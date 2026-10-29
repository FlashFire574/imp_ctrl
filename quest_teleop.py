import threading, time
import numpy as np
from oculus_reader import OculusReader

# Same Quest (y-up) -> z-up conversion as your publisher
C = np.array([[1, 0, 0],
              [0, 0, -1],
              [0, 1, 0]], dtype=float)

class QuestTeleop:
    def __init__(self, sides, scale=1.0, yaw_deg=0.0, hz=60):
        self.reader = OculusReader()
        self.sides = sides
        self.scale = scale
        # rotate about z so "forward" for you matches the robot's forward
        a = np.deg2rad(yaw_deg)
        Rz = np.array([[np.cos(a), -np.sin(a), 0],
                       [np.sin(a),  np.cos(a), 0],
                       [0, 0, 1]])
        self.R = Rz @ C
        self.poses, self.buttons = {}, {}
        self.state = {s: dict(active=False, p_c0=None, x_e0=None, target=None)
                      for s in sides}
        self._lock = threading.Lock()
        self._hz = hz
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                poses, buttons = self.reader.get_transformations_and_buttons()
                with self._lock:
                    if poses:
                        self.poses, self.buttons = poses, buttons
            except Exception as e:
                print("Quest read error:", e)
            time.sleep(1 / self._hz)

    def get_target(self, side, x_current):
        """Returns the Cartesian target for this arm; holds when grip is released."""
        k = "r" if side == "right" else "l"
        grip_key = f"{side}Grip"
        st = self.state[side]
        with self._lock:
            T = self.poses.get(k)
            grip = self.buttons.get(grip_key, (0.0,))[0] > 0.5
        if st["target"] is None:
            st["target"] = x_current.copy()
        if T is None:
            return st["target"]

        p_c = self.R @ T[:3, 3]
        if grip and not st["active"]:            # clutch engaged
            st.update(active=True, p_c0=p_c.copy(), x_e0=st["target"].copy())
        elif not grip:
            st["active"] = False
        if st["active"]:
            st["target"] = st["x_e0"] + self.scale * (p_c - st["p_c0"])
        return st["target"]

    def trigger(self, side):
        with self._lock:
            return self.buttons.get(f"{side}Trig", (0.0,))[0]