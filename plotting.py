import numpy as np
import matplotlib.pyplot as plt


def report(log, x_start):
    t = np.array(log["t"])
    pos = np.array(log["pos"])
    tgt = np.array(log["target"])

    err_2d = np.linalg.norm(pos[:, :2] - tgt[:, :2], axis=1)
    err_3d = np.linalg.norm(pos - tgt, axis=1)
    z_dev = (pos[:, 2] - x_start[2]) * 1000

    print(f"mean XY tracking error: {err_2d.mean()*1000:.1f} mm, max: {err_2d.max()*1000:.1f} mm")
    print(f"mean 3D tracking error: {err_3d.mean()*1000:.1f} mm, max: {err_3d.max()*1000:.1f} mm")
    print(f"z drift (peak-to-peak): {(pos[:,2].max()-pos[:,2].min())*1000:.1f} mm")
    print(f"max orientation error: {np.degrees(max(log['ori_err'])):.2f} deg")

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    ax[0].plot(tgt[:, 0], tgt[:, 1], "--", label="target")
    ax[0].plot(pos[:, 0], pos[:, 1], label="actual")
    ax[0].set_aspect("equal")
    ax[0].set(title="XY Spatial Path Comparison", xlabel="X Position (m)", ylabel="Y Position (m)")
    ax[0].legend(); ax[0].grid(True, linestyle="--", alpha=0.6)

    ax[1].plot(t, err_2d * 1000, label="2D Error (XY Plane)", color="crimson")
    ax[1].plot(t, z_dev, label="Z Deviation from Start", color="royalblue", alpha=0.7)
    ax[1].set(title="Tracking Deviations Over Time", xlabel="Time (s)", ylabel="Error (mm)")
    ax[1].legend(); ax[1].grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.show()