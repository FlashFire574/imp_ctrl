import mujoco
from geodude_assets import get_geodude_path

m = mujoco.MjModel.from_xml_path(str(get_geodude_path()))

print("JOINTS:")
for i in range(m.njnt):
    print("  ", m.joint(i).name)

print("SITES:")
for i in range(m.nsite):
    print("  ", m.site(i).name)

print("ACTUATORS:")
for i in range(m.nu):
    print("  ", m.actuator(i).name)

print("KEYFRAMES:", [m.key(i).name for i in range(m.nkey)])