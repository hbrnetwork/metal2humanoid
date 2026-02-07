"""
Render Asimov robot using MuJoCo's native viewer
This will definitely show the robot with all meshes properly loaded
"""
import mujoco
import mujoco.viewer
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('embodiments/asimov-v0/xmls/asimov.xml')
data = mujoco.MjData(model)

# Set initial pose - pelvis position (x, y, z) and orientation (quaternion w, x, y, z)
data.qpos[0:3] = [0, 0, 0.739]  # Position: x, y, z
data.qpos[3:7] = [1, 0, 0, 0]   # Orientation: w, x, y, z (quaternion)

# Option: Pin the pelvis in place by disabling the floating base joint
# This will keep the robot at the starting position with gravity on
# Uncomment the next line to pin the base:
# model.jnt_type[0] = mujoco.mjtJoint.mjJNT_FREE  # Keep as free joint but we'll control it

# Print model info
print(f"Model name: {model.names.decode('utf-8')}")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of degrees of freedom: {model.nv}")

# Launch interactive viewer
print("\nLaunching MuJoCo viewer...")
print("Controls:")
print("  - Left mouse: rotate view")
print("  - Right mouse: move view")
print("  - Scroll: zoom")
print("  - Double-click: select body")
print("  - Ctrl+right mouse: apply force")
print("  - Space: pause/resume")
print("  - Backspace: reset simulation")
print("  - Esc: exit")

# Use launch() instead of launch_passive() for macOS compatibility
mujoco.viewer.launch(model, data)
