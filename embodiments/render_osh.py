"""
Render Frankenstein robot (Asimov legs + Atom01 torso/arms) using MuJoCo's native viewer
"""
import mujoco
import mujoco.viewer
import numpy as np

# Load the Frankenstein robot model
model = mujoco.MjModel.from_xml_path('embodiments/frankenstein-v1.xml')
data = mujoco.MjData(model)

# Set initial pose - pelvis position (x, y, z) and orientation (quaternion w, x, y, z)
data.qpos[0:3] = [0, 0, 0.739]  # Position: x, y, z
data.qpos[3:7] = [1, 0, 0, 0]   # Orientation: w, x, y, z (quaternion - upright)

# Turn off gravity
model.opt.gravity[:] = [0, 0, -0.01]
print(f"Gravity: {model.opt.gravity}")

# Print model info
print(f"Model name: {model.names.decode('utf-8')}")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of degrees of freedom: {model.nv}")
print(f"Initial base height: {data.qpos[2]:.3f}m")
print("\nRendering Frankenstein Robot (Asimov legs + Atom01 torso/arms)")

# Initialize simulation state
mujoco.mj_forward(model, data)

# Launch interactive viewer
print("\nLaunching MuJoCo viewer...")
print("Controls:")
print("  - Left mouse: rotate view")
print("  - Right mouse: move view")
print("  - Scroll: zoom")
print("  - Double-click: select body")
print("  - Tab: enable/disable joint control sliders")
print("  - Space: pause/resume")
print("  - Backspace: reset simulation")
print("  - Esc: exit")

mujoco.viewer.launch(model, data)
