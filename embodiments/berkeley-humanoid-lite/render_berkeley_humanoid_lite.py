"""
Render Berkeley Humanoid Lite using MuJoCo's native viewer
"""
import mujoco
import mujoco.viewer
import numpy as np

# Load the Berkeley Humanoid Lite model
model = mujoco.MjModel.from_xml_path('embodiments/berkeley-humanoid-lite/xmls/berkeley_humanoid_lite_torso.xml')
data = mujoco.MjData(model)

# Set initial pose at a good height
# Base link is around 0.675m from URDF, adjust for standing pose
data.qpos[0:3] = [0, 0, 0.75]  # Position: x, y, z (base height)
data.qpos[3:7] = [1, 0, 0, 0]   # Orientation: w, x, y, z (upright)

# Turn on zero gravity
model.opt.gravity[:] = [0, 0, 0]
print(f"Gravity: {model.opt.gravity}")

# Print model info
print(f"Model name: {model.names.decode('utf-8')}")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of degrees of freedom: {model.nv}")
print(f"Initial base height: {data.qpos[2]:.3f}m")
print("\nRendering Berkeley Humanoid Lite")

# Launch interactive viewer
print("\nLaunching MuJoCo viewer (physics paused)...")
print("The robot will stay in place. Use Tab to enable joint sliders to pose it.")
print("\nControls:")
print("  - Left mouse: rotate view")
print("  - Right mouse: move view")
print("  - Scroll: zoom")
print("  - Double-click: select body")
print("  - Tab: enable/disable joint control sliders")
print("  - Space: pause/resume (starts paused)")
print("  - Backspace: reset simulation")
print("  - Esc: exit")

# Initialize simulation state
mujoco.mj_forward(model, data)

# Launch viewer - it starts with physics running
# The viewer will handle the rendering, user can pause with spacebar
mujoco.viewer.launch(model, data)
