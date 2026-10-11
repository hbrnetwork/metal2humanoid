"""standwalk: train stand / walk PPO policies for Metal2Humanoid embodiments in MuJoCo.

Modules
-------
embodiments  registry of robots (XML path, joints, PD gains, default pose) + model loader
env          HumanoidVecEnv: vectorised MuJoCo env with PD control, locomotion rewards
ppo          actor-critic, PPO trainer, checkpoints, policy export
render       offscreen rendering and video helpers
tools        sanity checks and benchmarks (python -m standwalk.tools)
"""
import os
import sys

# Headless Linux boxes (the GPU tower over SSH, Colab) need EGL for offscreen rendering.
# This must be set before `mujoco` is imported anywhere.
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

from .embodiments import REGISTRY, Embodiment, load_embodiment, list_embodiments  # noqa: E402
from .env import HumanoidVecEnv, TaskConfig, stand_config, walk_config  # noqa: E402

__all__ = [
    "REGISTRY", "Embodiment", "load_embodiment", "list_embodiments",
    "HumanoidVecEnv", "TaskConfig", "stand_config", "walk_config",
]
__version__ = "0.1.0"
