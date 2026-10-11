"""Offscreen rendering + video for a HumanoidVecEnv (works headless with MUJOCO_GL=egl)."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


def tracking_camera(env, distance=None, azimuth=135.0, elevation=-20.0):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = env.le.base_body
    cam.distance = distance or max(1.5, 2.5 * env.nominal_com_height)
    cam.azimuth, cam.elevation = azimuth, elevation
    return cam


class EnvRenderer:
    def __init__(self, env, width=640, height=480, camera=None):
        self.env = env
        self.renderer = mujoco.Renderer(env.model, height=height, width=width)
        self.cam = camera or tracking_camera(env)
        self.opt = mujoco.MjvOption()

    def frame(self, env_idx: int = 0) -> np.ndarray:
        data = self.env.sync_data(env_idx)
        self.renderer.update_scene(data, camera=self.cam, scene_option=self.opt)
        return self.renderer.render().copy()

    def close(self):
        self.renderer.close()


def save_video(frames, path, fps=50) -> Path:
    """Save frames (list of HxWx3 uint8) as mp4 via imageio-ffmpeg, falling back to gif."""
    import imageio.v2 as imageio
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    try:
        imageio.mimwrite(str(path.with_suffix(".mp4")), frames, fps=fps, codec="libx264",
                         quality=8, macro_block_size=None)
        return path.with_suffix(".mp4")
    except Exception:  # no ffmpeg
        imageio.mimwrite(str(path.with_suffix(".gif")), frames[::2], duration=2.0 / fps)
        return path.with_suffix(".gif")


def rollout_video(env, policy_fn, seconds=8.0, command=None, path=None, width=640, height=480, env_idx=0,
                  deterministic=True, frame_skip=1):
    """Run policy_fn(obs)->actions in `env`, record env `env_idx`, return (frames, stats)."""
    if command is not None:
        env.set_commands(command, fixed=True)
    obs, _ = env.reset()
    r = EnvRenderer(env, width, height)
    frames, ret, steps, fell = [], 0.0, 0, False
    n = int(seconds / env.dt)
    for t in range(n):
        a = policy_fn(obs)
        obs, _, rew, term, trunc, info = env.step(a)
        ret += float(rew[env_idx]); steps += 1
        if t % frame_skip == 0:
            frames.append(r.frame(env_idx))
        if term[env_idx] or trunc[env_idx]:
            fell = bool(term[env_idx]); break
    r.close()
    stats = {"return": ret, "steps": steps, "seconds": steps * env.dt, "fell": fell,
             "mean_vel_xy": None}
    out = save_video(frames, path, fps=int(round(1 / env.dt / frame_skip))) if path else None
    return frames, stats, out
