#!/usr/bin/env python3
"""Evaluate a checkpoint: metrics over parallel episodes, a video, and an optional export for the robot.

Examples
  python evaluate.py --checkpoint runs/asimov_stand/best.pt --video videos/asimov_stand.mp4
  python evaluate.py --checkpoint runs/asimov_walk/best.pt --cmd 0.5 0 0 --push-interval 3 --export export/asimov_walk
"""
import argparse
import json

import numpy as np

import standwalk  # noqa: F401
from standwalk.env import HumanoidVecEnv, TaskConfig
from standwalk.ppo import Policy, export_policy
from standwalk.render import rollout_video


def evaluate(policy: Policy, env: HumanoidVecEnv, seconds: float, cmd):
    env.set_commands(cmd, fixed=True)
    obs, _ = env.reset()
    n = int(seconds / env.dt)
    alive = np.ones(env.N, bool); fell = np.zeros(env.N, bool); ret = np.zeros(env.N); steps = np.zeros(env.N)
    vel_err, torque = [], []
    for t in range(n):
        obs, _, r, term, trunc, info = env.step(policy(obs))
        ret += r * alive; steps += alive
        vel_err.append(np.linalg.norm(env.base_lin_vel[:, :2] - env.commands[:, :2], axis=1)[alive].mean() if alive.any() else np.nan)
        torque.append(np.abs(env.torques)[alive].mean() if alive.any() else np.nan)
        fell |= term & alive
        alive &= ~(term | trunc)
        if not alive.any():
            break
    return {"command": list(map(float, cmd)), "episodes": env.N, "mean_return": float(ret.mean()),
            "mean_survival_s": float((steps * env.dt).mean()), "fall_rate": float(fell.mean()),
            "mean_vel_error_mps": float(np.nanmean(vel_err)), "mean_abs_torque_nm": float(np.nanmean(torque))}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--episodes", type=int, default=16)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--cmd", type=float, nargs=3, default=None, help="vx vy yaw_rate (default: 0 0 0 for stand, 0.5 0 0 for walk)")
    p.add_argument("--push-interval", type=float, default=0.0)
    p.add_argument("--obs-noise", type=float, default=0.0)
    p.add_argument("--video", default=None, help="output video path (.mp4)")
    p.add_argument("--video-seconds", type=float, default=8.0)
    p.add_argument("--export", default=None, help="write <export>.pt (TorchScript) + <export>.json for deployment")
    p.add_argument("--device", default="cpu")
    p.add_argument("--embodiments-dir", default=None)
    a = p.parse_args()

    pol = Policy(a.checkpoint, a.device)
    ck = pol.ckpt
    task = TaskConfig(**ck["task_config"])
    task.push_interval_seconds = a.push_interval; task.obs_noise = a.obs_noise
    cmd = a.cmd if a.cmd is not None else ([0.0, 0.0, 0.0] if task.task == "stand" else [0.5, 0.0, 0.0])
    print(f"checkpoint: {a.checkpoint}  embodiment={ck['embodiment']} task={task.task} "
          f"trained {ck['meta'].get('total_steps', '?'):,} steps (iteration {ck['meta'].get('iteration', '?')})")
    env = HumanoidVecEnv(ck["embodiment"], num_envs=a.episodes, task=task, seed=1000, embodiments_root=a.embodiments_dir)
    m = evaluate(pol, env, a.seconds, cmd)
    print(json.dumps(m, indent=2))
    env.close()
    if a.video:
        ev = HumanoidVecEnv(ck["embodiment"], num_envs=1, task=task, seed=7, n_threads=1, embodiments_root=a.embodiments_dir)
        _, stats, path = rollout_video(ev, pol, seconds=a.video_seconds, command=cmd, path=a.video)
        print(f"video: {path}  survived {stats['seconds']:.1f}s fell={stats['fell']}")
        ev.close()
    if a.export:
        out = export_policy(a.checkpoint, a.export)
        print(f"exported {out} and {out.with_suffix('.json')}")


if __name__ == "__main__":
    main()
