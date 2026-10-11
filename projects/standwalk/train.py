#!/usr/bin/env python3
"""Train a stand or walk PPO policy for one embodiment.

Examples
  python train.py --embodiment asimov --task stand --total-steps 5e6
  python train.py --embodiment asimov --task walk  --total-steps 100e6 --resume runs/asimov_stand/best.pt
"""
import argparse
import time
from pathlib import Path

import standwalk  # noqa: F401  (sets MUJOCO_GL on headless linux)
from standwalk.embodiments import REGISTRY
from standwalk.env import HumanoidVecEnv, stand_config, walk_config
from standwalk.ppo import PPOConfig, train


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--embodiment", default="asimov", choices=list(REGISTRY))
    p.add_argument("--task", default="stand", choices=["stand", "walk"])
    p.add_argument("--num-envs", type=int, default=256)
    p.add_argument("--total-steps", type=float, default=10e6)
    p.add_argument("--out", default=None, help="run directory (default runs/<embodiment>_<task>)")
    p.add_argument("--resume", default=None, help="checkpoint to initialise from (e.g. a stand policy)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--threads", type=int, default=None, help="simulation threads (default: min(8, cpu count))")
    p.add_argument("--device", default="auto")
    p.add_argument("--embodiments-dir", default=None)
    p.add_argument("--n-steps", type=int, default=24)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--hidden", default="512,256,128")
    p.add_argument("--push-interval", type=float, default=0.0, help="seconds between random pushes (0 = off)")
    p.add_argument("--obs-noise", type=float, default=0.0, help="observation noise level (1.0 = legged_gym defaults)")
    p.add_argument("--episode-seconds", type=float, default=20.0)
    p.add_argument("--save-every", type=int, default=50)
    p.add_argument("--video", action="store_true", help="record a short evaluation video at the end")
    a = p.parse_args()

    make = stand_config if a.task == "stand" else walk_config
    task = make(push_interval_seconds=a.push_interval, obs_noise=a.obs_noise, episode_seconds=a.episode_seconds)
    out = Path(a.out or f"runs/{a.embodiment}_{a.task}")
    t0 = time.time()
    env = HumanoidVecEnv(a.embodiment, num_envs=a.num_envs, task=task, seed=a.seed, n_threads=a.threads,
                         embodiments_root=a.embodiments_dir)
    print(f"[{a.embodiment}/{a.task}] {env.N} envs, {env.n} actuated joints, stands under PD: {env.le.stands}, "
          f"env build {time.time()-t0:.1f}s, out={out}")
    cfg = PPOConfig(total_steps=int(a.total_steps), n_steps=a.n_steps, lr=a.lr, seed=a.seed, device=a.device,
                    hidden=tuple(int(h) for h in a.hidden.split(",")), save_every=a.save_every)
    train(env, cfg, out, resume=a.resume)
    env.close()
    if a.video:
        from standwalk.ppo import Policy
        from standwalk.render import rollout_video
        ev = HumanoidVecEnv(a.embodiment, num_envs=1, task=task, seed=123, n_threads=1, embodiments_root=a.embodiments_dir)
        pol = Policy(out / "best.pt" if (out / "best.pt").exists() else out / "latest.pt")
        cmd = [0.0, 0.0, 0.0] if a.task == "stand" else [0.5, 0.0, 0.0]
        _, stats, path = rollout_video(ev, pol, seconds=8.0, command=cmd, path=out / "video.mp4")
        print(f"video: {path}  {stats}")
        ev.close()


if __name__ == "__main__":
    main()
