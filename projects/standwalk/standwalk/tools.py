"""Sanity checks and benchmarks.  python -m standwalk.tools {check,bench,render} [...]"""
from __future__ import annotations

import argparse
import warnings


def cmd_check(args):
    from .embodiments import REGISTRY, load_embodiment, describe
    names = args.embodiment or list(REGISTRY)
    print(f"{'embodiment':24s} {'DoF':>4s} {'mass':>7s} {'CoM/base h':>11s} {'PD-hold tilt':>13s} stands")
    for n in names:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            le = load_embodiment(n, embodiments_root=args.embodiments_dir)
        print(f"{n:24s} {le.n_act:4d} {le.total_mass:6.1f}kg {le.nominal_height:10.3f}m {le.settle_tilt_deg:12.1f}° "
              f"{'yes' if le.stands else 'no'}")
        if args.verbose:
            print(describe(le)); print()


def cmd_bench(args):
    from .env import benchmark
    sps = benchmark(args.embodiment, args.num_envs, args.steps, args.threads, embodiments_root=args.embodiments_dir)
    print(f"{args.embodiment}: {sps:,.0f} env steps/s with {args.num_envs} envs, {args.threads or 'auto'} threads")


def cmd_render(args):
    from .env import HumanoidVecEnv, stand_config
    from .render import EnvRenderer
    import imageio.v2 as imageio
    env = HumanoidVecEnv(args.embodiment, num_envs=1, task=stand_config(init_joint_noise=0.0, init_yaw_random=False),
                         embodiments_root=args.embodiments_dir)
    r = EnvRenderer(env, 640, 480)
    imageio.imwrite(args.out, r.frame(0))
    print("wrote", args.out)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m standwalk.tools")
    p.add_argument("--embodiments-dir", default=None)
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check", help="load every embodiment, settle it, print a table"); c.add_argument("embodiment", nargs="*")
    c.add_argument("-v", "--verbose", action="store_true"); c.set_defaults(fn=cmd_check)
    b = sub.add_parser("bench", help="environment steps/second"); b.add_argument("--embodiment", default="asimov")
    b.add_argument("--num-envs", type=int, default=128); b.add_argument("--steps", type=int, default=100)
    b.add_argument("--threads", type=int, default=None); b.set_defaults(fn=cmd_bench)
    r = sub.add_parser("render", help="render the standing pose to an image"); r.add_argument("--embodiment", default="asimov")
    r.add_argument("--out", default="pose.png"); r.set_defaults(fn=cmd_render)
    args = p.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
