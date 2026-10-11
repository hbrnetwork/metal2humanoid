#!/usr/bin/env python3
"""Training status of the stand / walk policies for every embodiment.

  python status.py              # reads ./runs on this machine
  python status.py --tower      # runs the same script on the GPU tower over ssh (live view)
  python status.py --watch 60   # refresh every 60 s

Status: trained (budget reached), training (log updated < 3 min ago), interrupted (partial run, the
queue will redo it), queued (planned, not started).  ETAs assume the queue runs in the planned order.
"""
import argparse
import csv
import datetime as dt
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

EMBODIMENTS = [("asimov", "asimov"), ("berkeley", "bhl"), ("berkeley (no arms)", "bhl_noarms"),
               ("robot-origin (atom01)", "atom01"), ("frankenstein-v0", "frankenstein_v0"), ("frankenstein-v1", "frankenstein_v1")]
TASKS = ["stand", "walk"]
# planned queue order and step budgets (queue 1, then queue 2)
PLAN = [("asimov_stand", 10e6), ("bhl_stand", 15e6), ("asimov_walk", 120e6),
        ("bhl_walk", 80e6), ("frankenstein_v1_stand", 10e6), ("frankenstein_v1_walk", 80e6),
        ("frankenstein_v0_stand", 10e6), ("frankenstein_v0_walk", 80e6),
        ("atom01_stand", 10e6), ("atom01_walk", 80e6),
        ("bhl_noarms_stand", 10e6), ("bhl_noarms_walk", 80e6)]
STALE_S = 180          # a run whose log is older than this is not training any more
FALLBACK_SPS = 7000.0  # used for ETAs when nothing has been measured yet
TOWER_HOST = "kompot"                       # the GPU tower where the queues run
TOWER_DIR = "~/Documents/GitHub/standwalk"  # the project directory on the tower (for --tower over ssh)


def inspect(run_dir: Path, budget: float) -> dict:
    info = {"status": "queued", "steps": 0.0, "total": budget, "sps": None, "ret": None, "falls": None,
            "minutes": None, "age": None, "finished_at": None}
    cfg = run_dir / "config.json"
    if cfg.exists():
        try:
            info["total"] = float(json.load(open(cfg))["ppo"]["total_steps"])
        except Exception:
            pass
    prog = run_dir / "progress.csv"
    if not prog.exists():
        return info
    try:
        with open(prog) as f:
            rows = list(csv.DictReader(f))
    except Exception:
        rows = []
    if not rows:
        return info
    last = rows[-1]
    tail = rows[-20:]
    info["steps"] = float(last["total_steps"])
    info["minutes"] = float(last["time_s"]) / 60
    info["sps"] = sum(float(r["sps"]) for r in tail) / len(tail)
    rets = [float(r["return"]) for r in tail if r.get("return") not in ("", None)]
    falls = [float(r["fall_rate"]) for r in tail if r.get("fall_rate") not in ("", None)]
    info["ret"] = sum(rets) / len(rets) if rets else None
    info["falls"] = sum(falls) / len(falls) if falls else None
    mtime = prog.stat().st_mtime
    info["age"] = time.time() - mtime
    if (run_dir / "DONE").exists() or info["steps"] >= 0.995 * info["total"]:
        info["status"] = "trained"
        info["finished_at"] = dt.datetime.fromtimestamp(mtime)
    elif info["age"] < STALE_S:
        info["status"] = "training"
    else:
        info["status"] = "interrupted"
    return info


def network_device(root: Path) -> str:
    dev = None
    for log in sorted(root.glob("runs/queue*.log"), key=lambda p: p.stat().st_mtime):
        for line in open(log, errors="ignore"):
            m = re.search(r"device=(\w+)", line)
            if m:
                dev = m.group(1)
    return dev or "unknown"


def fmt_dur(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h, m = divmod(seconds // 60, 60)
    return f"{h}h {m:02d}m" if h else f"{m}m"


def build_table(root: Path):
    runs = {name: inspect(root / "runs" / name, budget) for name, budget in PLAN}
    # ETA bookkeeping: walk the plan in order; queued runs start after the ones before them
    sps_est = next((r["sps"] for r in runs.values() if r["status"] == "training" and r["sps"]), None) \
        or next((r["sps"] for r in reversed(list(runs.values())) if r["status"] == "trained" and r["sps"]), None) \
        or FALLBACK_SPS
    clock = time.time()
    etas = {}
    for name, _ in PLAN:
        r = runs[name]
        if r["status"] == "trained":
            continue
        remaining = r["total"] - (r["steps"] if r["status"] == "training" else 0.0)
        rate = r["sps"] if (r["status"] == "training" and r["sps"]) else sps_est
        clock += remaining / rate + 60  # + a minute for evaluation / video
        etas[name] = clock
    rows = []
    for label, key in EMBODIMENTS:
        for task in TASKS:
            name = f"{key}_{task}"
            r = runs[name]
            prog = f"{r['steps']/1e6:5.1f}M / {r['total']/1e6:5.1f}M"
            pct = f"({100*r['steps']/r['total']:3.0f}%)" if r["status"] != "trained" and r["steps"] else "      "
            ret = f"{r['ret']:6.1f}" if r["ret"] is not None else "     -"
            falls = f"{r['falls']:5.2f}" if r["falls"] is not None else "    -"
            if r["status"] == "trained":
                when = f"done in {r['minutes']:.0f} min ({r['finished_at']:%a %H:%M})"
            elif name in etas:
                when = f"ETA {fmt_dur(etas[name]-time.time())} (~{dt.datetime.fromtimestamp(etas[name]):%a %H:%M})"
            else:
                when = ""
            rows.append((label, task, r["status"], f"{prog} {pct}", ret, falls, when))
    return rows


def print_table(root: Path):
    now = dt.datetime.now().strftime("%a %Y-%m-%d %H:%M:%S")
    dev = network_device(root)
    hint = "" if dev == "cuda" else "  (CPU only: reboot the tower to restore CUDA)" if dev == "cpu" else ""
    print(f"standwalk training status  {now}   host={os.uname().nodename}   network device: {dev}{hint}")
    if os.uname().nodename != TOWER_HOST:   # not on the tower: the runs here are copies
        print(f"reading copies in {root/'runs'} -- for live progress and ETAs: python3 status.py --tower")
    print()
    hdr = ("embodiment", "task", "status", "progress", "return", "falls", "time / ETA")
    rows = build_table(root)
    widths = [max(len(str(x)) for x in col) for col in zip(hdr, *rows)]
    line = "  ".join(f"{h:<{w}}" for h, w in zip(hdr, widths))
    print(line); print("-" * len(line))
    marks = {"trained": "✓", "training": "▶", "interrupted": "!", "queued": "·"}
    for row in rows:
        row = list(row); row[2] = f"{marks.get(row[2], ' ')} {row[2]}"
        print("  ".join(f"{str(x):<{w}}" for x, w in zip(row, widths)))
    print()
    print("return = mean episode return over the last 20 iterations (20 s episodes; ~45 is the ceiling), "
          "falls = fall rate over the same window")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", default=str(Path(__file__).resolve().parent), help="project directory containing runs/")
    p.add_argument("--tower", action="store_true", help="run this script on the GPU tower over ssh")
    p.add_argument("--host", default=TOWER_HOST, help="ssh host for --tower")
    p.add_argument("--watch", type=int, default=0, help="refresh every N seconds")
    a = p.parse_args()
    if a.tower:
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", a.host,
               f"cd {TOWER_DIR} && .venv/bin/python status.py" + (f" --watch {a.watch}" if a.watch else "")]
        sys.exit(subprocess.call(cmd))
    while True:
        if a.watch:
            print("\033[2J\033[H", end="")
        print_table(Path(a.root))
        if not a.watch:
            break
        time.sleep(a.watch)


if __name__ == "__main__":
    main()
