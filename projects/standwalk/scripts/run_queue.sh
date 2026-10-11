#!/bin/bash
# Training queue for the standwalk project (runs sequentially; each uses all 8 cores + the GPU)
cd "$(dirname "$(readlink -f "$0")")/.."   # the project directory (this script lives in scripts/)
export PYTHONUNBUFFERED=1 PYTHONWARNINGS=ignore
P=.venv/bin/python
run() { echo "=== $(date "+%F %T") START: $*"; "$@"; echo "=== $(date "+%F %T") END (exit $?): $*"; }
run $P train.py --embodiment asimov --task stand --num-envs 256 --total-steps 10e6 --out runs/asimov_stand --video
run $P evaluate.py --checkpoint runs/asimov_stand/best.pt --video videos/asimov_stand.mp4 --export export/asimov_stand
run $P evaluate.py --checkpoint runs/asimov_stand/best.pt --push-interval 3 --episodes 32
run $P train.py --embodiment berkeley_humanoid_lite --task stand --num-envs 256 --total-steps 15e6 --out runs/bhl_stand --video
run $P evaluate.py --checkpoint runs/bhl_stand/best.pt --video videos/bhl_stand.mp4 --export export/bhl_stand
run $P train.py --embodiment asimov --task walk --num-envs 512 --total-steps 120e6 --out runs/asimov_walk --video
run $P evaluate.py --checkpoint runs/asimov_walk/best.pt --cmd 0.5 0 0 --video videos/asimov_walk.mp4 --export export/asimov_walk
run $P evaluate.py --checkpoint runs/asimov_walk/best.pt --cmd 0 0 0 --episodes 32
run $P evaluate.py --checkpoint runs/asimov_walk/best.pt --cmd 0.3 0 0.5 --episodes 32
echo "=== QUEUE DONE $(date)"
