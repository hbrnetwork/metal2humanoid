#!/bin/bash
# Queue 3: Berkeley Humanoid Lite without arms (stand + walk). Waits for queue 2 to finish first. Resumable.
cd "$(dirname "$(readlink -f "$0")")/.."   # the project directory (this script lives in scripts/)
export PYTHONUNBUFFERED=1 PYTHONWARNINGS=ignore
P=.venv/bin/python
until grep -q "QUEUE2 DONE" runs/queue2.log 2>/dev/null; do sleep 120; done
run() { echo "=== $(date "+%F %T") START: $*"; "$@"; echo "=== $(date "+%F %T") END (exit $?): $*"; }
train() { local name=$1; shift; [ -f runs/$name/DONE ] && { echo "=== skip $name (done)"; return; }
          run $P train.py "$@" --out runs/$name --video && touch runs/$name/DONE; }
emb=berkeley_humanoid_lite_noarms
train bhl_noarms_stand --embodiment $emb --task stand --num-envs 256 --total-steps 10e6
[ -f runs/bhl_noarms_stand/EVAL ] || { run $P evaluate.py --checkpoint runs/bhl_noarms_stand/best.pt --video videos/bhl_noarms_stand.mp4 --export export/bhl_noarms_stand
                                       run $P evaluate.py --checkpoint runs/bhl_noarms_stand/best.pt --push-interval 3 --episodes 32; touch runs/bhl_noarms_stand/EVAL; }
train bhl_noarms_walk --embodiment $emb --task walk --num-envs 512 --total-steps 80e6
[ -f runs/bhl_noarms_walk/EVAL ] || { run $P evaluate.py --checkpoint runs/bhl_noarms_walk/best.pt --cmd 0.5 0 0 --video videos/bhl_noarms_walk.mp4 --export export/bhl_noarms_walk
                                      run $P evaluate.py --checkpoint runs/bhl_noarms_walk/best.pt --cmd 0 0 0 --episodes 32
                                      run $P evaluate.py --checkpoint runs/bhl_noarms_walk/best.pt --cmd 0.3 0 0.5 --episodes 32
                                      run $P evaluate.py --checkpoint runs/bhl_noarms_walk/best.pt --cmd -0.3 0 0 --episodes 32; touch runs/bhl_noarms_walk/EVAL; }
echo "=== QUEUE3 DONE $(date)"
