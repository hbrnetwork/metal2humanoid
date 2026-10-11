#!/bin/bash
# Queue 2: BHL walk, then stand + walk for frankenstein_v1, frankenstein_v0, atom01.
# Idempotent: a run with runs/<name>/DONE is skipped, so the queue can be relaunched after a reboot.
cd "$(dirname "$(readlink -f "$0")")/.."   # the project directory (this script lives in scripts/)
export PYTHONUNBUFFERED=1 PYTHONWARNINGS=ignore
P=.venv/bin/python
run() { echo "=== $(date "+%F %T") START: $*"; "$@"; echo "=== $(date "+%F %T") END (exit $?): $*"; }
train() {  # $1 name, rest = train.py args
  local name=$1; shift
  if [ -f runs/$name/DONE ]; then echo "=== skip $name (done)"; return; fi
  run $P train.py "$@" --out runs/$name --video && touch runs/$name/DONE
}
walk_evals() {
  run $P evaluate.py --checkpoint runs/$1/best.pt --cmd 0.5 0 0 --video videos/$1.mp4 --export export/$1
  run $P evaluate.py --checkpoint runs/$1/best.pt --cmd 0 0 0 --episodes 32
  run $P evaluate.py --checkpoint runs/$1/best.pt --cmd 0.3 0 0.5 --episodes 32
  run $P evaluate.py --checkpoint runs/$1/best.pt --cmd -0.3 0 0 --episodes 32
}
train bhl_walk --embodiment berkeley_humanoid_lite --task walk --num-envs 512 --total-steps 80e6
[ -f runs/bhl_walk/EVAL ] || { walk_evals bhl_walk; touch runs/bhl_walk/EVAL; }
for emb in frankenstein_v1 frankenstein_v0 atom01; do
  train ${emb}_stand --embodiment $emb --task stand --num-envs 256 --total-steps 10e6
  [ -f runs/${emb}_stand/EVAL ] || { run $P evaluate.py --checkpoint runs/${emb}_stand/best.pt --video videos/${emb}_stand.mp4 --export export/${emb}_stand
                                     run $P evaluate.py --checkpoint runs/${emb}_stand/best.pt --push-interval 3 --episodes 32; touch runs/${emb}_stand/EVAL; }
  train ${emb}_walk --embodiment $emb --task walk --num-envs 512 --total-steps 80e6
  [ -f runs/${emb}_walk/EVAL ] || { walk_evals ${emb}_walk; touch runs/${emb}_walk/EVAL; }
done
echo "=== QUEUE2 DONE $(date)"
