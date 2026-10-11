# standwalk: stand & walk PPO policies for Metal2Humanoid embodiments

Train reinforcement-learning locomotion policies for the course robots
(Asimov, Berkeley Humanoid Lite, Atom01, the Frankensteins, plus the MuJoCo humanoid as a baseline)
in MuJoCo, with a control and observation interface designed to transfer to the real hardware.

* **Control**: the policy outputs joint-position targets at 50 Hz; a PD controller (MuJoCo `<position>` actuators,
  gains and torque limits from the registry) turns them into torques at 250 Hz, like a real motor driver.
* **Observations**: IMU angular velocity, gravity direction, joint positions/velocities, last action, velocity command,
  gait clock. Nothing a real robot cannot measure. The critic additionally sees privileged state.
* **Simulation**: hundreds of robots stepped in parallel with `mujoco.rollout` (C++ thread pool), ~20k env steps/s
  on an 8-core desktop, ~50k on an Apple M3. The policy network trains on the GPU if there is one.
* **Algorithm**: PPO in the rsl_rl / legged_gym style (asymmetric actor-critic, GAE with truncation bootstrapping,
  observation normalisation, KL-adaptive learning rate), ~250 lines in `standwalk/ppo.py`.

```
standwalk/
  embodiments.py   registry of robots + XML rewriting + loader (PD actuators, sensors, settle test)
  env.py           HumanoidVecEnv: parallel envs, observations, reward terms, resets, pushes, noise
  ppo.py           ActorCritic, PPO trainer, checkpoints, Policy wrapper, TorchScript export
  render.py        offscreen rendering, videos
  tools.py         python -m standwalk.tools {check,bench,render}
  embodiments_extra/   generated robot XMLs the course repository does not ship (no-arms Berkeley Humanoid Lite)
train.py           train a stand or walk policy from the command line
evaluate.py        metrics over parallel episodes, video, export for the robot
status.py          status table of every embodiment's stand / walk run (python status.py [--watch 60])
scripts/           run_queue*.sh: the sequential training queues that produced runs/, checkpoints/, export/, videos/
checkpoints/       best.pt of every run, self-describing (see checkpoints/README.txt)
export/            TorchScript + JSON per policy, everything the robot code needs
standwalk_project.ipynb               the assignment notebook (explanations + experiments)
standwalk_colab_policy_torques.ipynb  Colab: run a checkpoint, video with torque panel, motor-sizing table
gpu_bench/         side experiment: GPU benchmark (tower vs Colab), unrelated to the policies
```

## Setup

```bash
# python >= 3.10; uv or venv
uv venv --python 3.12 .venv && source .venv/bin/activate
uv pip install -r requirements.txt          # CUDA torch: install the wheel for your CUDA version first
# the embodiments: found automatically when this folder lives inside the metal2humanoid repository;
# standalone, clone metal2humanoid into or next to this directory (or set STANDWALK_EMBODIMENTS)
git clone https://github.com/hbrnetwork/metal2humanoid.git
python -m standwalk.tools check            # loads + settles every robot, prints a table
python -m standwalk.tools bench            # env steps/s on this machine
```

Headless Linux (SSH, Colab): rendering uses EGL automatically (`MUJOCO_GL=egl`).

## Train and evaluate

```bash
# stand: ~10 minutes on 8 cores
python train.py --embodiment asimov --task stand --num-envs 256 --total-steps 10e6 --out runs/asimov_stand --video
# walk: 1-3 hours on 8 cores
python train.py --embodiment asimov --task walk --num-envs 512 --total-steps 120e6 --out runs/asimov_walk --video
# robustness variants
python train.py --embodiment asimov --task stand --push-interval 3 --obs-noise 1 --out runs/asimov_stand_robust

python evaluate.py --checkpoint runs/asimov_walk/best.pt --cmd 0.5 0 0 --video videos/asimov_walk.mp4
python evaluate.py --checkpoint runs/asimov_walk/best.pt --push-interval 3 --episodes 32
python evaluate.py --checkpoint runs/asimov_walk/best.pt --export export/asimov_walk   # .pt + .json for the robot
```

The training queues in `scripts/` (`run_queue.sh`, `run_queue2.sh`, `run_queue3.sh`) are idempotent and resumable;
they produced every run in this folder. `python3 status.py` prints a table with the stand/walk status (trained / training / queued, return, fall rate, ETA) of every
embodiment from the local `runs/`; `python3 status.py --tower` shows the live state on the GPU tower; `--watch 60` refreshes.

Each run directory contains `config.json`, `progress.csv` (one row per iteration: return, episode length, fall rate,
per-term reward contributions, losses, KL, learning rate, steps/s), `latest.pt` and `best.pt`.

Embodiments: `asimov`, `berkeley_humanoid_lite`, `berkeley_humanoid_lite_noarms` (full torso + legs, arms removed; a
generated XML that the course repository does not ship, kept in `standwalk/embodiments_extra/` and used whenever the
embodiments folder lacks it), `atom01`, `frankenstein_v1`, `frankenstein_v0`, `mjc_humanoid`.

## Run a policy in Google Colab (video with torque panel + motor-sizing plots)

`standwalk_colab_policy_torques.ipynb` loads a trained checkpoint, runs it through a command schedule and produces a video
with every motor's torque (Nm and % of its limit) in the top-right corner, the full torque time series at the physics
rate, and a motor-sizing table (peak / RMS torque, utilisation, peak power). Nothing to upload: the notebook clones this
repository and takes the code, the checkpoints and the robot models from it.

* [Open the torque notebook in Colab](https://colab.research.google.com/github/hbrnetwork/metal2humanoid/blob/main/projects/standwalk/standwalk_colab_policy_torques.ipynb)
* [Open the assignment notebook in Colab](https://colab.research.google.com/github/hbrnetwork/metal2humanoid/blob/main/projects/standwalk/standwalk_project.ipynb)

After retraining, copy the new policy into `checkpoints/` (`cp runs/<run>/best.pt checkpoints/<run>.pt`) and commit it.

## How it fits together

1. `load_embodiment(name)` rewrites the robot XML (absolute asset paths, one `<position>` actuator per joint, an IMU
   site, touch-sensor boxes on the feet and base, a sensor block), compiles it, sets kp/kd/torque limits from the
   registry, places the robot on the floor in its default pose and holds it with the PD controller for 1.5 s
   (`stands` tells you whether PD alone keeps it up; only the light Asimov legs do, the rest need the policy).
2. `HumanoidVecEnv(name, num_envs, task)` keeps one physics-state vector per env and advances all of them with a single
   `rollout` call per control step. Everything the reward and policy need comes back as batched sensor arrays.
3. `train(env, PPOConfig, out_dir)` runs PPO; `Policy(ckpt)` runs a trained policy; `export_policy` writes the
   deployment files.

Reward weights live in `standwalk/env.py` (`DEFAULT_REWARDS`); `stand_config()` and `walk_config()` build task
configs and accept overrides, e.g. `walk_config(rewards={"feet_air_time": 2.0}, push_interval_seconds=3.0)`.

## Adding an embodiment

Add an `Embodiment(...)` entry to `REGISTRY` in `standwalk/embodiments.py` (XML path relative to the embodiments
directory, base body, foot bodies, PD rules per joint-name regex, default pose, swing-joint regex). The XML needs a
free joint on the base, named hinge joints and collision geoms on the feet. Then `python -m standwalk.tools check
<name> -v`.

## Troubleshooting

* **Jupyter kernel dies / segfault when the training cell starts (Linux)**: caused by Triton's C extension being loaded
  after MuJoCo's EGL renderer on a machine whose NVIDIA libraries do not match the running kernel driver (typically
  after an unattended driver upgrade). `standwalk.ppo` now imports `torch._dynamo` early to avoid it; rebooting the
  machine fixes the underlying mismatch, as does `pip uninstall triton` if you do not use `torch.compile`.
* **`cuda available: False` although the machine has a GPU**: same driver mismatch; reboot.
* **No video / rendering errors over SSH**: `MUJOCO_GL=egl` is set automatically on headless Linux; on a machine
  without EGL use `MUJOCO_GL=osmesa` (`apt install libosmesa6`).

## Known limitations / next steps

* PD gains and torque limits in the registry are plausible guesses, not measured values: set them to the real
  motor-driver gains before expecting transfer.
* No terrain, no per-env dynamics randomisation yet (the rollout API supports one model per env, so it is a small
  change), no action delay.
* The MuJoCo humanoid entry is a pipeline sanity check, not a robot.
