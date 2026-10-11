"""Vectorised MuJoCo environment for stand / walk tasks.

Design (mirrors legged_gym / IsaacLab / mujoco_playground, scaled down to a CPU):
  * N independent MuJoCo worlds share one MjModel. Each control step is ONE call to
    mujoco.rollout, which advances every world by `decimation` physics steps in a C++
    thread pool and hands back batched state + sensor arrays (no per-env Python loop).
  * Everything the policy or the reward needs is read from MuJoCo sensors that the loader
    adds to the model (IMU gyro/velocimeter/orientation, foot positions, touch sensors...).
  * The policy runs at 50 Hz and outputs joint-position *offsets* from a default pose.
    MuJoCo <position> actuators turn targets into torques with a PD law at the physics
    rate, exactly like the PD loop on a real robot's motor driver.
  * Observations are only things a real robot can measure: IMU angular velocity,
    gravity direction from the IMU, joint positions/velocities, the last action and the
    velocity command. The critic additionally sees privileged state (base velocity,
    foot contacts, height) which speeds learning without leaking into the deployed actor.
  * Rewards are a weighted sum of named terms; weights are per-second rates multiplied
    by the control timestep, so they do not depend on the control frequency.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field, asdict

import mujoco
import numpy as np
from mujoco import rollout as mj_rollout

from .embodiments import LoadedEmbodiment, load_embodiment

# ---------------------------------------------------------------------------------------
# Task configuration
# ---------------------------------------------------------------------------------------

DEFAULT_REWARDS = {
    # task
    "tracking_lin_vel": 1.0,     # exp(-|cmd_xy - v_xy|^2 / sigma)
    "tracking_ang_vel": 0.5,     # exp(-(cmd_yaw - w_z)^2 / sigma)
    "alive": 0.2,                # constant, as long as the episode is running
    # gait shaping
    "feet_air_time": 1.0,        # reward long swings (touchdown event, walking only)
    "gait_contact": 0.5,         # feet follow the stance/swing schedule of the gait clock
    "feet_swing_height": 0.5,    # swing foot tracks a clearance profile
    # regularisation / sim2real
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -2.0,         # keep the base upright
    "base_height": -10.0,        # (com height - nominal)^2
    "torques": -0.05,            # sum (tau / tau_max)^2
    "dof_acc": -2.5e-7,
    "dof_vel": 0.0,
    "action_rate": -0.02,
    "posture": -1.0,             # non-swing joints stay near the default pose
    "legs_pose": 0.0,            # leg pitch joints stay near the default pose (stand task)
    "dof_pos_limits": -1.0,
    "feet_slip": -0.1,
    # one-off penalty when the robot falls (NOT scaled by dt)
    "termination": -2.0,
}


@dataclass
class TaskConfig:
    task: str = "walk"
    episode_seconds: float = 20.0
    # velocity commands [vx, vy, yaw rate] sampled uniformly from these ranges
    cmd_vx: tuple = (-0.4, 0.8)
    cmd_vy: tuple = (-0.3, 0.3)
    cmd_yaw: tuple = (-0.8, 0.8)
    cmd_resample_seconds: float = 5.0
    cmd_zero_prob: float = 0.2          # fraction of commands that are "stand still"
    cmd_deadband: float = 0.1           # |cmd| below this counts as standing
    rewards: dict = field(default_factory=lambda: dict(DEFAULT_REWARDS))
    tracking_sigma: float = 0.25
    air_time_target: float | None = None  # [s]; None = half a gait cycle (0.5 / gait_freq), consistent with the gait clock
    use_gait_phase: bool = True
    # termination
    max_tilt_deg: float = 50.0
    min_com_height_ratio: float = 0.5   # fallen if CoM height < ratio * nominal
    terminate_on_body_contact: bool = True
    # randomisation
    init_joint_noise: float = 0.05      # [rad]
    init_yaw_random: bool = True
    push_interval_seconds: float = 0.0  # 0 = off
    push_max_vel: float = 0.5           # [m/s]
    obs_noise: float = 0.0              # 0 = off, 1.0 = legged_gym default noise levels
    # observation scales (keep raw observations O(1))
    scale_ang_vel: float = 0.25
    scale_lin_vel: float = 2.0
    scale_dof_vel: float = 0.05
    scale_cmd: tuple = (2.0, 2.0, 0.25)
    action_clip: float = 5.0

    def to_dict(self):
        return asdict(self)


def walk_config(**overrides) -> TaskConfig:
    cfg = TaskConfig(task="walk")
    for k, v in overrides.items():
        if k == "rewards":
            cfg.rewards.update(v)
        else:
            setattr(cfg, k, v)
    return cfg


def stand_config(**overrides) -> TaskConfig:
    """Stand = walk with a zero velocity command plus a pull towards the default leg pose."""
    cfg = TaskConfig(task="stand", cmd_vx=(0.0, 0.0), cmd_vy=(0.0, 0.0), cmd_yaw=(0.0, 0.0), cmd_zero_prob=1.0)
    cfg.rewards.update({"legs_pose": -1.0, "feet_air_time": 0.0, "feet_swing_height": 0.0})
    for k, v in overrides.items():
        if k == "rewards":
            cfg.rewards.update(v)
        else:
            setattr(cfg, k, v)
    return cfg


# ---------------------------------------------------------------------------------------
# Small math helpers (batched over environments)
# ---------------------------------------------------------------------------------------

def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate world vectors v (N,3) into the body frame given MuJoCo quats q (N,4) = (w,x,y,z)."""
    w, xyz = q[:, :1], q[:, 1:]
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(xyz, v) * (2.0 * w)
    c = xyz * (np.sum(xyz * v, axis=1, keepdims=True) * 2.0)
    return a - b + c


# ---------------------------------------------------------------------------------------
# The environment
# ---------------------------------------------------------------------------------------

class HumanoidVecEnv:
    """N copies of one embodiment, stepped in lock-step. All arrays are (N, ...) numpy float32."""

    def __init__(self, embodiment: str | LoadedEmbodiment, num_envs: int = 64, task: TaskConfig | None = None,
                 seed: int = 0, n_threads: int | None = None, embodiments_root=None):
        self.le: LoadedEmbodiment = embodiment if isinstance(embodiment, LoadedEmbodiment) else \
            load_embodiment(embodiment, embodiments_root=embodiments_root)
        self.cfg = task or walk_config()
        self.model = self.le.model
        self.N = int(num_envs)
        self.n = self.le.n_act
        self.dt = self.le.emb.ctrl_dt
        self.decimation = self.le.decimation
        self.rng = np.random.default_rng(seed)
        self.n_threads = n_threads or max(1, min(8, os.cpu_count() or 1))
        self.rollout = mj_rollout.Rollout(nthread=self.n_threads)
        self.datas = [mujoco.MjData(self.model) for _ in range(self.n_threads)]   # scratch, one per thread
        self.render_data = mujoco.MjData(self.model)

        le, m = self.le, self.model
        # physics state of every env: [time, qpos, qvel, act] (mjSTATE_FULLPHYSICS)
        self.nq, self.nv = m.nq, m.nv
        self.state = np.zeros((self.N, le.nstate))
        self._ctrl = np.zeros((self.N, le.decimation, self.n))          # same target on every substep
        self._models = [m] * self.N                                      # one (shared) model per env
        self._state_out = np.zeros((self.N, le.decimation, le.nstate))
        self._sens_out = np.zeros((self.N, le.decimation, m.nsensordata))
        ss = le.sensor_slices
        self._s_quat, self._s_gyro, self._s_vel = ss["base_quat"], ss["base_gyro"], ss["base_velocimeter"]
        self._s_pos, self._s_com = ss["base_pos"], ss["com"]
        self._s_foot_pos = [ss["foot_pos_L"], ss["foot_pos_R"]]
        self._s_foot_vel = [ss["foot_vel_L"], ss["foot_vel_R"]]
        self._s_tau = slice(ss[f"tau_{le.joint_names[0]}"].start, ss[f"tau_{le.joint_names[-1]}"].stop)
        self.max_steps = int(round(self.cfg.episode_seconds / self.dt))
        self.air_time_target = self.cfg.air_time_target if self.cfg.air_time_target is not None else 0.5 / le.emb.gait_freq
        self.resample_steps = max(1, int(round(self.cfg.cmd_resample_seconds / self.dt)))
        self.push_steps = int(round(self.cfg.push_interval_seconds / self.dt)) if self.cfg.push_interval_seconds > 0 else 0
        self.default_pose = le.default_pose.astype(np.float32)
        self.swing_mask = le.swing_mask
        self.posture_mask = ~le.swing_mask
        jr = le.joint_range
        mid, half = (jr[:, 0] + jr[:, 1]) / 2, (jr[:, 1] - jr[:, 0]) / 2
        self.soft_lo, self.soft_hi = (mid - 0.9 * half).astype(np.float32), (mid + 0.9 * half).astype(np.float32)

        # nominal CoM height in the standing pose (robust even when the base frame sits at floor level)
        d0 = self.render_data
        d0.qpos[:] = le.init_qpos
        mujoco.mj_forward(m, d0)
        self.nominal_com_height = float(d0.subtree_com[le.base_body, 2])

        # ---- state buffers ----
        N, n = self.N, self.n
        f32 = np.float32
        self.q = np.zeros((N, n), f32); self.qd = np.zeros((N, n), f32); self.qd_prev = np.zeros((N, n), f32)
        self.torques = np.zeros((N, n), f32)
        self.base_quat = np.zeros((N, 4)); self.base_pos = np.zeros((N, 3)); self.com = np.zeros((N, 3))
        self.base_lin_vel = np.zeros((N, 3), f32); self.base_ang_vel = np.zeros((N, 3), f32)
        self.proj_grav = np.zeros((N, 3), f32)
        self.foot_pos = np.zeros((N, 2, 3)); self.foot_vel = np.zeros((N, 2, 3))
        self.foot_contact = np.zeros((N, 2), bool); self.last_contact = np.zeros((N, 2), bool)
        self.body_contact = np.zeros(N, bool)
        self.air_time = np.zeros((N, 2), f32)
        self.actions = np.zeros((N, n), f32); self.prev_actions = np.zeros((N, n), f32)
        self.commands = np.zeros((N, 3), f32)
        self.fixed_commands = False
        self.phase = np.zeros(N, f32)
        self.ep_step = np.zeros(N, int)
        self.ep_return = np.zeros(N, f32)
        self.reward_names = list(self.cfg.rewards.keys())
        self.ep_term_sums = np.zeros((N, len(self.reward_names)), f32)
        self._stats: list[dict] = []
        self._gravity = np.array([[0.0, 0.0, -1.0]]).repeat(N, axis=0)

        # observation layout (documented so the same vector can be built on the robot)
        self.obs_layout = [("command", 3), ("base_ang_vel", 3), ("projected_gravity", 3),
                           ("joint_pos_minus_default", n), ("joint_vel", n), ("last_action", n)]
        if self.cfg.use_gait_phase:
            self.obs_layout.append(("gait_phase_sin_cos", 2))
        self.obs_dim = sum(s for _, s in self.obs_layout)
        self.critic_extra_layout = [("base_lin_vel", 3), ("foot_contact", 2), ("com_height_error", 1)]
        self.critic_obs_dim = self.obs_dim + sum(s for _, s in self.critic_extra_layout)
        self.act_dim = n
        self.reset()

    # ------------------------------------------------------------------ properties
    @property
    def joint_names(self):
        return self.le.joint_names

    @property
    def num_envs(self):
        return self.N

    # ------------------------------------------------------------------ physics
    def _physics_step(self):
        """Advance all envs by one control period (decimation substeps) in the C++ thread pool."""
        self.rollout.rollout(self._models, self.datas, self.state, self._ctrl, nstep=self.decimation,
                             state=self._state_out, sensordata=self._sens_out, skip_checks=True)
        self.state[:] = self._state_out[:, -1]
        self._gather(self._sens_out[:, -1])

    def _gather(self, sens):
        """Batched read of the quantities the env needs from the state and sensor arrays."""
        le, nq = self.le, self.nq
        qpos = self.state[:, 1:1 + nq]
        qvel = self.state[:, 1 + nq:1 + nq + self.nv]
        self.q[:] = qpos[:, le.qpos_adr]
        self.qd[:] = qvel[:, le.dof_adr]
        self.torques[:] = sens[:, self._s_tau]
        self.base_quat[:] = sens[:, self._s_quat]
        self.base_ang_vel[:] = sens[:, self._s_gyro]
        self.base_lin_vel[:] = sens[:, self._s_vel]
        self.base_pos[:] = sens[:, self._s_pos]
        self.com[:] = sens[:, self._s_com]
        for f in range(2):
            self.foot_pos[:, f] = sens[:, self._s_foot_pos[f]]
            self.foot_vel[:, f] = sens[:, self._s_foot_vel[f]]
            self.foot_contact[:, f] = sum(sens[:, sl].sum(1) for sl in le.touch_feet[f]) > 1.0
        self.body_contact[:] = (sum(sens[:, sl].sum(1) for sl in le.touch_base) > 1.0) if le.touch_base else False
        self.proj_grav[:] = quat_rotate_inverse(self.base_quat, self._gravity)

    @property
    def substep_torques(self) -> np.ndarray:
        """Actuator torques at every physics substep of the last control step: (N, decimation, n_act) [Nm]."""
        return self._sens_out[:, :, self._s_tau]

    @property
    def substep_joint_vel(self) -> np.ndarray:
        """Actuated joint velocities at every physics substep of the last control step: (N, decimation, n_act)."""
        return self._state_out[:, :, 1 + self.nq + self.le.dof_adr]

    def sync_data(self, i: int, data: mujoco.MjData | None = None) -> mujoco.MjData:
        """Load env i's physics state into an MjData (for rendering / inspection)."""
        data = data or self.render_data
        mujoco.mj_setState(self.model, data, self.state[i], mujoco.mjtState.mjSTATE_FULLPHYSICS)
        mujoco.mj_forward(self.model, data)
        return data

    # ------------------------------------------------------------------ resets
    def _sample_commands(self, idx):
        c = self.cfg
        k = len(idx)
        cmd = np.stack([self.rng.uniform(*c.cmd_vx, k), self.rng.uniform(*c.cmd_vy, k),
                        self.rng.uniform(*c.cmd_yaw, k)], axis=1).astype(np.float32)
        zero = self.rng.random(k) < c.cmd_zero_prob
        cmd[zero] = 0.0
        small = np.linalg.norm(cmd[:, :2], axis=1) < c.cmd_deadband
        cmd[small, :2] = 0.0
        cmd[np.abs(cmd[:, 2]) < c.cmd_deadband, 2] = 0.0
        self.commands[idx] = cmd

    def set_commands(self, cmd, fixed: bool = True):
        """Force a command on every env (used for evaluation). cmd = [vx, vy, yaw_rate]."""
        self.commands[:] = np.asarray(cmd, np.float32)
        self.fixed_commands = fixed

    def _reset_envs(self, idx):
        le, m, c = self.le, self.model, self.cfg
        idx = np.asarray(idx)
        k = len(idx)
        qa, bq, nq = le.qpos_adr, le.base_qpos_adr, self.nq
        qpos = np.repeat(le.init_qpos[None, :], k, axis=0)
        qpos[:, qa] += self.rng.uniform(-c.init_joint_noise, c.init_joint_noise, (k, self.n))
        if c.init_yaw_random:
            yaw = self.rng.uniform(-math.pi, math.pi, k)
            qpos[:, bq + 3] = np.cos(yaw / 2); qpos[:, bq + 4:bq + 6] = 0.0; qpos[:, bq + 6] = np.sin(yaw / 2)
        self.state[idx] = 0.0
        self.state[idx, 1:1 + nq] = qpos
        # sensors for the fresh state: evaluate once through MjData (only on reset, cheap)
        sens = np.zeros((k, m.nsensordata))
        d = self.render_data
        for r, i in enumerate(idx):
            mujoco.mj_setState(m, d, self.state[i], mujoco.mjtState.mjSTATE_FULLPHYSICS)
            d.ctrl[:] = le.default_pose
            mujoco.mj_forward(m, d)
            sens[r] = d.sensordata
        full = self._sens_out[:, -1].copy() if hasattr(self, "_sens_out") else np.zeros((self.N, m.nsensordata))
        full[idx] = sens
        self._gather(full)
        self.qd_prev[idx] = self.qd[idx]
        self.actions[idx] = 0.0; self.prev_actions[idx] = 0.0
        self.air_time[idx] = 0.0; self.last_contact[idx] = False
        self.phase[idx] = self.rng.uniform(0, 2 * math.pi, len(idx))
        self.ep_step[idx] = 0; self.ep_return[idx] = 0.0; self.ep_term_sums[idx] = 0.0
        if not self.fixed_commands:
            self._sample_commands(idx)

    def reset(self):
        self._reset_envs(np.arange(self.N))
        return self._compute_obs()

    # ------------------------------------------------------------------ observations
    def _compute_obs(self):
        c = self.cfg
        cmd = self.commands * np.asarray(c.scale_cmd, np.float32)
        ang = self.base_ang_vel * c.scale_ang_vel
        grav = self.proj_grav
        dq = self.q - self.default_pose
        qd = self.qd * c.scale_dof_vel
        parts = [cmd, ang, grav, dq, qd, self.actions]
        if c.obs_noise > 0:
            lvl = c.obs_noise
            ang = ang + self.rng.normal(0, 0.2 * c.scale_ang_vel * lvl, ang.shape).astype(np.float32)
            grav = grav + self.rng.normal(0, 0.05 * lvl, grav.shape).astype(np.float32)
            dq = dq + self.rng.normal(0, 0.01 * lvl, dq.shape).astype(np.float32)
            qd = qd + self.rng.normal(0, 1.5 * c.scale_dof_vel * lvl, qd.shape).astype(np.float32)
            parts = [cmd, ang, grav, dq, qd, self.actions]
        if c.use_gait_phase:
            parts.append(np.stack([np.sin(self.phase), np.cos(self.phase)], axis=1).astype(np.float32))
        obs = np.concatenate(parts, axis=1).astype(np.float32)
        extra = np.concatenate([
            self.base_lin_vel * c.scale_lin_vel,
            self.foot_contact.astype(np.float32),
            (self.com[:, 2:3] - self.nominal_com_height).astype(np.float32) * 5.0,
        ], axis=1)
        critic_obs = np.concatenate([obs, extra], axis=1).astype(np.float32)
        return obs, critic_obs

    # ------------------------------------------------------------------ rewards
    def _compute_rewards(self):
        c, dt, le = self.cfg, self.dt, self.le
        w = c.rewards
        cmd = self.commands
        moving = np.linalg.norm(cmd[:, :2], axis=1) > c.cmd_deadband
        turning = np.abs(cmd[:, 2]) > c.cmd_deadband
        active = moving | turning

        # gait clock: left foot phase, right foot phase + pi. swing when sin > 0
        ph = np.stack([self.phase, self.phase + math.pi], axis=1)
        s = np.sin(ph)
        swing = (s > 0) & active[:, None]
        foot_h = self.foot_pos[:, :, 2] - le.foot_z_offset[None, :]
        h_des = np.where(swing, le.emb.swing_height * np.clip(s, 0, None), 0.0)
        foot_vel = self.foot_vel
        contact = self.foot_contact

        # feet air time (legged_gym): reward at touchdown proportional to how long the foot was up
        contact_filt = contact | self.last_contact
        first_contact = (self.air_time > 0) & contact_filt
        self.air_time += dt
        air = np.sum((self.air_time - self.air_time_target) * first_contact, axis=1) * moving
        self.air_time *= ~contact_filt
        self.last_contact[:] = contact

        qdd = (self.qd - self.qd_prev) / dt
        dq = self.q - self.default_pose
        t = {}
        t["tracking_lin_vel"] = np.exp(-np.sum((cmd[:, :2] - self.base_lin_vel[:, :2]) ** 2, axis=1) / c.tracking_sigma)
        t["tracking_ang_vel"] = np.exp(-((cmd[:, 2] - self.base_ang_vel[:, 2]) ** 2) / c.tracking_sigma)
        t["alive"] = np.ones(self.N, np.float32)
        t["feet_air_time"] = air
        t["gait_contact"] = 0.5 * np.sum(contact == ~swing, axis=1)
        t["feet_swing_height"] = np.sum(swing * np.exp(-((foot_h - h_des) ** 2) / (0.5 * le.emb.swing_height) ** 2), axis=1)
        t["lin_vel_z"] = self.base_lin_vel[:, 2] ** 2
        t["ang_vel_xy"] = np.sum(self.base_ang_vel[:, :2] ** 2, axis=1)
        t["orientation"] = np.sum(self.proj_grav[:, :2] ** 2, axis=1)
        t["base_height"] = (self.com[:, 2] - self.nominal_com_height) ** 2
        t["torques"] = np.sum((self.torques / le.torque_limit) ** 2, axis=1)
        t["dof_acc"] = np.sum(qdd ** 2, axis=1)
        t["dof_vel"] = np.sum(self.qd ** 2, axis=1)
        t["action_rate"] = np.sum((self.actions - self.prev_actions) ** 2, axis=1)
        t["posture"] = np.sum((dq[:, self.posture_mask]) ** 2, axis=1)
        t["legs_pose"] = np.sum((dq[:, self.swing_mask]) ** 2, axis=1)
        t["dof_pos_limits"] = np.sum(np.clip(self.soft_lo - self.q, 0, None) + np.clip(self.q - self.soft_hi, 0, None), axis=1)
        t["feet_slip"] = np.sum(contact * np.sum(foot_vel[:, :, :2] ** 2, axis=2), axis=1)
        t["termination"] = self._terminated.astype(np.float32)

        total = np.zeros(self.N, np.float32)
        for k, name in enumerate(self.reward_names):
            scale = w[name] * (1.0 if name == "termination" else dt)
            term = (t[name] * scale).astype(np.float32)
            self.ep_term_sums[:, k] += term
            total += term
        return total

    def _check_termination(self):
        c = self.cfg
        fallen = self.proj_grav[:, 2] > -math.cos(math.radians(c.max_tilt_deg))
        fallen |= self.com[:, 2] < c.min_com_height_ratio * self.nominal_com_height
        if c.terminate_on_body_contact:
            fallen |= self.body_contact
        return fallen

    # ------------------------------------------------------------------ step
    def step(self, actions: np.ndarray):
        """actions: (N, n_act). Returns obs, critic_obs, reward, terminated, truncated, info."""
        c = self.cfg
        actions = np.clip(np.asarray(actions, np.float32), -c.action_clip, c.action_clip)
        self.prev_actions[:] = self.actions
        self.actions[:] = actions
        self._ctrl[:] = (self.default_pose + self.le.emb.action_scale * actions)[:, None, :]

        # optional random pushes: kick the base velocity
        if self.push_steps:
            push = (self.ep_step % self.push_steps == 0) & (self.ep_step > 0)
            if push.any():
                da = 1 + self.nq + self.le.base_dof_adr
                self.state[push, da:da + 2] += self.rng.uniform(-c.push_max_vel, c.push_max_vel, (int(push.sum()), 2))

        self.qd_prev[:] = self.qd
        self._physics_step()
        self.ep_step += 1
        if c.use_gait_phase:
            self.phase = np.mod(self.phase + 2 * math.pi * self.le.emb.gait_freq * self.dt, 2 * math.pi).astype(np.float32)

        self._terminated = self._check_termination()
        reward = self._compute_rewards()
        truncated = (self.ep_step >= self.max_steps) & ~self._terminated
        done = self._terminated | truncated
        self.ep_return += reward

        # resample commands mid-episode
        if not self.fixed_commands:
            res = (self.ep_step % self.resample_steps == 0) & ~done
            if res.any():
                self._sample_commands(np.where(res)[0])

        obs, critic_obs = self._compute_obs()
        info = {}
        if done.any():
            idx = np.where(done)[0]
            info["final_obs"] = obs[idx].copy()
            info["final_critic_obs"] = critic_obs[idx].copy()
            info["done_idx"] = idx
            for i in idx:
                self._stats.append({"return": float(self.ep_return[i]), "length": int(self.ep_step[i]),
                                    "fell": bool(self._terminated[i]), "terms": self.ep_term_sums[i].copy()})
            self._reset_envs(idx)
            new_obs, new_cobs = self._compute_obs()
            obs[idx] = new_obs[idx]; critic_obs[idx] = new_cobs[idx]
        return obs, critic_obs, reward, self._terminated.copy(), truncated, info

    def pop_stats(self) -> dict:
        """Mean episode statistics since the last call (empty dict if no episode finished)."""
        if not self._stats:
            return {}
        s = self._stats; self._stats = []
        terms = np.mean([x["terms"] for x in s], axis=0)
        out = {"episodes": len(s), "return": float(np.mean([x["return"] for x in s])),
               "length": float(np.mean([x["length"] for x in s])), "fall_rate": float(np.mean([x["fell"] for x in s]))}
        out.update({f"rew/{n}": float(v) for n, v in zip(self.reward_names, terms)})
        return out

    def close(self):
        self.rollout.close() if hasattr(self.rollout, "close") else None


def benchmark(embodiment="asimov", num_envs=64, steps=200, n_threads=None, **kw):
    """Steps/second of the environment with random actions."""
    import time
    env = HumanoidVecEnv(embodiment, num_envs=num_envs, n_threads=n_threads, **kw)
    rng = np.random.default_rng(0)
    env.step(rng.normal(size=(env.N, env.act_dim)).astype(np.float32))
    t0 = time.perf_counter()
    for _ in range(steps):
        env.step(rng.normal(0, 0.3, size=(env.N, env.act_dim)).astype(np.float32))
    dt = time.perf_counter() - t0
    env.close()
    return steps * num_envs / dt
