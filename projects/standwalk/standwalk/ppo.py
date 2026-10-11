"""PPO for locomotion (the recipe from rsl_rl / legged_gym, written out in plain PyTorch).

What makes this "locomotion PPO" rather than textbook PPO:
  * separate actor and critic MLPs, the critic sees privileged observations
  * a Gaussian policy with a learned, state-independent standard deviation
  * running observation normalisation (statistics are saved with the policy)
  * value bootstrapping on time-limit truncation (an episode ending at 20 s is not a failure)
  * adaptive learning rate driven by the measured KL divergence between old and new policy
"""
from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

# PyTorch imports torch._dynamo (and with it Triton's C extension) lazily the first time an optimizer is
# created. On Linux boxes whose NVIDIA user-space libraries do not match the loaded kernel driver, that
# import segfaults if MuJoCo's EGL renderer was created earlier in the same process (e.g. a notebook that
# renders a robot before training). Importing it here, before any rendering, avoids the crash.
try:
    import torch._dynamo  # noqa: F401
except Exception:  # pragma: no cover - torch builds without dynamo
    pass

from .env import HumanoidVecEnv, TaskConfig


# ---------------------------------------------------------------------------------------
# Networks
# ---------------------------------------------------------------------------------------

class RunningNorm(nn.Module):
    """Welford running mean/variance; normalises inputs to ~N(0,1) and clips to [-10, 10]."""

    def __init__(self, dim: int, eps: float = 1e-2):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.tensor(eps))

    @torch.no_grad()
    def update(self, x: torch.Tensor):
        x = x.reshape(-1, x.shape[-1])
        bm, bv, bc = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        delta = bm - self.mean
        tot = self.count + bc
        new_mean = self.mean + delta * bc / tot
        m2 = self.var * self.count + bv * bc + delta ** 2 * self.count * bc / tot
        self.mean.copy_(new_mean); self.var.copy_(m2 / tot); self.count.copy_(tot)

    def forward(self, x):
        return torch.clamp((x - self.mean) / torch.sqrt(self.var + 1e-8), -10.0, 10.0)


def mlp(in_dim, hidden, out_dim, act=nn.ELU):
    layers, d = [], in_dim
    for h in hidden:
        layers += [nn.Linear(d, h), act()]
        d = h
    layers.append(nn.Linear(d, out_dim))
    return nn.Sequential(*layers)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, critic_obs_dim, act_dim, hidden=(512, 256, 128), init_std=1.0):
        super().__init__()
        self.actor = mlp(obs_dim, hidden, act_dim)
        self.critic = mlp(critic_obs_dim, hidden, 1)
        self.log_std = nn.Parameter(torch.full((act_dim,), math.log(init_std)))
        # NOTE: default PyTorch init, as in rsl_rl. A tiny last-layer init looks tempting (actions start
        # at the default pose) but it breaks the KL-based learning-rate adaptation: the network is
        # insensitive at first, the rate grows to its cap, and once the last layer inflates the rate
        # crashes to the floor and never recovers.

    def dist(self, obs):
        mean = self.actor(obs)
        std = torch.exp(self.log_std.clamp(-4.0, 1.0)).expand_as(mean)
        return torch.distributions.Normal(mean, std)

    @torch.no_grad()
    def act(self, obs):
        d = self.dist(obs)
        a = d.sample()
        return a, d.log_prob(a).sum(-1), d.mean, d.stddev

    def value(self, critic_obs):
        return self.critic(critic_obs).squeeze(-1)

    def evaluate(self, obs, critic_obs, actions):
        d = self.dist(obs)
        return d.log_prob(actions).sum(-1), d.entropy().sum(-1), self.value(critic_obs), d.mean, d.stddev


# ---------------------------------------------------------------------------------------
# Config + checkpoints
# ---------------------------------------------------------------------------------------

@dataclass
class PPOConfig:
    total_steps: int = 20_000_000
    n_steps: int = 24                 # rollout length per env per iteration
    epochs: int = 5
    minibatches: int = 4
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    ent_coef: float = 0.005
    vf_coef: float = 1.0
    lr: float = 1e-3
    adaptive_lr: bool = True          # rsl_rl-style: shrink/grow lr to keep KL near desired_kl
    desired_kl: float = 0.01
    max_grad_norm: float = 1.0
    init_std: float = 1.0
    hidden: tuple = (512, 256, 128)
    normalize_obs: bool = True
    seed: int = 0
    device: str = "auto"
    save_every: int = 50              # iterations
    log_every: int = 1


def pick_device(name: str = "auto") -> torch.device:
    if name != "auto":
        return torch.device(name)
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def save_checkpoint(path, policy, obs_norm, cobs_norm, env: HumanoidVecEnv, cfg: PPOConfig, meta: dict):
    le = env.le
    ckpt = {
        "model": policy.state_dict(),
        "obs_norm": obs_norm.state_dict(), "cobs_norm": cobs_norm.state_dict(),
        "ppo_config": asdict(cfg), "task_config": env.cfg.to_dict(),
        "embodiment": le.emb.name, "obs_dim": env.obs_dim, "critic_obs_dim": env.critic_obs_dim,
        "act_dim": env.act_dim, "obs_layout": env.obs_layout,
        "joint_names": le.joint_names, "default_pose": le.default_pose.tolist(),
        "kp": le.kp.tolist(), "kd": le.kd.tolist(), "torque_limit": le.torque_limit.tolist(),
        "action_scale": le.emb.action_scale, "ctrl_dt": le.emb.ctrl_dt,
        "meta": meta,
    }
    torch.save(ckpt, path)


def load_checkpoint(path, device="cpu"):
    ckpt = torch.load(path, map_location=device, weights_only=False)
    cfg = PPOConfig(**ckpt["ppo_config"])
    policy = ActorCritic(ckpt["obs_dim"], ckpt["critic_obs_dim"], ckpt["act_dim"], tuple(cfg.hidden), cfg.init_std).to(device)
    policy.load_state_dict(ckpt["model"])
    obs_norm = RunningNorm(ckpt["obs_dim"]).to(device); obs_norm.load_state_dict(ckpt["obs_norm"])
    cobs_norm = RunningNorm(ckpt["critic_obs_dim"]).to(device); cobs_norm.load_state_dict(ckpt["cobs_norm"])
    policy.eval()
    return policy, obs_norm, cobs_norm, ckpt


class Policy:
    """Inference wrapper: numpy obs -> numpy action (deterministic = mean action)."""

    def __init__(self, path, device="cpu"):
        self.device = torch.device(device)
        self.net, self.obs_norm, _, self.ckpt = load_checkpoint(path, self.device)
        self.task = TaskConfig(**self.ckpt["task_config"])

    @torch.no_grad()
    def __call__(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        x = self.obs_norm(torch.as_tensor(obs, dtype=torch.float32, device=self.device))
        d = self.net.dist(x)
        a = d.mean if deterministic else d.sample()
        return a.cpu().numpy()


class DeployablePolicy(nn.Module):
    """obs -> action with normalisation folded in; exported with TorchScript for the robot."""

    def __init__(self, policy: ActorCritic, obs_norm: RunningNorm):
        super().__init__()
        self.register_buffer("mean", obs_norm.mean.clone())
        self.register_buffer("std", torch.sqrt(obs_norm.var + 1e-8))
        self.actor = policy.actor

    def forward(self, obs):
        return self.actor(torch.clamp((obs - self.mean) / self.std, -10.0, 10.0))


def export_policy(ckpt_path, out_path) -> Path:
    """Write <out_path>.pt (TorchScript) and <out_path>.json (everything the robot code needs)."""
    policy, obs_norm, _, ckpt = load_checkpoint(ckpt_path, "cpu")
    dep = DeployablePolicy(policy, obs_norm).eval()
    for prm in dep.parameters():          # inference only: no autograd on the robot
        prm.requires_grad_(False)
    out = Path(out_path)
    with torch.no_grad():
        scripted = torch.jit.trace(dep, torch.zeros(1, ckpt["obs_dim"]))
    scripted.save(str(out.with_suffix(".pt")))
    meta = {k: ckpt[k] for k in ("embodiment", "obs_layout", "joint_names", "default_pose", "kp", "kd",
                                 "torque_limit", "action_scale", "ctrl_dt", "obs_dim", "act_dim")}
    tc = ckpt["task_config"]
    meta["obs_scales"] = {"cmd": tc["scale_cmd"], "base_ang_vel": tc["scale_ang_vel"], "joint_vel": tc["scale_dof_vel"]}
    meta["how_to_use"] = ("obs = concat(obs_layout) with the listed scales; action = model(obs); "
                          "target_q = default_pose + action_scale * action; torque = kp*(target_q - q) - kd*qd "
                          "clipped to torque_limit, at the motor driver rate; policy runs every ctrl_dt seconds.")
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    return out.with_suffix(".pt")


# ---------------------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------------------

def train(env: HumanoidVecEnv, cfg: PPOConfig, out_dir, resume: str | None = None, log=print,
          callback=None) -> list[dict]:
    """Run PPO on `env`. Writes progress.csv, latest.pt and best.pt into out_dir. Returns the log rows."""
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    device = pick_device(cfg.device)
    torch.manual_seed(cfg.seed); np.random.seed(cfg.seed)
    N, T = env.N, cfg.n_steps
    policy = ActorCritic(env.obs_dim, env.critic_obs_dim, env.act_dim, tuple(cfg.hidden), cfg.init_std).to(device)
    obs_norm, cobs_norm = RunningNorm(env.obs_dim).to(device), RunningNorm(env.critic_obs_dim).to(device)
    start_iter, total_steps = 0, 0
    if resume:
        _, on, cn, ck = load_checkpoint(resume, device)
        policy.load_state_dict(ck["model"]); obs_norm.load_state_dict(on.state_dict()); cobs_norm.load_state_dict(cn.state_dict())
        log(f"resumed weights from {resume} (embodiment {ck['embodiment']}, {ck['meta'].get('total_steps', '?')} steps)")
    opt = torch.optim.Adam(policy.parameters(), lr=cfg.lr)
    lr = cfg.lr

    (out / "config.json").write_text(json.dumps({"ppo": asdict(cfg), "task": env.cfg.to_dict(),
                                                 "embodiment": env.le.emb.name, "num_envs": N}, indent=2))
    log(f"device={device} envs={N} steps/iter={N*T:,} obs={env.obs_dim} critic_obs={env.critic_obs_dim} act={env.act_dim} "
        f"params={sum(p.numel() for p in policy.parameters()):,}")

    # rollout storage
    f32 = torch.float32
    S = dict(obs=torch.zeros(T, N, env.obs_dim, device=device), cobs=torch.zeros(T, N, env.critic_obs_dim, device=device),
             act=torch.zeros(T, N, env.act_dim, device=device), logp=torch.zeros(T, N, device=device),
             val=torch.zeros(T, N, device=device), rew=torch.zeros(T, N, device=device),
             done=torch.zeros(T, N, device=device), mu=torch.zeros(T, N, env.act_dim, device=device),
             sigma=torch.zeros(T, N, env.act_dim, device=device))

    obs_np, cobs_np = env.reset()
    obs = torch.as_tensor(obs_np, device=device); cobs = torch.as_tensor(cobs_np, device=device)
    rows, best_return, t_start = [], -np.inf, time.time()
    from collections import deque
    recent = deque(maxlen=max(32, N // 2))   # best.pt tracks the mean return of the most recent episodes
    n_iters = max(1, cfg.total_steps // (N * T))
    csv_file = open(out / "progress.csv", "w", newline="")
    writer = None

    for it in range(start_iter, n_iters):
        t_iter = time.time()
        policy.eval()
        for t in range(T):
            if cfg.normalize_obs:
                obs_norm.update(obs); cobs_norm.update(cobs)
            o, co = obs_norm(obs), cobs_norm(cobs)
            a, logp, mu, sigma = policy.act(o)
            with torch.no_grad():
                v = policy.value(co)
            obs_np, cobs_np, rew, term, trunc, info = env.step(a.cpu().numpy())
            rew_t = torch.as_tensor(rew, device=device)
            if trunc.any():  # bootstrap time-limit truncations with V(final state)
                idx = np.where(trunc)[0]
                pos = {j: k for k, j in enumerate(info["done_idx"])}
                fin = torch.as_tensor(info["final_critic_obs"][[pos[j] for j in idx]], device=device)
                with torch.no_grad():
                    rew_t[idx] += cfg.gamma * policy.value(cobs_norm(fin))
            S["obs"][t] = o; S["cobs"][t] = co; S["act"][t] = a; S["logp"][t] = logp; S["val"][t] = v
            S["rew"][t] = rew_t; S["done"][t] = torch.as_tensor(term | trunc, dtype=f32, device=device)
            S["mu"][t] = mu; S["sigma"][t] = sigma
            obs = torch.as_tensor(obs_np, device=device); cobs = torch.as_tensor(cobs_np, device=device)
        total_steps += N * T
        t_rollout = time.time() - t_iter

        # GAE
        with torch.no_grad():
            last_v = policy.value(cobs_norm(cobs))
            adv = torch.zeros(T, N, device=device); gae = torch.zeros(N, device=device)
            for t in reversed(range(T)):
                nv = last_v if t == T - 1 else S["val"][t + 1]
                nonterm = 1.0 - S["done"][t]
                delta = S["rew"][t] + cfg.gamma * nv * nonterm - S["val"][t]
                gae = delta + cfg.gamma * cfg.lam * nonterm * gae
                adv[t] = gae
            ret = adv + S["val"]
            adv_n = (adv - adv.mean()) / (adv.std() + 1e-8)

        # PPO update
        policy.train()
        B = T * N; mb = B // cfg.minibatches
        flat = {k: v.reshape(B, *v.shape[2:]) for k, v in S.items()}
        adv_f, ret_f = adv_n.reshape(B), ret.reshape(B)
        stats = dict(policy_loss=0.0, value_loss=0.0, entropy=0.0, kl=0.0, clip_frac=0.0); n_upd = 0
        for _ in range(cfg.epochs):
            perm = torch.randperm(B, device=device)
            for s in range(0, B, mb):
                i = perm[s:s + mb]
                logp, ent, v, mu, sigma = policy.evaluate(flat["obs"][i], flat["cobs"][i], flat["act"][i])
                ratio = torch.exp(logp - flat["logp"][i])
                surr = -torch.min(ratio * adv_f[i], torch.clamp(ratio, 1 - cfg.clip, 1 + cfg.clip) * adv_f[i]).mean()
                v_clip = flat["val"][i] + (v - flat["val"][i]).clamp(-cfg.clip, cfg.clip)
                v_loss = torch.max((v - ret_f[i]) ** 2, (v_clip - ret_f[i]) ** 2).mean()
                loss = surr + cfg.vf_coef * v_loss - cfg.ent_coef * ent.mean()
                with torch.no_grad():  # KL(old || new) for Gaussians, as in rsl_rl
                    os_, om = flat["sigma"][i], flat["mu"][i]
                    kl = (torch.log(sigma / os_) + (os_ ** 2 + (om - mu) ** 2) / (2 * sigma ** 2) - 0.5).sum(-1).mean().item()
                opt.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(policy.parameters(), cfg.max_grad_norm)
                opt.step()
                stats["policy_loss"] += surr.item(); stats["value_loss"] += v_loss.item(); stats["entropy"] += ent.mean().item()
                stats["kl"] += kl; stats["clip_frac"] += ((ratio - 1).abs() > cfg.clip).float().mean().item(); n_upd += 1
        stats = {k: v / n_upd for k, v in stats.items()}
        # Adaptive learning rate (rsl_rl idea, applied once per iteration). The KL between the rollout
        # policy and the current policy grows over the epochs, so the per-iteration mean is the right
        # quantity to control: too much drift -> smaller steps, too little -> larger steps.
        if cfg.adaptive_lr:
            if stats["kl"] > 2.0 * cfg.desired_kl:
                lr = max(1e-5, lr / 1.5)
            elif stats["kl"] < cfg.desired_kl / 2.0:
                lr = min(1e-2, lr * 1.5)
            for g in opt.param_groups:
                g["lr"] = lr

        # logging
        ep = env.pop_stats()
        elapsed = time.time() - t_start
        row = {"iteration": it + 1, "total_steps": total_steps, "time_s": round(elapsed, 1),
               "sps": round(N * T / (time.time() - t_iter)), "rollout_s": round(t_rollout, 2),
               "lr": lr, "action_std": float(torch.exp(policy.log_std.detach()).mean().item()), **stats}
        row.update({k: v for k, v in ep.items()})
        rows.append(row)
        if writer is None and ep:
            writer = csv.DictWriter(csv_file, fieldnames=list(row.keys())); writer.writeheader()
        if writer is not None and ep:
            writer.writerow(row); csv_file.flush()
        if ep and (it + 1) % cfg.log_every == 0:
            log(f"it {it+1:5d}/{n_iters} steps {total_steps:>11,} sps {row['sps']:6,} | return {ep['return']:7.2f} "
                f"len {ep['length']:6.1f} falls {ep['fall_rate']:.2f} | vloss {stats['value_loss']:.3f} kl {stats['kl']:.4f} "
                f"std {row['action_std']:.2f} lr {lr:.1e} | {elapsed/60:.1f} min")
        if ep:
            recent.extend([ep["return"]] * min(ep["episodes"], recent.maxlen))
            smoothed = float(np.mean(recent))
            row["return_smoothed"] = smoothed
            if len(recent) == recent.maxlen and smoothed > best_return:
                best_return = smoothed
                save_checkpoint(out / "best.pt", policy, obs_norm, cobs_norm, env, cfg,
                                {"iteration": it + 1, "total_steps": total_steps, "return": best_return})
        if (it + 1) % cfg.save_every == 0 or it + 1 == n_iters:
            save_checkpoint(out / "latest.pt", policy, obs_norm, cobs_norm, env, cfg,
                            {"iteration": it + 1, "total_steps": total_steps, "return": ep.get("return", float("nan"))})
        if callback is not None:
            callback(row)
    csv_file.close()
    log(f"done: {total_steps:,} steps in {(time.time()-t_start)/60:.1f} min, best return {best_return:.2f}, "
        f"checkpoints in {out}")
    return rows
