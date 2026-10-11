"""Embodiment registry and MuJoCo model loader.

Every robot in metal2humanoid/embodiments is described by an `Embodiment` record:
which XML to load, which body is the floating base, which bodies are the feet,
how the joints are grouped, the default (nominal) pose and the PD gains that turn
the policy's position targets into torques.

The loader rewrites the XML so that *every* robot is driven the same way:
    policy action -> joint position target -> MuJoCo <position> actuator (PD at sim rate)
This matches how real humanoids are controlled (joint-level PD on the motor driver),
which is the whole point of training policies that can later run on the hardware.
"""
from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

# --------------------------------------------------------------------------------------
# Registry
# --------------------------------------------------------------------------------------

PDRule = tuple[str, float, float, float]  # (joint-name regex, kp, kd, torque limit)


@dataclass
class Embodiment:
    name: str
    xml: str                       # path relative to the embodiments directory
    base_body: str                 # floating-base body
    feet: tuple[str, str]          # (left foot body, right foot body); their subtrees count as "foot"
    pd: tuple[PDRule, ...]         # first matching rule wins
    default_pose: dict[str, float] = field(default_factory=dict)  # joint regex -> radians
    swing_joints: str = r"(hip_pitch|knee|ankle_pitch)"            # leg pitch joints, free to move when walking
    passive_joints: tuple[str, ...] = ()                            # regexes of joints left unactuated
    action_scale: float = 0.25     # target = default + action_scale * action  [rad]
    sim_dt: float = 0.004          # physics timestep [s]  (5 substeps per control step)
    ctrl_dt: float = 0.02          # policy timestep [s]  (50 Hz)
    swing_height: float = 0.06     # desired foot clearance during swing [m]
    gait_freq: float = 1.5         # gait cycle frequency [Hz]
    notes: str = ""


# Asimov legs use mirrored joint axes: the right-side pitch joints rotate about -y,
# so a mirror-symmetric pose has opposite signs on the two sides.
_ASIMOV_LEG_PD: tuple[PDRule, ...] = (
    (r"hip", 100.0, 3.0, 80.0),
    (r"knee", 120.0, 3.0, 120.0),
    (r"ankle", 100.0, 3.0, 40.0),
)
# The Frankensteins carry a heavier torso on the same legs; stiffer hips/ankles keep them upright.
_FRANK_LEG_PD: tuple[PDRule, ...] = (
    (r"hip", 150.0, 4.0, 80.0),
    (r"knee", 150.0, 4.0, 120.0),
    (r"ankle", 150.0, 4.0, 40.0),
)
_ASIMOV_LEG_POSE = {
    r"left_hip_pitch": -0.10, r"right_hip_pitch": 0.10,
    r"left_knee": 0.20, r"right_knee": -0.20,
    r"left_ankle_pitch": -0.10, r"right_ankle_pitch": 0.10,
}
_ATOM_ARM_PD: tuple[PDRule, ...] = ((r"arm|elbow", 40.0, 1.5, 40.0),)
_BHL_ARM_PD: tuple[PDRule, ...] = ((r"shoulder|elbow", 20.0, 0.5, 20.0),)

REGISTRY: dict[str, Embodiment] = {
    "asimov": Embodiment(
        name="asimov",
        xml="asimov-v0/xmls/asimov.xml",
        base_body="pelvis_link",
        feet=("left_ankle_roll_link", "right_ankle_roll_link"),
        pd=_ASIMOV_LEG_PD,
        default_pose=dict(_ASIMOV_LEG_POSE),
        passive_joints=(r"toe",),
        notes="Menlo Research Asimov v0 legs (12 DoF) + passive sprung toes. ~16 kg.",
    ),
    "berkeley_humanoid_lite": Embodiment(
        name="berkeley_humanoid_lite",
        xml="berkeley-humanoid-lite/xmls/berkeley_humanoid_lite.xml",
        base_body="base",
        feet=("leg_left_ankle_roll", "leg_right_ankle_roll"),
        pd=((r"ankle_pitch", 80.0, 2.0, 20.0), (r"leg_", 40.0, 1.5, 20.0), (r"arm_", 20.0, 0.5, 20.0)),
        default_pose={r"hip_pitch": -0.20, r"knee_pitch": 0.40, r"ankle_pitch": -0.20},
        swing_joints=r"(hip_pitch|knee_pitch|ankle_pitch)",
        gait_freq=1.8,
        notes="Berkeley Humanoid Lite, 22 DoF, ~16 kg, 20 Nm actuators.",
    ),
    "berkeley_humanoid_lite_noarms": Embodiment(
        name="berkeley_humanoid_lite_noarms",
        xml="berkeley-humanoid-lite/xmls/berkeley_humanoid_lite_noarms.xml",
        base_body="base",
        feet=("leg_left_ankle_roll", "leg_right_ankle_roll"),
        pd=((r"ankle_pitch", 80.0, 2.0, 20.0), (r"leg_", 40.0, 1.5, 20.0)),
        default_pose={r"hip_pitch": -0.20, r"knee_pitch": 0.40, r"ankle_pitch": -0.20},
        swing_joints=r"(hip_pitch|knee_pitch|ankle_pitch)",
        gait_freq=1.8,
        notes="Berkeley Humanoid Lite without arms: full torso + 12-DoF legs, ~12.6 kg (generated XML).",
    ),
    "atom01": Embodiment(
        name="atom01",
        xml="roboto-origin/xmls/atom01.xml",
        base_body="base_link",
        feet=("left_ankle_roll_link", "right_ankle_roll_link"),
        pd=(
            (r"thigh", 150.0, 5.0, 120.0),
            (r"knee", 200.0, 5.0, 150.0),
            (r"ankle", 200.0, 5.0, 60.0),
            (r"torso", 100.0, 3.0, 80.0),
        ) + _ATOM_ARM_PD,
        default_pose={r"thigh_pitch": -0.15, r"knee": 0.30, r"ankle_pitch": -0.15},
        swing_joints=r"(thigh_pitch|knee|ankle_pitch)",
        swing_height=0.08,
        notes="Roboto Origin / Atom01, 23 DoF (torso yaw + arms), ~34 kg.",
    ),
    "frankenstein_v1": Embodiment(
        name="frankenstein_v1",
        xml="frankenstein-v1.xml",
        base_body="pelvis_link",
        feet=("left_ankle_roll_link", "right_ankle_roll_link"),
        pd=_FRANK_LEG_PD + _ATOM_ARM_PD,
        default_pose=dict(_ASIMOV_LEG_POSE),
        passive_joints=(r"toe",),
        notes="Asimov legs + Atom01 torso/arms + G1 head, 22 DoF, ~29 kg.",
    ),
    "frankenstein_v0": Embodiment(
        name="frankenstein_v0",
        xml="frankenstein-v0.xml",
        base_body="pelvis_link",
        feet=("left_ankle_roll_link", "right_ankle_roll_link"),
        pd=_FRANK_LEG_PD + _BHL_ARM_PD,
        default_pose=dict(_ASIMOV_LEG_POSE),
        passive_joints=(r"toe",),
        notes="Asimov legs + Berkeley Humanoid Lite torso/arms, 22 DoF.",
    ),
    "mjc_humanoid": Embodiment(
        name="mjc_humanoid",
        xml="misc/mjc-humanoid.xml",
        base_body="torso",
        feet=("foot_left", "foot_right"),
        pd=(
            (r"hip_y", 150.0, 5.0, 120.0),
            (r"knee", 100.0, 3.0, 80.0),
            (r"hip_|abdomen", 60.0, 2.0, 40.0),
            (r"ankle", 100.0, 3.0, 40.0),
            (r"shoulder", 20.0, 0.7, 20.0),
            (r"elbow", 40.0, 1.5, 40.0),
        ),
        swing_joints=r"(hip_y|knee|ankle_y)",
        swing_height=0.10,
        notes="DeepMind MuJoCo humanoid (not a real robot). Useful as a known-good baseline.",
    ),
}


def list_embodiments() -> list[str]:
    return list(REGISTRY)


# Generated models that the course repository does not ship (the no-arms Berkeley Humanoid Lite) live here, laid out
# like metal2humanoid/embodiments. They are used when the embodiments directory lacks them and borrow the meshes of
# the corresponding robot from it, so the project (and the Colab bundle) stays self-contained.
EXTRA_XML_DIR = Path(__file__).resolve().parent / "embodiments_extra"


def embodiments_dir(override: str | os.PathLike | None = None) -> Path:
    """Locate the embodiments folder: explicit arg > $STANDWALK_EMBODIMENTS > an enclosing metal2humanoid checkout
    > metal2humanoid cloned inside or next to this project."""
    if override:
        return Path(override).expanduser().resolve()
    env = os.environ.get("STANDWALK_EMBODIMENTS")
    if env:
        return Path(env).expanduser().resolve()
    here = Path(__file__).resolve().parent
    cands = [p / "embodiments" for p in here.parents]                  # this project inside the metal2humanoid repo
    cands += [here.parent / "metal2humanoid" / "embodiments",          # metal2humanoid cloned inside the project
              here.parent.parent / "metal2humanoid" / "embodiments",   # ... or next to it
              Path.cwd() / "metal2humanoid" / "embodiments"]
    for cand in cands:
        if (cand / "asimov-v0").is_dir():
            return cand
    raise FileNotFoundError(
        "Could not find metal2humanoid/embodiments. Clone https://github.com/hbrnetwork/metal2humanoid into or next "
        "to this project or set STANDWALK_EMBODIMENTS=/path/to/metal2humanoid/embodiments")


# --------------------------------------------------------------------------------------
# XML rewriting: absolute asset paths + uniform <position> actuators
# --------------------------------------------------------------------------------------

def _first_match(rules, name):
    for rule in rules:
        if re.search(rule[0], name):
            return rule
    return None


def _find_body(root, name):
    for b in root.iter("body"):
        if b.get("name") == name:
            return b
    raise KeyError(f"body '{name}' not found")


def _rewrite_xml(xml_path: Path, actuated: list[str], sensors: dict | None = None,
                 asset_base: Path | None = None) -> str:
    """Return the model XML as a string with absolute asset paths, uniform <position> actuators and,
    if `sensors` is given, an IMU site + touch sites + a <sensor> block laid out for the env.
    Relative asset directories are resolved against `asset_base` (default: the XML's own directory)."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    if root.find("include") is not None:
        raise NotImplementedError(f"{xml_path.name}: <include> is not supported by the loader")

    # 1. make asset directories absolute so the model can be compiled from a string
    comp = root.find("compiler")
    if comp is None:
        comp = ET.SubElement(root, "compiler")
    base_dir = Path(asset_base) if asset_base else xml_path.parent
    assetdir = comp.attrib.pop("assetdir", None)
    meshdir = comp.attrib.get("meshdir", assetdir or ".")
    texdir = comp.attrib.get("texturedir", assetdir or ".")
    comp.set("meshdir", str((base_dir / meshdir).resolve()))
    comp.set("texturedir", str((base_dir / texdir).resolve()))

    # 2. drop existing actuators, keyframes and sensors (we define our own layout)
    for tag in ("actuator", "keyframe", "sensor"):
        for el in root.findall(tag):
            root.remove(el)

    # 3. one <position> actuator per actuated joint, in our order; gains are set after compile
    act = ET.SubElement(root, "actuator")
    for j in actuated:
        ET.SubElement(act, "position", {"name": j, "joint": j, "ctrllimited": "false", "forcelimited": "false"})

    # 4. some exported models (Berkeley Humanoid Lite) come without a floor or lights
    wb = root.find("worldbody")
    if wb is None:
        raise ValueError(f"{xml_path.name}: no <worldbody>")
    if not any(g.get("type") == "plane" for g in wb.findall("geom")):
        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "texture", {"name": "sw_grid", "type": "2d", "builtin": "checker", "width": "512",
                                         "height": "512", "rgb1": ".2 .3 .4", "rgb2": ".1 .15 .2"})
        ET.SubElement(asset, "material", {"name": "sw_grid", "texture": "sw_grid", "texrepeat": "5 5",
                                          "texuniform": "true", "reflectance": ".2"})
        floor = ET.Element("geom", {"name": "floor", "type": "plane", "size": "0 0 0.05", "material": "sw_grid",
                                    "contype": "1", "conaffinity": "1"})
        wb.insert(0, floor)
        wb.insert(0, ET.Element("light", {"pos": "0 0 3", "dir": "0 0 -1", "directional": "true"}))

    # 5. sensors: everything the env observes comes out of mjData.sensordata
    if sensors:
        base = sensors["base_body"]
        fmt = lambda v: " ".join(f"{x:.6g}" for x in v)
        bel = _find_body(root, base)
        bel.append(ET.Element("site", {"name": "sw_imu", "pos": fmt(sensors["imu_pos"]), "size": "0.005", "group": "4"}))
        for body, site, center, half in sensors["touch_boxes"]:
            _find_body(root, body).append(ET.Element("site", {"name": site, "type": "box", "pos": fmt(center),
                                                               "size": fmt(half), "group": "4", "rgba": "1 0 0 0.2"}))
        sen = ET.SubElement(root, "sensor")
        ET.SubElement(sen, "framequat", {"name": "base_quat", "objtype": "xbody", "objname": base})
        ET.SubElement(sen, "gyro", {"name": "base_gyro", "site": "sw_imu"})
        ET.SubElement(sen, "velocimeter", {"name": "base_velocimeter", "site": "sw_imu"})
        ET.SubElement(sen, "framepos", {"name": "base_pos", "objtype": "xbody", "objname": base})
        ET.SubElement(sen, "subtreecom", {"name": "com", "body": base})
        for side, foot in zip("LR", sensors["feet"]):
            ET.SubElement(sen, "framepos", {"name": f"foot_pos_{side}", "objtype": "xbody", "objname": foot})
        for side, foot in zip("LR", sensors["feet"]):
            ET.SubElement(sen, "framelinvel", {"name": f"foot_vel_{side}", "objtype": "xbody", "objname": foot})
        for j in actuated:
            ET.SubElement(sen, "actuatorfrc", {"name": f"tau_{j}", "actuator": j})
        for body, site, center, half in sensors["touch_boxes"]:
            ET.SubElement(sen, "touch", {"name": f"touch_{site}", "site": site})
    return ET.tostring(root, encoding="unicode")


def _joint_names(xml_path: Path) -> list[str]:
    """Hinge/slide joint names in document order (free joints excluded)."""
    names = []
    for el in ET.parse(xml_path).getroot().iter("joint"):
        if el.get("type", "hinge") in ("hinge", "slide") and el.get("name"):
            names.append(el.get("name"))
    return names


# --------------------------------------------------------------------------------------
# Loaded model + resolved indices
# --------------------------------------------------------------------------------------

@dataclass
class LoadedEmbodiment:
    emb: Embodiment
    model: mujoco.MjModel
    joint_names: list[str]           # actuated joints, actuator i <-> joint_names[i]
    qpos_adr: np.ndarray             # qpos index per actuated joint
    dof_adr: np.ndarray              # qvel index per actuated joint
    default_pose: np.ndarray         # [rad]
    kp: np.ndarray
    kd: np.ndarray
    torque_limit: np.ndarray
    joint_range: np.ndarray          # (n, 2)
    swing_mask: np.ndarray           # bool, leg pitch joints
    base_body: int
    base_qpos_adr: int               # start of the 7-dim free joint in qpos
    base_dof_adr: int                # start of the 6-dim free joint in qvel
    feet_bodies: np.ndarray          # (2,) body ids
    geom_foot: np.ndarray            # (ngeom,) -1 = not a foot, 0 = left, 1 = right
    floor_geom: int
    init_qpos: np.ndarray            # settled standing configuration (base at x=y=0)
    nominal_height: float            # base height when standing in the default pose [m]
    foot_z_offset: np.ndarray        # (2,) foot body z when the foot is flat on the floor
    total_mass: float
    stands: bool                     # did the PD-held default pose stand on its own?
    settle_tilt_deg: float = 0.0     # base tilt at the end of the PD hold
    sensor_slices: dict = field(default_factory=dict)   # name -> slice into sensordata
    touch_feet: tuple = ((), ())     # per foot: tuple of touch-sensor slices
    touch_base: tuple = ()           # touch-sensor slices on the base body
    nstate: int = 0                  # size of the mjSTATE_FULLPHYSICS vector

    @property
    def n_act(self) -> int:
        return len(self.joint_names)

    @property
    def decimation(self) -> int:
        return int(round(self.emb.ctrl_dt / self.emb.sim_dt))


def _subtree_bodies(model, root_body):
    out = []
    for b in range(model.nbody):
        p = b
        while p != 0 and p != root_body:
            p = model.body_parentid[p]
        if p == root_body:
            out.append(b)
    return out


def _collision_box_in_body_frame(model, body, margin=0.02):
    """Axis-aligned box (center, half-size) in the body frame enclosing the body's collision geoms."""
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in np.where(model.geom_bodyid == body)[0]:
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        R = np.zeros(9); mujoco.mju_quat2Mat(R, model.geom_quat[g]); R = R.reshape(3, 3)
        c, h = model.geom_aabb[g, :3], model.geom_aabb[g, 3:]
        pts = model.geom_pos[g] + (c + corners * h) @ R.T
        lo, hi = np.minimum(lo, pts.min(0)), np.maximum(hi, pts.max(0))
    if not np.isfinite(lo).all():
        return None
    return (lo + hi) / 2, (hi - lo) / 2 + margin


def _lowest_point(model, data, exclude_geoms):
    """Lowest z of any collision geom's bounding box in world frame (after mj_forward)."""
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    zmin = np.inf
    for g in range(model.ngeom):
        if g in exclude_geoms or model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        c, h = model.geom_aabb[g, :3], model.geom_aabb[g, 3:]
        R = data.geom_xmat[g].reshape(3, 3)
        pts = data.geom_xpos[g] + (c + corners * h) @ R.T
        zmin = min(zmin, pts[:, 2].min())
    return zmin


def load_embodiment(name_or_emb: str | Embodiment, embodiments_root=None,
                    settle_time: float = 1.5, verbose: bool = False) -> LoadedEmbodiment:
    """Load an embodiment, attach PD actuators, and settle it into its standing pose."""
    emb = REGISTRY[name_or_emb] if isinstance(name_or_emb, str) else name_or_emb
    root = embodiments_dir(embodiments_root)
    xml_path, asset_base = root / emb.xml, None
    if not xml_path.exists():
        extra = EXTRA_XML_DIR / emb.xml
        if not extra.exists():
            raise FileNotFoundError(f"{xml_path} (and no copy in {EXTRA_XML_DIR})")
        xml_path, asset_base = extra, root / Path(emb.xml).parent   # our XML, the course repo's meshes

    all_joints = _joint_names(xml_path)
    actuated = [j for j in all_joints if not any(re.search(p, j) for p in emb.passive_joints)]

    # pass 1: compile without sensors to measure geometry (touch-site boxes, IMU position)
    m1 = mujoco.MjModel.from_xml_string(_rewrite_xml(xml_path, actuated, asset_base=asset_base))
    base1 = mujoco.mj_name2id(m1, mujoco.mjtObj.mjOBJ_BODY, emb.base_body)
    feet1 = [mujoco.mj_name2id(m1, mujoco.mjtObj.mjOBJ_BODY, f) for f in emb.feet]
    touch_boxes = []
    for tag, bodies in [("L", _subtree_bodies(m1, feet1[0])), ("R", _subtree_bodies(m1, feet1[1])), ("base", [base1])]:
        for b in bodies:
            box = _collision_box_in_body_frame(m1, b, margin=0.02)
            if box is not None:
                bname = mujoco.mj_id2name(m1, mujoco.mjtObj.mjOBJ_BODY, b)
                touch_boxes.append((bname, f"sw_touch_{tag}_{bname}", box[0], box[1]))
    sensors = {"base_body": emb.base_body, "feet": list(emb.feet), "imu_pos": m1.body_ipos[base1].tolist(),
               "touch_boxes": touch_boxes}
    # pass 2: the model we actually use
    model = mujoco.MjModel.from_xml_string(_rewrite_xml(xml_path, actuated, sensors, asset_base=asset_base))

    # physics settings shared by every robot (fast, stable Newton solver)
    model.opt.timestep = emb.sim_dt
    model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
    model.opt.iterations = min(int(model.opt.iterations), 50)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, 1280)
    model.vis.global_.offheight = max(model.vis.global_.offheight, 720)

    n = len(actuated)
    jid = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in actuated])
    qpos_adr = model.jnt_qposadr[jid]
    dof_adr = model.jnt_dofadr[jid]
    jrange = model.jnt_range[jid].copy()
    unlimited = model.jnt_limited[jid] == 0
    jrange[unlimited] = [-np.pi, np.pi]

    kp = np.zeros(n); kd = np.zeros(n); tl = np.zeros(n)
    for i, j in enumerate(actuated):
        rule = _first_match(emb.pd, j)
        if rule is None:
            raise ValueError(f"{emb.name}: no PD rule matches joint '{j}'")
        _, kp[i], kd[i], tl[i] = rule
    default = np.zeros(n)
    for i, j in enumerate(actuated):
        for pat, val in emb.default_pose.items():
            if re.search(pat, j):
                default[i] = val
    default = np.clip(default, jrange[:, 0], jrange[:, 1])
    swing_mask = np.array([bool(re.search(emb.swing_joints, j)) for j in actuated])

    # actuator i was created for joint actuated[i]; configure it as a PD position servo
    for i in range(n):
        assert model.actuator_trnid[i, 0] == jid[i]
        model.actuator_gainprm[i, :] = 0; model.actuator_biasprm[i, :] = 0
        model.actuator_gainprm[i, 0] = kp[i]
        model.actuator_biasprm[i, 1] = -kp[i]
        model.actuator_biasprm[i, 2] = -kd[i]
        model.actuator_ctrlrange[i] = jrange[i]
        model.actuator_ctrllimited[i] = 1
        model.actuator_forcerange[i] = [-tl[i], tl[i]]
        model.actuator_forcelimited[i] = 1

    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, emb.base_body)
    base_jnt = model.body_jntadr[base]
    assert model.jnt_type[base_jnt] == mujoco.mjtJoint.mjJNT_FREE, f"{emb.base_body} must carry a free joint"
    base_qadr = int(model.jnt_qposadr[base_jnt]); base_dadr = int(model.jnt_dofadr[base_jnt])

    feet = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f) for f in emb.feet])
    geom_foot = -np.ones(model.ngeom, dtype=int)
    for side, fb in enumerate(feet):
        for b in _subtree_bodies(model, fb):
            geom_foot[model.geom_bodyid == b] = side
    floor_candidates = [g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
    if not floor_candidates:
        raise ValueError(f"{emb.name}: no ground plane geom in {xml_path.name}")
    floor = floor_candidates[0]

    # ---- settle: find the standing height, then let the PD controller hold the pose ----
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[qpos_adr] = default
    data.qpos[base_qadr:base_qadr + 3] = 0.0
    data.qpos[base_qadr + 3:base_qadr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    world_geoms = set(np.where(model.geom_bodyid == 0)[0])
    data.qpos[base_qadr + 2] = -_lowest_point(model, data, world_geoms) + 0.01
    data.ctrl[:] = default
    mujoco.mj_forward(model, data)
    placed_qpos = data.qpos.copy()
    placed_qpos[base_qadr + 2] -= 0.01          # feet exactly on the floor
    placed_foot_z = data.xpos[feet, 2] - 0.01
    nsteps = int(settle_time / emb.sim_dt)
    heights = []
    for k in range(nsteps):
        mujoco.mj_step(model, data)
        if k > nsteps * 0.8:
            heights.append(data.xpos[base, 2])
    R = data.xmat[base].reshape(3, 3)
    tilt_deg = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    stands = tilt_deg < 5.0 and abs(data.xpos[base, 2] - np.mean(heights)) < 0.01
    if stands:
        init_qpos = data.qpos.copy()
        foot_z = data.xpos[feet, 2].copy()
        nominal = float(np.mean(heights))
    else:  # fall back to the geometric placement; the RL policy will have to do the balancing
        init_qpos = placed_qpos
        foot_z = placed_foot_z
        nominal = float(placed_qpos[base_qadr + 2])
    init_qpos[base_qadr:base_qadr + 2] = 0.0
    total_mass = float(np.sum(model.body_mass[_subtree_bodies(model, base)]))
    if verbose:
        print(f"[{emb.name}] {n} actuated joints, mass {total_mass:.1f} kg, nominal base height "
              f"{nominal:.3f} m, tilt after {settle_time:.1f}s PD hold {tilt_deg:.1f} deg, stands={stands}")
    if not stands:
        import warnings
        warnings.warn(f"{emb.name}: the PD-held default pose did not stand still (tilt {tilt_deg:.1f} deg). "
                      "Check default_pose / PD gains in standwalk/embodiments.py")
    # sensor layout
    slices = {}
    for sid in range(model.nsensor):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sid)
        slices[nm] = slice(int(model.sensor_adr[sid]), int(model.sensor_adr[sid] + model.sensor_dim[sid]))
    touch_feet = tuple(tuple(slices[f"touch_{site}"] for (_, site, _, _) in touch_boxes if site.startswith(f"sw_touch_{tag}_"))
                       for tag in ("L", "R"))
    touch_base = tuple(slices[f"touch_{site}"] for (_, site, _, _) in touch_boxes if site.startswith("sw_touch_base_"))
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    assert nstate == 1 + model.nq + model.nv + model.na, "unexpected physics state layout"

    return LoadedEmbodiment(
        emb=emb, model=model, joint_names=actuated, qpos_adr=qpos_adr, dof_adr=dof_adr,
        default_pose=default, kp=kp, kd=kd, torque_limit=tl, joint_range=jrange, swing_mask=swing_mask,
        base_body=base, base_qpos_adr=base_qadr, base_dof_adr=base_dadr, feet_bodies=feet,
        geom_foot=geom_foot, floor_geom=floor, init_qpos=init_qpos, nominal_height=nominal,
        foot_z_offset=foot_z, total_mass=total_mass, stands=bool(stands), settle_tilt_deg=tilt_deg,
        sensor_slices=slices, touch_feet=touch_feet, touch_base=touch_base, nstate=nstate,
    )


def describe(le: LoadedEmbodiment) -> str:
    """Human-readable summary table of a loaded embodiment."""
    m = le.model
    lines = [f"{le.emb.name}: {le.emb.notes}",
             f"  bodies={m.nbody} joints={m.njnt} nq={m.nq} nv={m.nv} actuated={le.n_act} "
             f"mass={le.total_mass:.1f} kg  nominal height={le.nominal_height:.3f} m  stands={le.stands}",
             f"  sim dt={le.emb.sim_dt*1000:.0f} ms  ctrl dt={le.emb.ctrl_dt*1000:.0f} ms  decimation={le.decimation}",
             f"  {'joint':34s} {'range [rad]':>18s} {'default':>8s} {'kp':>6s} {'kd':>5s} {'tau':>6s} swing"]
    for i, j in enumerate(le.joint_names):
        lo, hi = le.joint_range[i]
        lines.append(f"  {j:34s} [{lo:7.2f},{hi:7.2f}] {le.default_pose[i]:8.2f} {le.kp[i]:6.0f} "
                     f"{le.kd[i]:5.1f} {le.torque_limit[i]:6.0f} {'*' if le.swing_mask[i] else ''}")
    return "\n".join(lines)
