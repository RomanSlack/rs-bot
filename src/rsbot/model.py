"""Model loading and shared geometry."""

import math
from pathlib import Path

import mujoco
import numpy as np

XML = Path(__file__).parent / "model" / "rsbot.xml"

WHEEL_R = 0.040
WHEEL_HALF_W = 0.012
THIGH_L = 0.110
SHIN_L = 0.110

# Ankle roll: 0 is wheel mode, pi/2 lays the disc flat so it becomes the foot.
ROLL_WHEEL = 0.0
ROLL_FOOT = math.pi / 2


def axle_height(roll):
    """Floor-to-axle height at a given roll, i.e. the tilted disc's low point.

    Rises 1.76 mm at 16.7 deg as the rim corner leads, then falls monotonically
    to WHEEL_HALF_W. That 1.76 mm is this mechanism's entire dig; the flat
    plate it replaces dug 11-28 mm and went airborne doing it.
    """
    return WHEEL_R * math.cos(roll) + WHEEL_HALF_W * math.sin(roll)


def ankle_pitch_level(hip, knee, bias=0.0):
    """Ankle pitch holding the foot face level with the floor.

    Hip, knee and ankle pitch all turn about +y and compound, so the face is
    level when they sum to zero. `bias` tips the foot on purpose, which is the
    ankle flex available in foot mode.
    """
    return bias - (hip + knee)


def leg_ik(height, shift=0.0):
    """Hip/knee placing the ankle `height` below and `shift` ahead of the hip.

    shift=0 is the symmetric pose, hip = q, knee = -2q, which keeps the ankle
    directly under the hip so leg length is the only thing that changes.
    """
    r = math.hypot(shift, height)
    c = min(max(r / (THIGH_L + SHIN_L), -1.0), 1.0)
    phi = math.acos(c)
    psi = math.atan2(-shift, height)
    return psi + phi, -2.0 * phi


def make_ctrl(hip, knee, apitch, aroll, wheel):
    """Assemble the 10-actuator command, same values both legs."""
    return np.array([hip, knee, apitch, aroll, wheel,
                     hip, knee, apitch, aroll, wheel])


# --- Leg construction, with optional gear backlash ----------------------------
#
# Backlash is modelled physically, not as a command deadband: each actuated
# joint gets a second "lash" joint in series, free to float within +/- lash/2.
# The servo drives and senses the proximal side; the link hangs off the distal
# side and can move without the motor knowing. That floating deadzone in the
# kinematic chain is what actually hurts a balancer, and a deadband on the
# command would not reproduce it.
#
# The link bodies keep their canonical names (thigh_l, shin_l, wheel_l, ...)
# so sensors, contact excludes and lookups work either way.

GEAR_MASS = 0.002        # gearbox-side inertia stub, one per lashed joint
LASH_DAMPING = 0.001
LASH_ARMATURE = 0.0001
LASH_FRICTION = 0.0005

_LINKS = {
    "hip": ("thigh", '<geom class="leg" name="thigh_{s}" '
                     'fromto="0 0 0  0 0 -0.110" mass="0.130"/>'),
    "knee": ("shin", '<geom class="leg" name="shin_{s}" '
                     'fromto="0 0 0  0 0 -0.094" mass="0.100"/>'),
    "ankle_pitch": ("ankle", '<geom class="ankle" name="ankle_{s}" mass="0.120"/>'),
}
_RANGE = {"hip": "-0.60 1.40", "knee": "-2.00 0.05",
          "ankle_pitch": "-1.60 1.60", "ankle_roll": "-0.10 1.75"}


def _seg(joint, side, axis, body, pos, backlash, depth):
    """One joint plus its link body, with a lash joint spliced in if asked."""
    ind = "  " * depth
    jn = f"{joint}_{side}"
    rng = f' range="{_RANGE[joint]}"' if joint in _RANGE else ""
    j = f'<joint name="{jn}" axis="{axis}"{rng}/>'
    if backlash <= 0:
        return f'{ind}<body name="{body}" pos="{pos}">\n{ind}  {j}\n', f"{ind}</body>\n"
    b = backlash / 2.0
    head = (f'{ind}<body name="{jn}_drv" pos="{pos}">\n{ind}  {j}\n'
            f'{ind}  <inertial pos="0 0 0" mass="{GEAR_MASS}" '
            f'diaginertia="1e-6 1e-6 1e-6"/>\n'
            f'{ind}  <body name="{body}" pos="0 0 0">\n'
            f'{ind}    <joint name="{jn}_lash" axis="{axis}" '
            f'range="{-b:.6f} {b:.6f}" damping="{LASH_DAMPING}" '
            f'armature="{LASH_ARMATURE}" frictionloss="{LASH_FRICTION}"/>\n')
    return head, f"{ind}  </body>\n{ind}</body>\n"


def _leg(side, backlash):
    y = 0.060 if side == "l" else -0.060
    roll_axis = "1 0 0" if side == "l" else "-1 0 0"
    opens, closes = [], []

    for joint, pos in (("hip", f"0 {y} 0"), ("knee", "0 0 -0.110"),
                       ("ankle_pitch", "0 0 -0.110")):
        body, geom = _LINKS[joint]
        depth = 3 + len(opens) * (2 if backlash > 0 else 1)
        h, t = _seg(joint, side, "0 1 0", f"{body}_{side}", pos, backlash, depth)
        ind = "  " * (depth + (2 if backlash > 0 else 1))
        opens.append(h + ind + geom.format(s=side) + "\n")
        closes.append(t)

    depth = 3 + len(opens) * (2 if backlash > 0 else 1)
    ind = "  " * depth
    if backlash > 0:
        # Roll lash and the spin drive share one body, then the spin lash
        # carries the wheel. Every jointed body needs its own inertia, so a
        # bare carrier body is not an option.
        b = backlash / 2.0
        lash = (f'damping="{LASH_DAMPING}" armature="{LASH_ARMATURE}" '
                f'frictionloss="{LASH_FRICTION}"')
        stub = f'<inertial pos="0 0 0" mass="{GEAR_MASS}" diaginertia="1e-6 1e-6 1e-6"/>'
        body = (
            f'{ind}<body name="ankle_roll_{side}_drv" pos="0 0 0">\n'
            f'{ind}  <joint name="ankle_roll_{side}" axis="{roll_axis}" range="{_RANGE["ankle_roll"]}"/>\n'
            f'{ind}  {stub}\n'
            f'{ind}  <body name="wheel_{side}_drv" pos="0 0 0">\n'
            f'{ind}    <joint name="ankle_roll_{side}_lash" axis="{roll_axis}" range="{-b:.6f} {b:.6f}" {lash}/>\n'
            f'{ind}    <joint name="wheel_{side}" axis="0 1 0"/>\n'
            f'{ind}    {stub}\n'
            f'{ind}    <body name="wheel_{side}" pos="0 0 0">\n'
            f'{ind}      <joint name="wheel_{side}_lash" axis="0 1 0" range="{-b:.6f} {b:.6f}" {lash}/>\n'
            f'{ind}      <geom class="wheel" name="wheel_{side}" zaxis="0 1 0" mass="0.060"/>\n'
            f'{ind}    </body>\n{ind}  </body>\n{ind}</body>\n')
    else:
        body = (
            f'{ind}<body name="wheel_{side}" pos="0 0 0">\n'
            f'{ind}  <joint name="ankle_roll_{side}" axis="{roll_axis}" range="{_RANGE["ankle_roll"]}"/>\n'
            f'{ind}  <joint name="wheel_{side}" axis="0 1 0"/>\n'
            f'{ind}  <geom class="wheel" name="wheel_{side}" zaxis="0 1 0" mass="0.060"/>\n'
            f'{ind}</body>\n')

    return "".join(opens) + body + "".join(reversed(closes))


def _excludes():
    """Contact excludes for every link pair inside a leg.

    MuJoCo only auto-excludes DIRECT parent-child pairs. The ankle body already
    made shin and wheel grandparent/grandchild, and switching backlash on adds
    a drive body at every joint, which breaks the remaining pairs too. Without
    these the leg collides with itself: 14 contacts at rest and qacc of 2e4.
    Listing them all is cheap and does not depend on the chain's shape.
    """
    links = ["thigh", "shin", "ankle", "wheel"]
    out = []
    for s in ("l", "r"):
        out.append(f'    <exclude body1="torso" body2="thigh_{s}"/>')
        for i, a in enumerate(links):
            for b in links[i + 1:]:
                out.append(f'    <exclude body1="{a}_{s}" body2="{b}_{s}"/>')
    return "  <contact>\n" + "\n".join(out) + "\n  </contact>"


def _com_offset(m, d):
    mujoco.mj_forward(m, d)
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    return d.subtree_com[1][0] - axle[0]


STANCE = {"hip": 0.35, "knee": -0.70, "ankle_pitch": 0.35,
          "ankle_roll": 0.0, "wheel": 0.0}


def _write_stance(m):
    """Rewrite keyframe 0 by joint NAME.

    The lash joints interleave into the joint ordering, so a hand-written
    qpos vector silently scrambles across the wrong joints the moment
    backlash is switched on. Addressing by name is the only safe way.
    """
    d = mujoco.MjData(m)
    d.qpos[:] = 0.0
    d.qpos[0:3] = [0.0, 0.0, 0.247]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        if not name or name == "root":
            continue
        key = name.rsplit("_", 1)[0] if name.endswith(("_l", "_r")) else None
        d.qpos[m.jnt_qposadr[i]] = 0.0 if key is None else STANCE.get(key, 0.0)
    m.key_qpos[0] = d.qpos

    ctrl = np.zeros(m.nu)
    for i in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        key = {"apit": "ankle_pitch", "aroll": "ankle_roll"}.get(
            name.rsplit("_", 1)[0], name.rsplit("_", 1)[0])
        ctrl[i] = STANCE.get(key, 0.0)
    m.key_ctrl[0] = ctrl


def load(trim=None, backlash=0.0):
    """Return (model, data) reset to the stance keyframe.

    `backlash` is total gear lash per joint in radians, split +/- either side.
    Zero builds the rigid model with no extra bodies, so joint indices are
    unchanged. See docs/backlash.md.

    `trim` is the fore/aft offset of the torso mass, solved for rather than
    hardcoded: build once, measure how far the standing CoM sits from the axle,
    shift the torso to null it. Linear, so one pass is exact. Any residual
    offset shows up as a permanent standing lean.
    """
    base = (XML.read_text()
            .replace("<!--LEGS-->", _leg("l", backlash) + _leg("r", backlash))
            .replace("<!--EXCLUDES-->", _excludes()))
    marker = 'pos="0.0085 0 0.090"'
    assert marker in base

    if trim is None:
        probe = mujoco.MjModel.from_xml_string(base)
        _write_stance(probe)
        pd = mujoco.MjData(probe)
        mujoco.mj_resetDataKeyframe(probe, pd, 0)
        trim = 0.0085 - _com_offset(probe, pd) * (probe.body_subtreemass[1]
                                                  / probe.body_mass[1])

    m = mujoco.MjModel.from_xml_string(
        base.replace(marker, f'pos="{trim:.6f} 0 0.090"'))
    _write_stance(m)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d
