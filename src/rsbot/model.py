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


# --- Real part dimensions, metres --------------------------------------------
#
# Visual-only geometry built from the actual parts, so the render shows what
# would really be bolted together. It carries no mass and no collision: mass
# lives on the simple collision shapes, which are moved to geom group 4 and
# hidden. Nothing here changes the dynamics. See docs/bom.md.

SERVO = (0.0452, 0.0247, 0.0354)      # STS3215 body, L x W x H
SERVO_HORN_R = 0.0100                 # 25T output horn
PI5 = (0.085, 0.056, 0.017)
BATT_3S = (0.105, 0.034, 0.024)       # 2200 mAh 3S pack
DRIVER = (0.050, 0.030, 0.010)        # TTL bus adapter
WHEEL_TIRE_T = 0.008                  # tyre wall thickness

C_SERVO = "0.13 0.13 0.15 1"
C_HORN = "0.72 0.73 0.76 1"
C_PRINT = "0.88 0.45 0.13 1"          # printed PETG
C_TIRE = "0.09 0.09 0.10 1"
C_HUB = "0.55 0.56 0.60 1"
C_PCB = "0.05 0.33 0.17 1"
C_BATT = "0.16 0.16 0.38 1"
C_PLATE = "0.62 0.64 0.68 0.30"   # translucent, so the internals show


def _v(name, gtype, size, pos, rgba, euler=None):
    """A visual-only geom: no mass, no collision, hidden group for the solids."""
    e = f' euler="{euler}"' if euler else ""
    return (f'<geom name="{name}" type="{gtype}" size="{size}" pos="{pos}"'
            f'{e} rgba="{rgba}" contype="0" conaffinity="0" mass="0" group="0"/>')


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

SEG_MASS = {"thigh": 0.130, "shin": 0.100, "ankle": 0.060,
            "rollbracket": 0.060, "wheel": 0.060}
_RANGE = {"hip": "-0.60 1.40", "knee": "-2.00 0.05",
          "ankle_pitch": "-1.60 1.60", "ankle_roll": "-0.10 1.75"}


def _link_geoms(link, side, sgn):
    """Collision shape (group 4, carries the mass) plus the visual build."""
    L, W, H = SERVO
    sl, sw, sh = L / 2, W / 2, H / 2
    m = SEG_MASS[link]
    col, vis = [], []

    if link == "thigh":
        col.append(f'<geom class="leg" name="thigh_{side}" fromto="0 0 0  0 0 -0.110" '
                   f'mass="{m}" group="4"/>')
        vis += [_v(f"vhipsv_{side}", "box", f"{sh:.5f} {sw:.5f} {sl:.5f}",
                   f"0 0 {-sl:.5f}", C_SERVO),
                _v(f"vhiphorn_{side}", "cylinder", f"{SERVO_HORN_R} 0.004",
                   f"0 {sgn*0.016:.4f} 0", C_HORN, euler="1.5708 0 0"),
                _v(f"vthighbr_{side}", "box", f"0.010 0.016 0.0324",
                   "0 0 -0.0776", C_PRINT)]
    elif link == "shin":
        col.append(f'<geom class="leg" name="shin_{side}" fromto="0 0 0  0 0 -0.094" '
                   f'mass="{m}" group="4"/>')
        vis += [_v(f"vkneesv_{side}", "box", f"{sh:.5f} {sw:.5f} {sl:.5f}",
                   f"0 0 {-sl:.5f}", C_SERVO),
                _v(f"vkneehorn_{side}", "cylinder", f"{SERVO_HORN_R} 0.004",
                   f"0 {sgn*0.016:.4f} 0", C_HORN, euler="1.5708 0 0"),
                _v(f"vshinbr_{side}", "box", f"0.010 0.016 0.0324",
                   "0 0 -0.0776", C_PRINT)]
    elif link == "ankle":
        # Ankle-pitch servo. Must sit ABOVE the axle: in foot mode the wheel
        # face is only 12 mm below it, and anything lower becomes the contact.
        col.append(f'<geom class="ankle" name="ankle_{side}" '
                   f'size="{sh:.5f} {sw:.5f} {sl:.5f}" pos="0 0 {sl:.5f}" '
                   f'mass="{m}" group="4"/>')
        vis.append(_v(f"vanksv_{side}", "box", f"{sh:.5f} {sw:.5f} {sl:.5f}",
                      f"0 0 {sl:.5f}", C_SERVO))
    elif link == "rollbracket":
        col.append(f'<inertial pos="0 0 0.01" mass="{m}" '
                   f'diaginertia="4e-5 4e-5 4e-5"/>')
        # Roll servo: output on the fore/aft axis, so the body runs fore/aft.
        # Offset OUTBOARD in y, because the 90 deg roll maps y onto z: anything
        # sitting at y=0 ends up level with the axle and grazes the floor.
        vis.append(_v(f"vrollsv_{side}", "box", f"{sl:.5f} {sw:.5f} {sh:.5f}",
                      f"-0.034 {sgn*0.022:.4f} 0.020", C_SERVO))
        # Wheel-drive servo, OUTBOARD so the 90 deg roll swings it up, not
        # down into the floor.
        vis.append(_v(f"vwhlsv_{side}", "box", f"{sh:.5f} {sl:.5f} {sw:.5f}",
                      f"0 {sgn*(0.012+sl):.5f} 0", C_SERVO))
        # Compact and hugging the axle. A plate reaching 45 mm up swings 18 mm
        # BELOW the floor once the ankle rolls, which the abstract model hid.
        vis.append(_v(f"vrollbr_{side}", "box", "0.018 0.010 0.010",
                      f"0 {sgn*0.006:.4f} 0.018", C_PRINT))
    elif link == "wheel":
        col.append(f'<geom class="wheel" name="wheel_{side}" zaxis="0 1 0" '
                   f'mass="{m}" group="4"/>')
        vis += [_v(f"vtire_{side}", "cylinder", "0.040 0.012", "0 0 0", C_TIRE,
                   euler="1.5708 0 0"),
                _v(f"vhub_{side}", "cylinder", "0.024 0.0115", "0 0 0", C_HUB,
                   euler="1.5708 0 0")]
    return col + vis


# joint, link body, offset from the parent
CHAIN = [("hip", "thigh", None), ("knee", "shin", "0 0 -0.110"),
         ("ankle_pitch", "ankle", "0 0 -0.110"),
         ("ankle_roll", "rollbracket", "0 0 0"), ("wheel", "wheel", "0 0 0")]


def _leg(side, backlash):
    """One leg: hip pitch, knee pitch, ankle pitch, ankle ROLL, wheel.

    The roll bracket is its own body because the wheel-drive servo bolts to it
    and must NOT spin with the wheel.
    """
    sgn = 1 if side == "l" else -1
    y = sgn * 0.060
    b = backlash / 2.0
    lash_attrs = (f'damping="{LASH_DAMPING}" armature="{LASH_ARMATURE}" '
                  f'frictionloss="{LASH_FRICTION}"')
    stub = f'<inertial pos="0 0 0" mass="{GEAR_MASS}" diaginertia="1e-6 1e-6 1e-6"/>'

    opens, closes = [], []
    depth = 3
    for joint, link, pos in CHAIN:
        pos = pos or f"0 {y} 0"
        axis = ("1 0 0" if side == "l" else "-1 0 0") if joint == "ankle_roll" else "0 1 0"
        jn = f"{joint}_{side}"
        rng = f' range="{_RANGE[joint]}"' if joint in _RANGE else ""
        ind = "  " * depth
        joint_xml = f'<joint name="{jn}" axis="{axis}"{rng}/>'
        body = f"{link}_{side}"

        if backlash > 0:
            opens.append(
                f'{ind}<body name="{jn}_drv" pos="{pos}">\n{ind}  {joint_xml}\n'
                f'{ind}  {stub}\n'
                f'{ind}  <body name="{body}" pos="0 0 0">\n'
                f'{ind}    <joint name="{jn}_lash" axis="{axis}" '
                f'range="{-b:.6f} {b:.6f}" {lash_attrs}/>\n')
            closes.append(f"{ind}  </body>\n{ind}</body>\n")
            gind = "  " * (depth + 2)
            depth += 2
        else:
            opens.append(f'{ind}<body name="{body}" pos="{pos}">\n{ind}  {joint_xml}\n')
            closes.append(f"{ind}</body>\n")
            gind = "  " * (depth + 1)
            depth += 1

        for g in _link_geoms(link, side, sgn):
            opens[-1] += gind + g + "\n"

    return "".join(opens) + "".join(reversed(closes))


def _torso_visual():
    """Chassis and the parts inside it, drawn at real size.

    Two things this makes obvious that a plain box did not: the 3S pack is
    105 mm long and will not lie flat in a 90 mm bay, so it stands upright;
    and once the Pi and the pack are in, the bay is essentially full.
    """
    x = 0.0085
    g = []
    for sgn in (1, -1):
        g.append(_v(f"vside{sgn}", "box", "0.045 0.0015 0.090",
                    f"{x} {sgn*0.0345:.4f} 0.090", C_PLATE))
    g.append(_v("vtop", "box", "0.045 0.035 0.0015", f"{x} 0 0.1785", C_PLATE))
    g.append(_v("vfloor", "box", "0.045 0.035 0.0015", f"{x} 0 0.0015", C_PLATE))
    g.append(_v("vpi", "box", f"{PI5[0]/2} {PI5[1]/2} {PI5[2]/2}",
                f"{x} 0 0.030", C_PCB))
    # Stood on end: 105 mm will not fit across a 90 mm bay.
    g.append(_v("vbatt", "box", f"{BATT_3S[2]/2} {BATT_3S[1]/2} {BATT_3S[0]/2}",
                f"{x} 0 0.115", C_BATT))
    g.append(_v("vdriver", "box", f"{DRIVER[0]/2} {DRIVER[1]/2} {DRIVER[2]/2}",
                f"{x} 0 0.058", C_PCB))
    return "\n      ".join(g)


def _excludes():
    """Contact excludes for every link pair inside a leg.

    MuJoCo only auto-excludes DIRECT parent-child pairs. The ankle body already
    made shin and wheel grandparent/grandchild, and switching backlash on adds
    a drive body at every joint, which breaks the remaining pairs too. Without
    these the leg collides with itself: 14 contacts at rest and qacc of 2e4.
    Listing them all is cheap and does not depend on the chain's shape.
    """
    links = ["thigh", "shin", "ankle", "rollbracket", "wheel"]
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
            .replace("<!--EXCLUDES-->", _excludes())
            .replace("<!--TORSO_VIS-->", _torso_visual()))
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
