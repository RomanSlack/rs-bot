"""Model loading and shared geometry constants."""

import math
from pathlib import Path

import mujoco

XML = Path(__file__).parent / "model" / "rsbot.xml"

WHEEL_R = 0.040
THIGH_L = 0.110
SHIN_L = 0.110

def sole_angle(hip, knee, phi):
    """Ankle command that holds the strut at world angle `phi`.

    The ankle encoder is relative to the shin, and the shin pitches as the
    robot squats, so a constant ankle angle is NOT a constant strut angle.
    phi = 0 points the strut straight down; positive swings the tip forward.
    """
    return phi - (hip + knee)

# Joint/actuator names in the order the controller uses them.
LEGS = ("l", "r")


# --- Outrigger foot -----------------------------------------------------------
#
# A strut on a pivot part-way down the shin, swinging down INBOARD of the wheel,
# with a pad on the end. Replaces the flat plate that pivoted on the wheel axle;
# see docs/stage-0b.md for why that one could not work.
#
# Two things follow from moving the pivot off the axle:
#
#  1. The pad no longer has to cradle the wheel, so the pivot-to-pad distance is
#     free. Corner dig goes as roughly L^2/2R, so a longer strut shrinks it.
#  2. The pad's contact surface can be an ARC centred on the pivot, in which
#     case every point stays the same distance from the pivot as it swings and
#     the dig is exactly zero. That is what PAD_SEGMENTS approximates.
#
# The pads touch down while the WHEELS ARE STILL ON THE GROUND. That overlapping
# contact is the whole point: the robot is never airborne, so the balancer keeps
# authority right through the transition.

STRUT_PIVOT = 0.055        # pivot, measured down the shin from the knee
STRUT_LEN = 0.0825         # pivot to pad contact face
# The pad is biased FORWARD of the strut axis. The balancer necessarily hands
# over with the CoM above the wheel contact (x=0), so a pad lying entirely
# behind the axle puts the CoM on the polygon's forward edge and it tips. How
# far forward it can reach is capped by the swing: no pad corner may be further
# from the pivot than the pivot's height during the swing, or it digs.
PAD_HALF_LEN = 0.034
PAD_OFFSET = 0.0094
PAD_THICK = 0.004
PAD_HALF_W = 0.010
PAD_Y = 0.024              # inboard offset from the leg plane, clears the wheel

# The transition is a two-move sequence, and the leg lengths are what make it
# safe. STRUT_LEN equals the pivot height at STAND_HEIGHT, so with the strut
# straight down the pad sits exactly on the floor WITH THE WHEELS STILL DOWN.
# That is the critical state: both contacts live, robot never airborne.
#
# The swing happens at SWING_HEIGHT instead, where the pivot is 93 mm up. The
# pad's furthest corner is hypot(STRUT_LEN, PAD_HALF_LEN) = 89.6 mm from the
# pivot, so it clears the floor by ~3.6 mm through the whole swing and cannot
# dig. Squatting from SWING_HEIGHT to STAND_HEIGHT then lowers the pad onto
# the ground under full balancer authority.
SWING_HEIGHT = 0.213
STAND_HEIGHT = 0.170
DEPLOY_PHI = 0.0           # strut straight down
RETRACT_PHI = -2.30        # tucked up and back


# Shin angle at which BOTH contacts are satisfied at once: pad on the floor
# (pivot at STRUT_LEN) and wheel on the floor (axle at WHEEL_R). The pivot sits
# a fixed distance up the shin from the axle, so this pins the shin angle -- it
# is not a free choice. Shifting the body must hold it, or the pads gouge into
# the floor and lift the wheels.
SHIN_ANGLE_STAND = math.acos((STRUT_LEN - WHEEL_R) / (SHIN_L - STRUT_PIVOT))


def foot_pose(shift=0.0):
    """Hip/knee for foot mode, with the axle `shift` ahead of the hip.

    Holds the shin angle at SHIN_ANGLE_STAND so the pad and the wheel stay
    exactly co-planar with the floor while the torso walks backwards.
    """
    b = -SHIN_ANGLE_STAND
    a = math.asin(min(max(-shift / THIGH_L - math.sin(b), -1.0), 1.0))
    return a, b - a


def pivot_height(leg_len):
    """World height of the strut pivot for a given leg length."""
    q, _ = leg_ik(leg_len)
    return WHEEL_R + STRUT_PIVOT * math.cos(q)


def _foot(side):
    """XML for one outrigger foot. `side` is 'l' or 'r'."""
    y = -PAD_Y if side == "l" else PAD_Y
    return (
        f'<body name="foot_{side}" pos="0 {y:.6f} {-STRUT_PIVOT:.6f}">\n'
        f'            <joint name="ankle_{side}" axis="0 1 0" range="-3.00 1.00"/>\n'
        f'            <geom name="strut_{side}" type="capsule" size="0.005" '
        f'fromto="0 0 0  0 0 {-(STRUT_LEN - PAD_THICK):.6f}" mass="0.008" '
        f'rgba="0.75 0.76 0.78 1"/>\n'
        f'            <geom class="sole" name="pad_{side}" '
        f'size="{PAD_HALF_LEN} {PAD_HALF_W} {PAD_THICK}" '
        f'pos="{PAD_OFFSET:.6f} 0 {-(STRUT_LEN - PAD_THICK):.6f}" mass="0.012"/>\n'
        f'          </body>')


def _xml(trim):
    txt = XML.read_text()
    txt = txt.replace("<!--FOOT_L-->", _foot("l"))
    txt = txt.replace("<!--FOOT_R-->", _foot("r"))
    old_trim = 'pos="0.0085 0 0.090"'
    assert old_trim in txt
    return txt.replace(old_trim, f'pos="{trim:.6f} 0 0.090"')


def _com_offset(m, d):
    mujoco.mj_forward(m, d)
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    return d.subtree_com[1][0] - axle[0]


def load(trim=None):
    """Return (model, data) reset to the stance keyframe.

    The retracted foot's position moves the standing CoM, so `trim` is solved
    for rather than hardcoded: build once, measure the CoM offset, shift the
    torso mass to null it. The relationship is linear, so one pass is exact.
    """
    if trim is None:
        probe = mujoco.MjModel.from_xml_string(_xml(0.0))
        pd = mujoco.MjData(probe)
        mujoco.mj_resetDataKeyframe(probe, pd, 0)
        total = probe.body_subtreemass[1]
        torso_mass = probe.body_mass[1]
        trim = -_com_offset(probe, pd) * total / torso_mass

    m = mujoco.MjModel.from_xml_string(_xml(trim))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d


def leg_ik(height, shift=0.0):
    """Hip/knee angles placing the wheel axle `height` below and `shift` ahead
    of the hip axis.

    `shift` is what lets the robot stand on its outrigger pads. The pads land
    behind the axle, so with the body directly over the wheel the CoM sits on
    the forward edge of the support polygon and it topples every time. Driving
    the axle forward moves the torso back over the pads, which is the same
    thing a person does rocking onto their heels. shift=0 is the symmetric
    wheel-mode pose, hip = q, knee = -2q.
    """
    r = math.hypot(shift, height)
    c = min(max(r / (THIGH_L + SHIN_L), -1.0), 1.0)
    phi = math.acos(c)                  # half the included knee angle
    psi = math.atan2(-shift, height)    # tilt of the hip->axle line
    return psi + phi, -2.0 * phi
