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


def _com_offset(m, d):
    mujoco.mj_forward(m, d)
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    return d.subtree_com[1][0] - axle[0]


def load(trim=None):
    """Return (model, data) reset to the stance keyframe.

    `trim` is the fore/aft offset of the torso mass, solved for rather than
    hardcoded: build once, measure how far the standing CoM sits from the axle,
    shift the torso to null it. Linear, so one pass is exact. Any residual
    offset shows up as a permanent standing lean.
    """
    base = XML.read_text()
    marker = 'pos="0.0085 0 0.090"'
    assert marker in base

    if trim is None:
        probe = mujoco.MjModel.from_xml_string(base)
        pd = mujoco.MjData(probe)
        mujoco.mj_resetDataKeyframe(probe, pd, 0)
        trim = 0.0085 - _com_offset(probe, pd) * (probe.body_subtreemass[1]
                                                  / probe.body_mass[1])

    m = mujoco.MjModel.from_xml_string(
        base.replace(marker, f'pos="{trim:.6f} 0 0.090"'))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d
