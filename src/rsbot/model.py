"""Model loading and shared geometry constants."""

from pathlib import Path

import mujoco

XML = Path(__file__).parent / "model" / "rsbot.xml"

WHEEL_R = 0.040
THIGH_L = 0.110
SHIN_L = 0.110

# How far the sole swings between deployed and retracted, in world terms.
RETRACT_SWEEP = 2.27


def sole_angle(hip, knee, phi):
    """Ankle command that holds the sole at world angle `phi`.

    The ankle encoder is relative to the shin, and the shin pitches as the
    robot squats, so a constant ankle angle is NOT a constant sole angle.
    phi = 0 puts the plate flat on the ground; phi = RETRACT_SWEEP tucks it up.
    """
    return phi - (hip + knee)

# Joint/actuator names in the order the controller uses them.
LEGS = ("l", "r")


def load():
    """Return (model, data) reset to the stance keyframe."""
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d


def leg_ik(height):
    """Hip/knee angles that put the wheel axle `height` below the hip axis.

    Symmetric two-link: hip = q, knee = -2q keeps the wheel directly under
    the hip, so leg length is the only thing that changes.
    """
    c = height / (THIGH_L + SHIN_L)
    c = min(max(c, -1.0), 1.0)
    import math

    q = math.acos(c)
    return q, -2.0 * q
