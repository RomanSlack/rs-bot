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


# Baseline sole: flat plate, half-length 40 mm, contact face 55 mm below the
# ankle pivot. Both are swept in soles.py, because the baseline does not work.
SOLE_HALF_LEN = 0.040
SOLE_FACE_R = 0.055
SOLE_THICK = 0.003


def _xml(sole_half_len, sole_face_r, trim):
    # The sole cradles the wheel, so its INNER face has to clear the tyre.
    # Violating this makes MuJoCo generate sole-wheel contacts that look like
    # a control failure but are really an impossible part.
    inner = sole_face_r - 2 * SOLE_THICK
    if inner <= WHEEL_R:
        raise ValueError(
            f"sole_face_r={sole_face_r*1000:.1f} mm puts the plate inside the "
            f"wheel: inner face {inner*1000:.1f} mm < wheel radius "
            f"{WHEEL_R*1000:.1f} mm. Minimum is "
            f"{(WHEEL_R + 2*SOLE_THICK)*1000:.1f} mm.")
    txt = XML.read_text()
    old_size = 'size="0.040 0.016 0.003"'
    old_pos = 'pos="0 0 -0.052"'
    old_trim = 'pos="0.0085 0 0.090"'
    assert txt.count(old_size) == 2 and txt.count(old_pos) == 2
    txt = txt.replace(old_size, f'size="{sole_half_len:.6f} 0.016 {SOLE_THICK}"')
    txt = txt.replace(old_pos, f'pos="0 0 {-(sole_face_r - SOLE_THICK):.6f}"')
    return txt.replace(old_trim, f'pos="{trim:.6f} 0 0.090"')


def _com_offset(m, d):
    mujoco.mj_forward(m, d)
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    return d.subtree_com[1][0] - axle[0]


def load(sole_half_len=SOLE_HALF_LEN, sole_face_r=SOLE_FACE_R, trim=None):
    """Return (model, data) reset to the stance keyframe.

    Sole geometry is parameterised because the retracted sole's position moves
    the standing CoM, so changing it invalidates the torso trim. When `trim` is
    None it is solved for: build once, measure the CoM offset, and shift the
    torso mass to null it. The relationship is linear, so one pass is exact.
    """
    if trim is None:
        probe = mujoco.MjModel.from_xml_string(_xml(sole_half_len, sole_face_r, 0.0))
        pd = mujoco.MjData(probe)
        mujoco.mj_resetDataKeyframe(probe, pd, 0)
        total = probe.body_subtreemass[1]
        torso_mass = probe.body_mass[1]
        trim = -_com_offset(probe, pd) * total / torso_mass

    m = mujoco.MjModel.from_xml_string(_xml(sole_half_len, sole_face_r, trim))
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
