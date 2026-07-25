"""Sole sweep geometry. Guards the mechanism, not the controller."""

import numpy as np
import pytest
import mujoco

from src.rsbot.model import load, leg_ik, sole_angle, RETRACT_SWEEP, WHEEL_R
from clearance import sweep

HEIGHTS = (0.207, 0.170, 0.140)


@pytest.mark.parametrize("h", HEIGHTS)
def test_sole_never_hits_shin_or_wheel(h):
    a = sweep(h, verbose=False)
    assert a[:, 2].min() > 0.0, "sole clips the shin somewhere in the sweep"
    assert a[:, 3].min() > 0.0, "sole clips the wheel somewhere in the sweep"


@pytest.mark.parametrize("h", HEIGHTS)
def test_deployed_lift_is_stance_independent(h):
    """Deploying must lift the wheel by the same amount at any squat depth."""
    a = sweep(h, verbose=False)
    assert -a[0, 1] == pytest.approx(0.015, abs=1e-3)


def test_sole_angle_is_world_referenced():
    """A constant world sole angle must survive a change of leg configuration."""
    m, d = load()
    zs = []
    for h in HEIGHTS:
        hip, knee = leg_ik(h)
        d.qpos[2] = h + WHEEL_R
        d.qpos[7], d.qpos[8] = hip, knee
        d.qpos[11], d.qpos[12] = hip, knee
        d.qpos[10] = d.qpos[14] = sole_angle(hip, knee, RETRACT_SWEEP)
        mujoco.mj_forward(m, d)
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "sole_l")
        # World pitch of the sole plate, extracted from its rotation matrix.
        R = d.geom_xmat[gid].reshape(3, 3)
        zs.append(np.arctan2(R[0, 2], R[2, 2]))
    assert max(zs) - min(zs) < 1e-9


def test_retracted_sole_clears_the_floor_at_every_squat_depth():
    a = np.array([sweep(h, verbose=False) for h in HEIGHTS])
    # Last row of each sweep is the retracted end.
    assert a[:, -1, 1].min() > 0.005
