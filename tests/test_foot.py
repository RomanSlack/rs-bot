"""Outrigger foot geometry. Guards the mechanism, not the controller."""

import numpy as np
import mujoco
import pytest

from src.rsbot.model import (DEPLOY_PHI, RETRACT_PHI, STAND_HEIGHT, SWING_HEIGHT,
                            WHEEL_R, foot_pose, leg_ik, load, sole_angle)


def _pad_corners(m, d):
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pad_l")
    R = d.geom_xmat[g].reshape(3, 3)
    c, s = d.geom_xpos[g], m.geom_size[g]
    return np.array([c + R @ np.array([sx * s[0], 0, sz * s[2]])
                     for sx in (-1, 1) for sz in (-1, 1)])


def _pose(m, d, height, phi, pose=None):
    hip, knee = pose if pose else leg_ik(height)
    a = sole_angle(hip, knee, phi)
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = height + WHEEL_R, 1.0
    d.qpos[7], d.qpos[8], d.qpos[10] = hip, knee, a
    d.qpos[11], d.qpos[12], d.qpos[14] = hip, knee, a
    mujoco.mj_forward(m, d)
    return _pad_corners(m, d)


def test_swing_never_touches_the_floor():
    """The whole point of swinging at SWING_HEIGHT: no dig, at any angle."""
    m, d = load()
    lowest = min(_pose(m, d, SWING_HEIGHT, phi)[:, 2].min()
                 for phi in np.linspace(RETRACT_PHI, DEPLOY_PHI, 120))
    assert lowest > 0.002


def test_deployed_pad_sits_on_the_floor_with_wheels_down():
    """The critical state: pad and wheel co-planar, so nothing goes airborne."""
    m, d = load()
    corners = _pose(m, d, STAND_HEIGHT, DEPLOY_PHI, pose=foot_pose(0.0))
    assert corners[:, 2].min() == pytest.approx(0.0, abs=5e-4)


def test_pad_straddles_the_handover_point():
    """The balancer hands over with the CoM at x=0, so the polygon must
    contain x=0 or the robot tips forward the instant the wheels are cut."""
    m, d = load()
    corners = _pose(m, d, STAND_HEIGHT, DEPLOY_PHI, pose=foot_pose(0.0))
    assert corners[:, 0].min() < 0.0 < corners[:, 0].max()


def test_foot_pose_holds_the_shin_angle():
    """Shifting the body must not change the shin angle, or the pads gouge."""
    angles = [sum(foot_pose(s)) for s in (0.0, 0.03, 0.06, 0.09)]
    assert max(angles) - min(angles) < 1e-9


def test_retracted_foot_clears_the_floor_while_driving():
    m, d = load()
    for h in (0.140, 0.170, 0.207, 0.213):
        assert _pose(m, d, h, RETRACT_PHI)[:, 2].min() > 0.02
