"""Stage-0 exit criteria, as executable checks.

Run: uv run python -m pytest tests -q
"""

import math

import mujoco
import numpy as np
import pytest

from src.rsbot.model import load, leg_ik, WHEEL_R
from src.rsbot.sim import rollout


def test_model_mass_matches_budget():
    m, _ = load()
    assert 1.80 <= m.body_subtreemass[1] <= 1.95


def test_stance_geometry():
    """Nominal stance: wheels on the ground, soles clear, head near 43 cm."""
    m, d = load()
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    assert d.xpos[torso][2] == pytest.approx(0.247, abs=1e-3)

    def gz(name):
        return d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)][2]

    assert gz("wheel_l") == pytest.approx(WHEEL_R, abs=2e-3)
    # Retracted soles must not touch the floor or they steal wheel traction.
    assert gz("sole_l") > WHEEL_R and gz("sole_r") > WHEEL_R


def test_com_over_wheel_axle():
    """Any fore/aft CoM offset shows up as a permanent standing lean."""
    m, d = load()
    mujoco.mj_forward(m, d)
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    assert abs(d.subtree_com[1][0] - axle[0]) < 1e-3


def test_leg_ik_roundtrip():
    for h in (0.14, 0.18, 0.207):
        hip, knee = leg_ik(h)
        # Symmetric two-link, wheel stays directly under the hip.
        assert 0.110 * math.sin(hip) + 0.110 * math.sin(hip + knee) == pytest.approx(0, abs=1e-9)
        assert 0.110 * math.cos(hip) + 0.110 * math.cos(hip + knee) == pytest.approx(h, abs=1e-9)


def test_stands_unattended():
    r = rollout(duration=60.0)
    assert not r["fell"]
    assert abs(r["drift"]) < 0.05
    assert abs(r["max_pitch"]) < math.radians(5)


@pytest.mark.parametrize("impulse", [0.35, -0.35, 1.4, -1.4])
def test_rejects_shove(impulse):
    r = rollout(duration=8.0, shove=(2.0, impulse))
    assert not r["fell"]
    assert abs(r["drift"]) < 0.30


def test_tracks_velocity_command():
    r = rollout(duration=12.0, v_des=lambda t: 0.25 if 2 < t < 8 else 0.0)
    assert not r["fell"]
    # 6 s at 0.25 m/s = 1.5 m, allow 15% for accel/decel transients.
    assert r["drift"] == pytest.approx(1.5, rel=0.15)
