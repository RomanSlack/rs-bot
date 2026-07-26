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
    # Look the torso up by name: the scale-reference human is also a
    # top-level body, so index 1 is not the robot.
    t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    # 1.71 kg, derived rather than estimated: see cad/masses.py. The printed
    # structure turned out far lighter than the hand-assigned numbers.
    assert 1.73 <= m.body_subtreemass[t] <= 1.80


def test_stance_geometry():
    """Nominal stance: wheels on the ground, head near 43 cm."""
    m, d = load()
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    assert d.xpos[torso][2] == pytest.approx(0.247, abs=1e-3)

    def gz(name):
        return d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)][2]

    assert gz("wheel_l") == pytest.approx(WHEEL_R, abs=2e-3)


def test_com_over_wheel_axle():
    """Any fore/aft CoM offset shows up as a permanent standing lean."""
    m, d = load()
    mujoco.mj_forward(m, d)
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    assert abs(d.subtree_com[t][0] - axle[0]) < 1e-3


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


# 1.0 N.s, not 1.4: the robot lost 340 g, so the same impulse is a bigger
# velocity change. Threshold re-measured, not relaxed to make a test pass.
@pytest.mark.parametrize("impulse", [0.35, -0.35, 1.0, -1.0])
def test_rejects_shove(impulse):
    r = rollout(duration=8.0, shove=(2.0, impulse))
    assert not r["fell"]
    assert abs(r["drift"]) < 0.30


def test_tracks_velocity_command():
    r = rollout(duration=12.0, v_des=lambda t: 0.25 if 2 < t < 8 else 0.0)
    assert not r["fell"]
    # 6 s at 0.25 m/s = 1.5 m, allow 15% for accel/decel transients.
    assert r["drift"] == pytest.approx(1.5, rel=0.15)


@pytest.mark.parametrize("yaw", [0.4, 0.8, 1.2, -0.8])
def test_steers_without_falling(yaw):
    """There is no steering joint: turning is a wheel speed difference,
    closed on the z gyro so it holds a rate."""
    import numpy as np
    from src.rsbot.model import load
    from src.rsbot.sim import CTRL_HZ, obs
    from src.rsbot.balance import Balancer, pitch_from_quat

    m, d = load()
    bal = Balancer()
    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    rates = []
    for k in range(int(9.0 / m.opt.timestep)):
        t = k * m.opt.timestep
        if k % decim == 0:
            d.ctrl[:] = bal(obs(m, d), dt, yaw_des=yaw if t > 2.0 else 0.0)
        mujoco.mj_step(m, d)
        o = obs(m, d)
        assert abs(pitch_from_quat(o["quat"])) < 1.0, f"fell turning at {yaw}"
        if t > 4.5:
            rates.append(o["gyro"][2])
    achieved = float(np.mean(rates))
    assert achieved / yaw > 0.80, f"only tracked {achieved:.2f} of {yaw}"
