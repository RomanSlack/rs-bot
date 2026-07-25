"""Wheel-flip foot mode: the wheel face IS the sole."""

import math

import mujoco
import numpy as np
import pytest

from src.rsbot.model import (ROLL_FOOT, ROLL_WHEEL, WHEEL_HALF_W, WHEEL_R,
                            ankle_pitch_level, axle_height, leg_ik, load)
from src.rsbot.sim import rollout_cycle, rollout_deploy
from src.rsbot.transition import DeployCfg


def _axle_z(m, d):
    return float(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                          "wheel_l")][2])


def test_flip_dig_is_tiny():
    """The rim corner leads briefly. This is the whole cost of the mechanism."""
    peak = max(axle_height(r) for r in np.linspace(0, ROLL_FOOT, 2000))
    assert peak - WHEEL_R == pytest.approx(0.00176, abs=1e-4)
    assert axle_height(ROLL_FOOT) == pytest.approx(WHEEL_HALF_W, abs=1e-9)


def test_axle_descends_monotonically_after_the_bump():
    rolls = np.linspace(0.30, ROLL_FOOT, 500)
    h = [axle_height(r) for r in rolls]
    assert all(b <= a + 1e-12 for a, b in zip(h, h[1:]))


def test_nothing_hangs_below_the_wheel_face():
    """In foot mode the face is only 12 mm under the axle. Anything lower
    becomes the real contact and steals the 80 mm foot."""
    m, d = load()
    hip, knee = leg_ik(0.195)
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = 0.195 + WHEEL_HALF_W, 1.0
    for base in (7, 12):
        d.qpos[base], d.qpos[base + 1] = hip, knee
        d.qpos[base + 2] = ankle_pitch_level(hip, knee)
        d.qpos[base + 3] = ROLL_FOOT
    mujoco.mj_forward(m, d)
    mujoco.mj_collision(m, d)
    touching = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
                for c in d.contact[:d.ncon]}
    assert touching <= {"wheel_l", "wheel_r"}, f"unexpected ground contact: {touching}"


def test_shin_does_not_collide_with_its_own_wheel():
    """The ankle body makes these grandparent/grandchild, which MuJoCo does
    not auto-exclude."""
    m, d = load()
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(m.ngeom)]
    pairs = {tuple(sorted((names[c.geom1], names[c.geom2]))) for c in d.contact[:d.ncon]}
    assert ("shin_l", "wheel_l") not in pairs
    assert ("shin_r", "wheel_r") not in pairs


def test_ankle_pitch_levels_the_foot_at_any_leg_length():
    for h in (0.140, 0.170, 0.195, 0.210):
        hip, knee = leg_ik(h)
        assert hip + knee + ankle_pitch_level(hip, knee) == pytest.approx(0, abs=1e-12)


def test_transition_reaches_stand_and_holds():
    r = rollout_deploy(cfg=DeployCfg(flip_rate=2.0), duration=40.0)
    assert not r["fell"]
    assert r["state"] == "STAND"
    assert r["stand_duration"] > 30.0
    assert abs(r["drift"]) < 0.05
    # Standing on the faces, not perched on the rims.
    assert abs(r["wheel_clear"]) < 0.002


@pytest.mark.parametrize("rate", [0.5, 2.0, 8.0])
def test_transition_is_robust_across_flip_rates(rate):
    """A 16x range of flip rates all work, which is the point: this mechanism
    is not living on a knife edge the way the earlier two were."""
    r = rollout_deploy(cfg=DeployCfg(flip_rate=rate), duration=25.0)
    assert not r["fell"]
    assert r["state"] == "STAND"


def test_round_trip_and_keeps_driving():
    """Stage 0b is only done if it can get back OUT of foot mode and stay
    useful, not just get into it."""
    r = rollout_cycle(cfg=DeployCfg(flip_rate=2.0), stand_for=4.0, duration=30.0)
    assert not r["fell"]
    assert r["reached_stand"] and r["back_on_wheels"]
    assert r["state"] == "WHEEL"
    # It stood still while standing, then carried on well past where it flipped.
    assert abs(r["slip_while_standing"]) < 0.02
    assert r["x_end"] > r["x_at_flip"] + 1.0


def test_twenty_consecutive_transitions():
    """The stage-2 hardware criterion, run in sim."""
    import mujoco
    from src.rsbot.sim import CTRL_HZ, obs
    from src.rsbot.transition import DeployMachine, STAND, WHEEL
    from src.rsbot.balance import pitch_from_quat

    m, d = load()
    mach = DeployMachine(cfg=DeployCfg(flip_rate=2.0))
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep

    cycles, phase, mark = 0, "drive", 2.0
    for k in range(int(400 / m.opt.timestep)):
        t = k * m.opt.timestep
        if phase == "drive" and t >= mark and mach.state == WHEEL:
            mach.start_deploy(); phase = "toFoot"
        elif phase == "toFoot" and mach.state == STAND:
            phase, mark = "standing", t + 1.5
        elif phase == "standing" and t >= mark:
            mach.start_retract(); phase = "toWheel"
        elif phase == "toWheel" and mach.state == WHEEL:
            cycles += 1; phase, mark = "drive", t + 1.5
            if cycles >= 20:
                break
        if k % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt,
                             v_des=0.30 if phase == "drive" else 0.0)
        mujoco.mj_step(m, d)
        assert abs(pitch_from_quat(obs(m, d)["quat"])) < 1.0, f"fell, cycle {cycles+1}"
        assert d.xpos[torso][2] > 0.12
    assert cycles == 20
