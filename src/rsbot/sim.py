"""Headless rollouts and the shove test."""

import numpy as np
import mujoco

from .model import load, WHEEL_R
from .balance import Balancer, Gains, pitch_from_quat

CTRL_HZ = 200


def _sensor(m, d, name):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
    adr, dim = m.sensor_adr[i], m.sensor_dim[i]
    return d.sensordata[adr:adr + dim]


def obs(m, d):
    return {
        "quat": np.array(_sensor(m, d, "imu_quat")),
        "gyro": np.array(_sensor(m, d, "imu_gyro")),
        "wheel_vel": np.array([_sensor(m, d, "wheel_l_vel")[0],
                               _sensor(m, d, "wheel_r_vel")[0]]),
        "ankle_pos": np.array([_sensor(m, d, "ankle_l_pos")[0],
                               _sensor(m, d, "ankle_r_pos")[0]]),
    }


def rollout(gains=None, duration=10.0, shove=None, v_des=0.0, viewer=None):
    """Run the balancer. `shove` is (time_s, impulse_N_s) applied to the torso +x.

    Returns a metrics dict: fell, max_pitch, drift, recovery time.
    """
    m, d = load()
    bal = Balancer(gains)
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")

    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    n = int(duration / m.opt.timestep)
    dt = decim * m.opt.timestep

    max_pitch = 0.0
    fell = False
    shove_t = shove[0] if shove else None
    recovered_at = None
    pitch = 0.0

    for k in range(n):
        t = k * m.opt.timestep

        if k % decim == 0:
            o = obs(m, d)
            pitch = pitch_from_quat(o["quat"])
            vd = v_des(t) if callable(v_des) else v_des
            d.ctrl[:] = bal(o, dt, vd)

        # Impulse over a single 10 ms window, applied at the torso CoM.
        d.xfrc_applied[torso] = 0.0
        if shove and shove_t <= t < shove_t + 0.010:
            d.xfrc_applied[torso, 0] = shove[1] / 0.010

        mujoco.mj_step(m, d)
        if viewer is not None:
            viewer(m, d, t)

        if shove and t > shove_t:
            max_pitch = max(max_pitch, abs(pitch))
            if recovered_at is None and t > shove_t + 0.05 \
               and abs(pitch) < 0.03 and abs(bal.v_filt) < 0.05:
                recovered_at = t - shove_t
        else:
            max_pitch = max(max_pitch, abs(pitch))

        if d.xpos[torso][2] < 0.12 or abs(pitch) > 1.0:
            fell = True
            break

    return {
        "fell": fell,
        "t_end": d.time,
        "max_pitch": max_pitch,
        "drift": float(d.xpos[torso][0]),
        "recovery": recovered_at,
        "final_pitch": pitch,
    }


def rollout_deploy(cfg=None, gains=None, trigger=2.0, duration=12.0, viewer=None,
                   sole=None):
    """Balance, deploy the feet, then stand there. Returns metrics + state log."""
    from .transition import DeployMachine, STAND, NAMES

    m, d = load(**(sole or {}))
    mach = DeployMachine(gains, cfg)
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    wheel_l = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "wheel_l")

    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    fired = False
    fell = False
    peak_pitch = 0.0
    t_stand = None
    pitch = 0.0

    for k in range(int(duration / m.opt.timestep)):
        t = k * m.opt.timestep
        if not fired and t >= trigger:
            mach.start_deploy()
            fired = True

        if k % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt)

        mujoco.mj_step(m, d)
        if viewer is not None:
            viewer(m, d, t)

        pitch = pitch_from_quat(obs(m, d)["quat"])
        if fired:
            peak_pitch = max(peak_pitch, abs(pitch))
        if mach.state == STAND and t_stand is None:
            t_stand = t

        if d.xpos[torso][2] < 0.12 or abs(pitch) > 1.0:
            fell = True
            break

    # Wheel off the ground = the soles really are carrying the robot.
    wheel_clear = d.geom_xpos[wheel_l][2] - WHEEL_R

    return {
        "fell": fell,
        "state": NAMES[mach.state],
        "t_stand": t_stand,
        "stand_duration": (d.time - t_stand) if t_stand else 0.0,
        "peak_pitch": peak_pitch,
        "final_pitch": pitch,
        "wheel_clear": wheel_clear,
        "drift": float(d.xpos[torso][0]),
        "log": mach.log,
    }


def cost(gains, duration=8.0):
    """Scalar score for tuning. Lower is better; falling is heavily penalised."""
    total = 0.0
    trials = [
        dict(duration=duration, shove=None),
        dict(duration=duration, shove=(2.0, 0.35)),
        dict(duration=duration, shove=(2.0, -0.35)),
        dict(duration=duration, v_des=lambda t: 0.25 if 2.0 < t < 5.0 else 0.0),
    ]
    for kw in trials:
        r = rollout(gains, **kw)
        if r["fell"]:
            total += 100.0 + 10.0 * (duration - r["t_end"])
            continue
        total += abs(r["drift"]) * 2.0 + r["max_pitch"] * 3.0
        if kw.get("shove"):
            total += (r["recovery"] if r["recovery"] is not None else 3.0)
    return total
