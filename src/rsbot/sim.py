"""Headless rollouts and the shove test."""

import numpy as np
import mujoco

from .model import load, WHEEL_R, WHEEL_HALF_W
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
        "roll_pos": np.array([_sensor(m, d, "aroll_l_pos")[0],
                              _sensor(m, d, "aroll_r_pos")[0]]),
        "apitch_pos": np.array([_sensor(m, d, "apit_l_pos")[0],
                                _sensor(m, d, "apit_r_pos")[0]]),
    }


def _yaw_err(hist):
    if not hist:
        return 0.0
    tail = np.array(hist[len(hist) // 2:])
    return float(np.abs(tail[:, 1] - tail[:, 0]).mean())


def _yaw_rms(hist):
    """Deviation from the COMMANDED rate, root-mean-square. A robot cycling
    +/-2 rad/s about a commanded 0 has a mean error of zero and an RMS of 2."""
    if not hist:
        return 0.0
    tail = np.array(hist[len(hist) // 2:])
    return float(np.sqrt(((tail[:, 1] - tail[:, 0]) ** 2).mean()))


def rollout(gains=None, duration=10.0, shove=None, v_des=0.0, viewer=None,
            backlash=0.0, yaw_des=0.0):
    """Run the balancer. `shove` is (time_s, impulse_N_s) applied to the torso +x.

    `yaw_des` is a commanded turn rate, constant or a function of time. It
    exists because without it the tuning cost could not see the yaw loop at
    all: a set of gains once passed every trial while limit-cycling in yaw at
    +/-2 rad/s, counter-rotating the wheels at 20 rad/s and walking backwards a
    metre. Mean wheel speed stayed near zero throughout, so the odometry
    reported a robot standing perfectly still.

    Returns a metrics dict: fell, max_pitch, drift, recovery time, and yaw
    tracking.
    """
    m, d = load(backlash=backlash)
    bal = Balancer(gains)
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")

    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    n = int(duration / m.opt.timestep)
    dt = decim * m.opt.timestep

    max_pitch = 0.0
    pitch_hist = []
    yaw_hist = []
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
            yd = yaw_des(t) if callable(yaw_des) else yaw_des
            d.ctrl[:] = bal(o, dt, vd, yaw_des=yd)
            yaw_hist.append((yd, o["gyro"][2]))

        # Impulse over a single 10 ms window, applied at the torso CoM.
        d.xfrc_applied[torso] = 0.0
        if shove and shove_t <= t < shove_t + 0.010:
            d.xfrc_applied[torso, 0] = shove[1] / 0.010

        mujoco.mj_step(m, d)
        if viewer is not None:
            viewer(m, d, t)
        pitch_hist.append(pitch)

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

    # Steady-state oscillation, measured over the back half. A controller can
    # avoid falling and still limit-cycle violently; max_pitch alone does not
    # tell them apart, and tuning on max_pitch alone picks the limit cycle.
    tail = np.array(pitch_hist[len(pitch_hist) // 2:]) if pitch_hist else np.zeros(1)
    return {
        "fell": fell,
        "t_end": d.time,
        "max_pitch": max_pitch,
        "pitch_rms": float(np.sqrt((tail ** 2).mean())),
        "pitch_ptp": float(tail.max() - tail.min()) if len(tail) else 0.0,
        "drift": float(d.xpos[torso][0]),
        "recovery": recovered_at,
        "final_pitch": pitch,
        # Yaw over the back half: how well the rate was tracked, and how much
        # it thrashed doing it. The RMS is the one that catches a limit cycle,
        # because a symmetric oscillation averages to the right answer.
        "yaw_err": _yaw_err(yaw_hist),
        "yaw_rms": _yaw_rms(yaw_hist),
    }


def rollout_deploy(cfg=None, gains=None, trigger=2.0, duration=20.0, viewer=None,
                   backlash=0.0):
    """Balance, deploy the feet, then stand there. Returns metrics + state log."""
    from .transition import DeployMachine, STAND, NAMES

    m, d = load(backlash=backlash)
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

    # In foot mode the axle sits at the wheel's half-width, not its radius.
    axle_z = float(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                            "wheel_l")][2])
    wheel_clear = axle_z - WHEEL_HALF_W

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


def rollout_cycle(cfg=None, gains=None, flip_at=4.0, stand_for=4.0,
                  duration=26.0, v_des=0.35, viewer=None, backlash=0.0):
    """Drive, flip to feet, stand, flip back, drive on. Returns metrics.

    This is the round trip: the exit criterion for stage 0b is not just
    reaching foot mode but getting back out of it and still being useful.
    """
    from .transition import DeployMachine, STAND, WHEEL, NAMES

    m, d = load(backlash=backlash)
    mach = DeployMachine(gains, cfg)
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep

    fired = stood_at = retracted = None
    fell = False
    x_flip = x_back = x_stand0 = x_stand1 = None
    pitch = 0.0

    for k in range(int(duration / m.opt.timestep)):
        t = k * m.opt.timestep

        if fired is None and t >= flip_at:
            mach.start_deploy()
            fired = t
            x_flip = float(d.xpos[torso][0])
        if (stood_at is not None and retracted is None
                and t >= stood_at + stand_for):
            mach.start_retract()
            retracted = t
        if mach.state == STAND and stood_at is None:
            stood_at = t
            x_stand0 = float(d.xpos[torso][0])
        if retracted is not None and x_stand1 is None:
            x_stand1 = float(d.xpos[torso][0])
        if retracted is not None and mach.state == WHEEL and x_back is None:
            x_back = t

        drive = v_des if (fired is None or x_back is not None) else 0.0
        if k % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt, v_des=drive)

        mujoco.mj_step(m, d)
        if viewer is not None:
            viewer(m, d, t)

        pitch = pitch_from_quat(obs(m, d)["quat"])
        if d.xpos[torso][2] < 0.12 or abs(pitch) > 1.0:
            fell = True
            break

    return {
        "fell": fell,
        "state": NAMES[mach.state],
        "reached_stand": stood_at is not None,
        "back_on_wheels": x_back is not None,
        "x_at_flip": round(x_flip, 3) if x_flip is not None else None,
        "x_end": round(float(d.xpos[torso][0]), 3),
        # How far it wandered while it was supposed to be standing still.
        "slip_while_standing": (round(x_stand1 - x_stand0, 4)
                                if x_stand0 is not None and x_stand1 is not None
                                else None),
        "stand_seconds": (round(retracted - stood_at, 2)
                          if stood_at is not None and retracted is not None
                          else None),
        "final_pitch": round(pitch, 4),
        "log": mach.log,
    }


def cost(gains, duration=8.0, backlash=0.0):
    """Scalar score for tuning. Lower is better; falling is heavily penalised."""
    total = 0.0
    # Shoves at the RATED 1.0 N.s as well as a gentle 0.35. Tuning against only
    # a disturbance smaller than the one the robot is specified to survive
    # returns gains that pass the search and fail the tests.
    #
    # `want` is how far the robot is SUPPOSED to travel. It is not always zero,
    # and treating it as though it were is a real trap: penalising raw drift on
    # the drive trial scores a robot that refuses to move as perfect, and that
    # is exactly what one round of this produced - a set of gains that held
    # position beautifully and tracked 3 cm of a 75 cm velocity command.
    trials = [
        dict(duration=duration, shove=None, want=0.0),
        dict(duration=duration, shove=(2.0, 0.35), want=0.0),
        dict(duration=duration, shove=(2.0, -0.35), want=0.0),
        dict(duration=duration, shove=(2.0, 1.0), want=0.0),
        dict(duration=duration, shove=(2.0, -1.0), want=0.0),
        dict(duration=duration, want=0.75,
             v_des=lambda t: 0.25 if 2.0 < t < 5.0 else 0.0),
        dict(duration=duration, want=None, yaw_des=0.8),
        dict(duration=duration, want=None, yaw_des=-1.2),
    ]
    for kw in trials:
        want = kw.pop("want")
        r = rollout(gains, backlash=backlash, **kw)
        if r["fell"]:
            total += 100.0 + 10.0 * (duration - r["t_end"])
            continue
        # Oscillation dominates: a robot that wobbles +/-16 deg forever is not
        # balancing, even though it never falls.
        # Position error at 40, not 2. The acceptance criterion is 50 mm of
        # drift over a minute; at a weight of 2 that is worth 0.1 of cost
        # against pitch terms worth tens, so the search happily trades all of
        # it away for a slightly smoother pitch trace - which it did, returning
        # gains that stood beautifully while wandering off the desk.
        if want is not None:
            total += abs(r["drift"] - want) * 40.0
        total += (r["max_pitch"] * 3.0
                  + r["pitch_rms"] * 60.0 + r["pitch_ptp"] * 30.0
                  # Yaw RMS, not mean: a symmetric limit cycle averages to the
                  # commanded rate and looks like perfect tracking.
                  + r["yaw_rms"] * 12.0)
        if kw.get("shove"):
            total += (r["recovery"] if r["recovery"] is not None else 3.0)
    return total
