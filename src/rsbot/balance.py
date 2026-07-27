"""Pitch balancer for the wheeled-biped mode.

Cascade, because the wheels are velocity-source (STS3215 continuous rotation
takes a speed command, not a torque):

    outer  position + velocity error  ->  pitch target
    inner  pitch error                ->  wheel speed command

Everything it reads is available on the real robot: IMU quaternion, IMU gyro,
and wheel joint velocity. No cheating on base velocity from the simulator.
"""

import math
from dataclasses import dataclass

import numpy as np

from .model import (ROLL_WHEEL, WHEEL_R, ankle_pitch_level, leg_ik,
                    make_ctrl)


@dataclass
class Gains:
    # Tuned against 0 AND 1 deg of gear lash at the CAD-derived mass of 1.863 kg,
    # with the CAD-derived INERTIA TENSORS rather than the primitive capsules.
    # Gains depend on the mass distribution, not just the mass: correcting the
    # distribution alone - same total, same centre of mass to within a few mm -
    # broke shove rejection, steering and the whole transition, and needed this
    # re-tune. The ankle was the worst of them, with its centre of mass 75 mm
    # from where the stand-in box put it. See docs/backlash.md.
    # inner loop: pitch -> wheel speed
    kp: float = 10.714
    kd: float = 1.058
    # outer loop: odometry -> pitch target
    kv: float = 0.35
    kx: float = 0.4
    pitch_max: float = 0.309  # rad, cap on commanded lean
    tau_odom: float = 0.11  # s, low-pass on wheel-derived speed
    # Low-pass on the wheel command itself. With gear lash, slamming the
    # command across the deadzone is what drives the limit cycle.
    tau_cmd: float = 0.112
    # Yaw: closed on the IMU's z gyro, differenced across the two wheels.
    kyaw: float = 0.486
    yaw_max: float = 1.2     # rad/s; 1.5 and up tips it over


# Kept as a name for the tuned-for-lash set, which is now simply the default:
# one gain set handles rigid and lashed alike.
LASH_GAINS = Gains()


def pitch_from_quat(q):
    """Pitch about +y from a wxyz quaternion. Positive = leaning forward."""
    w, x, y, z = q
    s = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, s)))


class Balancer:
    def __init__(self, gains=None, height=0.207):
        self.g = gains or Gains()
        self.height = height
        self.v_filt = 0.0
        self.x = 0.0
        self.x_ref = 0.0
        self.w_cmd = 0.0
        self.outer_enabled = True

    def reset(self):
        self.v_filt = 0.0
        self.x = 0.0
        self.x_ref = 0.0
        self.w_cmd = 0.0
        self.outer_enabled = True

    def __call__(self, obs, dt, v_des=0.0, yaw_des=0.0, pitch_bias=0.0):
        """obs: dict with quat(4), gyro(3), wheel_vel(2). Returns ctrl(8).

        `yaw_des` is a turn rate in rad/s, positive to the left. There is no
        steering joint: it becomes a speed difference between the wheels,
        closed on the gyro so it holds a rate rather than a wheel offset.

        `pitch_bias` offsets the commanded lean. The transition uses it to rock
        the robot back onto its pads: the inner loop drives the wheels forward
        until the body reaches the biased lean, and past that the CoM is behind
        the axle and the pads catch the fall.
        """
        g = self.g
        pitch = pitch_from_quat(obs["quat"])
        pitch_rate = obs["gyro"][1]

        # Ground speed from wheel odometry. The wheel joint is measured
        # relative to the shin, so add the body pitch rate back in.
        w = float(np.mean(obs["wheel_vel"])) + pitch_rate
        v = WHEEL_R * w
        a = dt / (g.tau_odom + dt)
        self.v_filt += a * (v - self.v_filt)
        self.x += self.v_filt * dt

        # Driving is a moving setpoint, so the position term never fights it.
        self.x_ref += v_des * dt

        # Outer: to go forward, lean forward. Disabled once the pads carry
        # load, because unloaded wheels free-spin and the odometry derived from
        # them is meaningless -- it reads over 1 m/s of motion that isn't there.
        if self.outer_enabled:
            pitch_des = g.kv * (v_des - self.v_filt) + g.kx * (self.x_ref - self.x)
        else:
            pitch_des = 0.0
        pitch_des = max(-g.pitch_max, min(g.pitch_max, pitch_des)) + pitch_bias

        # Inner: catch the lean by driving the wheels under the CoM.
        w_cmd = (g.kp * (pitch - pitch_des) + g.kd * pitch_rate) / WHEEL_R
        if g.tau_cmd > 0:
            a = dt / (g.tau_cmd + dt)
            self.w_cmd += a * (w_cmd - self.w_cmd)
            w_cmd = self.w_cmd

        # Steering. Positive yaw is to the left, so the right wheel runs
        # faster. Gated by the caller in foot mode, where a flat wheel would
        # just scrub against the floor.
        yaw_des = max(-g.yaw_max, min(g.yaw_max, yaw_des))
        dw = g.kyaw * (yaw_des - obs["gyro"][2]) / WHEEL_R

        hip, knee = leg_ik(self.height)
        # Wheels upright, foot face held level so it is ready to plant.
        return make_ctrl(hip, knee, ankle_pitch_level(hip, knee),
                         ROLL_WHEEL, w_cmd - dw, w_cmd + dw)
