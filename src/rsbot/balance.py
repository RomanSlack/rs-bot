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
    # Tuned by tune.py against the stage-0 cost. See docs/stage-0.md.
    # inner loop: pitch -> wheel speed
    kp: float = 48.0
    kd: float = 1.719
    # outer loop: odometry -> pitch target
    kv: float = 0.250
    kx: float = 0.859
    pitch_max: float = 0.30  # rad, cap on commanded lean
    tau_odom: float = 0.05   # s, low-pass on wheel-derived speed


# Gains re-tuned against 1 deg of gear lash. The rigid-model defaults above
# survive NO backlash at all, so this is the set to start from on hardware.
# It is mostly the outer loop that has to change. See docs/backlash.md.
LASH_GAINS = Gains(kp=48.0, kd=1.719, kv=0.640, kx=0.537, tau_odom=0.0312)


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
        self.outer_enabled = True

    def reset(self):
        self.v_filt = 0.0
        self.x = 0.0
        self.x_ref = 0.0
        self.outer_enabled = True

    def __call__(self, obs, dt, v_des=0.0, pitch_bias=0.0):
        """obs: dict with quat(4), gyro(3), wheel_vel(2). Returns ctrl(8).

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

        hip, knee = leg_ik(self.height)
        # Wheels upright, foot face held level so it is ready to plant.
        return make_ctrl(hip, knee, ankle_pitch_level(hip, knee),
                         ROLL_WHEEL, w_cmd)
