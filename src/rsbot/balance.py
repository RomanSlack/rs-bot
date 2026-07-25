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

from .model import WHEEL_R, leg_ik


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

    def reset(self):
        self.v_filt = 0.0
        self.x = 0.0
        self.x_ref = 0.0

    def __call__(self, obs, dt, v_des=0.0):
        """obs: dict with quat(4), gyro(3), wheel_vel(2). Returns ctrl(8)."""
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

        # Outer: to go forward, lean forward.
        pitch_des = g.kv * (v_des - self.v_filt) + g.kx * (self.x_ref - self.x)
        pitch_des = max(-g.pitch_max, min(g.pitch_max, pitch_des))

        # Inner: catch the lean by driving the wheels under the CoM.
        w_cmd = (g.kp * (pitch - pitch_des) + g.kd * pitch_rate) / WHEEL_R

        hip, knee = leg_ik(self.height)
        ankle = 2.27  # retracted; feet are stage 2
        return np.array([hip, knee, ankle, w_cmd,
                         hip, knee, ankle, w_cmd])
