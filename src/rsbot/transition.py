"""Wheel mode <-> foot mode transition.

Deploy is a state machine over the balancer:

    WHEEL    balancing normally
    SQUAT    ramp leg length down, still balancing (lower CoM = less to shed)
    SETTLE   wait for pitch, pitch rate and forward speed to all be small
    DEPLOY   sweep the soles down at a fixed rate, still balancing
    LOAD     soles are carrying weight: ramp the wheel command to zero
    STAND    static. Wheels commanded to zero, balancer not running.

Contact is inferred from ankle servo position error, because an STS3215 has
no force sensing. When the sole starts carrying the robot, the servo can no
longer reach its commanded angle, and the tracking error is the signal. This
is deliberately the same information the real robot will have.
"""

from dataclasses import dataclass

import numpy as np

from .balance import Balancer, Gains, pitch_from_quat
from .model import RETRACT_SWEEP, leg_ik, sole_angle

WHEEL, SQUAT, SETTLE, DEPLOY, LOAD, STAND = range(6)
NAMES = ["WHEEL", "SQUAT", "SETTLE", "DEPLOY", "LOAD", "STAND"]


@dataclass
class DeployCfg:
    stand_height: float = 0.170   # leg length to transition at
    squat_rate: float = 0.10      # m/s of leg length
    deploy_rate: float = 1.5      # rad/s of sole world angle
    settle_pitch: float = 0.020   # rad
    settle_rate: float = 0.15     # rad/s
    settle_speed: float = 0.03    # m/s
    settle_timeout: float = 3.0   # s before giving up on a quiet moment
    contact_err: float = 0.06     # rad of ankle tracking error = loaded
    contact_hold: float = 0.05    # s that error must persist
    unload_time: float = 0.30     # s to ramp the wheel command out


class DeployMachine:
    def __init__(self, gains: Gains = None, cfg: DeployCfg = None,
                 cruise_height: float = 0.207):
        self.cfg = cfg or DeployCfg()
        self.bal = Balancer(gains, height=cruise_height)
        self.cruise_height = cruise_height
        self.state = WHEEL
        self.t = 0.0
        self.t_state = 0.0
        self.phi = RETRACT_SWEEP     # sole world angle
        self.contact_for = 0.0
        self.unload = 1.0            # wheel command scale
        self.log = []

    def _go(self, s):
        self.log.append((round(self.t, 3), NAMES[self.state], NAMES[s]))
        self.state = s
        self.t_state = 0.0

    def start_deploy(self):
        if self.state == WHEEL:
            self._go(SQUAT)

    def __call__(self, obs, dt):
        c = self.cfg
        self.t += dt
        self.t_state += dt

        pitch = pitch_from_quat(obs["quat"])
        rate = obs["gyro"][1]

        if self.state == SQUAT:
            target = c.stand_height
            step = c.squat_rate * dt
            self.bal.height += np.clip(target - self.bal.height, -step, step)
            if abs(self.bal.height - target) < 1e-4:
                self._go(SETTLE)

        elif self.state == SETTLE:
            quiet = (abs(pitch) < c.settle_pitch
                     and abs(rate) < c.settle_rate
                     and abs(self.bal.v_filt) < c.settle_speed)
            # Don't wait forever. A robot that never gets a quiet moment still
            # has to commit, and the timeout is a real failure mode to measure.
            if quiet or self.t_state > c.settle_timeout:
                self.timed_out = self.t_state > c.settle_timeout
                self._go(DEPLOY)

        elif self.state == DEPLOY:
            self.phi = max(0.0, self.phi - c.deploy_rate * dt)
            hip, knee = leg_ik(self.bal.height)
            err = np.max(np.abs(obs["ankle_pos"] - sole_angle(hip, knee, self.phi)))
            self.contact_for = self.contact_for + dt if err > c.contact_err else 0.0
            if self.contact_for >= c.contact_hold:
                self._go(LOAD)
            elif self.phi <= 0.0:
                # Fully swept with no load detected. Commit anyway rather than
                # hang: on hardware this is a miscalibrated ankle or a step.
                self._go(LOAD)

        elif self.state == LOAD:
            self.phi = max(0.0, self.phi - c.deploy_rate * dt)
            self.unload = max(0.0, self.unload - dt / c.unload_time)
            if self.unload <= 0.0 and self.phi <= 0.0:
                self._go(STAND)

        # --- command assembly -------------------------------------------------
        hip, knee = leg_ik(self.bal.height)

        if self.state == STAND:
            ankle = sole_angle(hip, knee, 0.0)
            return np.array([hip, knee, ankle, 0.0, hip, knee, ankle, 0.0])

        ctrl = self.bal(obs, dt, v_des=0.0)
        ctrl[2] = ctrl[6] = sole_angle(hip, knee, self.phi)
        ctrl[3] *= self.unload
        ctrl[7] *= self.unload
        return ctrl
