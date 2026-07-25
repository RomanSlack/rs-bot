"""Wheel mode <-> foot mode by rolling the wheels flat.

The wheel IS the foot. A 90 deg ankle roll stands the spin axis vertical, the
disc lies down, and its 80 mm face becomes the sole. That gives the one thing
the two earlier mechanisms could not: a support polygon CENTRED under the axle,
where the CoM already is. No fore/aft margin to scrounge for.

    WHEEL   balancing normally
    SETTLE  wait for a quiet moment, judged against the balancer's trim lean
    FLIP    roll both wheels to 90 deg. The disc's low point falls from 40 mm
            to 12 mm (after a 1.76 mm bump at 16.7 deg), so the robot simply
            settles 28 mm as it goes. Contact is never broken.
    STAND   flat on both faces, wheels braked, balancer off
    UNFLIP  roll back to upright and hand control back to the balancer

Wheel drive authority falls off as cos(roll), so the balancer is fading out
exactly as the polygon appears. That handover is what flip_rate trades against,
and UNFLIP runs it in reverse: authority grows back as the polygon shrinks.
"""

from dataclasses import dataclass

import numpy as np

from .balance import Balancer, Gains, pitch_from_quat
from .model import (ROLL_FOOT, ROLL_WHEEL, ankle_pitch_level, leg_ik,
                    make_ctrl)

WHEEL, SETTLE, FLIP, STAND, UNFLIP = range(5)
NAMES = ["WHEEL", "SETTLE", "FLIP", "STAND", "UNFLIP"]


@dataclass
class DeployCfg:
    flip_rate: float = 2.0        # rad/s of ankle roll
    stand_height: float = 0.195   # leg length held through the flip
    ankle_bias: float = 0.0       # deliberate foot tilt in foot mode
    settle_pitch: float = 0.015   # rad, measured against the trim lean
    settle_rate: float = 0.15     # rad/s
    settle_speed: float = 0.03    # m/s
    settle_timeout: float = 4.0
    trim_tau: float = 0.5
    unload_time: float = 0.20     # s to ramp the wheel command out
    reload_time: float = 0.30     # s to ramp it back in on the way up


class DeployMachine:
    def __init__(self, gains: Gains = None, cfg: DeployCfg = None,
                 cruise_height: float = 0.207):
        self.cfg = cfg or DeployCfg()
        self.bal = Balancer(gains, height=cruise_height)
        self.state = WHEEL
        self.t = 0.0
        self.t_state = 0.0
        self.roll = ROLL_WHEEL
        self.unload = 1.0
        self.trim = 0.0
        self.cruise_height = cruise_height
        self.timed_out = False
        self.log = []

    def _go(self, s):
        self.log.append((round(self.t, 3), NAMES[self.state], NAMES[s]))
        self.state = s
        self.t_state = 0.0

    def start_deploy(self):
        if self.state == WHEEL:
            self._go(SETTLE)

    def start_retract(self):
        if self.state == STAND:
            # The balancer has been idle and its odometry is stale: x still
            # holds wherever the robot was when it stopped balancing. Clearing
            # it stops the position term yanking the robot back there.
            self.bal.reset()
            self.bal.outer_enabled = False
            self.bal.height = self.cfg.stand_height
            self._go(UNFLIP)

    def toggle(self):
        if self.state == WHEEL:
            self.start_deploy()
        elif self.state == STAND:
            self.start_retract()

    def _ramp(self, value, target, rate, dt):
        return value + np.clip(target - value, -rate * dt, rate * dt)

    def __call__(self, obs, dt, v_des=0.0):
        c = self.cfg
        self.t += dt
        self.t_state += dt
        if self.state != WHEEL:
            v_des = 0.0

        pitch = pitch_from_quat(obs["quat"])
        rate = obs["gyro"][1]

        # The balancer parks at a non-zero lean that moves with squat depth, so
        # "quiet" is judged against that lean rather than against zero.
        if self.state in (WHEEL, SETTLE):
            a = dt / (c.trim_tau + dt)
            self.trim += a * (pitch - self.trim)

        if self.state == SETTLE:
            quiet = (abs(pitch - self.trim) < c.settle_pitch
                     and abs(rate) < c.settle_rate
                     and abs(self.bal.v_filt) < c.settle_speed)
            if quiet or self.t_state > c.settle_timeout:
                self.timed_out = self.t_state > c.settle_timeout
                self.bal.height = c.stand_height
                self._go(FLIP)

        elif self.state == FLIP:
            self.roll = self._ramp(self.roll, ROLL_FOOT, c.flip_rate, dt)
            # Odometry assumes an upright wheel; past halfway it is nonsense,
            # so drop the outer loop and hold the trim lean on pitch alone.
            if self.roll > ROLL_FOOT * 0.5:
                self.bal.outer_enabled = False
            if self.roll >= ROLL_FOOT - 1e-4:
                self._go(STAND)

        elif self.state == STAND:
            self.unload = max(0.0, self.unload - dt / c.unload_time)

        elif self.state == UNFLIP:
            self.roll = self._ramp(self.roll, ROLL_WHEEL, c.flip_rate, dt)
            # Feed the wheels back in as the disc comes upright, since a
            # near-flat disc just scrubs rather than driving.
            self.unload = min(1.0, self.unload + dt / c.reload_time)
            if self.roll < ROLL_FOOT * 0.5:
                self.bal.outer_enabled = True
            if self.roll <= ROLL_WHEEL + 1e-4:
                self._go(WHEEL)

        elif self.state == WHEEL:
            self.bal.height = self._ramp(self.bal.height, self.cruise_height,
                                         0.10, dt)

        hip, knee = leg_ik(self.bal.height)
        apitch = ankle_pitch_level(hip, knee, c.ankle_bias)

        if self.state == STAND:
            return make_ctrl(hip, knee, apitch, self.roll, 0.0)

        ctrl = self.bal(obs, dt, v_des=v_des,
                        pitch_bias=self.trim if not self.bal.outer_enabled else 0.0)
        ctrl[2] = ctrl[7] = apitch
        ctrl[3] = ctrl[8] = self.roll
        return ctrl
