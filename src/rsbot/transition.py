"""Wheel mode <-> foot mode transition.

The ordering is the whole design, and it comes from one rule: the robot must
never be airborne. Wheel-biped transformation work calls this passing through a
"critical state" where wheel and foot are both in ground contact. The earlier
axle-pivoted flat sole violated it and toppled every time (docs/stage-0b.md).

    WHEEL    balancing normally
    EXTEND   stretch the legs to SWING_HEIGHT. This lifts the strut pivot so
             the pad can swing without touching the floor at all.
    SETTLE   wait for a quiet moment, measured against the balancer's own
             trim lean rather than against zero
    SWING    rotate the struts down to vertical. No ground contact, ~3.6 mm
             of clearance, balancer fully in charge throughout.
    PLANT    squat back to STAND_HEIGHT. The pads come down onto the floor
             while the wheels are still carrying. This is the critical state.
    SHIFT    drive the axles forward, which walks the torso back over the
             pads, until the CoM sits in the middle of the support polygon.
    LOAD     ramp the wheel command out
    STAND    static, balancer off

Contact is inferred from ankle servo position error, because an STS3215 has no
force sensing. That is deliberately the same information the real robot has.
"""

from dataclasses import dataclass

import numpy as np

from .balance import Balancer, Gains, pitch_from_quat
from .model import (DEPLOY_PHI, RETRACT_PHI, STAND_HEIGHT, SWING_HEIGHT,
                    foot_pose, leg_ik, sole_angle)

WHEEL, EXTEND, SETTLE, SWING, PLANT, SHIFT, ROCK, LOAD, STAND = range(9)
NAMES = ["WHEEL", "EXTEND", "SETTLE", "SWING", "PLANT", "SHIFT", "ROCK",
         "LOAD", "STAND"]


@dataclass
class DeployCfg:
    height_rate: float = 0.10     # m/s of leg length
    swing_rate: float = 2.0       # rad/s of strut world angle
    shift_rate: float = 0.08      # m/s of body shift
    body_shift: float = 0.057     # puts the CoM mid-polygon, see docs/stage-0b
    settle_pitch: float = 0.015   # rad, measured against the trim lean
    settle_rate: float = 0.15     # rad/s
    settle_speed: float = 0.03    # m/s
    settle_timeout: float = 4.0
    trim_tau: float = 0.5         # s, low-pass that learns the trim lean
    rock_lean: float = 0.16       # rad of backward lean to plant the pads
    contact_err: float = 0.05     # rad of ankle tracking error = pads loaded
    contact_hold: float = 0.04    # s that error must persist
    rock_timeout: float = 2.0
    unload_time: float = 0.40     # s to ramp the wheel command out


class DeployMachine:
    def __init__(self, gains: Gains = None, cfg: DeployCfg = None,
                 cruise_height: float = 0.207):
        self.cfg = cfg or DeployCfg()
        self.bal = Balancer(gains, height=cruise_height)
        self.state = WHEEL
        self.t = 0.0
        self.t_state = 0.0
        self.phi = RETRACT_PHI
        self.shift = 0.0
        self.unload = 1.0
        self.contact_for = 0.0
        self.rock_timed_out = False
        self.trim = 0.0            # learned steady-state lean
        self.timed_out = False
        self.log = []

    def _go(self, s):
        self.log.append((round(self.t, 3), NAMES[self.state], NAMES[s]))
        self.state = s
        self.t_state = 0.0
        # Must reset, or a previous state's tracking error (the swing ramp
        # produces plenty) counts as contact the moment the next state starts.
        self.contact_for = 0.0

    def start_deploy(self):
        if self.state == WHEEL:
            self._go(EXTEND)

    def _legs(self):
        """Hip/knee for the current phase. Foot mode holds the shin angle."""
        if self.state in (PLANT, SHIFT, ROCK, LOAD, STAND):
            return foot_pose(self.shift)
        return leg_ik(self.bal.height, 0.0)

    def _pads_loaded(self, obs, c, dt):
        """Ankle servo tracking error as a load proxy. No force sensor needed."""
        hip, knee = self._legs()
        err = np.max(np.abs(obs["ankle_pos"] - sole_angle(hip, knee, self.phi)))
        self.contact_for = self.contact_for + dt if err > c.contact_err else 0.0
        return self.contact_for >= c.contact_hold

    def _ramp(self, value, target, rate, dt):
        step = rate * dt
        return value + np.clip(target - value, -step, step)

    def __call__(self, obs, dt, v_des=0.0):
        c = self.cfg
        if self.state != WHEEL:
            v_des = 0.0
        self.t += dt
        self.t_state += dt

        pitch = pitch_from_quat(obs["quat"])
        rate = obs["gyro"][1]

        # The balancer parks at a non-zero lean that moves with squat depth, so
        # "quiet" has to be judged against that lean, not against zero. This is
        # why SETTLE used to time out instead of ever firing.
        if self.state in (WHEEL, EXTEND, SETTLE):
            a = dt / (c.trim_tau + dt)
            self.trim += a * (pitch - self.trim)

        if self.state == EXTEND:
            self.bal.height = self._ramp(self.bal.height, SWING_HEIGHT,
                                         c.height_rate, dt)
            if abs(self.bal.height - SWING_HEIGHT) < 1e-4:
                self._go(SETTLE)

        elif self.state == SETTLE:
            quiet = (abs(pitch - self.trim) < c.settle_pitch
                     and abs(rate) < c.settle_rate
                     and abs(self.bal.v_filt) < c.settle_speed)
            if quiet or self.t_state > c.settle_timeout:
                self.timed_out = self.t_state > c.settle_timeout
                self._go(SWING)

        elif self.state == SWING:
            self.phi = self._ramp(self.phi, DEPLOY_PHI, c.swing_rate, dt)
            if abs(self.phi - DEPLOY_PHI) < 1e-4:
                self._go(PLANT)

        elif self.state == PLANT:
            # Squat until the pads reach the floor. foot_pose(0) is exactly the
            # pose where pad and wheel are both on the ground: the critical
            # state. Going further would lift the wheels.
            self.bal.height = self._ramp(self.bal.height, STAND_HEIGHT,
                                         c.height_rate, dt)
            if self.bal.height <= STAND_HEIGHT + 1e-4:
                self.bal.outer_enabled = False
                self._go(LOAD)

        elif self.state == SHIFT:
            # Wheels are already braked and the pads are down, so this is
            # quasi-static: the torso walks back over the pads with nothing
            # driving. Doing it the other way round makes the balancer chase an
            # equilibrium that no longer exists and drive off across the room.
            self.shift = self._ramp(self.shift, c.body_shift, c.shift_rate, dt)
            if abs(self.shift - c.body_shift) < 1e-4:
                self._go(STAND)

        elif self.state == ROCK:
            if self._pads_loaded(obs, c, dt) or self.t_state > c.rock_timeout:
                self.rock_timed_out = self.t_state > c.rock_timeout
                self._go(LOAD)

        elif self.state == LOAD:
            self.unload = max(0.0, self.unload - dt / c.unload_time)
            if self.unload <= 0.0:
                self._go(SHIFT)

        # --- command assembly -------------------------------------------------
        hip, knee = self._legs()
        ankle = sole_angle(hip, knee, self.phi)

        if self.state in (SHIFT, STAND):
            return np.array([hip, knee, ankle, 0.0, hip, knee, ankle, 0.0])

        ctrl = self.bal(obs, dt, v_des=v_des)
        ctrl[0] = ctrl[4] = hip
        ctrl[1] = ctrl[5] = knee
        ctrl[2] = ctrl[6] = ankle
        ctrl[3] *= self.unload
        ctrl[7] *= self.unload
        return ctrl
