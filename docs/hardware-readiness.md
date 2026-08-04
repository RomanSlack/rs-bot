# Hardware readiness

Two questions decide what to buy. Both now have answers from the sim.

## 1. Are STS3215s enough? Yes.

Stall is 2.9 N.m; safe continuous is roughly 0.6-0.9 N.m. What matters is RMS,
not peak, because peaks are what stall torque exists for.

| joint | RMS (worst case) | peak | % time > 0.9 N.m |
|---|---|---|---|
| hip | 0.08 | 1.68 | 0.0% |
| knee | **0.36** | 2.90 | 0.4% |
| ankle pitch | 0.07 | 1.63 | 0.1% | (no servo: a linkage holds it) |
| ankle roll | 0.18 | 0.92 | 0.0% |
| wheel | 0.38 | 2.90 | 2.6% |

Worst case is the flip cycle plus driving. The knee is the hardest-working
joint at 0.36 N.m RMS, comfortably inside continuous. Knee and wheel do touch
the 2.9 N.m clamp, but for 0.1-1.8% of the time - transient, which is fine.

**No actuator upgrade needed.** The design fits the cheap servo class.

## 2. How fast does the control loop need to run? At least 50 Hz.

This one shapes the electronics, because 8 servos on one TTL bus will not give
you 200 Hz if you read every joint back: a sync-write of 8 positions is quick,
but 8 individual position reads are 8 round trips. It was 10 until the
ankle-pitch servos were deleted (`docs/deleting-the-ankle-pitch-servo.md`),
which takes two round trips off the worst case and does not change the
conclusion below.

| control rate | balancing | flip round trip |
|---|---|---|
| 200 Hz | 1.2 deg pitch, 17 mm drift | ok |
| 100 Hz | 1.3 deg pitch, 17 mm drift | ok |
| 66 Hz | 7.3 deg pitch, 235 mm drift | ok |
| 50 Hz | 7.6 deg pitch, 289 mm drift | ok |
| 33 Hz | 13.8 deg pitch | ok, marginal |
| 25 Hz | 18.8 deg pitch | **falls** |

100 Hz is comfortable, 50 Hz is usable but visibly worse, 25 Hz is dead.

**The architecture that follows:** the balancer only needs the IMU (local,
fast, I2C/SPI) and the two wheel velocities. It does not need to read the leg
joints back at loop rate. So sync-write all 8 commands, read only the 2 wheel
servos, and run at 100 Hz. Reading every joint every cycle is what would drop
you into the 33 Hz danger zone.

## What sim still cannot tell you

Listed so nobody mistakes a green test suite for a working robot:

- ~~Backlash~~ **now modelled** - see [backlash.md](backlash.md). Short version:
  wheel mode survives 1-2 deg after re-tuning but wobbles badly, and foot mode
  falls over while standing. Lash is a purchasing spec, not a detail.
- **Serial bus latency and jitter.** Modelled only as a control rate, not as
  variable delay.
- **Speed-dependent servo torque.** The model is a flat 2.9 N.m clamp; real
  servos lose torque as they speed up.
- **Terrain.** A perfect friction plane. No carpet, no USB cable, no lip
  between floorboards. At 80 mm wheels these are all obstacles.
- **Yaw and steering.** Never tested. The controller has no differential drive
  and the robot has never turned.
- **Roll disturbances.** Passively stable by wheel separation, never tested.
- **Foot-mode disturbance rejection.** Passive only; tips at 0.35-0.45 N.s.
  The ankle strategy that would fix it is unbuilt.
