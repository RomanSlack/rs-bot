# Stage 0: sim balancer

**Exit criterion: rejects a shove, attitude recovery under 1 s. Met.**

## The robot

43 cm to the top of the torso, 24.7 cm hip height in nominal stance, 1.87 kg.
Two legs, each hip / knee / wheel / ankle, all pitch axes. 8 DOF.
Wheels are 80 mm, hips are 120 mm apart laterally.

Nominal stance is hip +0.35, knee -0.70. The two-link leg is driven
symmetrically (`knee = -2 * hip`) so the wheel stays directly under the hip and
leg length is the only free variable. That keeps squatting from coupling into
pitch, which matters a lot at stage 0b.

## The controller

The wheels are velocity-source. STS3215 servos in continuous-rotation mode take
a speed command and close their own loop internally; you do not get torque.
That rules out the textbook LQR-on-a-wheeled-inverted-pendulum formulation,
which assumes wheel torque is the input. So it's a cascade instead:

```
outer   position + velocity error (wheel odometry)  ->  pitch target
inner   pitch error (IMU)                           ->  wheel speed command
```

Everything the controller reads exists on the real robot: IMU quaternion, IMU
gyro, wheel joint velocity. Base velocity is reconstructed from wheel odometry
as `v = R * (omega_wheel + pitch_rate)` (the wheel encoder is relative to the
shin, so the body pitch rate has to be added back) and low-passed at 50 ms.
Nothing reads ground truth out of the simulator, so the controller ports to
hardware as-is.

Physics runs at 1 kHz, control at 200 Hz.

Gains came from `tune.py`, a coordinate-descent search over a cost combining
quiet standing, a forward shove, a backward shove and a drive command:

```
kp = 48.0    kd = 1.719    kv = 0.250    kx = 0.859    pitch_max = 0.30
```

## Measured

| Test | Result |
|---|---|
| Stand 5 min unattended | no fall, 18 mm total drift, 1.3 deg peak pitch |
| Drive 0.25 m/s for 6 s | 1.47 m travelled (1.50 m commanded) |
| Drive 0.60 m/s for 6 s | 3.56 m travelled (3.60 m commanded) |
| Shove 0.35 N.s | 6.3 deg peak lean, attitude recovery 0.65 s |
| Shove 0.70 N.s | 13.2 deg peak lean, attitude recovery 0.70 s |
| Shove 1.40 N.s | 18.3 deg peak lean, recovers |
| Shove 1.80 N.s | **falls** |

Fall threshold is between 1.4 and 1.8 N.s, i.e. an impulse that would instantly
give the robot roughly 0.8-0.95 m/s. That is a hard flick with a finger, and
it's a reasonable place to be for a first controller.

The binding constraint at the limit is the 0.30 rad cap on commanded lean, not
servo torque or wheel speed. Raising it trades disturbance rejection for
stability margin, so leave it until there's hardware to argue with.

Full-recovery-to-origin (attitude *and* position back to zero) takes 1.5-2.5 s.
That's the outer loop walking the robot home and is not what the < 1 s criterion
is about. Two of the attitude-recovery numbers above look implausibly fast
(0.06 s at 1.4 N.s) because the detector triggers as the pitch swings back
through the threshold; trust the 0.35 and 0.70 N.s rows.

## Known soft spots

- **Residual 0.8 deg standing lean.** The controller is P-only in the outer
  loop, so it parks at whatever lean the position term demands after the
  startup transient. Stable, not drifting. An integral term would null it;
  not worth adding before hardware, where the real trim errors are bigger.
- **Roll and yaw are untested.** Both are open-loop here. Roll is passively
  stable from the 120 mm wheel separation, and yaw needs differential wheel
  speed that the controller doesn't command yet. Stage 1.
- **Contact model is a plane with friction 1.6.** No carpet, no USB cables, no
  compliance in the tyre. Desk-scale terrain is going to be worse than this.
- **The servo model is optimistic.** Position actuators with `kp=12, kv=0.35`
  and a hard 2.9 N.m clamp. Real STS3215s have 1-2 deg of backlash, a serial
  bus with real latency, and speed-dependent torque. None of that is modelled.
  Expect the hardware to be mushier.

## Next: stage 0b

The rocker soles are already in the model, retracted and held. What's missing
is the transition controller: squat, drive pitch error to zero, deploy, detect
ground contact, ramp wheel speed to zero. And the reverse.
