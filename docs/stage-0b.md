# Stage 0b: the transition

**Status: deploy does not work with a flat-plate sole. The destination works;
the journey does not.**

## What was built

`transition.py` is a state machine over the balancer:

    WHEEL -> SQUAT -> SETTLE -> DEPLOY -> LOAD -> STAND

Contact is inferred from ankle servo position error, not from simulator contact
forces, because an STS3215 has no force sensing. When the sole starts carrying
the robot the servo stops reaching its commanded angle, and that tracking error
is the signal. It works: first contact is detected within 0.13 rad of where
kinematics says the sole touches.

## The result

Every configuration falls. 60 physically valid combinations of sole
half-length (20-60 mm), face radius (47-65 mm) and deploy rate (1.5-6 rad/s):
**0 reached STAND.**

Controller-side fixes were tried first and none helped: unloading the wheels at
flat instead of at contact, deploy rates from 1.5 to 200 rad/s, and squatting to
140 mm before committing. At 60 rad/s the plate slams the floor and launches the
robot 135 mm into the air, so faster is actively worse.

## Why, precisely

A flat plate on a revolute pivot at the wheel axle has corners farther from that
pivot than its face: `corner_radius = hypot(face_radius, half_length)`. The face
radius has a hard floor, because the plate has to cradle the wheel without
intersecting it (`face_r >= wheel_r + 2 * thickness`, i.e. >= 46 mm). So the
smallest achievable "jack" - how far the plate lifts the robot above its final
resting height while sweeping through - is about 11 mm, and 21 mm for a sole
long enough to be worth having.

During that jack the wheels are off the ground and the balancer has no
authority at all. Worse, the contact is a single edge sitting ~53 mm *behind*
the axle, which applies a strong forward pitching moment. The robot pitches
through 57 degrees in 0.4 s. The pendulum time constant is 0.17 s and the
unavoidable sweep is ~1.6 rad, so there is no deploy rate that fits.

This is geometry, not tuning. Flat and constant-radius are incompatible.

## The part that does work

Started already in foot mode - soles deployed, wheels off the ground, balancer
completely off - the robot stands indefinitely and tolerates real tilt:

| sole half-length | polygon | survives initial tilt |
|---|---|---|
| 60 mm | 120 mm | 8.0 deg, stood 8 s |
| 40 mm | 80 mm | 8.0 deg, stood 8 s |
| 30 mm | 60 mm | 5.7 deg (fails at 8.0) |

It settles at about 3.4 degrees of forward lean, resting on the toe. So foot
mode itself is sound, and the tilt tolerance at touchdown is ~8 degrees. The
whole problem is arriving there with less than 8 degrees of pitch error.

## Two bugs found on the way

- **Invalid geometry looked like a control failure.** Face radii of 41.5 and
  44 mm put the plate *inside* the wheel, and MuJoCo generated sole-wheel
  contacts that threw the robot during SETTLE. `model.load()` now refuses to
  build a sole that intersects the wheel.
- **SETTLE never fires; it times out.** The quiet test compares pitch against
  zero, but the balancer's steady-state lean is not zero and moves with squat
  depth. So the machine commits to deploying with ~2.5 deg of pitch error
  instead of ~0. That error is the initial condition for the whole airborne
  phase, so it matters a lot. Not yet fixed.

## Where it goes next

The fix has to make the contact point stay under the axle during the rise, so
there is no driving moment and the robot only drifts. Drifting from a
well-settled 0.6 deg over a 0.4 s rise lands around 3 deg, inside the 8 deg
polygon. Options are in the handoff notes; the leading candidate is a spiral
cam sole whose contact radius grows smoothly from the wheel radius.
