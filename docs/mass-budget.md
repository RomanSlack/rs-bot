# Mass budget

Target is the **final** robot, arms and all. The sim model carries the
not-yet-built parts as torso ballast so the legs are sized and the balancer is
tuned against real inertia from day one. On the stage-1 hardware build this is
literal: a ~600 g block where the arms and head will go.

**These are now derived, not estimated.** Run `uv run python -m cad.masses`.
Each body is its printed structure (volume x PETG x infill) plus the servos
whose *cases* bolt to it plus the bought parts it carries. The shin uses its
real CAD mass from `cad/shin.py`.

| Item | Count | Each | Total |
|---|---|---|---|
| Leg servos, STS3215 | 10 | 55 g | 550 g |
| Printed structure (PETG @ 60% infill) | | | 165 g |
| 3S pack | 1 | 180 g | 180 g |
| Wheels | 2 | 60 g | 120 g |
| Pi 5 | 1 | 45 g | 45 g |
| Bus adapter + IMU | | | 15 g |
| Arm/head ballast (stage 4) | | | 600 g |
| **Total** | | | **1713 g** |

The printed structure came in at **165 g against a 400 g estimate**, which is
where most of the 337 g went. Estimating structure by eye overshoots badly.

## How that maps onto the sim model

A servo case bolts to the body **proximal** to the joint it drives, which is
what fixes the old error of leaving 100 g in the shin for a wheel servo that
actually lives on the roll bracket, 110 mm further out.

| Body | Mass | Printed | Carries |
|---|---|---|---|
| torso | 1069 g | 119 g | 2 hip servos, Pi, pack, adapter, IMU, 600 g ballast |
| thigh x2 | 72.9 g | 17.9 g | knee servo |
| shin x2 | 72.3 g | **17.3 g (CAD)** | ankle-pitch servo |
| ankle x2 | 56.8 g | 1.8 g | ankle-roll servo |
| rollbracket x2 | 59.8 g | 4.8 g | wheel servo |
| wheel x2 | 60 g | - | tyre and hub, bought |
| **Total** | **1713 g** | | |

## Torso fore/aft trim

The torso mass is offset fore/aft so the standing CoM lands over the wheel
axle. It is **solved for at load time** rather than hardcoded (`model.load()`):
build once, measure the offset, shift the torso, which is exact in one pass
because the relationship is linear.

This is not cosmetic. Any residual offset becomes a permanent standing lean
that eats lean authority. When the feet were separate parts that parked behind
the wheel, the untrimmed CoM sat 5.7 mm aft and forced a 1.2 deg lean; nulling
it cut the residual standing lean from 2.1 to 0.8 deg. Solving for it means
changing the leg can no longer silently reintroduce the error.

On the real build, this is where the battery goes. Expect to trim it again on
hardware once the arms are on, since they move the CoM forward.

### What the correction cost

Losing 337 g made the robot twitchier, and the gains are mass-dependent: the
old set fell over at 2 deg of lash once the robot got lighter. Re-tuned, and
shove rejection settles at 1.0 N.s rather than 1.4 - the same impulse is a
larger velocity change on a lighter robot.

**The +120 g is the cost of the wheel-flip foot.** It buys an ankle roll joint
per leg, and that joint is what turns the wheel into an 80 mm foot centred
under the axle. It replaces every separate foot part: no soles, no struts,
no pads.
