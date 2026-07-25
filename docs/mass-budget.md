# Mass budget

Target is the **final** robot, arms and all. The sim model carries the
not-yet-built parts as torso ballast so the legs are sized and the balancer is
tuned against real inertia from day one. On the stage-1 hardware build this is
literal: a ~600 g block where the arms and head will go.

| Item | Count | Each | Total |
|---|---|---|---|
| Leg servos (hip, knee, ankle pitch, ankle **roll**, wheel x2) | 10 | 60 g | 600 g |
| Arm servos (shoulder pitch/roll, elbow, gripper x2) | 8 | 60 g | 480 g |
| Head servos (pan/tilt) | 2 | 60 g | 120 g |
| Printed structure (PETG) | | | 400 g |
| Pi 5 + cameras + IMU + harness | | | 250 g |
| 3S pack | | | 200 g |
| **Total** | | | **2050 g** |

## How that maps onto the sim model

| Body | Mass | Stands in for |
|---|---|---|
| torso | 1230 g | structure, Pi, battery, **plus arm/head ballast** |
| thigh x2 | 130 g | hip + knee servo, thigh shell |
| shin x2 | 100 g | wheel servo, shin shell |
| ankle x2 | 120 g | ankle pitch + ankle roll servos |
| wheel x2 | 60 g | wheel, tyre |
| **Total** | **2050 g** | |

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

**The +120 g is the cost of the wheel-flip foot.** It buys an ankle roll joint
per leg, and that joint is what turns the wheel into an 80 mm foot centred
under the axle. It replaces every separate foot part: no soles, no struts,
no pads.
