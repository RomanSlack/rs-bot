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
| Printed structure (all seven parts) | | | 341 g |
| Arm/head ballast + IMU (stage 4) | | | 605 g |
| 3S pack | 1 | 180 g | 180 g |
| Wheels, printed body + TPU tyre + horn | 2 | 54 g | 108 g |
| Belt drive: pulleys, belt, shafts | 2 | 34 g | 68 g |
| Pi 5 | 1 | 45 g | 45 g |
| Bus adapter | | | 10 g |
| **Total** | | | **1907 g** |

The printed structure came in at **341 g against a 400 g estimate**, which is
much closer than the 165 g this page claimed until 2026-08-02. That 165 was the
legs only, quoted as if it were the robot: the chassis alone is 135 g and was
never in the figure. Estimating structure by eye overshoots, but not by the
two-to-one this page was advertising.

## How that maps onto the sim model

A servo case bolts to the body **proximal** to the joint it drives, which is
what fixes the old error of leaving 100 g in the shin for a wheel servo that
actually lives on the roll bracket, 110 mm further out.

| Body | Mass | Printed | Carries |
|---|---|---|---|
| torso | 1085.1 g | 135.1 g | 2 hip servos, Pi, pack, adapter, IMU, 600 g ballast |
| thigh x2 | 96.3 g | 41.3 g | knee servo |
| shin x2 | 128.9 g | 39.9 g | ankle-pitch servo, belt drive |
| ankle x2 | 69.8 g | 14.8 g | ankle-roll servo |
| rollbracket x2 | 62.0 g | 7.0 g | wheel servo |
| wheel x2 | 54.0 g | - | body, tyre and horn |
| **Total** | **1907.1 g** | **341.1 g** | |

> **The sim does not agree with this table.** It weighs 1874.8 g, because
> `SEG_MASS` in `src/rsbot/model.py` is a hand-copied snapshot that nobody
> re-ran after the servo cradles landed. `cad.twin` fails on it per body. The
> table above is the CAD's, which is what gets built; the gap is open on
> purpose because closing it changes what the balancer was tuned against.

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
