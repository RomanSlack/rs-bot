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
| Leg servos, STS3215 | 8 | 55 g | 440 g |
| Printed structure (chassis, thighs, shins, yokes, brackets) | | | 325 g |
| The parallelogram, printed: rod, idler, rod | 2 | 17 g | 34 g |
| Arm/head ballast + IMU (stage 4) | | | 605 g |
| 3S pack | 1 | 180 g | 180 g |
| Wheels, printed body + TPU tyre + horn | 2 | 55 g | 110 g |
| Pi 5 | 1 | 45 g | 45 g |
| Bus adapter | | | 10 g |
| **Total** | | | **1749 g** |

**1907 to 1800 g on 2026-08-03**, and every gram of it is the ankle-pitch
servos and their belt drive coming out: 2 x 55 g of servo and 2 x 34 g of
pulleys, belt and shafts, against 2 x 35 g of linkage going in. See
`docs/deleting-the-ankle-pitch-servo.md`.

The parallelogram row said **2 x 35 g** until 2026-08-04 and the rows did not
sum to the total, which nobody noticed because the total was written out
separately. 35 g a leg was right when the fin, the stub and the arm were loose
solids of their own; they are features of the chassis, the shin and the yoke
now, so they are inside the row above and the rods and idler alone are 17 g.

**Then 1800 to 1770 g on 2026-08-04**, which is the shin giving up the MOUNT
for the servo that had already gone: a standoff, two cradle walls, an end wall
and a clearance pocket, 53.05 cm3 down to 45.11, about 9 g a leg. It sat there
for a day with a note on it saying it was dead. Nothing failed while it did,
because dead material is conservative in every check this repo runs - which is
exactly why nothing was going to remove it on its own.

**Then 1770 to 1771 g on 2026-08-05**, the only ADDITION in the run: a seating
face on each of the four servos that were held by nothing - an end wall under
the two hip cases, a datum wall on the two roll cradles. About 1 g total, and it
buys the load path a face to bear on instead of 0.4 mm of air.

**Then 1771 to 1749 g the same day**, and no part moved: the printed structure
had been weighed at PETG's 1.270 g/cm3, the density it was COSTED in, while it
is ordered in PA6-CF at 1.19 (docs/materials.md). That density was in five files
and is now in one, cad/material.py, imported by the CAD and the sim alike. The
22 g is the correction. The balancer was re-checked against it, not assumed:
still stands 60 s and rejects a 1.0 N.s shove (tests/test_stage0.py).

That 107 g is not just lighter, it is lighter IN THE RIGHT PLACE. It came off
the far end of the leg, and foot mode now survives 4 degrees of gear lash where
the same linkage on the old mass failed at 4 ten times out of ten.

The printed structure came in at **380 g against a 400 g estimate**, which is
much closer than the 165 g this page claimed until 2026-08-02. That 165 was the
legs only, quoted as if it were the robot: the chassis alone is 135 g and was
never in the figure. Estimating structure by eye overshoots, but not by the
two-to-one this page was advertising.

> **Still quoted at PETG's 1.27 g/cm3, and these parts are ordered in PA6-CF at
> 1.19.** About 26 g the model carries that the robot will not. Not fixed,
> because the material is not settled: `road-to-order.md` item 8 put three PA12
> variants back in play when the roll bracket went from 99% to 55%.

## How that maps onto the sim model

A servo case bolts to the body **proximal** to the joint it drives, which is
what fixes the old error of leaving 100 g in the shin for a wheel servo that
actually lives on the roll bracket, 110 mm further out.

| Body | Mass | Printed | Carries |
|---|---|---|---|
| torso | 1109.9 g | 135.1 g | 2 hip servos, Pi, pack, adapter, IMU, 600 g ballast, 2 linkage fins |
| thigh x2 | 102.9 g | 41.1 g | knee servo, linkage rod 1 |
| shin x2 | 54.1 g | 40.4 g | linkage rod 2, idler and stub. NO SERVO |
| ankle x2 | 70.8 g | 13.7 g | ankle-roll servo, linkage arm |
| rollbracket x2 | 62.6 g | 7.6 g | wheel servo |
| wheel x2 | 54.8 g | - | body, tyre and horn |
| **Total** | **1800.4 g** | **340.7 g** | |

The shin is the row that moved: **128.9 g to 54.1 g**. It was carrying a 55 g
servo and 34 g of belt drive out at y = 84.5, and it is now the lightest link
in the leg. Its centre of mass moved 22.7 mm and most of its inertia went with
the pulleys.

> **The sim agrees with this table to a tenth of a gram**, per body, checked by
> `uv run python -m cad.twin`. It has not always: it weighed 1874.8 g for a
> fortnight against a CAD that said 1907, because `SEG_INERTIA` is pasted from
> `cad.inertia --emit` and nobody re-ran it. What follows is that history.
>
> ~~The sim does not agree with this table.~~ It weighed 1874.8 g, because
> `SEG_MASS` in `src/rsbot/model.py` was a hand-copied snapshot that nobody
> re-ran after the servo cradles landed. `cad.twin` failed on it per body. The
> table above is the CAD's, which is what gets built; the gap was open on
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
