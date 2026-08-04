# The order sheet

Everything you have to buy or have made, with a quantity against every line.
Someone who is not you should be able to place this order without asking a
question.

`bom.md` is the reasoning: what the parts are, why the servos sit where they
sit, and what modelling them at real size revealed. This is the shopping list.

**Read `road-to-order.md` before spending anything.** Three lines here are still
unverified and one servo will settle all three.

> **BLOCKING, 2026-08-03: three linkage pieces are drawn as separate parts and
> belong to other parts.** `lk_fin` is chassis, `lk_stub` is shin, `lk_arm` is
> ankle yoke. Until they are cut into their hosts, this sheet describes a robot
> that cannot be assembled, and the shin should shrink when it happens - its
> outboard carrier only reaches y = 136 because the deleted belt forced the
> ankle bearing out there.

---

## 1. Printed parts

All from a service. **PA6-CF, FDM**, from Unionfab or Weerg: it is the only
material that passes every part, and `materials.md` has the numbers plus the
reason the obvious vendors (JLCPCB, PCBWay) do not work.

Files are written into `cad/out/` by `uv run python -m cad.robot`.

| file | qty | material | mass ea |
|---|---|---|---|
| `chassis.step` | 1 | PA6-CF | 135 g |
| `thigh.step` | 2 | PA6-CF | 41 g |
| `shin.step` | 2 | PA6-CF | 40 g |
| `ankle_yoke.step` | 2 | PA6-CF | 14 g |
| `roll_bracket.step` | 2 | PA6-CF | 8 g |
| `wheel_body.step` | 2 | PA6-CF | 27 g |
| `wheel_tyre.step` | 2 | **TPU 95A** | 27 g |
| `lk_rod1.step` | 2 | PA6-CF | 7 g |
| `lk_rod2.step` | 2 | PA6-CF | 7 g |
| `lk_idler.step` | 2 | PA6-CF | 4 g |

> **THREE MORE LINKAGE PIECES ARE NOT ON THIS LIST AND THAT IS WHY YOU CANNOT
> ORDER YET.** `lk_fin` (12 g), `lk_stub` (3 g) and `lk_arm` (2 g) are FEATURES
> of the chassis, the shin and the ankle yoke, drawn separately in
> cad/linkage.py and not yet cut into their hosts. Ordering today would buy
> three loose pieces with nothing to bolt them to. See the blocking list below.
>
> `lk_rod2` is also drawn as one piece and specified as two, clamped, because
> its length has to be adjustable at assembly. cad/linkage.py's ADJUST_MM says
> why and tests/test_linkage.py measures what happens without it.

Right-hand parts are the left mirrored in y, except the two wheels, which are
the **same part flipped over**. One part number, print two.

**Specify 60% infill or better.** The entire strength study assumes it and
service FDM often defaults far lower. That is a line on the order form, not an
assumption you get to make afterwards.

**Cost: get a real quote, because this is the line nobody can estimate.**
19 physical pieces, 430 g of PA6-CF plus 53 g of TPU.

> The 183 g this line used to claim was the LEGS ONLY, quoted as if it were the
> robot, with the 135 g chassis never in it. It is in `docs/how-checks-fail.md`
> as entry 7. The figure above is `uv run python -m cad.masses` summed over the
> printed column, which is where it should have come from all along.
>
> **It is computed at PETG's 1.27 g/cm3 and these parts are ordered in PA6-CF
> at 1.19.** That is about 26 g the model is carrying that the robot will not
> be, and it is not fixed here because the material is not settled: three PA12
> variants came back into play when the roll bracket went from 99% to 55%
> (`road-to-order.md` item 8), and the density follows the material. Service FDM in PA6-CF
prices per part rather than per gram, with a setup charge that dominates on
small parts like the 6 g roll bracket, so expect somewhere between **$180 and
$420** and treat any single number as fiction until Unionfab or Weerg have
quoted the actual STEP files. This is plausibly the largest line on the whole
order, larger than the servos. Upload `cad/out/*.step` and find out before
planning around the total below.

## 2. Bought parts

Prices are USD, checked 2026-07-28. **`v` was verified against a live listing
that day; `e` is an estimate and wants checking before you rely on the total.**
Nobody should be surprised at the checkout page.

| part | qty | unit | total | src | note |
|---|---|---|---|---|---|
| **Feetech STS3215-C018** (12 V, 1:345) | 8 | $16.00 | $128 | v | EIGHT, not ten: see the note below |
| 25T **metal** servo horn | 8 | $2.50 | $20 | e | pattern measured, see `cad/servo.py`. Metal, not the POM horn that may ship in the box |
| 623ZZ bearing, 3 x 10 x 4 | 6 | $1.00 | $6 | e | ankle pitch and the linkage idler, both legs. Sold in 10-packs |
| 3 mm shaft, 30 mm | 4 | $1.00 | $4 | e | ankle pitch and ankle roll. Or cut from stock |
| 3 mm pin, 18 mm | 8 | $0.50 | $4 | e | the parallelogram's four pivots a leg. Cut from the same stock |
| Raspberry Pi 5 | 1 | $80.00 | $80 | e | 8 GB |
| 3S LiPo, 2200 mAh | 1 | $25.00 | $25 | e | |
| TTL bus adapter, FE-URT-1 class | 1 | $10.00 | $10 | e | needed for the validation order too, buy it early |
| IMU, BNO085 class | 1 | $25.00 | $25 | e | breakout board |
| Servo bus cable, 3-pin, 100 mm | 4 | $1.50 | $6 | e | |
| Servo bus cable, 3-pin, 150 mm | 4 | $1.50 | $6 | e | hip and roll joint, which need slack. `cad.wiring` now wants a 9 mm service loop, up from 6 |
| | | | **$314** | | |

> **TWO SERVOS AND A WHOLE BELT DRIVE CAME OFF THIS LIST.** $81 of parts
> deleted against $6 added, so $78 net, and 178 g out against 70 g of linkage
> back in, so 107 g net. The ankle-pitch servo is replaced by a passive parallelogram: see
> `docs/deleting-the-ankle-pitch-servo.md`, and `cad/linkage.py` for the
> mechanism. Gone with it: two 25T horns, a 20T GT2 pulley on a 4.40 mm hub
> that was not a catalogue part, a 40T pulley that sat 2.23 mm below the floor
> in foot mode, and a 188 mm x **15 mm** belt whose width this sheet flagged as
> uncommon and possibly needing a specialist. That sourcing risk is gone.

**Order by part number, not by name.** Feetech sells at least five STS3215
variants that look identical in photographs and are not interchangeable:

| part | V | torque | ratio | |
|---|---|---|---|---|
| **C018** | **12** | **30 kg.cm** | **1/345** | **this robot** |
| C001 | 7.4 | 19.5 kg.cm | 1/345 | |
| C044 | 7.4 | 16-27 kg.cm | 1/191 | |
| C046 | 7.4 | 14.4 kg.cm | 1/147 | |

Most listings that surface first are the 7.4 V ones, because those are what
SO-ARM100/101 uses. Searching `STS3215 C018` filters correctly in one step.
$16 is OpenELAB's price; Amazon runs about $24.50 a unit for next-day.

The belt is 15 mm and not 9 because the measured joint peak is 132 N of
tension, and the 2:1 ratio is pinned from both sides: below it the belt is
overloaded, above it the servo runs out of travel.

## 3. Fasteners

Generated by `uv run python -m cad.fasteners`, which prints the interface each
one belongs to.

| part | qty |
|---|---|
| M2 x 10 self-tapping | 40 |
| M3 x 8 socket cap | 32 |
| M2.5 x 8 socket cap | 4 |
| M2.5 x 10 socket cap | 8 |
| M2.5 x 6 socket cap | 4 |
| M2.5 heat-set insert, 3.5 mm bore x 4.0 deep | 16 |

Buy 25% spares. They cost nothing and one missing M2 stops a build for a week.

### What threads into what

| interface | screw | threads into |
|---|---|---|
| part to servo **case** | M2 self-tapping | the servo's own 1.5 mm pilot |
| part to servo **horn** | **M3** | the horn, which is metal and tapped |
| part to part | M2.5 | a heat-set insert |

The horn screw is M3 and not M2.5, and that is measured rather than assumed:
TheRobotStudio's own SO-ARM101 parts carry 3.0-3.2 mm clearance holes on the
horn pattern. A clearance hole in the plastic means the thread is in the horn,
so a horn joint needs **no insert** - which is why the insert count fell from
48 to 16.

Inserts for anything taken apart more than twice, which is every servo mount on
a robot that has not been built yet.

## 4. Tools

| tool | ~cost | why |
|---|---|---|
| 2 mm hex key, short | $8 | four of the thigh's screws will not take a full-size driver's handle. `cad/toolaccess.py` found that, rather than the bench |
| soldering iron with M2.5 insert tip | $15 | 16 inserts |
| **digital calipers, 150 mm, 0.01 mm** | $40 | the validation order is worthless without them. Do not buy the $25 tier: the headline check is a Ø19.95 horn in a Ø20.00 pocket, and at 0.05 mm a cheap caliper measures itself |
| pin gauges or drill bits | $0-25 | calipers cannot measure hole **positions**, and the case mounting pattern is exactly that. Pin the holes, measure over the pins, subtract one pin diameter |

A flatbed scanner, if you own one, beats all of the above for the case pattern:
1200 dpi is 0.021 mm per pixel and it captures the whole 2D pattern at once
instead of pair by pair.

## 5. Cable channels: cut, with one exception

The thigh, shin and ankle yoke now carry a 6 mm channel along the cable run,
cut for BOTH poses because the paths move between them. Wheel mode is clear on
all eight runs; foot mode has one left.

**The roll-to-wheel run is not solved and cannot be by a channel.** It crosses
the ankle roll joint, so its far end is on a bracket that turns 90 degrees, and
its shortest path passes through the WHEEL in foot mode. You cannot groove a
part that spins. That run needs a routed path tucked inboard of the wheel, held
by a clip, taking up the 6.3 mm of length change as a service loop with an
8 mm bend radius.

Decide that route before ordering: it may want a tie point on the yoke, and a
tie point is a printed feature.

---

## 6. What the whole thing costs

Added 2026-07-28. Before this, `bom.md` promised that "the quantities and prices
live in the order sheet" and only the quantities did.

| line | low | high | confidence |
|---|---|---|---|
| Printed parts, PA6-CF + TPU | $180 | $420 | **needs a real quote** |
| Bought parts | $314 | $314 | servos verified, rest estimated |
| Fasteners and inserts, incl. 25% spares | $40 | $40 | estimated |
| **Build subtotal** | **$534** | **$774** | |
| Tools, one-time | $63 | $88 | estimated |
| **Total** | **$597** | **$862** | |

Three things worth reading off that table:

- **The printed parts cost more than the servos**, and by more than they used
  to: the servo line is down to $128 and the printed mass is up to 431 g. Until the STEP
  files are quoted, the largest number here is the one nobody has priced.
- **Buying the servos on Amazon adds about $85** to the total ($24.50 against
  $16 a unit) for next-day delivery. Worth it for the validation order,
  wasteful for the other eight.
- **This is cheap for what it is.** A single Dynamixel XL430 is around $50 and
  a DIY QDD actuator is $150+, which is why the whole LeRobot ecosystem
  standardised on this servo. See the 2026-07-28 status report for why building
  custom actuators was considered and rejected.

## Do not order all of this at once

`road-to-order.md` argues for a validation order first: **about $60 in parts,
or $100 including the calipers you do not yet own.** These are the reasons, in
order of what they would cost to get wrong:

1. ~~The horn bolt pattern is assumed.~~ **Now measured** off three
   TheRobotStudio SO-ARM101 parts that bolt to the same horn, six patterns
   agreeing to 0.00 mm. It moved the screw from M2.5 to M3 and grew every horn
   hole in this robot by 0.7 mm.
2. **The servo case mounting positions are still assumed.** What is verified is
   that every servo hole runs along the shaft axis and that all 36 holes in the
   printed parts agree with it.
3. **There is no central horn screw anywhere in the CAD.** A horn is fixed to
   its spline by an axial screw. Nothing models it, sizes it, or checks you can
   reach it, and it is the *only* screw actually fitted on the robot at a horn
   joint, because the four pattern screws are done on the bench.
4. ~~Cable channels do not exist.~~ **Cut**, in the thigh, shin and ankle yoke,
   for both poses. One run is left and it needs a ROUTE rather than a groove:
   roll-to-wheel passes through the wheel in foot mode, and you cannot channel
   a part that spins. It wants a clip and a tie point, and a tie point is a
   printed feature - so decide it before ordering.

**Buy two servos, a metal horn and a bus adapter first.** About $60, and a
week. It settles 2 and 3, plus the printed bore tolerance and the 0.35 mm tyre
press fit.

**Two, not one, and the reason is the same one that recovered the horn.** A
single servo gives you a *measurement* of the case mounting pattern. Two give
you a *check*: if both agree, the pattern is real, and if they disagree, that
disagreement is the finding, which matters a great deal when the plan is to put
40 M2 screws into those holes. Agreement between independent parts is the
measuring instrument here, exactly as it was for the horn.

The adapter is on the list because backlash is an angle, not a length: command
the servo to hold, twist the output by hand, read the position back. The 12-bit
encoder resolves 0.088 deg, so it measures 2-3 deg of lash easily and no
caliper can.

## Mass, for the record

**1.907 kg**, and every line below is `uv run python -m cad.masses`, summed,
rather than a number carried forward from the last time someone wrote this out:

| | g |
|---|---|
| printed structure, all seven parts | 341 |
| servos, 10 x STS3215 | 550 |
| arm/head ballast + IMU (stage 4 stand-in) | 605 |
| torso electronics: Pi 5, 3S pack, bus adapter | 235 |
| wheels, 2 x (printed body + TPU tyre + horn) | 108 |
| belt drive, 2 x (pulleys, belt, shafts) | 68 |
| **total** | **1907** |

It said 1.875 kg until 2026-08-02, which was the SIM's number, and the sim had
not been told about the cradles. The figure above is the CAD's, because the CAD
is what gets printed and this is the sheet you order from.

The printed line said **183 g** for weeks and was briefly "corrected" to 197 g
on 2026-08-02 by adding the cradles to the stale figure instead of re-deriving
it. It is 341 g. Incrementing a number you have not checked carries its error
forward and puts your name on it, and it happened here in the one file where
being wrong costs money.

> **The sim still says 1.875 kg.** `cad.twin` fails on that gap, per body, and
> it is open on purpose: closing it changes what the balancer was tuned
> against. It does not affect anything you would buy, which is why this sheet
> can be correct while that stays open.
