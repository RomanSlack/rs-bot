# Before you order anything

Read this first if you are picking up rs-bot. It says what the project is
actually trying to prove, what has been verified against reality and how, and
what is still fiction.

> **Current state, 2026-08-06 (this supersedes stale numbers further down).**
> The robot is **1749 g**, EIGHT servos, no belt and no ankle-pitch servo - a
> passive parallelogram replaced them - and PA6-CF throughout. Sim mass = CAD
> mass to the milligram. The body below still says 1.907 kg, ten servos, a belt,
> a 39.60 mm servo (it is 36.50 now), and "53 tests"; those are history. The
> verification suite is ~106 checks and green, and the linkage rod now has a
> real FEA behind it.
>
> **Two things below are no longer true and matter.** "Fasteners are specified"
> is only half right: the bolt list exists, but the **16 M2.5 heat-set insert
> bores it calls for are not modelled in any part** (found 2026-08-06), so the
> part-to-part joints have nowhere to seat an insert. And the order sheet's
> fastener rows are stale (40 deleted case screws; missing the horn centre screw
> and rod-clamp bolts). See `road-to-order.md` for both. Everything else still
> open is physical - material coupon, bore fits, backlash - and wants the $30
> validation order, not more simulation.

## The mission

Prove that a simulator plus an agentic AI can do the whole loop - CAD, physics,
control, verification - well enough that the hardware works **first time**.

That last part is the whole point. The operator gets **one order**. So the
question is never "does it look right", it is **"what in this model is still a
number somebody made up?"** Every section below exists because the answer was
once "quite a lot".

## The one rule this project runs on

> A check that cannot fail is worse than no check at all, because it reads as
> evidence.

Every significant fault here was found by a check that was *made able to fail*,
and several were hidden for weeks by checks that could not be. Before trusting
any check you add, break the thing it tests on purpose and confirm it screams.

Examples, all real:

| check | how it was silently passing |
|---|---|
| whole-robot interference | MuJoCo skips collision between static world children - it reported clean with a part shoved 100 mm into the robot |
| chassis stress | held nodes and loaded nodes were the same set, so nothing could move: 0.00 MPa |
| fit checker | skipped same-body pairs as lap joints, hiding a bracket 2597 mm3 inside a servo case |
| tuning cost | penalised falling at 100 and oscillation at 3, so it returned gains that never fell and limit-cycled at +/-16 deg |
| tuning cost, again | had no yaw input at all, so a set of gains limit-cycled at +/-2 rad/s and walked a metre backwards while scoring well |
| FEA solve | SuperLU diverged silently - same geometry and load, 22.7 MPa in one material and 132.8 in another |

## What is verified, and against what

| | how | status |
|---|---|---|
| Geometry, both modes and through the flip | boolean intersection of 19 real solids, plus an OBB sweep over 26 poses | clean |
| Every body is one rigid piece | connectivity with a tolerance tight enough to catch a gap inside a part | clean |
| Nothing touches the floor in foot mode | support-point check, both modes | clean |
| Loads | `d.cfrc_int` off the sim - exact, inertial and contact terms included | closes against statics, 6.12 N vs 6.12 N |
| Stress | TET10 FEA, `cad/fea.py` | cantilever benchmark: +1.9% deflection, +3.1% stress |
| Mass and inertia | composed from real B-rep solids plus bought-part boxes | mass closes against `cad/masses.py`; tensors positive definite |
| Printability | overhang, wall thickness, bed footprint | all five parts fit, no sub-1.2 mm features |
| Servo torque | measured over the whole envelope | duty above continuous 0-1.9% |
| Balance, transition, backlash to 2 deg | 53 tests | passing |
| **Servo dimensions** | **measured off a real STEP model** | see below |

### The servo is the one that mattered

Every servo dimension was invented from a product listing. Four of the five
printed parts bolt to a servo, so those numbers set every mounting face in the
robot. `cad/servo.py` now reads them from TheRobotStudio's SO-ARM100 STEP:

| | assumed | real |
|---|---|---|
| case height | 35.4 | **39.60 mm** - and it is along the OUTPUT SHAFT |
| horn bolts | r = 8.0 at 45 deg | **9.9 x 10.0 rectangle** - every hole 0.71 mm out |
| horn screw | M2 | 2.5 mm clearance |

Applying it removed 4.2 mm of clearance that never existed and broke **16
clearance pairs**. Fixing them moved the shin's whole outboard route, the
chassis shelves, the ankle servo and the yoke. Nothing about that was visible
before the real number arrived.

**The lesson for whoever is next: a bought part's dimensions are not a
detail, they are the boundary conditions of every part that touches them. Get
the real CAD before drawing anything against it.**

## What is still fiction

Most of the original list has been closed. What is left is what CAD cannot
answer.

1. **The servo horn bolt pattern is assumed.** The horn is a bought part and
   its pattern is not in the servo STEP, so it was never measured, and four of
   the five printed parts bolt to a horn. There is a decoy in that file: the
   case has four screws at each end on a 9.9 x 9.9 rectangle, almost exactly
   what this project believes the horn to be. This is now the single most
   load-bearing assumption in the robot.
2. **The servo case mounting positions are assumed**, same reason. What IS
   verified is that every servo hole runs along the shaft axis and that all 28
   holes in the printed parts agree with it.
3. **There is no central horn screw in the CAD.** A horn is fixed to its spline
   by an axial screw; nothing models it or checks access to it, and it is the
   only screw actually fitted on the robot at a horn joint.
4. **Bearing bores are nominal.** A printed bore needs measuring and
   offsetting, usually 0.1-0.2 mm. So does the 0.35 mm tyre press fit.
5. **Cable channels do not exist.** The runs are modelled now
   (`cad/wiring.py`) and three of them pass through printed structure, so the
   thigh, shin and yoke need channels cut before they are ordered.
6. **Material and fatigue properties are published figures**, not coupons off
   the machine that will make these parts. Expect +/- 20%.

Closed since this list was written: fasteners are specified
(`cad/fasteners.py`), tool access is checked in build order
(`cad/toolaccess.py`), tolerance is stacked along the chain rather than
pair-by-pair, mesh convergence is run on the two parts near their limit and
both have settled, and fatigue is solved on the flip at 1x with PA6-CF passing
everything at 70% worst.

## What to buy, and what not to

**Printed parts USED to be treated as not the risk**, on the grounds that you
can reprint a bracket for pennies. That is false here: the operator has no
printer, so every part comes from a service and a reprint is real money and one
to two weeks. Printed-part correctness now matters about as much as bought-part
correctness, and `road-to-order.md` re-ranks the remaining work on that basis.

The bought parts are still where the irreversible spend is:

| | decision | why |
|---|---|---|
| Servos | 10x Feetech STS3215 | torque verified over the envelope; duty cycle fine, peaks saturate for milliseconds |
| Belt | **GT2 94T (188 mm), 15 mm wide** | centre distance falls on a stock size to a tenth of a tooth. 9 mm is over-tensioned at 132% |
| Pulleys | GT2 20T and 40T | 2:1 is pinned from both sides - below it the belt is overloaded, above it the servo runs out of travel |
| Bearings | 623ZZ | 3 mm shafts throughout |
| Wheels | **print them** | no commercial wheel has a usable side face, and this one is also the foot. Pololu's 25T range tops out at 90x10 |
| Material | **PA6-CF** | it is the only one that passes every part. ABS fails the shin and the roll bracket |

## Run these before believing anything

```bash
uv run python -m pytest tests -q     # 53 checks, ~80 s
uv run python fitcheck.py            # overlap, connectivity, flip sweep
uv run python -m cad.assemble_check  # real solids, every pair, both modes
uv run python -m cad.envelope        # where a feature is allowed to go
uv run python -m cad.stress          # FEA, every part, every material
uv run python -m cad.printability    # overhangs, walls, bed
uv run python -m cad.loads           # joint loads, with its statics control test
uv run python -m cad.servo           # the measured servo, vs what was assumed
uv run python -m cad.fasteners       # the bolt list, and do the screws line up
uv run python -m cad.toolaccess      # can a driver reach every screw
uv run python -m cad.wiring          # where the cables go, and how much slack
uv run python serve.py               # drive it, localhost:8781
```

`cad/envelope.py` is the one to reach for when adding geometry: it holds the
forbidden volumes as solids, so you intersect against them instead of
reasoning about them. It is deliberately conservative - it sweeps roll and
pitch independently, so a small overlap there may still be fine on the real
trajectory. `fitcheck.py` and `cad/assemble_check.py` settle that, because they
sweep the manoeuvre the robot actually performs.

## Where it stands

**1.907 kg**, every gram derived from geometry rather than assigned. It read
1.858 until 2026-08-02: five servo cradles and a closed chassis, none of which
had reached this page.

The robot balances, drives, steers, flips both ways, and survives 2 deg of gear
lash against a measured 0.87 in the real servo.

**It is not ready to order, and the fix is cheap.** Buy ONE servo and one horn
first. That settles items 1, 2 and 3 - the whole top of the list - plus the
printed bore tolerance, for the price of a coffee and a week. See
`road-to-order.md` and `order-sheet.md`.
