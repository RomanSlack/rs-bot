# The road to ordering

> **2026-08-03: the ankle-pitch servos and their belt drive are deleted.** A
> passive parallelogram replaces them (`cad/linkage.py`,
> `docs/deleting-the-ankle-pitch-servo.md`). Ten servos to eight, 1907 g to
> 1800, $78 off the bought-parts line, and one sourcing risk gone with the
> 15 mm belt. The order sheet is updated and now BLOCKS on a new item: three
> linkage pieces are drawn as separate parts and belong to the chassis, the
> shin and the ankle yoke.
>
> **NEW, and it outranks everything below: the servo case mounting is not
> just unverified, it may be the wrong idea.** 40 of this robot's screws go
> into holes in the servo case. TheRobotStudio's own holder, which is the
> reference design for this servo, does not use those holes at all - it is a
> cradle that captures the servo in a cavity. That is why the pattern cannot be
> recovered the way the horn's was: a mating dimension gets encoded exactly by
> everyone who mates with it, and nobody mates with these. See `cad/servo.py`.
>
> **Investigated, and the answer is: capture is right, but it is a redesign,
> not a feature.** There IS room - 3 to 4 mm free at the hip, knee and ankle
> mounts, and the roll and wheel servos are already enclosed on three or four
> faces. But a rim will not attach: tried on the shin at both ends of the servo
> and at 6, 10 and 16 mm depth, it fuses as a separate floating solid every
> time. Every servo mount in this robot is a PLATE on the case's end face with
> no material anywhere around the case perimeter, so there is nothing for a rim
> to grow from.
>
> `cad.servo.cradle()` exists and its geometry is right; it is deliberately not
> wired in. Doing this properly means changing how four parts meet their
> servos, with packaging re-checked each time. Decide it BEFORE printing.
>
> **Cable channels are cut** in the thigh, shin and ankle yoke, for BOTH poses,
> because the runs move between them. Wheel mode is clear on all eight; foot
> mode has one left, and it needs a ROUTE rather than a groove: roll-to-wheel
> passes through the WHEEL in foot mode, and you cannot channel a part that
> spins. It wants a path tucked inboard with a clip, taking up the 6.3 mm of
> length change as a service loop.
>
> **Status, 2026-07-27: items 1 to 7 are all done.** What each one found is
> recorded in place below. The conclusion did not change: do not order all of
> it at once. Buy one servo and one horn first, because the three things CAD
> cannot settle all live on that interface.

What is left between here and placing an order, ranked by what actually stops a
build. Written 2026-07-27, after the wheel, clearance and assembled-strength
work landed.

Read `before-you-order.md` for what has been verified and against what. This
document is the other half: what has NOT been, and in what order to fix it.

---

## First, a correction to how this project ranks risk

`before-you-order.md` says:

> **Printed parts are not the risk.** You can reprint a bracket for pennies.
> The irreversible spend is the bought parts, so that is where the checking
> went.

That was written for someone with a printer on the desk. **The operator does
not have one.** Every printed part comes from a service, so a reprint is real
money and one to two weeks, and a reprint of something already built around is
worse than that.

So the sentence is false here, and it has been quietly steering where the
effort goes. Corrected:

| | with a printer | with a print service |
|---|---|---|
| reprint a bracket | pennies, same day | ~$20-40, 1-2 weeks |
| wrong hole diameter | reprint, lose an hour | reprint, lose a fortnight |
| discover it at assembly | annoying | the build stops |

**Printed-part correctness now matters roughly as much as bought-part
correctness.** Everything below is ranked on that basis, which is not the
basis the earlier work used.

---

## The strategy: order twice, not once

The stated mission is that hardware works first time, off one order. The way
to actually achieve that is **two orders, not one**:

1. **A validation order, roughly $30.** One printed coupon set plus the two or
   three cheapest bought parts. Its only job is to turn measured-nowhere
   numbers into measured numbers.
2. **The real order**, placed against corrected geometry.

This is not a retreat from the goal. Four of the seven items on the fiction
list are *material and process properties*, and no amount of simulation
produces them. They can only be measured off a part that exists. Spending $30
to stop guessing is the cheapest risk reduction available, and it is far
cheaper than reprinting the structure.

What the validation order answers:

| unknown | how the coupon answers it |
|---|---|
| bearing bore tolerance | print a bore, measure it, offset the CAD |
| the 0.35 mm tyre press fit | print a rim section and a tyre section, push them together |
| material properties | a tensile coupon in the real material, on the real machine |
| service DFM | the vendor's own automated checks run on the real STLs |
| real cost | the quote, which nobody in this repo knows |

---

## The work, ranked

### 1. There is no order sheet

**Status: blocking. `docs/bom.md` cannot be ordered from.**

It predates the CAD port and most of what has been learned since:

| line | says | reality |
|---|---|---|
| servo | 45.2 x 24.7 x 35.4 | 45.23 x 24.73 x **36.50**, the manufacturer's drawing |
| wheel | bought, 80 x 24, 60 g, ~$8 | **printed**, two parts, 54 g |
| structure | "Printed PETG ~400 g" | **197 g of PA6-CF** |
| total | 2.05 kg | **1.907 kg** |

And it is missing entirely: **fasteners, bearings, the belt and pulleys,
shafts, and cables.** Those are most of the part count.

**Exit criterion.** A table where every row has a specific part, a quantity, a
supplier and a price, and every printed part has a file, a material and a
process. Someone who is not you can place the order from it without asking a
question.

**Why it is first.** Not because it is hard, but because writing it forces
every remaining decision to be made explicitly. You cannot fill in a fastener
row without deciding item 2.

---

### 2. Fasteners: undecided, and it is a CAD change

**Status: blocking, and it is #1 on the project's own fiction list.**

Heat-set inserts versus self-tapping screws is not a shopping preference. The
two need **different hole diameters and different boss wall thickness**:

- a heat-set insert for M2.5 wants roughly a 3.5 mm bore and enough wall around
  it to take the melt without bulging
- a self-tapping screw wants a smaller pilot, sized to the screw's core
  diameter and the material

Nothing in `cad/` currently models either. Decide this after printing and every
part with a threaded interface is scrap.

There is a second question hiding behind it: **the robot has never been given a
bolt list.** How many M2, in what lengths, where. Every bolt hole in the CAD is
a clearance hole into air right now.

**Exit criterion.** Every fastener in the robot appears in a table with
size, length, type and count; every corresponding hole in the CAD matches that
choice; and a check exists that fails if a hole diameter and its fastener
disagree.

**Rule of thumb worth writing down:** inserts for anything taken apart more
than twice. That is every servo mount.

---

### 3. Tool access: nothing checks a driver can reach a bolt

**Status: unchecked, and this design is dense enough that it matters.**

Three servos live within 45 mm of one axle. The ankle yoke, roll bracket and
wheel all crowd the same 60 mm. Nothing anywhere verifies that a screwdriver,
hex key or nut driver can physically reach each fastener, at the angle it needs,
in an order that lets the assembly be built up rather than requiring a part to
be in two places at once.

This is exactly the class of thing this project is good at checking, because it
is geometric: sweep a driver shaft and handle down each fastener axis and
intersect it against the assembled solids.

**Exit criterion.** For every fastener, a driver of realistic dimensions
(shaft diameter, shaft length, handle envelope) reaches its head without
intersecting any part, in a stated assembly order. Control-tested by moving a
bolt somewhere unreachable and confirming the check screams.

**Prediction, recorded so it can be wrong:** this finds at least one
unreachable fastener. If it finds none on the first run, be suspicious of the
check before believing the result.

**Outcome, 2026-08-02.** It found four, on each roll bracket, and held them for
a week. All four were false: they are the roll servo's horn screws, which are
fitted on the bench before the bracket goes near the robot, and the check had
been measuring each screw's distance to its servo from the CASE centre rather
than from the shaft, 12.5 mm away. The prediction told us to be suspicious of a
check that found nothing. The lesson is the other one: **be equally suspicious
of a check that finds something and then never resolves it.** A red that sits
still for a week is not evidence, it is furniture. See
[status/2026-08-02](../status/2026-08-02-status.md).

---

### 4. Wire routing: completely unmodelled

**Status: the item most likely to force a reprint.**

Ten daisy-chained bus servos. Cables cross every joint, including **one that
rotates 90 degrees**. There is not one cable, channel, tie point or strain
relief anywhere in the CAD.

Why this is dangerous rather than merely incomplete:

- cable bundles need volume, and this robot's spare volume is already spoken
  for. The envelope work found exactly one legal location for the ankle-pitch
  bearing; a cable run has to fit in what is left.
- the ankle roll joint turns 90 degrees. A cable crossing it needs a service
  loop with a bend radius, and it must not snag, chafe or bind at either end of
  travel or anywhere between. That is the same swept-envelope question the flip
  already needed, applied to a flexible object.
- strain relief has to bolt to something, and mounting features are exactly the
  kind of thing that gets discovered after printing.

**Exit criterion.** Every cable run modelled as a swept volume with a stated
minimum bend radius, checked against the assembled robot in both modes and
through the flip, with tie or channel features present in the parts that carry
them.

---

### 5. Tolerance stacks beyond the wheel

**Status: partially done. The wheel is covered; nothing else is.**

`cad/assemble_check.py` now reports minimum distance and fails a running
clearance under 0.8 mm, but `running_pair()` only treats the **wheel** as a
running pair, because it is the only continuously rotating body. That was the
right call for the check that caught the 0.1 mm rub, and it leaves two gaps:

- **static chains are unchecked.** Four parts in a row, each ±0.3 mm, is a
  ±0.6 mm worst-case relative error that no single pairwise gap sees.
- **limited-travel joints are only checked at poses.** `fitcheck.py` sweeps the
  flip in 26 poses, which is a sampling, not a proof, and it uses nominal
  geometry with no tolerance applied at all.

**Exit criterion.** A worst-case stack computed along each assembly chain, and
the flip sweep re-run with parts perturbed by the service tolerance rather than
at nominal.

**DONE, 2026-08-02.** `cad.assemble_check.sweep_stack()` applies the chain-depth
stack at every step of the flip, not just at the two rest poses. Nothing closes
anywhere: the tightest is the roll bracket against the wheel, 0.90 mm nominal
across 2 interfaces, so **0.30 mm at worst case**, and it holds that value right
through the manoeuvre because the two turn together. Control-tested by loosening
the service to +/-0.5 mm, which closes 14 pairs.

Analytic rather than perturbed, for the reason `stack()` already records:
offsetting these solids by 0.3 mm fails outright in OCC, and assuming every
error lines up the wrong way is what worst case means anyway.

**And the sweep had never been running.** `stack`, `sweep` and `sweep_stack` all
existed while `uv run python -m cad.assemble_check` called none of them - the
entry point did the two rest poses and stopped. So the flip sweep on real
solids, written 2026-08-01 and the first thing ever to sweep this robot on its
actual geometry, only ran when somebody imported it by hand. All four now run
from the one command, which takes a few minutes rather than one.

---

### 6. The FEA's own confidence

**Status: two known holes, both cheap to close.**

**Mesh convergence has never been run.** The solver is verified against a
cantilever (+1.9% deflection, +3.1% stress), which validates the *element*, not
the *mesh density on these parts*. Peaks could move with refinement, and the
margins that matter are thin: the shin sits at 86% and the roll bracket at 94%
in PA6-CF. A 20% peak shift changes the answer for both.

**There is no fatigue analysis.** Everything is static ultimate. Stage 2's exit
criterion is 20 consecutive transitions, and the design intent is thousands.
A part at 94% of ultimate can fail after ten thousand cycles at a fraction of
that load, and printed parts are worse at this than the handbook figures for
the bulk polymer.

**Exit criterion.** Each part solved at two or three mesh densities with the
peak reported, so the trend is visible rather than assumed; and a cycle count
with a knockdown applied to the two parts above 80%.

> **RETRACTED, later the same day. This item does NOT close, and the risk it
> describes is worse than when it was written.** Everything below was computed
> on meshes containing inverted elements - tets with a flipped Jacobian, which
> make the stiffness matrix indefinite and quietly UNDERSTATE stress. Six of
> the seven parts were affected. On valid meshes the roll bracket is at **99%**
> of PA6-CF, not 70%, and the shin is at **83%**, not 61%. See `docs/stress.md`.
>
> So the two thin margins this item was written about are not gone. One of them
> is thinner than the 94% that prompted writing it, and the reason nobody
> noticed is that the solver returned plausible numbers from a broken mesh for
> weeks. `cad/fea.py` now refuses to return an invalid mesh at all.
>
> **Re-run, and the numbers hold.** Convergence on valid meshes:
>
> ```
> roll bracket   57.00 / 56.41 / 56.40 MPa   at 3.0 / 2.6 / 2.2 mm
> shin           47.59 / 47.83 / 48.07 MPa
> ```
>
> Both flat to 1%, so 56.4 MPa is the roll bracket's answer and not a meshing
> artifact. **99% of PA6-CF, converged.** Fatigue at 10 000 cycles is fine on
> both - the roll bracket is at 74% of its endurance limit in plane, 56% across
> layers - so fatigue is NOT the constraint. The static margin is, and there
> is none.
>
> **This is now a geometry problem, not an analysis problem**, and it is the
> thing that blocks an order. See item 8 below.

**Superseded, kept for the record.** Both parts, both studies, on meshes that
turned out to be invalid.

| | mesh 3.0 / 2.6 / 2.2 mm | static, PA6-CF | fatigue @ 10k |
|---|---|---|---|
| roll bracket | 40.69 / 39.78 / 39.94 MPa | **70%** | 52% in-plane, 32% interlayer |
| shin | 34.46 / 34.80 / 34.64 MPa | **61%** | 23% / 23% |

Both peaks are flat to within 2% across three densities, so they are answers
rather than artifacts of how finely the part was chopped. Neither is close to
its endurance limit, and **0 of 8 material pairs fail in fatigue for the shin**.

**Neither of the two "thin margins" this item was written about still exists.**
It said the shin sits at 86% and the roll bracket at 94%, and warned that a 20%
peak shift under refinement would change the answer for both. Refined: the peaks
barely move. The numbers themselves were simply old - the shin is 61% and has
been through redesigns since 86% was recorded, and the roll bracket came off 94%
when its servo cradle added material in the load path.

So the risk this item exists to flag is not merely quantified, it is gone, and
it had been gone for a while without anyone re-running the numbers to notice.
The remaining fiction is the material properties themselves, which are
literature values: that is the validation order's job, not the solver's.

---

### 7. Refresh the entry documents

`before-you-order.md` is the first thing anyone reads and it is now stale:
1.863 kg (it is 1.907 as of 2026-08-02, and was 1.858 when this was written -
the number has moved twice since, which is the argument for this item rather
than against it), the wheel before it had tread or retention, no mention of the
material sourcing conclusion, and the risk framing corrected at the top of this
file. Its mass line is now correct; the rest of the item stands.

`docs/bom.md` is covered by item 1.

---

## Suggested sequence

Items 1 to 3 are one conversation, because writing the order sheet forces the
fastener decision, and the fastener decision is what tool access is checked
against. Do them together.

```
  1 + 2 + 3   order sheet, fastener spec, tool access      <- start here
  4           wire routing
  5           tolerance stacks
  --- place the $30 validation order ---
  6           mesh convergence and fatigue, while it ships
  --- measure the coupons, correct the CAD ---
  7           refresh the entry docs
  --- place the real order ---
```

Item 6 goes while the validation order is in transit because it needs no
hardware and it is the only item on this list that can be done in parallel.

---

### 8. The roll bracket is at 99% of PA6-CF, and that now blocks the order

> **SOLVED the same day, and cheaply.** The web is 5.5 mm instead of 4.0 mm and
> the part is at **55% of PA6-CF**, not 99%. It also passes in all three PA12
> variants now - MJF 66%, SLS 70%, FDM 75% - where it was over in every one of
> them, so the material choice is open again rather than PA6-CF or nothing.
>
> ```
> web 4.0 mm   56.6 MPa   99%
> web 5.5 mm   31.4 MPa   55%
> web 7.0 mm   20.7 MPa   36%
> ```
>
> Stress in a plate bending goes as 1/thickness, so the last 1.5 mm was worth 44
> points of margin. The window between the yoke's post and the flat wheel was
> the whole budget, and the post was 6.0 mm housing a 4.0 mm bearing: the spare
> 1.5 mm was doing nothing. Three separate yoke boxes referenced the old web
> face and only one moved with it; `assemble_check` found the other two, at
> 477 mm3 and 45 mm3.
>
> Hub radius was tried first and is not the lever: 8 to 12 mm moves it 99% to
> 97%. The section was the problem, not a stress concentration.

**Status: was BLOCKING, added 2026-08-02, solved 2026-08-02.**

Converged, on valid meshes, at 56.4 MPa against PA6-CF's 57 MPa allowable. One
percentage point is not a margin - it is the same number twice. And PA6-CF is
now the ONLY material that passes this part: MJF PA12 is at 118%, SLS PA12GF at
126%, FDM PA12CF at 135%. Those are the three services this project was
choosing between.

This was invisible until today because every FEA in the repo ran on meshes
containing inverted elements, which understate stress. The part has read 94%,
then 70% after its servo cradle, and it is really 99%. The cradle did help; the
mesh error was hiding more than the cradle gained.

Fatigue is fine (74% of the endurance limit in plane), so this is a static
ultimate problem, not a cycles problem.

**What it needs, and it is one of these:**

- **More section in the load path.** The bracket is a plate from the arm to the
  roll axis and the peak is at the re-entrant corner where the web meets the
  hub. It already carries the one fillet the FEA asked for.
- **Less load through it.** The wheel's reaction arrives through the servo, so
  this is really about the arm's cantilever, which is set by the wheel's 40 mm
  radius and the 12.9 mm y-offset that keeps the arm off the tyre.
- **Accept PA6-CF only, at 99%, and print a spare.** Defensible only if the
  material properties are real, and they are literature values until the
  validation order says otherwise. At 99% a 10% property shortfall is a broken
  part.

**Do not order printed parts until this is decided.** It is exactly the class
of thing this project exists to catch before money is spent, and it was caught
by a mesh-quality guard that did not exist this morning.

## The rule this all runs on

Unchanged, and it applies to every check added for the work above:

> A check that cannot fail is worse than no check at all, because it reads as
> evidence.

Before trusting any of the checks proposed here, break the thing it tests on
purpose and confirm it screams. Every check added in the wheel and clearance
work was control-tested that way, and two of them were rewritten because the
first version passed a deliberately broken input.
