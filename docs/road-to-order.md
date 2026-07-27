# The road to ordering

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
| servo | 45.2 x 24.7 x 35.4 | 45.4 x 24.8 x **39.6**, measured off a real STEP |
| wheel | bought, 80 x 24, 60 g, ~$8 | **printed**, two parts, 54 g |
| structure | "Printed PETG ~400 g" | **165 g of PA6-CF** |
| total | 2.05 kg | **1.858 kg** |

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

---

### 7. Refresh the entry documents

`before-you-order.md` is the first thing anyone reads and it is now stale:
1.863 kg (it is 1.858), the wheel before it had tread or retention, no mention
of the material sourcing conclusion, and the risk framing corrected at the top
of this file.

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

## The rule this all runs on

Unchanged, and it applies to every check added for the work above:

> A check that cannot fail is worse than no check at all, because it reads as
> evidence.

Before trusting any of the checks proposed here, break the thing it tests on
purpose and confirm it screams. Every check added in the wheel and clearance
work was control-tested that way, and two of them were rewritten because the
first version passed a deliberately broken input.
