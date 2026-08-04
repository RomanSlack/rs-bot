# How checks fail here

This project's whole method is to catch mistakes in the model, because the
model is cheap and a reprint is a fortnight. That method has one weakness, and
it is not the one you would expect: **the checks are less trustworthy than the
parts.**

Every entry below is something that actually happened in this repo, most of it
inside two days. They are written as patterns rather than as history, because
the specific bug is never the one you will hit next. Read this before you trust
a green result, and before you write a new check.

---

## 1. A check that cannot fail

The oldest rule in the repo and still the most productive. `cad/fasteners.py`
reported "every servo screw runs along its servo's mounting axis" in green for a
week while three joints had screws that never touched a servo. It was not lying.
It checked that a hole was PARALLEL to a shaft, and read as though it checked
that the hole ARRIVED at one.

**The test:** break the thing on purpose and confirm it screams. Every check
here that was control-tested that way survived; two were rewritten because the
first version passed a deliberately broken input.

## 2. A check nobody runs

`cad/assemble_check.py` had `stack()` and `sweep()`, and
`uv run python -m cad.assemble_check` called neither. The flip sweep on real
solids - written the day before, and described in that day's report as "the
first time this robot has been swept on its actual geometry" - only ever ran
when somebody imported it by hand and remembered to.

A check nobody runs is not a weaker check, it is an absent one, and it is worse
than absent because the documents list it among the checks that pass.

## 3. A check whose INPUT is wrong

The one that cost the most. Every FEA in this repo ran on meshes containing
inverted elements - tets with a flipped Jacobian - for weeks. Six of the seven
parts. An inverted element contributes negative stiffness, so the matrix is not
positive definite, and **the solves mostly converged anyway**: they returned
numbers that looked like results and went into every strength document here.

On valid meshes the roll bracket is at 99% of PA6-CF, not the 70% that had been
published. The error was entirely in the direction that makes a part look safe.

> A check validates its logic and forgets its input. That is the same failure as
> #1, one layer down: `fasteners.py` checked a hole's direction and not its
> destination; the FEA checked its solver and not its mesh.

**Ask of any check: what does it consume, and who checks that?**

## 4. A threshold set to the number you already have

`test_every_screw_can_be_reached_when_it_is_fitted` asserted
`len(blocked) <= 4`, with a paragraph explaining why four unreachable screws
were acceptable. They were not unreachable at all - the check that produced the
4 was broken. The bound had been set to whatever was being reported, which is a
screenshot of a bug, not a threshold.

Its opposite is also on record and is the good case: a tight assertion
(`mass == 1.875 +/- 0.003`) is the one that would have caught a 32 g divergence
immediately, while a comfortable range (`1.83 <= m <= 1.90`) absorbed it in
silence for a fortnight.

**A bound wide enough to be comfortable is a bound that will absorb a real
fault.**

## 5. A number in two files

The repo's own rule, and it kept being broken. `SHAFT_X` = 12.5 mm was wrong in
FOUR separate files in two days: the sim, `cad/toolaccess.py`,
`cad/envelope.py`, and again in `ankle.ROLL_SV`. Each time it was fixed
somewhere, nobody asked who else had a copy.

**When a number turns out to be wrong in one file, the question is not whether
it is fixed. It is who else has a copy.**

The fix is a pure-data module both halves import: `cad/servo_dims.py`,
`cad/wheel_dims.py`. No build123d, so the physics can import it too.

## 6. A constant with no uses

`WHEEL_TIRE_T = 0.008` in the sim said the tread was 8 mm. The CAD builds 3 mm.
It survived because **nothing read it** - one definition, zero uses, wrong by
5 mm, in the file that describes the robot to the physics. `SEG_MASS` was the
same: dead, stale, and sitting under a comment reading "Derived, not estimated",
which sent an agent chasing the wrong constant for an hour.

A constant with no uses is not harmless. It is a wrong answer waiting for the
first person who trusts it, and it reads as verified because it sits among
numbers that are. It is the mirror image of #1: a fact nobody consults is never
contradicted.

## 7. Incrementing a number you have not checked

`docs/order-sheet.md` said 183 g of printed structure. Updating it for five new
cradles, an agent wrote 197 g by adding to the number already there. It is
341 g. The 183 was the legs only, quoted as if it were the robot; the chassis
alone is 135 g and had never been in it.

The error was not the increment, it was the base. **Adding to a stale number
makes it look freshly checked and puts your name on it.**

## 8. The reason a check gives for not checking something

`cad/twin.py`'s bought-box check shipped with one constant excused, on the
stated grounds that its frame was not pinned against the sim. That sounded
rigorous. It was disproved by the first line of the measurement: two of the
three axes matched to a thousandth of a millimetre, which is not something an
unverified frame conversion does. The third axis was 6.8 mm out.

**An exclusion deserves the same suspicion as an assertion.** Name what is not
covered, in the check's own output, so the excuse can be argued with.

## 9. Prose does not re-run itself

`docs/road-to-order.md` blocked an order on two thin margins, 86% and 94%. Both
numbers were stale before anyone worried about them: the parts had been
redesigned since. An item can sit on a blocking list long after the thing it
describes has stopped being true.

Status reports are worse, not better, because they are dated and therefore look
authoritative. One recorded "mass went 1875 g to 1904 g" - into a document,
while the model stayed at 1875 for a fortnight.

**Any number in a document is a copy. See #5.**

## 10. Nondeterminism makes every other check unreliable

gmsh's parallel mesher gave 49217, 49278 and 49370 nodes from the same STEP with
identical settings. While that was true, "does this converge" was a coin flip
and nothing downstream could be diagnosed at all. Single-threaded meshing costs
time and buys the ability to ask a question twice.

**A check whose answer changes between runs is not a check.** Fix that first;
everything else is guesswork until you do.

## 11. Look at it

Three faults were found by a human looking at a screen and none of them by
anything measuring: servos drawn as bounding boxes, a belt drawn as its
clearance envelope, and a shaft floating in front of the right tyre because
MuJoCo reports that joint's axis negated and the code trusted the sign.

The last one is the point. The geometry the CHECKS ran on was fine. The viewer
was assembling a different robot, and `0 interfering pairs` was true of both.

> **A check can measure the right number and still be looking at the wrong
> robot.**

Screenshot what you build. `cad/fitview.py` is the assembly viewer; headless
Chrome will render it, and it takes a minute.

## 12. An envelope is not a part

`cad.belt.swept()` is a box the width of the big pulley spanning the whole
centre distance. That is the correct shape for "what must nothing else occupy"
and a useless shape for "what does this look like". Likewise the sim's servo
BOXES are the conservative shape for interference and hide the horn end, the
boss end and the connector.

**An envelope is a claim about empty space. A part is a claim about material.**
Know which one you are holding, and never let a viewer show one while a check
uses the other without saying so.

## 13. Model the bought parts, not just the printed ones

The shin and the ankle yoke had nothing between them, because the joint - a
623ZZ bearing and a 3 mm shaft - existed only as HOLES. So did every non-servo
bought part: bearings, shafts, pulleys, belts, horns. Ten line items, about $80,
none of them drawn.

Nothing could then ask whether a shaft was long enough for its bore. Drawing
them produced the answer in a minute: 14.0 mm and 15.5 mm, against an order
sheet that said "30 mm, or cut from stock".

**The parts that do the assembling are the ones nobody checks.**

## 14. Measure the thing, not something adjacent to it

Two wrong ways to measure one shaft, in one afternoon:

- probing MATERIAL along the joint axis gave 120 mm, because it ran the length
  of the wheel and the roll servo. A shaft passes through HOLES, not material.
- scanning for coaxial bore FACES missed the shin's bore entirely and gave
  10 mm for a 14 mm joint. The bore is real: 350 mm3 of material rings it and
  its wall is exactly `SHAFT_R` from the axis.

Both were plausible, both were confidently wrong, and the second failed silently
in the safe-looking direction. When a measurement surprises you, measure it a
second way before believing it.

## 15. Symmetry: solve one side, mirror it

`cad/wiring.py` reached this first - choosing each side's cable pairing by
"whichever two are closest" picked a different pair on the right - and the roll
shaft repeated it, because MuJoCo reports the right ankle-roll axis negated.

The robot is symmetric. **Solve the left, mirror it, and never read a per-side
sign out of the physics** when the CAD already states where something lives.

---

## 16. An index that quietly changed meaning

A test injected a fault by writing `m.eq_data[0]` and `[1]`, which were the two
ankle tendons on the day it was written. Then the parallelogram's own bodies
went into the sim and added three equalities per leg, so index 1 became a
ROD's coupling. The test went on running, went on passing, and went on
reporting that the robot survived build offsets that used to floor it - because
it was no longer injecting the fault it named.

Nothing about the test changed. The world it indexed into did.

**Anything addressed by position is a bet that nothing will ever be inserted
before it.** `_write_stance` in `src/rsbot/model.py` was written for exactly
this reason - the lash joints interleave, so a hand-written qpos vector
scrambles the moment backlash is switched on - and the lesson had not been
carried across to the equalities. Look up by NAME.

## 17. A constraint that mj_forward does not solve

`fitcheck.pose()` clears qpos, writes the joints it knows by name, and calls
`mj_forward`. That is the whole posing mechanism behind every geometric check
in the repo. When the linkage's rods and idler became real bodies held by
equality constraints, `pose()` did not know their names, so it left them at
zero - the rods hanging straight down off their pins - and `mj_forward` did not
correct it, because forward kinematics does not run the constraint solver.

It was caught by a check written the same hour, which reported 32.8 degrees of
angular divergence with the POSITIONS exact to the micron. That signature is
worth remembering: right place, wrong attitude, means something set a position
and not an orientation.

**A constraint is not a fact about the model, it is a thing the solver does.**
If you did not run the solver, it has not happened.

## 18. "One connected group" is not "assembled"

Two parts count as joined when the gap between them is zero. A gap of zero is
also what you get when they are twenty-five millimetres INSIDE each other, so a
connectivity check happily reports one object for a robot that is
interpenetrating. It was the control test for `connected()` that showed this:
shoving a rod deep into its neighbours did not disconnect it, and only moving
it 250 mm clear did.

Connectivity is also only REACHABILITY. A part hanging on by a single 0.04 mm
graze passes, and so does a pin that enters one of the two parts it joins and
misses the other - which is exactly what this repo's linkage pins were doing,
engaging 3 mm of a 6 mm idler and none at all of the chassis fin, while every
check reported one connected group.

**Three questions, and they need three checks:** does it touch (connectivity),
does it touch without overlapping (interference), and is it HELD - how much of
the pin is actually inside the bore. `cad.linkage.assembled()` is the third
one, and it is the only one that found anything.

## 19. A bounding box cannot tell a fork from a slab

The thigh was one box in the physics model, 20 x 16 x 98, and it had been that
box for the whole life of the project. `cad.twin` measured it at **71.4% backed
by CAD material** and passed it, because the threshold for "ok" is lower than
that and 71% reads like a rounded-off part.

It was not a rounded-off part. The real thigh does not run straight down: it
reaches AROUND the knee servo to a fork on the INBOARD side, because the
outboard band is where the shin's hub has to be. The box described the outboard
spine running down to z = -102, and the real outboard material stops at -74.
The missing 28 mm out of 98 is the 28.6% - the box was describing material on
the wrong side of the leg.

**What finally showed it** was not twin. It was a new part: the idler's stub, a
boss on the knee axis. `fitcheck.py` had it 3.3 mm inside the thigh. On the
real solids the two are **22.11 mm apart** at every pose, so the collision was
entirely against material that does not exist.

Then the fix went wrong the same way twice more, and both are worth keeping:

- Replacing the one box with an inboard **plate** put it 10 mm inside the knee
  servo, while the real solids intersect at 0 mm3. The plate came from probing
  the solid and reading each slice's **bounding box**, and the y profile is
  continuous straight across the servo pocket - a fork and a slab have the same
  bounding box. The x profile is what distinguishes them: material at
  |x| = 12.5..15.3 with air between. Three boxes, not one.
- The fork wall then sat 0.5 mm inside the servo. `cad/thigh.py` says
  `SERVO_Y_LO = SERVO_Y_HI - SERVO_H  # inboard face, y = -20.4` and that
  comment has been wrong since `servo_dims.HEIGHT` was corrected to 36.50: the
  face is at -21.5. The comment was not load-bearing on its own. It became
  load-bearing the moment a second file was written off it.

**The lesson is not "raise twin's threshold".** It is that a percentage-backed
number says how MUCH of a box is wrong and says nothing about WHERE, and
"where" is the whole question when a part reaches around something. A box that
is 71% backed and 29% on the wrong side of the leg scores the same as a box
with a chamfer it does not model.

## 20. A check that got greener because its subject left the set

`cad.linkage.ground()` measures how close the linkage comes to the floor in
foot mode. It walks `placed()`, which walks `SOLIDS`. When the yoke arm stopped
being a linkage part and became a feature the yoke prints, it left `SOLIDS` -
correctly - and the check went from **8.1 mm to 13.0 mm**.

Both numbers were true and neither was the point. The arm really did rise, from
its section going 8 to 6 and its root moving off the ankle-pitch shaft bore. But
about half that move was the check no longer looking at the lowest thing the
linkage puts near the floor. The arm is the entire reason the function exists:
its docstring says so, and says the inboard route for stage 2 was ruled out
because the only arm that cleared the roll bracket dropped to 1 mm.

**The tell is the direction.** A refactor that moves a part from one owner to
another should not change a measurement. When a number improves on a change
that was supposed to be neutral, the question is not "what got better" but
"what is no longer in the loop". `mounts_placed()` puts the three mount
features back in front of `ground()`, for measurement only, and the lowest part
it reports is the arm again.

The same session had the mirror image of this: the whole-robot mass assertion in
`tests/test_foot.py` went stale for the third time on a change that was correct,
because it was a literal. It now derives from `cad.masses.table()`. A number
that goes stale every time something legitimate happens is not a tight check, it
is a second copy.

## 21. A global answer to a local question

"Is the robot one connected object?" has been green in three different checks
for the life of this project. `fitcheck.connectivity()` says one group at 4 mm,
`cad.assemble_check.connected()` says one group at 0.05 mm on the real solids,
and `tests/test_foot.py` asserts both.

All three were true while **four of the six servos were held by nothing** - the
two hip servos 0.400 mm off the chassis and the two roll servos 0.400 mm off the
ankle yoke, touching no part of the robot anywhere.

They stayed green because union-find over the whole robot at once cannot ask a
local question. Each floating servo is still joined to the assembly through its
own output horn, and a horn touching a servo is a real contact, so the graph
stays connected. What is broken is not the graph. It is that a servo and the
part it seats against belong to the SAME RIGID BODY, move as one lump, and have
nothing between them.

`cad/floating.py` asks the two local questions instead:

- **a rigid body has to be one piece.** Union-find WITHIN a body, at a seam
  rather than a running clearance.
- **a connector has to reach both ends.** A shaft touching one body drives
  nothing.

The two have opposite tells, which is why one check could not do both: a
bracket that touches one body is fine, a shaft that touches one body is not.

**Two traps in writing it, both of which produced a confident wrong answer:**

- Reach has to be TRANSITIVE. The first version reported all four ankle shafts
  as reaching nothing, because a shaft does not touch the parts it joins - it
  touches the BEARING pressed into them. Four false alarms is how you teach
  somebody to ignore a check.
- The control test has to push in a direction that actually separates. Nudging
  the knee servo 2 mm into its cradle left the gap at 0.000, because the thigh
  wraps it on three sides. The check looked broken and was not.

**And note what the fix to one of these was already called.** `cad.fasteners`
has been reporting "worst is 1.30 deg" of servo capture preload for weeks, and
that 1.3 degrees IS this 0.4 mm, measured from the other end. Two checks
describing one fault in units that do not obviously match is its own way of
staying invisible.

## The shape of all of them

Every entry here is the same mistake in different clothes: **something was
believed because it was displayed.** A green line, a printed table, a dated
report, a constant sitting among verified constants, a rendered picture.

The question that catches them is not "is this check passing". It is:

> **What would this look like if it were wrong, and would I be able to tell?**

If the answer is "it would look exactly like this", you have not checked
anything yet.
