---
description: Resume the 1:1 digital-twin loop
---

# What this project is actually for

rs-bot's deliverable is **a digital twin exact enough to build from once**.
Parts ordered once, printed once, assembled once, working. No revision rounds.

That is a hard constraint, not an ambition. The operator has no printer, so
every part comes from a service at real money and one to two weeks per attempt,
and a part that has already been built around is worse than that. The cheapest
place to find a mistake is in the model. The only mistake that costs nothing is
the one caught before the order.

> **The rule: anything the model asserts about the real robot has to trace to
> something measured.** A number that came from a product listing, a screenshot,
> or somebody's guess is a bug that has not surfaced yet.

The twin is really **two models pretending to be one**: `src/rsbot/model.py`
builds the robot out of boxes for the physics, `cad/` builds the same robot out
of solids for the printer. Nothing forces them to agree, and each half is
internally consistent, so drift is invisible from inside either one. The sim
balances. The parts fit. Neither notices they describe different robots.

# The loop

Run it. Do not ask whether to start.

1. **Establish the state.** Run the fidelity checks below, cheapest first. Do
   not trust the last report in the conversation or in `status/` - re-derive it.
2. **Rank what is wrong by what it would cost at the bench.** A servo that is
   not on its joint outranks a fillet. A part that cannot be printed outranks a
   part that is heavy.
3. **Take the top one.** Fix it, then verify with the check that found it, then
   re-run anything downstream of the change.
4. **Record why**, in the file, in the voice the rest of the repo uses: what was
   wrong, how it was found, what it would have cost, and what is still open.
5. **Repeat.** Stop to ask only when a fix needs a decision that changes the
   robot's proportions, mass budget, or kinematics.

# The checks, cheapest first

```
uv run python -m cad.drives          is every servo's shaft on the joint it turns
uv run python -m cad.twin            every sim box backed by CAD material, and
                                     every CAD copy of a bought part still the
                                     same box the sim has
uv run python -m cad.servo           the bench and drawing numbers, and what is open
uv run python -m cad.fasteners       every screw specified, and pointing at something
uv run python -m cad.wiring          can the cables physically get there
uv run python -m cad.floating        what is held by nothing: every rigid body
                                     one piece, every shaft reaching both ends
uv run python -m cad.assemble_check  interference, seating, the worst-case
                                     tolerance stack, and the flip swept on
                                     real solids. Four checks, a few minutes
uv run python fitcheck.py            interference through all 26 flip poses
uv run python -m cad.toolaccess      can a driver reach every screw
uv run python -m cad.printability    overhangs, thin walls, bridges
uv run python -m pytest tests -q     everything, about 20 minutes, run last
uv run python serve.py               look at it move: localhost:8781
uv run python -m cad.fitview         look at how it goes together, with every
                                     gap measured: localhost:8783
```

`cad.floating` is the one to run when something LOOKS wrong and everything
reads green: the connectivity checks answer "is this one object" and stay green
while a servo hovers off the plate it bolts to. It found four doing exactly
that, and cad/fitview.py paints whatever it reports red.

`cad.drives` and `cad.twin` are the two that exist specifically to catch
sim-versus-CAD divergence. They are the newest and the least exercised, so
extend them when you find a class of error they missed.

# What counts as a divergence

- A servo positioned by its **case** rather than its **output**. The shaft sits
  12.5 mm off the case centre, so these are never the same point.
- A sim box with **no CAD material behind it** - the balancer is tuned against
  structure that will not exist, and the clearance checks route around it.
- A sim box **smaller than** the CAD part it stands for. The dangerous
  direction: clearance gets checked against something thinner than reality.
- **A number that appears in two files.** One of them is already wrong or will
  be. Import it, do not copy it.
- A dimension traceable only to a listing, a render, or a guess.
- A check that **cannot fail**, which is worse than no check because it reads as
  evidence. `cad/fasteners.py` reported green for a week while three joints had
  screws that never touched a servo: it checked that a hole was parallel to a
  shaft, not that it arrived at one.

# Working agreements

- Measure rather than assume, and say where the number came from.
- Prefer a check that can fail over a comment that says it is fine.
- One source of truth per number. A second copy is a second place to be wrong.
- Fix the check as well as the bug when the check should have caught it.
- Match the surrounding voice: plain, direct, no em-dashes, and record the
  reasoning that would otherwise be re-litigated next week.

# Before you trust a green result

`docs/how-checks-fail.md` is twenty-one ways a check in this repo has lied,
all of them things that actually happened. The short version:

- a check that cannot fail, and a check nobody runs
- a check whose INPUT nobody validated - every FEA here ran on broken meshes
  for weeks and the solves converged anyway
- a threshold set to the number the bug was already producing
- a number copied into a second file, which then drifted. SHAFT_X was wrong in
  four files in two days
- a dead constant that was also wrong, believed because it sat among good ones
- a viewer showing a different robot from the one the checks ran on

> **What would this look like if it were wrong, and would I be able to tell?**
> If the answer is "exactly like this", nothing has been checked yet.

# Where the context is

- `README.md`, "The point: one shot"
- `status/` - dated reports, newest last, each one leading with what turned out
  to be wrong
- `docs/road-to-order.md` - what still blocks an order
- `docs/order-sheet.md` - the file that has to be correct on the day money is
  spent, and which goes stale every time a part or fastener changes

Start by running the checks and telling me, briefly, what is currently wrong and
what you are taking first.
