# rs-bot

A desk-scale wheeled biped whose wheels rotate flat to become its feet.
~43 cm tall, ~1.80 kg.

It balances on two wheels like a tiny Handle, and when it needs to hold still
or manipulate something, each ankle rolls 90 degrees: the wheels lie down and
their 80 mm faces become the soles. It then stands there statically with the
balance loop switched off. That transition is the point of the project - a
wheeled biped that can stop fighting its own balance loop before it reaches for
something - and the wheel-as-foot trick means it costs one joint per leg and no
extra parts.

Nothing here is bought yet. Everything runs in MuJoCo.

## The point: one shot

**The digital twin is the deliverable, not a preview of it.** The goal is a
model so exactly 1:1 with reality that the physical build happens once: parts
ordered once, printed once, assembled once, working. No revision rounds.

That is a real constraint rather than an ambition. The operator has no printer,
so every part comes from a service at real money and one to two weeks per
attempt, and a part that has already been built around is worse than that. The
cheapest place to find a mistake is in the model, and the only mistake that
costs nothing is the one caught before the order.

So the rule the whole repo is organised around:

> **Anything the model asserts about the real robot has to be traceable to
> something measured.** A number that came from a product listing, a
> screenshot, or somebody's guess is a bug that has not surfaced yet.

This is why the verification layer in `cad/` is bigger than the parts it checks:
interference and clearance through both poses, fastener specification, driver
access, tolerance stack-up, cable routing, FEA in five materials, printability.
It is also why the status reports read as lists of things that turned out to be
wrong. Finding those is the work.

Two examples of the failure mode, both real and both from 2026-08-01. The wheel
carried a Ø20.00 pocket for a horn that measures Ø19.93, which would have made
most wheels unable to accept the part they bolt to. And forty M2 screws went
into servo case holes whose positions no drawing publishes, with four different
parts each guessing a different pattern. Neither was visible until somebody put
calipers on the real servo.

## Status

**Stages 0 and 0b (sim) - done.** Balances, drives, rejects shoves, and flips
its wheels flat to stand for 87 s with the controller off.

Gear backlash is modelled, and it is the finding that most changed the plan.
The balancer needed much lower gain plus a low-pass on the wheel command,
without which it limit-cycles through the deadzone. The passive foot stance
needed a low-gain ankle loop too, until the ankle servo was deleted: a passive
parallelogram holds the foot level mechanically and does it better than the
loop did, standing 10/10 at 4 deg of lash where the servo managed 2/10 at 2.
See [docs/backlash.md](docs/backlash.md) and
[docs/deleting-the-ankle-pitch-servo.md](docs/deleting-the-ankle-pitch-servo.md).

Stage 1 hardware has not started.

## Roadmap

Each stage has a hard exit criterion. Hit it and move on; don't polish stage N
because stage N+1 is scary.

| # | Stage | Exit criterion | State |
|---|-------|----------------|-------|
| 0 | Sim balancer | Rejects a shove, attitude recovery < 1 s | **done** |
| 0b | Sim foot transition | Flips to foot mode and stands, balancer off | **done** |
| 1 | Legs in hardware | Stands 5 min unattended, survives a poke, drives, squats | |
| 2 | Feet in hardware | 20 consecutive transitions both directions, zero falls | |
| 3 | Head camera | Teleop drive from the camera feed alone | |
| 4 | One arm | Picks up a can, 8/10 attempts | |
| 5 | Second arm | Autonomous policies | |

## Design commitments

These are fixed up front so stage 4 doesn't invalidate stage 1.

- **The whole skeleton is designed now, built in pieces.** The sim model already
  carries the arms, head and compute as torso ballast, so the legs are tuned
  against final inertia (1.80 kg) rather than a bare stage-1 mass.
- **STS3215-class serial bus servos throughout.** 2.9 N.m stall. The leg joints
  run position mode; the wheels run continuous rotation, which means the
  balancer gets a *velocity* command, not a torque. The controller is built
  around that constraint rather than pretending otherwise.
- **The wheel is the foot.** 5 DOF per leg - hip pitch, knee pitch, ankle
  pitch, ankle roll, wheel - but only FOUR servos: ankle pitch is driven by a
  passive parallelogram off the hip and knee, not by a motor. Nothing deploys, nothing folds out. Two earlier mechanisms
  with separate feet both failed for the same reason, recorded in
  docs/stage-0b.md.
- **No hopping.** Bouncing needs real torque control at ~1 kHz and QDD ankles.
  Static feet mode gets most of the value; hopping is a different robot.

## Layout

```
src/rsbot/model/rsbot.xml   MJCF model
src/rsbot/model.py          loading, leg IK, foot geometry
src/rsbot/balance.py        the balancer
src/rsbot/transition.py     wheel mode <-> foot mode state machine
src/rsbot/sim.py            headless rollouts, shove test
serve.py                    live sim in the browser: drag to orbit, scroll to zoom
render.py                   render a demo run to renders/*.mp4
scale.py                    true-scale still beside a 5 ft 9 in person
fitcheck.py                 do the real parts actually fit? (oriented-box audit)
tune.py                     coordinate-descent gain search
view.py                     native viewer (needs a working GLX display)
cad/shin.py                 build123d proof of concept: STEP + MuJoCo mesh
cad/linkage.py              the passive ankle parallelogram, drawn and swept
cad/linkage_dims.py         its geometry, with no build123d, so the sim can
                            import it too
cad/linkage_anim.py         it moving, beside a leg that has none
cad/fitview.py              the assembled robot in the browser, every gap
                            measured: toggle parts, explode, both poses
cad/servo_view.py           the servo model against the part on the desk
tests/                      exit criteria as executable checks
```

## Run it

```bash
uv run python -m pytest tests -q   # all exit criteria, ~20 min (the FEA)
uv run python serve.py             # live sim at localhost:8781, drive it around
uv run python -m cad.fitview       # the assembly at localhost:8783, gaps measured
uv run python fitcheck.py          # part interpenetration audit, both modes
uv run python render.py            # render a demo to renders/
uv run python tune.py              # re-tune balancer gains
```

Latest status report: [status/2026-08-03-status.md](status/2026-08-03-status.md).
Each one leads with what turned out to be wrong that day, which is the fastest
way back into the project.

## Documents

**What the sim proved**

- [stage-0.md](docs/stage-0.md) - the balancer numbers
- [stage-0b.md](docs/stage-0b.md) - the foot transition, and the two mechanisms
  that failed before it
- [backlash.md](docs/backlash.md) - the gear-lash results, and the finding that
  most changed the plan
- [hardware-readiness.md](docs/hardware-readiness.md) - actuator and
  control-rate margins. Are STS3215s enough, and how fast must the loop run

**The parts**

- [cad.md](docs/cad.md) - why build123d, and the route from sketch to real CAD
- [porting-a-link.md](docs/porting-a-link.md) - the method used to port each
  part, and the clearance rules every part is checked against
- [real-parts-in-sim.md](docs/real-parts-in-sim.md) - putting the CAD solids
  back into the physics model
- [materials.md](docs/materials.md) - what to print in, and why
- [mass-budget.md](docs/mass-budget.md) - where the 1.80 kg goes
- [stress.md](docs/stress.md) - FEA, per part, in five materials
- [assembled-strength.md](docs/assembled-strength.md) - the leg solved as one
  solid rather than part by part

**Buying it**

- [order-sheet.md](docs/order-sheet.md) - **the file that has to be correct on
  the day money is spent.** Quantities, part numbers, prices
- [bom.md](docs/bom.md) - the real parts, and what modelling them at full size
  revealed
- [road-to-order.md](docs/road-to-order.md) - what still blocks an order
- [before-you-order.md](docs/before-you-order.md) - what has been verified, and
  against what

**How this project goes wrong**

- [how-checks-fail.md](docs/how-checks-fail.md) - **read this before trusting a
  green result.** Seventeen ways a check here has lied, all of them things that
  happened. The parts are in better shape than the things that verify them

**Questions settled, so they stay settled**

- [deleting-the-ankle-pitch-servo.md](docs/deleting-the-ankle-pitch-servo.md) -
  could a passive four-bar replace the ankle-pitch servo? Yes, and it stands
  10/10 in foot mode where the servo now manages 2/10. Drawn as a mechanism
  too: it fits, it beats the gear lash it replaces, and it needs one
  length-adjustable rod that nobody had thought of
- [deleting-the-roll-servo.md](docs/deleting-the-roll-servo.md) - could the
  wheel motor drive the ankle roll through a clutch? No, and the reason is not
  the one you would guess
