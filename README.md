# rs-bot

A desk-scale wheeled biped whose wheels rotate flat to become its feet.
~43 cm tall, ~2.05 kg.

It balances on two wheels like a tiny Handle, and when it needs to hold still
or manipulate something, each ankle rolls 90 degrees: the wheels lie down and
their 80 mm faces become the soles. It then stands there statically with the
balance loop switched off. That transition is the point of the project - a
wheeled biped that can stop fighting its own balance loop before it reaches for
something - and the wheel-as-foot trick means it costs one joint per leg and no
extra parts.

Nothing here is bought yet. Everything runs in MuJoCo.

## Status

**Stages 0 and 0b (sim) - done.** Balances, drives, rejects shoves, and flips
its wheels flat to stand for 87 s with the controller off.

Gear backlash is modelled, and it is the finding that most changed the plan.
The passive foot stance needs a low-gain ankle loop to survive it, and the
balancer needed much lower gain plus a low-pass on the wheel command, without
which it limit-cycles through the deadzone. Both are built, and the full round
trip now works with 2-3 deg of lash. See [docs/backlash.md](docs/backlash.md).

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
  against final inertia (2.05 kg) rather than a bare stage-1 mass.
- **STS3215-class serial bus servos throughout.** 2.9 N.m stall. The leg joints
  run position mode; the wheels run continuous rotation, which means the
  balancer gets a *velocity* command, not a torque. The controller is built
  around that constraint rather than pretending otherwise.
- **The wheel is the foot.** 5 DOF per leg: hip pitch, knee pitch, ankle pitch,
  ankle roll, wheel. Nothing deploys, nothing folds out. Two earlier mechanisms
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
tests/                      exit criteria as executable checks
```

## Run it

```bash
uv run python -m pytest tests -q   # all exit criteria, ~8 s
uv run python serve.py             # live sim at localhost:8781, drive it around
uv run python fitcheck.py          # part interpenetration audit, both modes
uv run python render.py            # render a demo to renders/
uv run python tune.py              # re-tune balancer gains
```

See [docs/cad.md](docs/cad.md) for the route to real CAD, and
[docs/porting-a-link.md](docs/porting-a-link.md) for the method used to port
each part.

Latest status report: [status/2026-07-25-status.md](status/2026-07-25-status.md).

See [docs/bom.md](docs/bom.md) for the real parts, costs and what modelling
them at full size revealed,
[docs/hardware-readiness.md](docs/hardware-readiness.md) for the actuator and
control-rate margins,
[docs/backlash.md](docs/backlash.md) for the gear-lash results (the one that
changes the plan),
[docs/stage-0.md](docs/stage-0.md) for the balancer numbers,
[docs/stage-0b.md](docs/stage-0b.md) for the transition and the two mechanisms
that failed first, and [docs/mass-budget.md](docs/mass-budget.md) for where the
2.05 kg goes.
