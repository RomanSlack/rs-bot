# rs-bot

A desk-scale wheeled biped with deployable rocker feet. ~43 cm tall, ~1.9 kg.

It balances on two wheels like a tiny Handle, and when it needs to hold still or
manipulate something, it flips a sole down over each wheel and becomes a
statically stable standing robot. That transition is the point of the project:
a wheeled biped that can stop fighting its own balance loop before it reaches
for something.

Nothing here is bought yet. Everything runs in MuJoCo.

## Status

**Stage 0 (sim balancer) - done.** Stands, drives, rejects shoves.
Stage 1 hardware has not started.

## Roadmap

Each stage has a hard exit criterion. Hit it and move on; don't polish stage N
because stage N+1 is scary.

| # | Stage | Exit criterion | State |
|---|-------|----------------|-------|
| 0 | Sim balancer | Rejects a shove, attitude recovery < 1 s | **done** |
| 0b | Sim foot transition | 20 consecutive transitions both ways, no falls | next |
| 1 | Legs in hardware | Stands 5 min unattended, survives a poke, drives, squats | |
| 2 | Rocker feet | 20 consecutive transitions both directions, zero falls | |
| 3 | Head camera | Teleop drive from the camera feed alone | |
| 4 | One arm | Picks up a can, 8/10 attempts | |
| 5 | Second arm | Autonomous policies | |

## Design commitments

These are fixed up front so stage 4 doesn't invalidate stage 1.

- **The whole skeleton is designed now, built in pieces.** The sim model already
  carries the arms, head and compute as torso ballast, so the legs are tuned
  against final inertia (1.87 kg) rather than a bare stage-1 mass.
- **STS3215-class serial bus servos throughout.** 2.9 N.m stall. The leg joints
  run position mode; the wheels run continuous rotation, which means the
  balancer gets a *velocity* command, not a torque. The controller is built
  around that constraint rather than pretending otherwise.
- **No hopping.** Bouncing needs real torque control at ~1 kHz and QDD ankles.
  Static feet mode gets most of the value; hopping is a different robot.

## Layout

```
src/rsbot/model/rsbot.xml   MJCF model
src/rsbot/model.py          loading, leg IK, geometry constants
src/rsbot/balance.py        the balancer
src/rsbot/sim.py            headless rollouts, shove test
tune.py                     coordinate-descent gain search
view.py                     interactive viewer (needs a display)
tests/test_stage0.py        exit criteria as executable checks
```

## Run it

```bash
uv run python -m pytest tests -q   # stage-0 exit criteria, ~3 s
uv run python view.py              # watch it. W/S drive, SPACE shove, R reset
uv run python tune.py              # re-tune gains
```

See [docs/stage-0.md](docs/stage-0.md) for measured numbers and
[docs/mass-budget.md](docs/mass-budget.md) for where the 1.87 kg goes.
