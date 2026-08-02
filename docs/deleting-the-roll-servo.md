# Deleting the ankle roll servo

> Could the wheel motor drive the ankle roll through a clutch, since the two
> never need to move at the same time? It would save two servos.

Asked 2026-08-01. The answer is **no for the clutch, yes for the premise**: the
roll joint really is over-actuated, but the saving comes from a latch and an
over-centre spring, not from sharing the wheel motor. Written down because it is
a good idea that fails for a non-obvious reason, and a good idea that fails
quietly gets re-invented.

## The premise is right

Two degrees of freedom that are never used in the same instant is exactly the
case where sharing an actuator pays. And the roll is better than that: it is
**binary**. Two positions, 0 and 90 degrees, nothing in between is ever
commanded. A binary joint does not need a servo, it needs a bistable actuator.

The numbers back it up. Ankle roll is the least-worked joint in the robot
(`docs/hardware-readiness.md`):

| joint | RMS N.m | peak N.m |
|---|---|---|
| knee | 0.36 | 2.90 |
| wheel | 0.38 | 2.90 |
| hip | 0.08 | 1.68 |
| ankle pitch | 0.07 | 1.63 |
| **ankle roll** | **0.18** | **0.92** |

And the work is trivial: **1.13 J** flipping down, **1.99 J** coming back, over
0.79 s. About 2.5 W. An STS3215 is 2.9 N.m of stall torque, sized to HOLD,
doing a job that needs two joules.

## Why sharing the wheel motor does not work

**The wheel motor is not free during the flip. It is at its busiest.**

`src/rsbot/transition.py` keeps the balancer running through FLIP. The outer
loop is only dropped past 50% of the roll, and the wheel command is not unloaded
until STAND. `docs/stage-0b.md` says what that means physically:

> "Contact is never broken and the balancer keeps authority the whole way down."

The manoeuvre is a race between wheel authority falling off as cos(roll) and the
support polygon growing as the discs flatten. This is a two-wheel balancer:
during the flip the wheels are the only thing holding it up. The one moment you
would want to borrow that motor is the one moment it cannot be spared.

**And the fix costs more than it saves.** The way to free the wheel motor is to
unload the leg, which is exactly what the real robot does. AgiBot's X2-N flips
one leg at a time, in the air, so the leg being flipped carries no weight.
`docs/stage-0b.md` already costed that for us: it needs a hip ROLL joint to shift
weight onto one leg.

    two more servos, to free up two servos

Net zero, plus a clutch, plus a mid-manoeuvre torque handover.

## What the prize actually is

| | |
|---|---|
| mass | 2 x 55 g = 110 g of 1904 g, **5.8%** |
| cost | 2 x $16 = **$32** of $675-940 |

Real, and worth an afternoon. Not worth a mechanism that can drop the robot.

## What to do instead

Three things stack, and the first is already on this project's wish list for a
different reason.

**1. A latch at each end state.** `docs/stage-0b.md`, on the X2-N teardown:

> "a clip that seats at each end state takes the driving loads off the ankle
> roll servo AND removes that joint's backlash in exactly the two poses where it
> hurts."

Foot mode is the pose that gear lash breaks (`docs/backlash.md`), so a detent
there is worth more than a better servo. With a latch, the roll actuator stops
needing to hold anything and only has to move the joint.

**2. The mechanism is ALREADY over-centre.** The axle climbs 1.76 mm at 16.7 deg
of roll, then falls 29.8 mm. That hump is what makes the two directions
different jobs:

| | climb | then | net energy at 2.05 kg |
|---|---|---|---|
| wheel -> foot | 1.76 mm | falls 29.8 mm | **0.035 J** |
| foot -> wheel | 29.8 mm | - | **0.599 J**, 17x more |

A spring stable at both ends means the actuator supplies only the crossing
energy, never the holding torque.

**3. There is a 3.5x speed surplus to trade for torque.** The flip needs 90 deg
in 0.79 s, about 114 deg/s. An STS3215 does roughly 400 deg/s. Gear that down
3.5:1 and a 0.3 N.m servo becomes 1.05 N.m, which covers the 0.92 N.m peak.
Around 15-20 g of servo plus 10 g of reduction, against 55 g.

**The honest ceiling on this.** The 2026-07-28 status found that at 1.8 kg the
integrated hobby servo is *mass-optimal, not a compromise*, because 55 g buys
motor, gearbox, driver and encoder in one case, and torque density is roughly
linear in mass in this class. So the saving comes from the latch and the spring
doing the HOLDING, not from finding a better actuator. Expect 25-30 g per leg,
not 55.

## The one place the clutch idea does have legs

**UNFLIP.** In STAND the robot is statically stable on flat feet with the
balancer off, so at that instant the wheel motors genuinely are free. And
foot-to-wheel is the hard direction: 0.599 J against 0.035 J, seventeen times
more, because it climbs the whole 29.8 mm back. Measured servo work agrees,
1.99 J against 1.13 J.

So a clutch that assists only the breakout and the climb, then hands back as the
balancer fades in, is not obviously impossible.

It is also a torque handover partway through a manoeuvre on a robot that is
falling into balance as it happens, which is the shape of thing that drops
robots. Build the latch, measure it, and only then decide whether anything else
is needed.

## What would change the answer

- **A squat-to-chassis sequence.** If the robot rested its chassis on the ground
  before flipping, the wheels would be free and the balancer off, and a clutch
  would have a real window. That is a different behaviour and it has to get back
  up again, but it is the one sequence change that makes the original idea work.
- **A bigger robot.** `docs/stage-0b.md` notes that flipping both legs at once
  only works because our over-centre barrier is tiny, and that one-leg-at-a-time
  becomes increasingly necessary as the robot grows. A robot that needs hip roll
  anyway gets the free wheel motor as a side effect, and then this trade flips.

## Status

Not doing the clutch. The latch is worth doing on its own merits and is already
justified by backlash; do that first, measure the roll joint's real loads with
it fitted, and revisit the actuator size afterwards.
