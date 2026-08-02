# Stage 0b: the transition

**Status: DONE, both directions. The robot drives, flips its wheels flat,
stands with the balancer off, flips back, and drives on.**

## Result

    4.00s  WHEEL  -> SETTLE     decelerates, waits for a quiet moment
    5.43s  SETTLE -> FLIP
    6.22s  FLIP   -> STAND      flip takes 0.79 s
   10.22s  STAND  -> UNFLIP
   11.01s  UNFLIP -> WHEEL      and it drives away

**20 consecutive round trips, zero falls**, over 120 s of sim, travelling
8.67 m between transitions. That is the stage-2 hardware criterion, met in sim.

Standing still, it stands 87.2 s of a 90 s run with 7.8 mm of drift and the
balancer off. Slip during a stand is 2-4 mm.

| | |
|---|---|
| Flip rates that work | 0.5 to 8 rad/s (16x range) |
| Axle at rest | 11.8 mm, i.e. flat on the faces, not perched on the rims |
| Ground contacts in foot mode | `wheel_l`, `wheel_r` only |
| Static tilt margin | 9.1 deg |
| Tips at | 0.35-0.45 N.s fore/aft, 1-2 N.s sideways |

## The mechanism

The wheel **is** the foot. A 90 deg ankle roll stands the spin axis vertical,
the disc lies down, and its 80 mm face becomes the sole. Each leg is hip pitch,
knee pitch, ankle pitch, ankle roll, wheel.

This is the third mechanism tried and the first that works, because it is the
only one whose support polygon is **centred under the axle** - which is exactly
where the CoM already is when the balancer hands over. The two earlier designs
both died trying to buy fore/aft margin they could not afford.

| | flat plate on the axle | outrigger struts | wheel flip |
|---|---|---|---|
| dig during deploy | 11-28 mm | 0 mm | **1.76 mm** |
| goes airborne | yes | no | no |
| polygon vs CoM | behind it | +8.5 mm forward | **centred, +/-40 mm** |
| extra parts | sole + servo | strut + pad + servo | **none, just a joint** |
| result | falls, 60/60 configs | falls during shift | **stands 87 s** |

The 1.76 mm dig is the rim corner briefly leading as the disc tips. It peaks at
16.7 deg of roll, and after that the axle descends monotonically from 40 mm to
12 mm, so the robot simply settles 28 mm as it goes. Contact is never broken
and the balancer keeps authority the whole way down.

Ankle pitch holds the face level as the leg pitches, and `ankle_bias` tips it
deliberately - the ankle flex available in foot mode.

## How the real one does it (AgiBot X2-N)

There is a paper on exactly this robot, and it settles some questions.

**It is an actuated ankle roll joint, not a passive mechanism.** The paper:
"the mode transformation is achieved by reusing the ankle roll joint to
actively drive the orientation of wheel motors from horizontal foot-contact to
vertical wheel-contact form." So the approach here is the right one. Their
transitions take about 1 s; ours takes 0.79 s.

(Whether the roll joint needs a servo of its own at all, and whether the wheel
motor could drive it through a clutch, is worked through in
`docs/deleting-the-roll-servo.md`. Short version: not the clutch, because the
wheel motor is busiest exactly when you would want to borrow it, but the joint
is genuinely over-actuated and the latch below is most of the answer.)

**But there IS a mechanical latch.** X2-N carries "a retaining clip structure
and multiple contact interfaces to stabilize the wheel twist during
locomotion", plus a sliding slot that guides the ankle into place. That is a
lock for *holding*, not for driving - and it is worth copying here, for a
reason specific to our problem: a clip that seats at each end state takes the
driving loads off the ankle roll servo AND removes that joint's backlash in
exactly the two poses where it hurts. Foot mode is the thing lash breaks
(docs/backlash.md), so a detent there is worth more than a better servo.

**They transform one leg at a time, in the air.** Wheel-to-foot happens "while
rotating the ankle and wheel during the air time of each step". The leg being
flipped is unloaded, so the mechanism never lifts the robot's weight. That is
the "hop" you see in their videos, and it is the clean way to do it.

We flip both legs at once, standing on both wheels, which only works because
our barrier is tiny. Doing it their way would need a hip ROLL joint to shift
weight onto one leg, which we do not have - two more servos. Not needed at
this scale; increasingly needed as the robot gets bigger.

## The energy asymmetry, and which way is downhill

The axle rises 1.76 mm at 16.7 deg of roll and then falls to 12 mm. That hump
is an over-centre mechanism, and it makes the two directions completely
different jobs:

| | climb | then | energy at 2.05 kg |
|---|---|---|---|
| wheel -> foot | 1.76 mm | falls 29.8 mm | **0.035 J** |
| foot -> wheel | 29.8 mm | - | **0.599 J**, 17x more |

Measured servo work bears it out: 1.13 J flipping down, 1.99 J coming back.
Peak ankle-roll torque is 0.83 N.m per servo either way, against ~0.9 N.m
continuous and 2.7 N.m stall at 3S - so it fits, but the return is the
direction that works the servos.

Note this is **inverted** relative to X2-N, whose foot-to-wheel transition
"follows the potential-energy-descent principle due to the CoM depression in
wheel-legged mode". Their wheel mode is lower; ours is higher, because our
wheels are small relative to the ankle. So their free direction is
foot-to-wheel and ours is wheel-to-foot.

## Getting back out

`UNFLIP` runs the handover in reverse: authority grows back as cos(roll) while
the support polygon shrinks. Two things it has to get right, both of which are
the kind of bug that only shows up on the return trip:

- **Reset the balancer's odometry.** It has been idle through STAND, so its
  integrated position still holds wherever the robot stopped. Without a reset
  the position term hauls the robot back there the instant it re-engages.
- **Feed the wheels back in gradually** (`reload_time`). A near-flat disc
  scrubs rather than drives, so commanding full wheel speed at 90 deg of roll
  just fights the floor.

## Cost

+120 g, two extra servos (one ankle roll per leg). Total 1.93 -> 2.05 kg. That
buys the deletion of every separate foot part: no soles, no struts, no pads.

## The honest limitation

Foot mode is **passively** stable. It tips at 0.35-0.45 N.s fore/aft, while
wheel mode survives 1.4 N.s, because wheel mode can actively catch itself and a
rigid stance cannot. Measured peak lean before tipping is 8.3 deg against a
9.1 deg geometric limit, so the model matches the energy calculation exactly:
this is a real property, not a tuning failure.

The fix is an ankle strategy - use the ankle pitch servos to shift the centre
of pressure inside the foot, which is what a person does when nudged. The DOF
is already there. Stage 2 work, on hardware.

## Bugs found while building this

- **The shin collided with its own wheel, 26 mm deep.** Inserting the ankle body
  between shin and wheel made them grandparent/grandchild, and MuJoCo only
  auto-excludes direct parent-child pairs. Explicit `<exclude>` now.
- **The robot stood on its ankle brackets, not its feet.** The bracket geom hung
  16 mm below the axle; the wheel face is only 12 mm below. Anything lower than
  the face silently steals the entire 80 mm foot.
- **Capsule end caps count.** The shin capsule ran to the axle, and its 14 mm
  end cap reached below the wheel face. Trimmed to stop at the axle.

Each of these presented as a control failure and was really a geometry error.
That is the recurring lesson of this stage: when the robot falls over, check
what is actually touching the floor before touching a gain.

## Superseded designs

Attempt 1, a rocker sole pivoting on the wheel axle, is dead for a geometric
reason worth keeping: `corner_radius = hypot(face_r, half_len)`, and
`face_r >= 46 mm` because the plate had to cradle the wheel, so it always
jacked the robot 11-21 mm onto a single edge behind the axle, airborne, with
the balancer powerless. 60 valid configurations, zero survived.

Attempt 2, outrigger struts beside the wheel, fixed the dig completely and held
the critical state (wheel and foot in contact simultaneously, never airborne)
but capped forward margin at +8.5 mm and tipped during the body shift.

The rule both attempts taught, which the wheel-biped literature states
directly: a wheel-to-foot transition must pass through a state where wheel and
foot are both in ground contact. The wheel flip satisfies it trivially, because
the wheel and the foot are the same object.
