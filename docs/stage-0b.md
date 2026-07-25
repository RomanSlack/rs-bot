# Stage 0b: the transition

**Status: DONE. The robot flips its wheels flat and stands on them with the
balancer completely off.**

## Result

    2.00s  WHEEL  -> SETTLE
    2.00s  SETTLE -> FLIP
    2.79s  FLIP   -> STAND      flip takes 0.79 s
    stands 87.2 s of a 90 s run, 7.8 mm drift, balancer off throughout

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
