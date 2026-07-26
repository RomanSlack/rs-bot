# Bill of materials, and what modelling it at real size revealed

The sim now draws the actual parts at their actual dimensions. The visual
geometry carries no mass and no collision - mass still lives on the simple
shapes, moved to geom group 4 and hidden - so nothing here changes the
dynamics. What it does change is that packaging problems become visible.

## Parts

| Part | Qty | Size (mm) | Mass | Unit | Total |
|---|---|---|---|---|---|
| Feetech STS3215 C018 (12 V, 1:345) | 10 | 45.2 x 24.7 x 35.4 | 55 g | ~$14-16 | ~$150 |
| Wheel / tyre, 80 dia x 24 | 2 | 80 x 24 | 60 g | ~$8 | ~$16 |
| Raspberry Pi 5 | 1 | 85 x 56 x 17 | 45 g | ~$80 | ~$80 |
| 3S LiPo 2200 mAh | 1 | 105 x 34 x 24 | 180 g | ~$25 | ~$25 |
| TTL bus adapter | 1 | 50 x 30 x 10 | 10 g | ~$12 | ~$12 |
| IMU (BNO085 class) | 1 | 25 x 20 x 3 | 5 g | ~$25 | ~$25 |
| Printed PETG structure | | | ~400 g | filament | ~$15 |
| **Legs only, stage 1** | | | **2.05 kg** | | **~$320** |

Servo torque at 3S is worth noting: 30 kg.cm is the 12 V figure, and a 3S pack
sits at 11.1 V nominal, so expect about **2.7 N.m** rather than 2.9. RMS demand
is 0.36 N.m so this is comfortable; it only means the peaks clip slightly
sooner than sim says.

## Five servos per leg, and where they go

hip pitch, knee pitch, ankle pitch, ankle **roll**, wheel drive.

The thigh and shin each carry one servo at their proximal end plus a printed
bracket. The interesting part is the ankle, which carries **three**: pitch,
roll, and the wheel drive. That is 165 g and three 45 mm bodies clustered at
the end of the leg.

## Does it actually fit? `uv run python fitcheck.py`

MuJoCo cannot answer this: geoms in the same body never collide, and the
visual build is non-colliding by design. So `fitcheck.py` runs its own
oriented-box (separating axis) test over every pair of parts, in both wheel
and foot mode, and reports interpenetration in millimetres. It is wired into
the test suite, so the layout cannot silently regress.

The first attempt had **102 interpenetrating pairs**, worst at 12 mm. Two
mistakes caused most of it:

- **Servos were attached to the wrong body.** A servo case bolts to the body
  *proximal* to the joint it drives, so the hip servos belong to the torso,
  the knee servo to the thigh, and so on. Hanging each one off the link it
  drives put every case inside the next link.
- **The output shaft was modelled along the body's long axis.** On a real bus
  servo it is perpendicular, near one end of the top face, so a servo driving
  a joint occupies the plane *perpendicular* to that joint's axis. Getting
  this backwards rotates every servo 90 degrees into its neighbours.

## What drawing it at real size caught

**The battery does not fit flat.** A 3S 2200 mAh pack is 105 mm long and the
torso bay is 90 mm across. It has to stand upright. Once it and the Pi are in,
the bay is essentially full - there is no room for the arms' electronics later
without growing the torso.

**The printed roll bracket swung 18 mm below the floor.** The ankle roll maps
y onto z, so a plate reaching 45 mm *up* from the axle ends up 45 mm *out* and
its far edge dips under the ground plane once the wheel goes flat. Anything
mounted on the roll bracket has to be checked in both orientations, not just
the one you drew it in.

**The roll servo grazed the floor for the same reason.** Sitting at y = 0 it
ends up level with the axle after the roll, and the axle is only 12 mm up in
foot mode. It has to be offset outboard so the roll carries it upward.

**The wheel-drive servo has to be outboard.** Mounted inboard it swings *down*
into the floor during the flip. Outboard it swings up.

**The flat wheel sweeps an 80 mm disc horizontally.** This is the one that is
genuinely easy to miss. Upright, the wheel is 24 mm wide and you plan
clearances in y. Flat, it becomes a 80 mm platter and anything within 40 mm of
the axle horizontally is in its way. The ankle-pitch servo, mounted outboard
purely to clear the upright wheel, ran straight into the flat one. It needs
*both* clearances.

Current clearances, worst part in each mode:

| | lowest part | clearance |
|---|---|---|
| wheel mode | tyre | 0 mm (it is the contact) |
| foot mode | tyre face | 0 mm (it is the contact) |

Interpenetration: **0 pairs in both modes.**

Overall size, for the record:

| | |
|---|---|
| height on wheels | 427 mm / 16.8 in |
| height on feet | 387 mm / 15.2 in |
| width | 219 mm / 8.6 in |
| depth | 131 mm / 5.2 in |
| mass | 2.05 kg / 4.5 lb |

## Is it assembled, or just non-overlapping?

Those are different questions, and the first pass only answered the second.
`fitcheck.py` now also builds a connectivity graph: two parts are joined if
their boxes come within 4 mm, and the whole robot should be one group. It was
**11 separate groups** - a cloud of parts that happened to miss each other,
which is exactly what "floating" looks like.

Fixing it meant giving every link a **spine** running from its own joint to
the child joint, with the child's servo bolted flush against it, and routing
the whole structure outboard of the wheel plane. Now **1 group in both modes,
0 interpenetration.**

The rule that shapes the entire lower leg: a non-rolling part must clear the
wheel in *both* of its shapes. Upright it is a 24 mm rim swept 80 mm
vertically; flat it is an 80 mm platter swept horizontally. Satisfying both
means either **|x| > 32 mm** (fore or aft of the disc) or **z > +12 mm**
(above it). Nothing may run straight down to the axle at y = 0, which is why
the shin stops 26 mm short and reaches the ankle bearing from behind.

## The end poses are not enough: check the sweep

Both end poses can be clean while the robot destroys itself in between.
`fitcheck.py` now walks the flip in 26 steps and checks every pose. The first
time it ran, the **wheel-drive servo swept 20.7 mm through the shin** at 64%
of the flip. That servo turns with the roll bracket, so it carves an annulus
**14-50 mm from the roll axis, for |x| < 23 mm**, and anything not on the roll
bracket has to stay out of it.

That single constraint reshaped the whole lower leg. The rules that survive:

| region | rule |
|---|---|
| clear of the upright wheel | \|y\| > 12 mm, or radius > 40 mm from the spin axis |
| clear of the flat wheel | \|x\| > 32 mm, or z > +12 mm above the axle |
| clear of the wheel-servo sweep | radius < 14 mm **or** > 50 mm from the roll axis, or \|x\| > 23 mm |

The third rule has a useful loophole: the annulus has a *hole*. Parts sitting
essentially **on** the roll axis are inside it and the sweep misses them
entirely, which is where the ankle bearing carrier and the tie back to the
wheel servo now live. The shin instead goes the other way and stops 55 mm up,
outside the annulus, reaching the ankle from behind.

The audit also distinguishes three things that all look like overlap:

- **interpenetration** between parts that can move relative to each other - a
  real fault
- **lap joints** between geoms on the same body, which cannot move relative to
  each other at all, so shared material is just how a bracket bolts on
  (MuJoCo skips these for the same reason)
- **designed mates**, like a hub inside its tyre

Current state: **0 interpenetration in both end poses, 0 through all 26 flip
poses, 1 connected group.**

## Is it stiff enough, and is anything actually floating?

Two separate worries, two separate checks.

**Floating.** The robot-wide connectivity check uses a 4 mm tolerance, which is
right for a *running clearance across a joint* - two links that rotate relative
to each other must not touch - but it is far too generous *inside* a rigid
part. Checked strictly, parts on the same body had to actually touch, and
**5 of 11 bodies were not one piece**: the hip servos were 0.6 mm off the
chassis, the shin's lower arm was detached from its own spine, and the
roll-bracket tie was floating. That is exactly what "floating servos" looks
like. All closed; a test now enforces one rigid piece per body.

**Stiffness.** Printed PETG at a conservative E = 2.0 GPa, loaded by the peak
joint torques measured in sim:

| member | section | length | load | tip deflection |
|---|---|---|---|---|
| thigh spine, fore/aft | 20 x 12 mm | 110 mm | 26.4 N (peak knee torque) | **0.73 mm** |
| thigh spine, sideways | 12 x 20 mm | 110 mm | 3.3 N (RMS) | 0.25 mm |
| shin spine | 20 x 12 mm | 55 mm | 26.4 N | 0.09 mm |
| shin aft post | 12 x 12 mm | 30 mm | 18.4 N (peak ankle roll) | 0.05 mm |

For scale, **0.87 deg of servo backlash on a 110 mm link is 1.67 mm of slop** -
more than twice the worst-case structural deflection, and that is at *peak*
torque, not RMS.

So the structure is not the weak link; the actuators are. That matches the
backlash results, and it means thickening these parts buys almost nothing
until the lash is dealt with.

## Not modelled

The ankle-pitch drive is not coaxial with its own joint, because the wheel
already owns that axle. The servo sits high on the shin and would drive down
through a belt on the real robot; that belt is not drawn.

**The three-DOF ankle remains the hard part of this robot.** Pitch, roll and
wheel drive all want to live within 45 mm of one axle, two of them have to
clear a wheel that changes shape halfway through the manoeuvre, and the third
sweeps an annulus through everything else. It is buildable - the layout here
is clean through the whole flip - but it is the part that will take real CAD
rather than a parametric sketch.

## Still open

**Mass distribution is not yet updated to match.** The model puts 100 g in the
shin and 60 g at the ankle, dating from when the wheel servo was assumed to
live in the shin. Physically the wheel servo is on the roll bracket, 110 mm
further out. Moving 55 g that far out raises leg inertia and would need the
balancer re-checked. Worth doing before committing to a print.
