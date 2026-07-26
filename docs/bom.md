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
into the floor during the flip. Outboard it swings up. That also sets the
robot's width: about 234 mm across the ankles.

Current clearances, worst part in each mode:

| | lowest part | clearance |
|---|---|---|
| wheel mode | tyre | 0 mm (it is the contact) |
| foot mode | tyre face | 0 mm (it is the contact) |
| foot mode, next-lowest | roll bracket | ~8 mm |

## Still open

**Mass distribution is not yet updated to match.** The model puts 100 g in the
shin and 60 g at the ankle, dating from when the wheel servo was assumed to
live in the shin. Physically the wheel servo is on the roll bracket, 110 mm
further out. Moving 55 g that far out raises leg inertia and would need the
balancer re-checked. Worth doing before committing to a print.
