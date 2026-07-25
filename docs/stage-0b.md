# Stage 0b: the transition

**Status: mechanism redesigned and mostly working. The robot now reaches foot
mode with the wheels never leaving the ground, but still tips forward during
the final body shift.**

## Attempt 1: rocker sole on the wheel axle. Dead.

60 valid sole geometries x deploy rates, zero reached STAND. The cause is
geometric, not tuning: `corner_radius = hypot(face_r, half_len)`, and
`face_r >= 46 mm` because the plate must cradle the wheel without intersecting
it. So the plate always jacked the robot 11-21 mm onto a single edge ~53 mm
behind the axle, wheels airborne, balancer powerless. It pitched 57 deg in
0.4 s against a 0.17 s pendulum time constant. No deploy rate fits; at
60 rad/s the plate slams the floor and launches the robot 135 mm.

## The rule that was being broken

Wheel-biped transformation work is explicit about this: the transition must
pass through a **critical state where wheel and foot are both in ground
contact**. BHR-W uses its knees as the second contact. The axle-pivoted sole
had no such state - it went straight from wheel-only to foot-only via an
airborne jack, and no controller can survive that.

## Attempt 2: outrigger struts

A strut on a pivot 55 mm down the shin, swinging down inboard of the wheel,
with a pad on the end. Moving the pivot off the axle removes the cradling
constraint, so the pivot-to-pad distance is free, and dig scales as
roughly L^2/2R.

The sequence is built entirely around never being airborne:

    WHEEL -> EXTEND -> SETTLE -> SWING -> PLANT -> LOAD -> SHIFT -> STAND

- **EXTEND** to 213 mm. This raises the strut pivot to 93 mm.
- **SWING** the struts down. The pad's furthest corner is 43.4 mm from the
  pivot, so it clears the floor by 7.2 mm at every angle and cannot dig. The
  balancer is fully in charge the whole time.
- **PLANT** by squatting to 170 mm, where `STRUT_LEN` equals the pivot height
  and the pad lands exactly co-planar with the wheel contact. Critical state.
- **LOAD** brakes the wheels *before* shifting, then **SHIFT** walks the torso
  back over the pads quasi-statically.

## What works

- Swing clearance 7.2 mm, verified across the whole sweep at every squat depth.
- The pad lands at 0.0 mm with the wheels still down: `wheel_clear = 0.0 mm`
  through the entire transition. **The robot is never airborne.**
- **Foot mode itself is solid.** Placed in the deployed pose with the balancer
  completely off and the wheels braked, it stands the full 10 s test for body
  shifts of +50 and +65 mm, settling at 4.0 and -1.3 deg. The stable band is
  CoM x in about [-67, -33] mm.
- SETTLE now fires in 0.32 s instead of timing out, because "quiet" is judged
  against the balancer's learned trim lean rather than against zero.

## What still fails

It tips forward during SHIFT. Drift is down from 4.1 m to 0.42 m, but the
handover pose is still marginal.

The reason is a hard cap. The balancer necessarily hands over with the CoM
above the wheel contact at x=0, so the pad polygon has to straddle x=0. How far
forward the pad can reach is limited by the swing: no pad corner may sit
further from the pivot than the pivot's height during the swing, which is
43.4 mm. With the pivot 34.9 mm behind the axle that buys only
**+8.5 mm of forward margin, about 1.8 deg** - not enough to absorb the
residual pitch and momentum at handover.

## Three ways to buy forward margin

1. **A second, forward-facing strut per leg.** Definitive: the polygon
   straddles the CoM by construction. Costs 2 more servos and ~120 g.
2. **Stand deeper.** Forward reach grows as the stand height drops, because
   the pivot travel between swing and stand grows. Rough numbers: +13.8 mm at
   a 140 mm stand height, +16.4 mm at 120 mm.
3. **Swing with the legs straighter.** Small gain, ~+3 mm, and it pushes the
   leg IK toward its singularity.

## Bugs found and fixed on the way

- **Invalid geometry masqueraded as a control failure.** Face radii of 41.5 and
  44 mm put the old plate *inside* the wheel; MuJoCo generated sole-wheel
  contacts that threw the robot during SETTLE. `load()` now refuses.
- **The contact latch was never reset between states,** so the swing ramp's
  tracking error counted as pad contact and PLANT exited 0.04 s in, before the
  pads were near the floor.
- **Unloaded wheels free-spin and poison the odometry.** When the pads first
  took load and lifted the wheels 1.5 mm, wheel-derived velocity read -1.4 m/s
  of motion that was not happening, and the balancer ran away. The outer loop
  is now disabled once the pads carry load.
- **Shifting the body changed the shin angle,** dropping the strut pivot 10 mm
  and gouging the pads into the floor (26.8 mm of wheel lift). Both contacts
  co-planar pins the shin angle; `foot_pose()` holds it.
- **Braking after shifting drove the robot 4 m across the room.** With the legs
  shifted, holding the torso upright puts the CoM permanently behind the wheels,
  so the balancer accelerates forever chasing an equilibrium that no longer
  exists. LOAD now comes before SHIFT.
