# Backlash

`load(backlash=radians)` builds the robot with gear lash. This was the last
thing that could still invalidate the design before spending money, and the
answer is: **it does not kill wheel mode, but it breaks foot mode.**

## How it is modelled

Physically, not as a command deadband. Every actuated joint gets a second
"lash" joint in series, free to float within +/- lash/2, with a small inertia
stub on the gearbox side:

    servo drives and senses  ->  [lash joint, free play]  ->  link

The servo commands and reads the proximal side; the link hangs off the distal
side and can move without the motor knowing. That floating deadzone in the
kinematic chain is what actually hurts a balancer. A deadband applied to the
command would not reproduce it, because the load could not move on its own.

All 10 joints get lash, including the wheel drive - an unloaded wheel that can
turn without the encoder seeing it is exactly what poisons the odometry.

`backlash=0` builds the rigid model with no extra bodies at all, so joint
indices are unchanged and every existing test still addresses the same model.

## Results

Gains tuned on the rigid model do not survive any lash at all. Re-tuning is
mandatory, and it is mostly the outer loop (`kv`, `kx`, `tau_odom`) that moves.

| lash | with rigid-tuned gains | after re-tuning | flip round trip |
|---|---|---|---|
| 0 | 1.2 deg pitch | - | ok |
| 0.5 deg | falls at 2.7 s | 20.4 deg pitch, survives | **fails** |
| 1.0 deg | falls at 4.5 s | 25.2 deg pitch, survives | **fails** |
| 2.0 deg | falls at 2.5 s | 24.2 deg pitch, survives | **fails** |
| 3.0 deg | falls at 1.6 s | **falls at 16.6 s** | fails |

Re-tuning at 1 deg took the cost from 564 to 13. Tuned gains at 1 deg:

    kp 48.0   kd 1.72   kv 0.64   kx 0.54   tau_odom 0.031

## What this actually means

**Wheel mode survives but stops being pretty.** It holds up to 2 deg of lash,
but at 20-25 deg of pitch swing rather than 1.2 deg. That is not balancing, it
is limit-cycling through the deadzone: visible, continuous wobbling. A robot
that looks drunk rather than one that looks solid.

**Foot mode is the real casualty, and this was the surprise.** The transition
itself completes - it reaches STAND at 8.79 s with 1 deg of lash - and then it
**falls over while standing still**. Passive static stability is precisely the
thing that depends on rigidity: lash across hip, knee and ankle lets the body
rotate 2-3 deg for free, which eats a third of the 9.1 deg tilt margin before
any servo can even feel it, and the robot builds momentum inside the deadzone.

So the headline claim from stage 0b - "stands with the controller completely
off" - is a rigid-model result. On real servos it will need a controller, just
a weak one.

## What to do about it

1. **Do not switch the controller fully off in foot mode.** Keep a low-gain
   ankle-pitch loop on the IMU. The support polygon means it only needs to be
   weak, but it cannot be nothing: something has to take up the lash. This is
   the same ankle strategy already noted as the fix for foot-mode disturbance
   rejection, and it is now load-bearing rather than a nice-to-have.
2. **Treat lash as a purchasing spec, not a detail.** The difference between
   0.5 and 3 deg is the difference between a robot that works and one that
   cannot stand up. Worth paying for lower-backlash servos, or preloading the
   ankle joints against a spring.
3. **Re-tune on hardware, not in sim.** The gains here are tuned against a lash
   figure that is a guess.

## Bugs this exposed

Inserting a drive body at every joint makes every adjacent link
grandparent/grandchild, and MuJoCo only auto-excludes *direct* parent-child
pairs. Without a full exclude list the leg collided with itself: 14 contacts at
rest and `qacc` of 2e4. `_excludes()` now lists every intra-leg pair
explicitly, which is cheap and does not depend on the chain's shape.

The lash joints also interleave into the joint ordering, so the hand-written
keyframe `qpos` silently scrambled across the wrong joints the moment backlash
was switched on. The stance is now written by joint *name* (`_write_stance`).

Both of these produced clean, plausible-looking falls that were nothing to do
with backlash. Twice now the same lesson: check what is touching the floor, and
check the pose is what you think it is, before believing a dynamics result.
