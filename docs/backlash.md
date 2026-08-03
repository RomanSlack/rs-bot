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

## A wrong turn worth recording

The first pass at this re-tuned the balancer against lash and reported that it
"survives up to 2 deg". That was wrong, and the cost function was why: falling
was penalised at 100+, oscillation at only 3x max_pitch. The search happily
returned a set that never fell and **limit-cycled at +/-16 deg with the lash
switched off entirely** - it had simply found a violent oscillation that stayed
upright. It looked fine in aggregate metrics and looked drunk on video.

The cost now weights steady-state pitch RMS and peak-to-peak heavily
(`pitch_rms * 60 + pitch_ptp * 30`, measured over the back half of a run), which
ranks that gain set at 130 against 15 for the rigid defaults. Lesson: "did not
fall" is not a fitness function.

## Results

Two changes fix it, and together they are worth more than any amount of gain
tuning.

**Much lower inner-loop gain.** Multi-start search, rather than coordinate
descent from the rigid optimum, lands at `kp` 10.4 instead of 48. A high-gain
loop through a deadzone chatters by construction.

**A low-pass on the wheel command** (`tau_cmd`, 80 ms). Slamming the command
across the deadzone is what drives the limit cycle, and filtering it is what
actually kills it: at 1 deg of lash the pitch swing drops from 17.7 deg
peak-to-peak to 0.9. Beyond ~150 ms the lag itself becomes the problem and at
250 ms the robot falls.

One gain set now covers both rigid and lashed:

    kp 10.435   kd 0.435   kv 0.300   kx 0.500   tau_odom 0.100   tau_cmd 0.08

| lash | quiet swing | max shove | drift 25 s | round trip |
|---|---|---|---|---|
| 0 | 0.1 deg pp | 1.4 N.s | 28 mm | ok |
| 0.5 deg | 0.1 deg pp | 1.4 N.s | 64 mm | ok |
| 1 deg | 0.9 deg pp | 1.4 N.s | 76 mm | ok |
| 2 deg | 9.9 deg pp | 1.4 N.s | 76 mm | ok |
| 3 deg | 29.6 deg pp | 1.0 N.s | 304 mm | ok |

The old `kp=48` set falls at 0.5 deg and every level above it. The new set is
identical to it on a rigid robot (same 1.4 N.s shove rejection, same quiet
pitch) and survives to 3 deg.

The one real cost is position hold: `kx` drops from 0.86 to 0.5, so standing
drift roughly doubles to about 28 mm over 60 s. Pushing `kx` back above 0.7
starts costing shove rejection.

## What this means for foot mode

**Foot mode was the real casualty, and this was the surprise.** The transition
itself completes - it reaches STAND at 8.79 s with 1 deg of lash - and then it
**falls over while standing still**. Passive static stability is precisely the
thing that depends on rigidity: lash across hip, knee and ankle lets the body
rotate 2-3 deg for free, which eats a third of the 9.1 deg tilt margin before
any servo can even feel it, and the robot builds momentum inside the deadzone.

So the headline claim from stage 0b - "stands with the controller completely
off" - is a rigid-model result. On real servos it needs a controller, just a
weak one.

## The fix: a low-gain ankle loop in foot mode. Built, and it works.

Foot mode no longer switches the controller fully off. It runs a weak
PID on ankle pitch against IMU pitch - the support polygon still does the
work, but *something* has to take up the lash:

    apitch += stand_kp * pitch + stand_kd * pitch_rate + integral
    stand_kp 1.2   stand_kd 0.12   stand_ki 0.6

Positive ankle pitch tips the toe down, and the reaction rotates the body
back, so a forward lean wants more ankle pitch. The integral absorbs the
steady offset from the CoM not sitting dead centre on the foot, which leaves
the P term free to fight disturbances.

> **SUPERSEDED, 2026-08-02. The table below no longer holds.** It was measured
> against the old inertia. `SEG_INERTIA` was regenerated from the CAD that day
> (1874.8 g to 1907.0 g, and the roll bracket's centre of mass moved 11.5 mm),
> and re-measured foot mode now stands **2/10 at 2 deg and 3/10 at 3 deg with
> the loop ON**, not reliably as recorded here.
>
> Correcting the mass distribution broke foot mode at the lash the servos
> actually have. See `docs/deleting-the-ankle-pitch-servo.md`, which measures a
> passive parallelogram standing 10/10 at both.

**Standing, 15 s:**

| lash | loop off | loop on |
|---|---|---|
| 0 | stands, 3.6 deg wobble | stands, **1.1 deg** |
| 0.5 deg | stands, 11.9 deg | stands, **3.1 deg** |
| 1 deg | stands, 13.8 deg | stands, **3.8 deg** |
| 2 deg | **falls at 1.1 s** | stands, **4.7 deg** |
| 3 deg | **falls at 1.0 s** | stands, **5.5 deg** |

**Largest shove survived:**

| lash | loop off | loop on |
|---|---|---|
| 0 | 0.4 N.s | 0.5 N.s |
| 1 deg | 0.2 N.s | **0.5 N.s** |
| 2 deg | falls unshoved | **0.5 N.s** |

Disturbance rejection becomes lash-independent, and 0.5 N.s is close to the
geometric ceiling - tipping over the 40 mm foot edge takes about 0.58 N.s of
energy, so there is little left to win without a bigger foot or a step.

**Full round trip** (drive, flip, stand, flip back, drive on):

| lash | loop off | loop on |
|---|---|---|
| 0.5 deg | ok | ok |
| 1 deg | **falls in STAND** | ok |
| 2 deg | **falls in STAND** | ok |

The gains sit right at the stability edge: anything above `stand_kp` 1.2
chatters through the deadzone and is unstable. Do not turn it up.

## Still to do about it

1. **Lash is still a purchasing spec.** Wheel mode is fine to 1 deg (0.9 deg of
   swing) and usable at 2 deg (9.9 deg), but 3 deg gives 30 deg of swing and
   300 mm of drift. Under about 1 deg, backlash stops mattering; over 2 deg it
   dominates everything.
2. **Re-tune on hardware.** `balance.LASH_GAINS` is a starting point tuned
   against a guessed lash figure, not an answer.

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
