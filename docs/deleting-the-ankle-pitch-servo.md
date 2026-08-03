# Deleting the ankle-pitch servo

> Ankle pitch does the least work of any joint in the robot and drags the worst
> subsystem in it. Can a passive parallelogram do the job instead?

Asked 2026-08-02. The answer is **yes, and it is not close**: with a linkage and
no ankle actuator at all, foot mode stands 10 out of 10 at the 2-3 degrees of
gear lash where the servo version now stands 2 or 3 out of 10.

Written up whichever way it landed, like `deleting-the-roll-servo.md`, because a
good idea that fails quietly gets reinvented - and because this one found
something worse on the way past.

## The premise

Ankle pitch is the least-worked joint in the robot, 0.07 N.m RMS against a servo
good for 0.6-0.9 continuous (`docs/hardware-readiness.md`). Its entire job in
wheel mode is `hip + knee + ankle = 0`, which holds the wheel's face parallel to
the torso, ready to plant. That is geometry, and geometry is what a four-bar
does for nothing.

What the one servo currently drags behind it:

- a 2:1 GT2 belt drive, needed only because the wheel already owns the ankle axle
- a 40T driven pulley that is **2.23 mm through the floor in foot mode**. With
  the wheel flat the axle sits at `WHEEL_HALF_W` = 12.0 mm and the pulley's pitch
  radius is 12.73, so it does not fit even with no flange
- a 20T pulley needing a 4.40 mm hub on the horn pattern, so it is no longer a
  catalogue part, and it is not on the order sheet
- the ankle-pitch bearing forced outboard, which is why each leg reaches y = 136
  and the robot is 306 mm wide
- about 180 g

## What was measured

`load(parallel=True)` builds the linkage as what it physically is: a rigid
kinematic constraint, `hip + knee + ankle_pitch = 0`, as a fixed tendon with an
equality. The ankle also loses its lash joint, because **a four-bar has no
gearbox in that path** - links and pin joints have no reduction and therefore no
backlash. The ankle loop is switched fully off, since there is no actuator left
to run it.

Foot mode, 15 s standing, 10 trials per cell varying only the moment the flip is
triggered:

| lash | servo + ankle loop | parallelogram, no loop |
|---|---|---|
| 0 | 10/10 | 10/10 |
| 1 deg | 10/10 | 10/10 |
| **2 deg** | **2/10** | **10/10** |
| **3 deg** | **3/10** | **10/10** |
| 4 deg | 2/10 | **0/10** |

2-3 degrees is the range that matters: it is what `docs/backlash.md` measures
for this servo class and what the whole control design was rebuilt around.

## And the linkage is doing the work, not just the lash removal

The obvious objection is that the win is simply an ankle with no free play, and
that the constraint is redundant because the controller already commands
`ankle = -(hip + knee)`. Control test, ankle loop off throughout, disabling the
equality at run time while keeping the rigid ankle:

| lash | rigid ankle only | rigid ankle + linkage |
|---|---|---|
| 2 deg | 2/5 | **5/5** |
| 3 deg | **0/5** | **5/5** |

So no. Removing the ankle's lash alone is worth almost nothing; at 3 degrees it
still falls every time. What the linkage adds is a **mechanical feedback path**:
the foot's angle is slaved to the hip and knee, so when the body rotates into
its remaining free play the ankle counter-rotates and keeps the foot planted.
A servo loop has to sense that rotation and react to it. A linkage cannot help
doing it, instantly and with no gain to tune.

## Where it fails, and it is worth knowing

**At 4 degrees the parallelogram falls 10 out of 10 and the servo loop manages
2.** Past some amount of free play a passive linkage cannot win, because the
lash is upstream of it in the hip and knee and the linkage has no authority of
its own. An active ankle can still push back. So this is not "linkages beat
control"; it is that at the lash this robot actually has, the mechanism wins.

## What this does NOT settle

**Packaging is not done.** The linkage has to be drawn and checked:
`cad.assemble_check` both poses and the flip sweep, `cad.wiring` both poses,
`cad.printability`, and `cad.hardware.floor` in both modes. Nothing here says a
four-bar fits in a shin that is already the tightest region of the robot. It
does say the thing it replaces provably does not fit.

**The constraint is an idealisation.** The tendon ties the DRIVE-side joint
angles. A real linkage ties the LINK angles, downstream of the hip and knee
lash, which should be better still - the model is conservative in that respect,
but it is not the same object and a drawn linkage has to be re-tested.

**A true parallelogram needs equal link lengths.** `hip + knee + ankle = 0` is
exact by construction here. A four-bar with unequal links only approximates it,
and the error over the squat range has to be swept once the geometry exists.

## The thing found on the way past

Setting the baseline turned up something that has nothing to do with linkages.

`docs/backlash.md` records foot mode standing at 2 and 3 degrees with the ankle
loop on. **It does not any more:** 2/10 and 3/10. That table was measured
against the old inertia, and `SEG_INERTIA` was regenerated from the CAD on
2026-08-02 - total mass 1874.8 g to 1907.0 g, and the roll bracket's centre of
mass moved 11.5 mm.

Correcting the mass distribution, which was unambiguously right to do, **broke
foot mode at the lash the servos actually have.** Nobody would have noticed,
because the claim lived in a document and documents do not re-run themselves.

> The result that justified the ankle loop is the result the ankle loop no
> longer delivers. The loop was added to make foot mode survive 2-3 degrees; it
> does not, and a linkage does.

That is the same lesson as everything in `docs/how-checks-fail.md`, one level up:
**a design decision is only as current as the number it was made from.**
