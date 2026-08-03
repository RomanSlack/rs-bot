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

> **Answered 2026-08-03 in `cad/linkage.py`, which draws it. Both open items
> below are closed, one of them badly.** See "The mechanism" at the end of this
> file; what remains here is what was open when this was written.

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

---

# The mechanism, 2026-08-03

`cad/linkage.py`. It fits, on the real solids, over squat and flip, both legs:
**nothing collides at any pose.** It costs 32.3 g per leg against 89 g of ankle
servo and belt drive, so 113 g comes off the robot.

But the tendon was flattering it in one respect and the drawn version says so.

## Two parallelograms, one idler

Holding the foot parallel to the TORSO across two joints needs the torso's
attitude carried past the knee first, so it is two stages and not one:

```
stage 1   torso -> thigh -> idler      holds the IDLER level
stage 2   idler -> shin  -> ankle      holds the FOOT level
```

The idler is one bar on the knee axis with a pin at each end, free to turn
against both neighbours. Five pivots, three new parts (two rods and the bar),
and two features on parts that already exist.

Both stages are EXACT parallelograms, so `hip + knee + ankle = 0` holds by
construction and not by fit, which closes the "unequal links only approximate
it" worry: the foot is level to 1e-14 degrees at every squat depth. That is a
solver result and not a tautology, and the solver is control-tested - a rod
1 mm long comes back as 2.03 degrees of foot tilt.

## Where it goes, and what that cost

Outboard, in the band the ankle-pitch bearing already occupies, with the parts
stacked across y:

```
31.0  torso fin    off the chassis side plate
38.0  rod 1
48.0  idler        on the knee axis
54.5  rod 2
61.0  yoke arm     inside YOKE_FWD's own y = 56..66 band
```

**Inboard was tried first and is wrong**, which is worth recording because it
looks obviously right: there is an empty 59 mm corridor between the legs, the
rods fit in it, and the chassis pin is easy to reach. The yoke is the problem.
It has no material inboard of y = -6, the roll bracket sweeps everything from
y = -23 to +24 as it flips, and the one straight arm that clears the bracket
ends up 1 mm off the floor in foot mode. Outboard the arm is 30 mm of YOKE_FWD
carried further forward and attaches to nothing new.

The offset is 30 mm. 34 grazes the shin's ankle carrier, 26 buys nothing.

## What the pins cost, which is the number that matters

The linkage is bought to remove 2-3 degrees of ankle gear lash and it pays with
four pin joints. Those are not one number:

| | worst case | what it is |
|---|---|---|
| free play, 0.10 mm printed bores | **0.97 deg** | the same animal as gear lash |
| free play, 623ZZ at each pin | **0.10 deg** | ditto, for $1.60 a leg |
| build offset, print +/-0.3 mm | **4.21 deg** | a fixed tilt, per leg |
| band spread after adjustment | **1.09 deg** | what no adjustment removes |

So the mechanism wins on the thing it was bought for, by 2x on plain printed
bores and by 20x on bearings.

**And it introduces something the servo version did not have.** 4.2 degrees of
fixed foot tilt is not lash - it is a part that comes out of the machine aimed
slightly wrong, and with no ankle actuator there is nothing to trim it out with
afterwards. In foot mode that is the torso standing 4 degrees off vertical with
both soles flat, and the two legs are free to be wrong in opposite directions.

> **One of the two rods has to be length-adjustable.** That is a requirement
> this design did not have an hour ago, and it came from drawing the thing.

Even then 1.09 degrees is left, because mis-placed pins stop the four-bar being
an exact parallelogram and its error then moves with squat depth, which a single
adjustment at a single pose cannot follow.

## The rest of it

- **Transmission angle** stays at sin >= 0.931 over the whole band, which is
  69 degrees at worst. The two idler pins are split 6 mm either side of the
  offset, and which one goes high is not free: stage 1 low and stage 2 high
  keeps the rods from crossing AND keeps stage 2's pin above the axle, which
  is only 12 mm off the floor in foot mode. The other way round crossed the
  rods for 950 mm3 and put the yoke arm on the ground.
- **Rod force is 38.7 N** at the measured 1.082 N.m ankle peak, against 705 N
  of Euler buckling. 18x. The rods are not the problem.
- **The chassis fin is the worst part of the design.** It reaches 79 mm off the
  side plate to hold stage 1's ground pin, and a pin that moves is a foot that
  tilts at about 2 degrees per millimetre. As a round boss it would deflect
  0.7 mm at the factored rod force, which is 1.3 degrees - more than the free
  play the whole mechanism is bought for. It is a 25 mm deep fin instead, at
  0.1 mm, and it wants triangulating back to the plate properly.

## And a trap, now closed

`rollout_deploy(parallel=True)` used to return **0 of 10** at both 2 and 3
degrees - worse than the servo it replaces. The flag built the linkage and left
the ankle ACTUATOR in, so the STAND loop drove the ankle to one angle while the
tendon held it at another and the two fought.

The 10/10 in the table above was real, but reproducing it needed the caller to
know to switch the ankle loop off, and nothing recorded that. The measurement
that produced the table was never committed. A parallelogram is the ABSENCE of
a servo, so `load(parallel=True)` now removes the ankle actuators, and
`tests/test_linkage.py` flips ten times at 2 and 3 degrees with the default
config and asserts 10 of 10.
