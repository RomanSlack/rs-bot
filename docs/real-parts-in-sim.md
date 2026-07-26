# The real robot in the simulator

```bash
uv run python -m cad.inertia          # the table, and the control tests
uv run python -m cad.inertia --emit   # the block that goes into model.py
uv run python serve.py                # drive it, localhost:8781
```

Three things changed, and only one of them is cosmetic.

| | before | now |
|---|---|---|
| what you see | stand-in boxes | the actual CAD parts |
| mass per body | CAD-derived | unchanged |
| **where that mass is, and how it is spread** | **a capsule or a box** | **CAD-derived** |

The third is the one that matters.

## Mass was right, the distribution was a guess

`cad/masses.py` derived every body's mass from geometry months of work ago. But
the *inertia* still came from whatever primitive was carrying that mass in the
sim - a 28 mm capsule standing in for an L-shaped part with a servo bolted to
one side. Each body is now composed properly:

    printed structure   the real B-rep solid, exact inertia from OCC
    servo cases         uniform boxes, at the positions the sim already uses
    bought parts        the same, for the wheels, Pi, pack and driver

combined with parallel axis about the composite centre of mass, rather than
added as scalars.

### How wrong the stand-ins were

| body | centre of mass moved | principal inertia, before -> after (g.cm2) |
|---|---|---|
| thigh | 25 mm | 1247 / 1247 / 83 -> 156 / 971 / 1049 |
| shin | 43 mm | 908 / 908 / 77 -> 310 / 460 / 614 |
| **ankle** | **75 mm** | 154 / 124 / 87 -> 91 / 125 / 160 |
| roll bracket | 32 mm | 400 / 400 / 400 -> 99 / 173 / 202 |
| torso | 6 mm | 33234 / 36086 / 11583 -> 13219 / 34991 / 37449 |
| wheel | 0 mm | 269 / 269 / 480 -> 191 / 191 / 326 |

Total mass is unchanged at **1.7565 kg** and the standing centre of mass is
still over the axle - the trim solve sees to that. Nothing about the robot's
statics moved.

**The ankle is the headline.** Its centre of mass is 75 mm from where the
stand-in box put it, because the roll servo's 55 g case really does hang 76 mm
aft of the ankle axle, and the box modelling it sat 23 mm *forward*. That is a
98 mm error on a 56 g body, on each leg, on a link that swings.

That cantilever is a real feature of the design, not a modelling artefact -
`_link_geoms` has always drawn the roll servo at x = -75.7 mm. It just was not
in the dynamics until now.

## What it cost

Correcting the distribution alone - same total mass, same standing centre of
mass to within a few millimetres - broke shove rejection, steering, and the
entire transition. Eleven tests. The gains had to be found again.

That is the point of the exercise, and it is worth stating plainly: **the
balancer was tuned against a mass distribution the robot does not have.**

### Three tuning traps, in one sitting

Every one of these produced gains that scored well and failed the tests.

**1. The cost could not see the yaw loop.** A set of gains passed every trial
in the search while limit-cycling in yaw at +/-2 rad/s, counter-rotating the
wheels at 20 rad/s and walking a metre backwards. It went unpunished because
`rollout` had no yaw command at all, and because mean wheel speed stayed near
zero throughout - so the robot's own odometry reported it standing perfectly
still. `kyaw` was not in the searched fields either. Both are now.

The tell was a *smaller* shove drifting further than a larger one: 0.35 N.s
drifted 1.04 m while 1.0 N.s recovered to 26 mm. A monotonic failure is a gain
problem; a non-monotonic one is a discrete event.

Yaw is scored on **RMS deviation from the commanded rate**, not mean error. A
symmetric limit cycle averages to exactly the commanded rate and reads as
perfect tracking.

**2. Drift was weighted at 2 against pitch terms worth tens.** The acceptance
criterion is 50 mm of drift in a minute. At a weight of 2 that is worth 0.1 of
cost, so the search traded all of it for a slightly smoother pitch trace and
returned gains that stood beautifully while wandering off the desk. Now 40.

**3. Then the drift weight punished the robot for driving.** Applied uniformly,
it scored a robot that refuses to move as perfect - and one round produced
exactly that, tracking 3 cm of a 75 cm velocity command. Each trial now carries
how far it is *supposed* to travel.

This is the same failure the tuning cost had once before, when it penalised
falling at 100 and oscillation at 3 and returned gains that never fell and
limit-cycled at +/-16 degrees. The lesson keeps arriving in new clothes: **a
search will find whatever the cost actually measures, and the cost is usually
measuring something narrower than the requirement.**

### And one in the inertia itself

OCC's `MatrixOfInertia` is already taken about the centre of mass, despite the
`GProp` being constructed at the origin. Subtracting the parallel-axis term as
well - the natural thing to write - drove the tensors indefinite. MuJoCo
accepts an indefinite inertia tensor without complaint and the robot simply
behaves strangely. It was caught by checking that the eigenvalues are positive
and satisfy the triangle inequality, which costs three lines.

## Meshes are visual, and opt-in

```python
load(meshes=True)     # the real parts
load()                # the stand-in boxes, still the default
```

`serve.py` uses the meshes. Everything else does not, deliberately:
`fitcheck.py` measures interference from the oriented bounding boxes of the
group-0 geoms, and a mesh geom has no meaningful `geom_size`, so it would
quietly start auditing the wrong shape. The collision geometry and the
inertials are identical either way, so this changes what you see and nothing
about what it does.

Right-hand parts are the left-hand STL with a negative y scale. Mirroring also
flips the sign of the centre of mass in y **and** of the Ixy and Iyz products
of inertia - invisible in any symmetric pose, and a slow drift in a turn if you
miss it.

## Control tests

| check | what it would catch |
|---|---|
| composite mass == `cad.masses` | the two derivations disagreeing |
| eigenvalues positive, triangle inequality | a double parallel-axis, a sign error in a product |
| total subtree mass unchanged at 1.7565 kg | mass appearing or vanishing in the port |

## Still open

- The 600 g standing in for stage-4 arms and head is unchanged, in the same
  place, as the same uniform box. It represents parts that do not exist, so
  there is nothing to derive; as a point mass at the origin instead it drops
  the standing centre of mass 57 mm, which is not a port, it is a different
  robot. When the arms are real, this changes and the gains change with it.
- Collision is still the primitive shapes. Mesh collision in MuJoCo is
  convex-hull based, which is wrong for every L-shaped part here, and slower.
- The infill is assumed uniform at 60%. A real print is a solid shell around a
  sparse core, so the true inertia is slightly higher than this for the same
  mass.
- **The roll servo's 76 mm cantilever is now in the dynamics and it is not
  free.** Whether that is worth designing out - by moving the roll actuation
  inboard, or driving it through a belt as the ankle pitch already must be - is
  a design question this only just made visible.
