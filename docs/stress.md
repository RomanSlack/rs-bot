# Stress: what the parts can actually take

```bash
uv run python -m cad.loads      # what each joint carries, measured off the sim
uv run python -m cad.fea        # the solver's own verification case
uv run python -m cad.stress     # every part, every material, PNGs to cad/out
```

Three modules, each of which refuses to be trusted until it has been checked
against something it cannot fake.

| | what it does | how it is checked |
|---|---|---|
| `cad/loads.py` | reads the wrench crossing every joint | closes against statics exactly, 6.12 N vs 6.12 N |
| `cad/fea.py` | TET10 linear-elastic solve | cantilever, +1.9% deflection, +3.1% stress |
| `cad/stress.py` | per-part boundary conditions, materials, pictures | interfaces must land on real features |

---

## The loads are measured, not assumed

`d.cfrc_int[i]` is the exact 6D interaction force between body i and its
parent, inertial and contact terms included. It is the whole load path with
nothing left out, so no part of this is an estimate.

The trap is the reference frame: cfrc_int is world-aligned but its torque is
taken about `subtree_com[rootid]`, not about the joint. Read straight it gives
a moment tens of times too big, because it folds in F x r for an r of a couple
of hundred millimetres. It has to be translated to the joint first.

**Control test:** balancing quietly, the wrench across the knee must equal
everything below the knee, weighed, minus the ground reaction under that wheel.
It comes out 6.12 N against 6.12 N. If that had not closed, every number below
would have been wrong in the same direction and none of the pictures would have
shown it.

### Four cases, and they are not the same kind of event

| case | what it is | factor |
|---|---|---|
| quiet | standing still | - |
| flip | a full wheel -> foot -> wheel cycle | **x3** (worst normal operation) |
| shove | the rated 1.0 N.s disturbance, **fore/aft** | **x1.5** (already a limit event) |
| tipover | the same impulse **sideways** | **x1** (a measured event, not a proxy) |

Factoring a limit case by 3 as well counts the same conservatism twice.

The fore/aft and lateral cases are separated because averaging them hides the
most important number in this whole exercise:

| | peak force at the hip |
|---|---|
| fore/aft 1.0 N.s | **6.5 N** - the wheels roll away, almost nothing reaches the structure |
| lateral 1.0 N.s | **472 N**, peaking 574 ms *later* |

That 574 ms is the tell: it is not the impulse. The robot rolls through -100
degrees and lands on its side, and 472 N is the floor. There is no hip roll
joint and no way to step sideways, so laterally the robot has **no compliance
whatever**. Same impulse, seventy times the load, and it is what sizes the
thigh and the shin.

Design loads that come out of this:

| part | wrench at the joint below | governed by |
|---|---|---|
| thigh | 107 N, 8.81 N.m | tipover |
| shin | 104 N, 6.34 N.m | tipover |
| ankle yoke | 47 N, 1.92 N.m | flip |
| roll bracket | 49 N, 1.87 N.m | flip |
| chassis | 55 N sideways + 25 N down | its own payload |

What this does **not** include: the reaction torque of a servo whose case bolts
to the part. That is an internal couple between two bolt patterns on the same
body, so it never crosses a joint and cfrc_int cannot see it. Bolt spacing is
checked separately, in each part's script.

## The solver

Ten-node tetrahedra, not four. A TET4 has a constant strain field, so it cannot
represent the linear strain gradient in a bending beam at all: it locks, and
under-reports peak stress by a factor that grows with slenderness. Every part
here is a slender member in bending, so TET4 would have returned a confident
and badly non-conservative answer.

Units are mm / N / MPa, which is self-consistent - stresses come out in MPa and
deflections in mm with no conversion anywhere.

**Two control tests, both of which caught something:**

*Cantilever.* 100 x 10 x 20 mm, 50 N at the tip. Deflection lands +1.9% (shear
deformation, which beam theory omits) and mid-span bending +3.1%. The first
attempt sampled stress at the built-in face and read **+44%** - that is not a
solver error, it is a real stress singularity at a sharp constraint, and it does
not converge under refinement, it just grows. Every peak in the results is
therefore quoted away from the supports.

*Node ordering.* If gmsh's mid-edge convention were not what the code assumes,
every shape function would be wrong and the solve would return plausible
nonsense. But mid-edge nodes are not at edge midpoints - gmsh projects them onto
the CAD surface, and an edge across a small bore can bow by nearly half its own
length. Testing the worst element cannot separate "curved" from "wrongly
permuted". The mean can: correct ordering runs a few percent, a scrambled mesh
runs 79%.

## Why the pictures look alike

In a linear-elastic solve with prescribed loads, the stress **field** barely
depends on the material. Stiffness cancels; only Poisson's ratio has any effect
and it is small. What changes between materials is how much of the allowable
that same stress uses up, and how far the part bends.

So the panels are coloured by **utilisation** - stress over allowable - not by
stress. Coloured by stress they would be identical, and that would be a picture
of nothing.

Printed parts get two numbers, because a printed part is not one material:
**in-plane** along the layers, and **interlayer** pulling them apart at roughly
half the strength. The second is how a printed part actually fails. The print
orientation is chosen per part from the load - see below, it is worth a factor
of three.

A 0.6 knockdown is applied to printed strength and stiffness, matching the 60%
infill the mass model assumes. That is conservative for bending, where the solid
perimeters sit at the extreme fibre. Print a coupon and correct it.

---

## Results

Worst of in-plane and interlayer, each part in its chosen print orientation.

> **Every number in this file before 2026-08-02 was computed on a mesh with
> INVERTED ELEMENTS, and they were all too low.** See "The meshes were broken"
> below. The table here is the first one from meshes that are valid.

| part | PETG | ABS | PA6-CF |
|---|---|---|---|
| thigh | 47% | 58% | **23%** |
| shin | **169%** | **209%** | **83%** |
| ankle yoke | 71% | **92%** | **41%** |
| roll bracket | **201%** | **248%** | **99%** |
| chassis | 27% | 33% | **13%** |
| wheel body | 8% | 10% | **4%** |
| leg assembly | 91% | **112%** | **45%** |

**PA6-CF still passes every part, and the roll bracket passes by 1%.** That is
not a margin, it is a coincidence. It was reported at 70% this morning and at
84% before that, and both of those came off broken meshes.

The roll bracket now also FAILS in MJF PA12 (118%), SLS PA12GF (126%) and FDM
PA12CF (135%). Those are the three print services this project was actually
choosing between, so the material decision is no longer "PA6-CF is comfortable,
the others are close" - it is **PA6-CF or nothing**, on one part, at 99%.

The shin went 60% to 83% for the same reason and is the second-worst part.

The leg assembly appears here for the first time. It never converged before,
because its mesh was the worst of the seven at -0.622, and a matrix that is not
positive definite does not have to fail loudly. At 45% it is well inside PA6-CF,
which says the joints are not where the trouble is - the individual parts are.

The shin and the roll bracket are the two parts that reach the ankle-pitch
joint, and that is not a coincidence: the bearing is forced 68 mm outboard and
110 mm down, and no amount of ribbing makes a long cantilever cheap.

### What the ribs bought

| part | before | after | what changed |
|---|---|---|---|
| thigh | 161% | **76%** | plate 6 -> 9 mm, two corner ribs |
| shin | 322% (interlayer) | **98%** | print orientation, plus one rib |
| roll bracket | 1596% | **146%** | a plate to the axis, not a crank |

Two lessons out of the shin, both worth more than the mass:

**Print orientation was worth more than any rib.** The shin's interlayer went
322% -> 91% for nothing but building it up in x instead of on its side. Its
old orientation had been chosen for a shape that no longer existed.

**A rib that stops in mid-air moves the concentration, it does not remove it.**
Two triangles bridging the shin's two hot corners dropped the peak from 167% to
116% - and the new peak appeared at y = 45, exactly where both of them ended.
One continuous rib, terminating on structure at both ends, is what worked.

**And the member could not simply be made deeper.** Taking the crossing from
9 mm to 15 mm buys 116% -> 107%, and then collides with the wheel-drive servo
5.3 mm deep at 64% of the flip. There is nowhere to put the extra section.

### Print orientation is free### Print orientation is free and it is not a detail

`layer` is chosen per part from the load, not by laying each on its biggest
face. Interlayer utilisation in PETG, across the three axes:

| part | y (on its side) | z (flat on the bed) | x (built up in x) |
|---|---|---|---|
| thigh | 74% | 136% | **36%** |
| shin | **13%** | 109% | 40% |
| roll bracket | 202% | **78%** | 222% |
| chassis | 76% | 81% | **35%** |

The roll bracket goes from 202% to 78% for nothing but turning it on the bed.
Get this wrong and a part that passes fails, with no other change to anything.

### Three faults this found


**1. The roll bracket's two members meet over a 0.5 mm sliver.**

`ROLL_ARM` and `ROLL_TIE` overlap in a box **2.00 x 0.50 x 2.65 mm** - 2.65 mm3
of contact - and the entire wheel load passes through it. It fails at 16x in
PETG and still fails at 1.65x in **aluminium**. The picture localises it
exactly: one magenta blob at the junction, everything else deep blue.

This is why the material comparison is worth having. No material fixes it. It
is a geometry fault wearing a stress fault's clothes.

Note that a topological connectivity check passes this part - it *is* one
connected solid. Connectivity was the check added after five bodies turned out
not to be one piece, and it has a tolerance of 4 mm. A 0.5 mm lap satisfies it
and is structurally worthless.

**2. The roll bracket has no holes at all.**

Its final volume equals its solid volume to the milligram: all four
subtractions - wheel axle bore, roll bearing bore, both wheel-servo bolts - cut
air. They are drawn at z = 0, on the wheel spin axis, but the arm they are meant
to pass through spans z = 12.35 .. 22.35. Nothing can be attached to this part.

**3. The ankle-pitch joint is not drawn.**

The sim puts the ankle-pitch axis at (0, 0, -110) in the shin's frame. The
shin's bearing bore is at (-48, 21, -88) - **52.8 mm away**. The ankle yoke has
no matching feature at all, and assembled, the two parts come no closer than
**10.3 mm** with no shaft between them.

This is the same class as the knee hub that sat 14 mm off its own axis, and a
bigger instance: the joint does not exist. It also means the shin's number above
is computed with the lever arm as drawn, not the kinematic one.

### What the materials are worth

- **PA6-CF** is the only printed material that passes everything except the
  roll bracket, and it halves deflection against PETG. It needs a hardened
  nozzle and drying, and it costs three to four times as much.
- **PLA** beats PETG here on both counts, which is what the numbers say and not
  what most people expect - it is stiffer, so it deflects less, and its layer
  bond is better. It creeps under sustained load in a warm room, which is
  exactly what a standing robot applies, so treat this as a caveat and not a
  recommendation.
- **ABS** is the worst of the four on every part. Its only argument is
  toughness in impact, which none of this captures.
- **Aluminium** ends every argument except the roll bracket, at 2.1x the density.

None of this is worth acting on until the three geometry faults are fixed,
because two of them change what the parts are.

## Still open

- No fatigue. These are static ultimate checks; a part that passes at 78% can
  still fail after ten thousand flips.
- No fasteners in the model. Bolts are modelled as their hole patterns, and
  bearing stress on a printed M2 hole is not checked anywhere.
- Anisotropy is checked, not modelled. The solve is isotropic and the layer
  direction is post-processed out of the stress tensor. A real orthotropic
  solve would redistribute load, usually making it slightly worse.
- Material properties are from published data, not from coupons off this
  printer with this filament. Expect +/- 20%.
