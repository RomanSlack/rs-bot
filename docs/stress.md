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
| `cad/loads.py` | reads the wrench crossing every joint | closes against statics to 0.01 N |
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
It comes out 6.10 N against 6.10 N. If that had not closed, every number below
would have been wrong in the same direction and none of the pictures would have
shown it.

### Three cases, three factors

| case | what it is | factor | why |
|---|---|---|---|
| quiet | standing still | - | reference |
| flip | a full wheel -> foot -> wheel cycle | **x3** | worst normal operation |
| shove | the rated 1.0 N.s disturbance | **x1.5** | already a limit event |

Factoring the shove by 3 as well would count the same conservatism twice and
turn an 87 N transient into a fictional 261 N. The shove is delivered as 100 N
for 10 ms, which is an impulse approximation and not a measurement of any real
knock - the same impulse over 50 ms would load the structure far less. Read the
shove rows as an upper bound on a hand-swipe.

Design loads that come out of this:

| part | wrench at the joint below | governed by |
|---|---|---|
| thigh | 131 N, 10.90 N.m | shove |
| shin | 95 N, 2.30 N.m | shove |
| ankle yoke | 46 N, 1.82 N.m | flip |
| roll bracket | 47 N, 1.81 N.m | flip |
| chassis | 60 N payload + shove | its own cargo |

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
half the strength. The second is how a printed part actually fails. Every part
is assumed printed lying on its largest face, layer normal along y.

A 0.6 knockdown is applied to printed strength and stiffness, matching the 60%
infill the mass model assumes. That is conservative for bending, where the solid
perimeters sit at the extreme fibre. Print a coupon and correct it.

---

## Results

| part | PETG | PLA | ABS | PA6-CF | Al 6061-T6 |
|---|---|---|---|---|---|
| thigh | **154%** | **133%** | **192%** | 78% | 16% |
| shin | 69% | 58% | 84% | 34% | 7% |
| ankle yoke | 9% | 4% | 6% | 3% | 1% |
| roll bracket | **1596%** | **1382%** | **2000%** | **799%** | **165%** |
| chassis | 35% (76% interlayer) | 30% | 44% (99%) | 18% | 4% |

Worst of in-plane and interlayer. Bold is over the allowable.

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

### Two things that were already suspected, now with numbers

**The thigh is the limiting printed part, and not where the widening went.**
The spine was widened from 12 to 16 mm because lateral bending at the hip is
reacted by structure alone. But the peak is on the **knee-servo mounting
plate**, at 154% in PETG - the 6 mm inboard plate the whole shin hangs off.
Widening the spine did not touch it.

**The chassis is soft, as suspected.** 13.8 mm of sway in PETG under the rated
lateral shove, and interlayer is the binding constraint at 76% (99% in ABS) -
side plates printed flat get their layers pulled apart in exactly this load
case. A rear brace, already on the open-items list, is the fix.

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
