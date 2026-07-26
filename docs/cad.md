# Going to real CAD

The current model is a parametric sketch: boxes and cylinders at real
dimensions, with masses I assigned by hand. To actually build the robot it has
to become real parts with bolt holes, bearing seats, wire routing and a belt,
and the masses have to come from the geometry rather than from my estimates.

The requirement is narrow, which makes the choice easy: it must be **scriptable
in Python**, export **STEP** for manufacture, export **meshes MuJoCo can load**,
and give **mass and inertia from the solid**. That last one matters most,
because "mass distribution is stale" is currently an open item on the project
and CAD is what closes it for good.

## Recommendation: build123d

Validated end to end in this repo (`cad/shin.py`, runs in about a second).

| | |
|---|---|
| Kernel | Open CASCADE - the same B-rep kernel FreeCAD uses, not a mesh hack |
| API | Python, context managers rather than CadQuery's method chaining |
| Exports | STEP, STL, 3MF, SVG, DXF |
| Mass properties | volume, centroid, full inertia matrix from the solid |
| Build time | 0.06 s for a bolted structural part |
| Licence | Apache 2.0 |

**Why not the alternatives:**

- **CadQuery** is the obvious rival and shares the same kernel. build123d is
  effectively its successor: same capability, a much less awkward API, and it
  is where the momentum is. Either would work; the API is the tiebreak.
- **OpenSCAD** is CSG-only with no real B-rep, no STEP export, and no mass
  properties. Fine for brackets, wrong for a machine.
- **FreeCAD** has a genuine kernel and a Python console, and RobotCAD /
  OVERCROSS can emit URDF from it. But it is GUI-first: driving it headless
  from a script is fighting the tool. Worth keeping as the **viewer** for STEP
  files build123d produces.
- **Onshape + onshape-to-robot** is the most mature CAD-to-simulation pipeline
  that exists, and it exports MuJoCo directly. It is also **not open source**,
  and the free tier makes your documents public. If that is acceptable it is
  arguably the strongest option; it was excluded here on the open-source
  requirement.

## Vendored, not just installed

build123d lives in this repo as a submodule at `vendor/build123d`, pointing at
a fork under `RomanSlack/build123d` with `gumyr/build123d` as `upstream`. It is
installed **editable**, so changes to the CAD library take effect immediately
and can be pushed back to the fork or upstream as a PR.

```bash
git clone --recurse-submodules <this repo>      # or, if already cloned:
git submodule update --init --recursive
uv sync
```

`pyproject.toml` pins it with `[tool.uv.sources]`. Without that pin a `uv sync`
quietly swaps in the published wheel and edits stop having any effect, which is
a miserable thing to debug.

**One conflict this surfaced, worth knowing:** build123d depends on
`threejs-materials`, which pins `pillow < 12.3`, while `render.py` had asked for
`pillow >= 12.3`. Our constraint was arbitrary - we only use `Image`,
`ImageDraw` and `ImageFont` - so we relaxed ours rather than patching a
dependency. Verified: renderer and all 43 tests still pass on pillow 12.2.

To pull upstream changes later:

```bash
cd vendor/build123d && git fetch upstream && git merge upstream/dev
```

## The validated pipeline

```
build123d  --export_step-->  STEP   ->  printer / machine shop
           --export_stl -->  STL    ->  MuJoCo <mesh>
MuJoCo     computes mass and inertia from the actual geometry
```

Measured agreement between CAD and MuJoCo: **0.000%**.

### Two traps, both found by measuring rather than reading docs

**Units.** build123d exports millimetres; MuJoCo assumes metres. Without a
scale the part comes out 10^9 times too heavy - the first test reported a
28,000 tonne shin.

```xml
<mesh name="spine" file="shin_spine.stl" scale="0.001 0.001 0.001"/>
```

**Convex hull.** MuJoCo computes mesh inertia from the convex hull by default,
which quietly fills every bolt hole and pocket. Tightening the STL tolerance
does nothing, because tessellation was never the problem:

| mesh `inertia=` | mass error vs CAD |
|---|---|
| `legacy` (default) | +2.146% |
| `convex` | +1.609% |
| `exact` | **+0.001%** |

Set `inertia="exact"` on every structural mesh. On a robot whose whole design
turns on a 2.05 kg mass budget and a CoM trimmed to a millimetre, a silent 2%
error on every part is not acceptable.

## Looking at a part

```bash
uv run python cad/view.py                 # renders cad/out/shin.stl
uv run python cad/view.py path/to.stl     # or any other export
```

Loads the STL into a bare MuJoCo scene and renders side / three-quarter /
front into one PNG. No CAD GUI needed, and it is worth doing every time: the
first version of the shin had a lightening pocket that looked reasonable in
code and turned out to cut a 10 mm slot clean through a 20 mm spine, leaving a
thin-necked keyhole. It saved about 2 g on a 17 g part and invalidated the
stiffness figure, which assumes a solid section. Removed after one look.

For a proper CAD GUI on the STEP files, FreeCAD opens them directly.

## Shin: manufacturability review

Reviewed before porting anything else, on the principle that the same mistakes
would otherwise be repeated seven more times. Three were real.

### Three faults, all fixed

**The knee hub had no attachment at all.** The horn bolt circle was at 7.5 mm
radius inside a 10 mm bore, so the four M2 holes cut nothing but air. The part
could not have been bolted to the knee servo. Now a 4 mm bore with the bolt
circle at 8 mm, inside a 13 mm hub: 2.9 mm of material to the bore, 3.9 mm to
the rim.

**The arm was the weak member.** Peak bending stress 18.8 MPa on a 3x design
load, only 2.1x against PETG's ~40 MPa in-plane - and uncomfortably close to
its ~20 MPa *across* layers, which is what you get if the print orientation is
wrong. Deepened 10 -> 14 mm, now 10.8 MPa and 3.7x.

**The ankle servo's bolts were 12 mm apart.** Its reaction torque peaks at
1.63 N.m measured, and a couple through bolts that close is **136 N per bolt**.
The standoff now runs the length of the servo case with bolts 40 mm apart:
41 N. This is the sort of thing that survives assembly and fails a week later.

**The hub was 14 mm off the knee axis.** Found only when placing the servos:
the hub sat at z = -14 while the knee joint is at z = 0, so the part would not
have pivoted about its own joint. No amount of stress margin saves that. Now
centred on the axis.

Plus a fifth from simply looking at the render: a lightening pocket that cut a
10 mm slot through a 20 mm spine, leaving a thin-necked keyhole. Removed.

### Stress, on measured loads

Peak ground reaction on one wheel, measured in sim across quiet standing,
driving, a 1.0 N.s shove and a full flip cycle: **18.3 N normal, 12.8 N
lateral, 3.8 N fore/aft**, worst case during the flip - about 2.2x static.
Applying a 3x design factor, because a desk robot will get knocked off a desk:

| member | section | peak stress | margin vs 40 MPa |
|---|---|---|---|
| post | 12 x 12 | 1.6 MPa | 25x |
| arm | 12 x 14 | 10.8 MPa | **3.7x** |
| spine | 20 x 12 | 8.2 MPa | 4.9x |

The arm remains the governing member. Everything is now above 3x on a load
that is itself 3x measured peak.

### Fitted with its servos

```bash
uv run python -m cad.assembly     # shin + knee servo + ankle servo
```

Two STS3215s touch this part: the knee servo, whose case bolts to the thigh
and whose horn drives the hub, and the ankle-pitch servo, whose case bolts to
the standoff. Both are placed by their mounting rather than by eye, and
checked by boolean intersection rather than by looking:

| pair | intersection |
|---|---|
| knee servo x shin | 0.0 mm3 |
| ankle servo x shin | 0.0 mm3 |
| servo x servo | 0.0 mm3 |

Assembly envelope **72 x 90 x 134 mm**. Note the 90 mm in y: the shin itself is
19 mm, and the two servo cases either side account for the rest. That is worth
carrying into the other links, because it is what actually sets the robot's
width.

## Thigh

Ported second. Same review, and one design decision fell out of it.

**The governing load in the whole leg is at the hip, and no servo reacts it.**
There is no hip roll joint, so a lateral force at the contact patch is carried
entirely by the structure: 38 N design load at 247 mm is **9.5 N.m**. At the
shin's 12 mm width that is a 2.0x margin, the thinnest anywhere in the robot.
The thigh spine is therefore **16 mm wide rather than 12**, which takes it to
3.6x. Fore/aft is never the problem there - the hip servo's own 1.67 N.m peak
works out at 25x.

**The knee servo mount has to reach around the servo, not bolt beside it.** The
case occupies y = -20.4 .. 15, which is exactly where the shin's hub also
wants to be. Bolting to the outboard face would put the bracket through the
shin. The thigh instead crosses over the top of the case and comes down the
inboard side to a 6 mm plate carrying all four mounting bolts.

Checked, not eyeballed:

| check | result |
|---|---|
| thigh x knee servo case | 0.0 mm3 |
| thigh x shin, knee swept 0 to -110 deg in 10 deg steps | 0.0 mm3 at every angle |

Sweeping the knee matters for the same reason sweeping the flip did: two links
that clear each other in one pose can still scissor in between.

Thigh is 31.7 g printed. The robot is now 1759 g with two links CAD-derived.

### One regression, stated plainly

The heavier, differently distributed leg costs a little backlash tolerance:
the full round trip now survives to **1.75 deg** rather than 2. Under 1 deg
nothing changes, and the measured STS3215 figure is 0.87 deg, so this sits
inside the design target - but it is a real narrowing and the margin at 2 deg
is gone.

### Printing it

Lay the part with its **y axis vertical**: 72 x 99 mm footprint, 19 mm tall.
That orientation is not arbitrary:

- the part is essentially a profile extruded along y, so there are no
  overhangs and it needs no support
- both bores - the knee hub and the ankle bearing - run along y, so they print
  as vertical circles with no bridging and no elephant-footing on the seat
- bending loads put tension along x and z, which are **in-plane**. Printed flat
  instead, the same loads would pull across layers at ~20 MPa, and the arm's
  10.8 MPa would be a 1.9x margin rather than 3.7x

### Still not verified

- **Beam theory, not FEA.** Fine for prismatic members, unreliable at the
  fillets and the hub, which are exactly where a real part cracks.
- **The bearing fit.** A 623ZZ counterbore is modelled at nominal 10 mm; a
  printed bore needs measuring and offsetting, usually 0.1-0.2 mm.
- **Thread engagement.** M2 clearance holes are drawn, but nothing specifies
  heat-set inserts versus self-tapping into plastic. Inserts, for anything
  that will be taken apart more than twice.
- **The ankle-pitch belt** is still neither drawn nor sized.

## What this buys, concretely

1. **The mass budget stops being a guess.** Every part's mass, centroid and
   inertia tensor comes from its geometry. The open item about the shin
   carrying 100 g on a stale assumption disappears - the number just becomes
   whatever the part weighs.
2. **The sim and the parts cannot drift apart.** One script generates the STEP
   you print and the STL the simulator loads. Today they are two separate
   descriptions of the robot that I have to keep in agreement by hand.
3. **The fit checks get sharper.** `fitcheck.py` currently approximates every
   part as an oriented box. Against real solids it can use true clearances.

## Method

The step-by-step process, the design rules that came out of it, and the traps -
written up in [porting-a-link.md](porting-a-link.md). Read that before porting
the next link; every rule in it exists because something broke.

## Suggested order of work

1. Port one link - the shin - to build123d with real bolt holes and a bearing
   seat, and drive its mass into the MuJoCo model from the solid.
2. Re-verify the balancer against the corrected mass distribution. This is the
   open item that most wants closing before anything gets printed.
3. Port the rest of the structure, then the ankle, which is the region that
   most needs real CAD: three DOF inside 45 mm, two of them clearing a wheel
   that changes shape, and a belt that is currently neither drawn nor sized.
4. Keep the servos, wheels, Pi and battery as simple blocks. They are bought
   parts; modelling them in detail buys nothing beyond their bounding box and
   mass, both of which are already known.

## Sources

- [build123d](https://dev.opencascade.org/project/build123d) on Open CASCADE
- [build123d vs CadQuery comparison](https://www.oreateai.com/blog/build123d-vs-cadquery-navigating-the-future-of-python-cad-modeling/b9e17e3134422786a0ab67c0a6d1eeda)
- [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) - URDF/SDF/MuJoCo export from Onshape
- [RobotCAD for FreeCAD](https://github.com/drfenixion/freecad.robotcad) - CAD to ROS2/URDF
