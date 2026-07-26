# Porting a link to CAD: the method

Written after the shin and thigh, for whoever does the ankle, roll bracket and
chassis. Every rule here exists because something broke; the commit history has
the receipts.

The point of the exercise is not to draw pretty parts. It is to replace
estimates with measurements, and to find the faults that a parametric sketch
hides, before anything is printed. The shin had **five faults**. One of them
(the hub 14 mm off the knee axis) would have made the part useless, and no
amount of stress margin would have saved it.

---

## The loop

Do these in order. Each step has caught something the previous one could not.

```
1. read the sim geometry        the sim is the source of truth for placement
2. measure the loads            never assume them
3. build the part               real bolt holes, bearing seats, mounts
4. check the geometry           bolt circles, wall thicknesses, joint axes
5. check the stress             beam theory against measured loads
6. check the fit                boolean intersection, through the joint's RANGE
7. LOOK at it                   uv run python cad/view.py
8. drive the mass into the sim  cad/masses.py -> SEG_MASS
9. re-verify                    full suite; re-tune if the mass moved much
10. write down what changed     especially anything that got worse
```

### 1. Read the sim geometry

The sim already places every part. Do not invent positions:

```bash
uv run python -c "
import mujoco; from src.rsbot.model import load
m,d=load()
b=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'shin_l')
for i in range(m.ngeom):
    if m.geom_bodyid[i]==b and m.geom_group[i]==0:
        print(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i),
              m.geom_pos[i]*1000, m.geom_size[i]*1000)"
```

If the CAD needs a dimension the sim does not have, change the sim too and
re-run `fitcheck.py`. They must not drift apart.

### 2. Measure the loads

Peak joint torques and peak ground reaction, from the sim, across every mode -
quiet, driving, shoved, and a full flip cycle. The flip is almost always the
worst case and it is easy to forget.

Current numbers, at 1.759 kg:

| | hip | knee | ankle pitch | ankle roll | wheel |
|---|---|---|---|---|---|
| peak N.m | 1.67 | 2.90 | 1.63 | 0.74 | 1.35 |

Ground reaction on **one** wheel, worst case (flip cycle):
**18.3 N normal, 12.8 N lateral, 3.8 N fore/aft** - about 2.2x static.

Apply **SF = 3** to those before sizing anything. A desk robot gets knocked off
a desk.

### 3. Build the part

Conventions:

- **Millimetres.** The sim is in metres; convert at the boundary and nowhere
  else.
- **Same frame as the sim body.** Shin origin at the knee, thigh origin at the
  hip, z negative downward.
- Solid model, then subtract holes. `.clean()` at the end.

### 4. Geometry checks

These are the ones that have actually bitten:

| rule | why |
|---|---|
| **A hub must be centred ON its joint axis** | the shin's sat 14 mm below the knee; the part would not have pivoted |
| **Bolt circle radius > bore radius** | the shin's four M2 holes were inside the bore, cutting air |
| **Bolt spacing vs reaction torque** | a servo's reaction is a force couple. 1.63 N.m through bolts 12 mm apart is 136 N each; at 40 mm it is 41 N |
| **Bearing seats need a counterbore** | a plain hole runs a steel shaft against printed plastic |
| **No lightening pockets without re-checking stress** | one cut a 10 mm slot through a 20 mm spine and silently invalidated the stiffness figure |
| **A servo case bolts to the body PROXIMAL to the joint it drives** | hip servos on the torso, knee servo on the thigh, and so on. Getting this wrong left 100 g in the wrong link for weeks |

### 5. Stress

Beam theory is enough for the prismatic members. `S = b*h^2/6`, `sigma = M/S`,
against **40 MPa** for PETG in-plane.

Bending stress acts **along the beam axis**, whichever direction the load comes
from. So as long as the beam axis lies in the layer plane, bending is in-plane
and 40 MPa is the right number. Only tension across layers, or torsion, drops
you to ~20 MPa.

Aim for **3x or better** on the SF=3 load, i.e. ~9x on measured peak. Anything
at 2x is the weakest thing in the robot and should be widened.

Remember loads that **no actuator reacts**. There is no hip roll joint, so the
entire lateral moment at the hip - 9.5 N.m at the design load - is carried by
the structure alone. That, not any servo torque, is what sizes the thigh.

### 6. Fit

Never trust a picture for this. Use boolean intersection, and check through the
joint's **whole range of motion**, not just the assembled pose:

```python
for deg in range(0, -120, -10):
    moved = bd.Pos(0, 0, KNEE_Z) * bd.Rot(0, deg, 0) * shin
    assert (thigh & moved).volume < 1.0
```

Two links that clear each other in one pose can still scissor in between. The
same mistake at the robot level - checking only the two end poses of the flip -
hid the wheel-drive servo sweeping 20.7 mm through the shin.

The clearance rules the wheel imposes on anything that does not rotate with it:

| must clear | rule |
|---|---|
| upright wheel | \|y\| > 12 mm, or radius > 40 mm from the spin axis |
| flat wheel | \|x\| > 32 mm, or z > +12 mm above the axle |
| wheel-servo sweep | radius **< 14 mm** or **> 50 mm** from the roll axis, or \|x\| > 23 mm |

That third rule has a hole in the middle: parts sitting essentially *on* the
roll axis are inside the annulus and the sweep misses them. That is where the
ankle bearing carrier lives.

### 7. Look at it

```bash
uv run python cad/view.py        # 6 views, timestamped, never overwrites
uv run python -m cad.assembly    # with the servos fitted
```

Two of the five shin faults were found by looking, not by computing. Render
close-ups of the faces that carry features - on this robot everything faces +y
and is invisible from every other angle.

### 8-9. Mass, then re-verify

```bash
uv run python -m cad.masses      # printed + servos + bought, per body
```

Put the result in `SEG_MASS` and the torso geom mass, then run the full suite.
**Gains are mass-dependent.** Dropping 340 g made the old set fall over at 2 deg
of backlash; it needed a multi-start re-tune. If the mass moves more than ~5%,
expect to re-tune and re-measure the thresholds rather than nudging the tests.

### 10. Write down what got worse

Porting the thigh cost backlash tolerance: the round trip went from surviving
2.0 deg to 1.75. That is inside the design target (measured servo lash is
0.87 deg) but it is a real narrowing, and burying it would make the next
regression invisible.

---

## Two traps that are not about CAD at all

**Measuring the wrong thing.** A tuning cost that penalised falling at 100+ and
oscillation at 3x returned gains that never fell and limit-cycled at +/-16 deg.
"Did not fall" is not a fitness function. Likewise a connectivity check with a
4 mm tolerance - correct as a running clearance across a joint, far too
generous inside a rigid part - hid five bodies that were not one piece.

**Geometry errors that present as control failures.** Five separate times a
robot "fell over" and the cause was that something was touching the floor, or
a pose was not what it looked like. Before touching a gain: check what is in
contact, and check the pose is what you think it is.

---

## What not to model

Servos, wheels, the Pi and the battery are **bought parts**. Their bounding box
and mass are all that matter; modelling their internals buys nothing. Draw them
as blocks in `cad/assembly.py` so the mounts can be checked against something,
and stop there.

## Still open, robot-wide

- the ankle-pitch belt is neither drawn nor sized
- bearing bores are nominal; a printed bore needs measuring and offsetting,
  usually 0.1-0.2 mm
- nothing specifies heat-set inserts versus self-tapping. Use inserts for
  anything that will be taken apart more than twice
- beam theory, not FEA - unreliable at fillets and hubs, which is exactly where
  a printed part cracks
