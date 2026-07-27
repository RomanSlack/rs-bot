# Strength with the robot assembled

`cad/stress.py` solves every part **alone**, rigidly clamped at its own bolt
holes. That is the right way to size a part, and it is wrong in two ways for
answering "will the robot work":

- a rigid clamp is a wall where a compliant neighbour should be, and
- nothing ever adds the deflections up.

So the leg is now also solved as **one fused solid**, held at the hip and
loaded at the wheel. `cad/assemble_check.py: fused_leg()` builds it from the
posed parts, `cad/stress.py` has it as `leg_assembly`.

## What it says

PA6-CF, at the factored (flip x3) load:

| | deflection |
|---|---|
| thigh | 2.01 mm |
| shin | 8.70 mm |
| ankle yoke | 1.28 mm |
| roll bracket | 2.46 mm |
| **sum of the isolated parts** | **14.45 mm** |
| **fused leg, one solve** | **13.23 mm** |

**The two agree to 8%.** That is the useful result, and it cuts the way you
want: it means the per-part method has not been quietly lying. The assembly
comes out slightly stiffer because fusing across a servo output shaft models
that joint as rigid (see the caveat below), and slightly softer load sharing
elsewhere does not make up the difference.

Full material table, `uv run python -m cad.stress leg_assembly`:

| material | peak MPa | in-plane | interlayer | deflection |
|---|---|---|---|---|
| PETG | 20.9 | 74% | 55% | 39.7 mm |
| PLA | 20.9 | 63% | 43% | 22.7 mm |
| ABS | 20.9 | 92% | 71% | 36.1 mm |
| **PA6-CF** | 20.9 | **37%** | **32%** | **13.2 mm** |
| MJF PA12 | 20.9 | 44% | 15% | 44.1 mm |
| SLS PA12GF | 20.9 | 47% | 16% | 30.5 mm |
| FDM PA12CF | 20.9 | 50% | 40% | 19.9 mm |
| Al 6061-T6 | 20.9 | 8% | 3% | 1.2 mm |

13.2 mm is at the **ultimate** load. At 1x service it is 4.4 mm.

## Three things this model is NOT

**It does not supersede the per-part stress numbers.** Its mesh is
curvature-driven and coarse in the bulk, so it under-resolves the very stress
concentrations the per-part solves exist to find - which is exactly why its
peak reads 21.2 MPa where the shin alone reads 49.6. Per-part remains
authoritative for stress; this model is for the global load path and for
deflection.

**It is a lower bound on deflection.** Fusing across the servo output shafts
models every joint as rigid. Real gearboxes wind up under load, and the servos
have 0.87 deg of measured backlash on top of that - 0.87 deg at the ankle is
1.7 mm at the wheel before the structure has bent at all. The real leg is
softer than 13.2 mm.

**It does not answer the running-clearance question.** The wheel and the roll
bracket are 0.9 mm apart and both hang off the end of the leg, so most of the
13.2 mm is global tip motion they make *together* and it does not close the
gap. What could close it is differential bending across the arm, and this model
still cannot see that, because the wheel itself is not in it - it turns, so it
cannot be fused to anything.

## What fusing it found: the wheel servo was on a standoff

Fusing is a check in its own right, because an exact boolean union only closes
if the parts really touch. The first attempt returned **two solids**. The
second was the wheel-drive servo: it sat **0.5 mm off** the roll bracket arm it
bolts to.

It came from `ROLL_ARM` starting at z = 12.9 when the servo's face is at 12.4 -
somebody cleared a 0.05 mm interference by moving 0.55 mm. It measured 0.500 mm
both at HEAD and after the arm was moved for wheel clearance, so it was old and
it was not a side effect of that work. Nothing had ever reported it, because
`assemble_check` only tested for interference and 0.5 mm of air reads exactly
like 20 mm of air.

Why it matters: the servo's reaction is a force couple across its mounting
bolts, and this project has already been bitten by exactly that. From the
2026-07-26 status report, on the *ankle* servo:

> Its bolts were 12 mm apart. Its 1.63 N.m reaction is a force couple, so that
> is 136 N per bolt. This is the class of fault that survives assembly and
> fails a week later.

On a 0.5 mm standoff those bolts also work in bending rather than shear, and
the joint has no friction preload at all.

**Fixed.** `ROLL_ARM` now starts at z = 12.40, exactly the servo face, so the
arm seats flat the way the other bolted faces in this robot do. The gap is
0.0000 mm with 0.00 mm3 of interference - the case is a bought part and the
bracket may not eat into it. `assemble_check` now counts 16 seated faces
instead of 14, the leg fuses into one solid, and the roll bracket came out
slightly stronger for the thicker arm (96 -> 94% in PA6-CF).

`test_every_servo_seats_flat_on_what_it_bolts_to` checks all four mounts, so
the next one cannot drift the same way.

### One modelling note that came out of it

With the servo fused in, the solve stopped converging - residual 17x the
applied load - and the cause was specific: the load was being applied to the
servo's own nodes. It is a bought box joined to the leg over a single
22.7 x 10.8 mm face, and pushing on the far side of it conditions the system
badly. The load now goes on the arm patch the servo bolts through, which is
where the wheel's reaction enters the *printed* structure and is the same
idealisation the per-part roll_bracket entry makes. The servo stays in the
solid, so it still has to seat and still carries its mass. Deflection agrees
between the two placements to 0.02 mm.

## Running it

```bash
uv run python -m cad.stress leg_assembly    # ~40 s, 38k nodes
uv run python -m cad.stress                 # everything, per part
```

The assembly is meshed with curvature-driven sizing rather than uniform.
Uniform sizing has to resolve the M2 holes everywhere, which the 230 mm leg
turns into 105k nodes and 314k DOF - and the direct solve does not converge
there, returning a residual 128x the applied load. `cad/fea.py: mesh_step` now
takes a `curvature` argument for this.

## CPU

`cad/__init__.py` caps BLAS, OpenMP and gmsh to half the machine's cores,
because these runs otherwise take every core and the desktop stops responding.
Override with `RSBOT_THREADS=n`, or `RSBOT_THREADS=0` for no cap.
