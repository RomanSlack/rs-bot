"""Stress in every printed part, in five candidate materials.

    uv run python -m cad.stress            # all parts, all materials
    uv run python -m cad.stress shin       # one part

Loads come from `cad.loads` (measured off the sim, not estimated), the solver
is `cad.fea` (verified against a cantilever before being pointed at anything),
and the output is one PNG per part: the same part under the same load in each
material, coloured by how close it is to breaking.

WHY THE PICTURES LOOK ALIKE: in a linear-elastic solve with prescribed loads,
the stress FIELD barely depends on the material. Stiffness cancels out; only
Poisson's ratio has any effect, and it is small. What changes between materials
is how much of the allowable that same stress uses up, and how far the part
bends. So the panels are coloured by UTILISATION - stress over allowable - not
by stress. If they were coloured by stress they would be identical, and that
would be a picture of nothing.

Printed parts get two checks, because a printed part is not one material:

    in-plane     along the layers. Strong. This is ordinary bending.
    interlayer   pulling the layers apart. Roughly half as strong, and how a
                 printed part actually fails. Reported separately.
"""

import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from cad import fea, loads

OUT = Path(__file__).parent / "out"

# A print is not solid. The mass model assumes 60% infill and so does this,
# knocked off both strength and stiffness. It is conservative for bending,
# where the solid perimeters sit at the extreme fibre and carry most of it.
# Print a coupon and correct it.
INFILL = 0.60

# name, E MPa, nu, in-plane ultimate MPa, interlayer ultimate MPa, g/cm3, note
MATERIALS = [
    ("PETG",      2000.0, 0.40,  47.0,  22.0, 1.27, "the baseline"),
    ("PLA",       3500.0, 0.36,  55.0,  28.0, 1.24, "stiffer, brittle, creeps warm"),
    ("ABS",       2200.0, 0.35,  38.0,  17.0, 1.04, "tough, warps"),
    ("PA6-CF",    6000.0, 0.38,  95.0,  38.0, 1.15, "nylon + carbon; needs a hot end"),
    ("Al 6061-T6", 69000.0, 0.33, 276.0, 276.0, 2.70, "machined, isotropic"),
]
PRINTED = {"PETG", "PLA", "ABS", "PA6-CF"}


def knockdown(name):
    return INFILL if name in PRINTED else 1.0


# --- node selection ----------------------------------------------------------

def near_axis(nodes, point, axis, radius, half_len=1e9):
    """Nodes within `radius` of a line, and within `half_len` along it. This is
    how a bore or a bolt hole gets picked out without naming CAD faces."""
    p, a = np.asarray(point, float), np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    r = nodes - p
    along = r @ a
    perp = np.linalg.norm(r - along[:, None] * a, axis=1)
    return np.where((perp < radius) & (np.abs(along) < half_len))[0]


def in_box(nodes, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.where(np.all((nodes >= lo) & (nodes <= hi), axis=1))[0]


# --- the parts ---------------------------------------------------------------
#
# Each entry says where the part is HELD and where the load COMES IN, and both
# have to be the real interfaces or the answer is fiction.
#
# `layer` is the print orientation: the normal to the layers, i.e. the axis the
# part is built up along. It is NOT "lay it on its biggest face" - it is chosen
# per part from the load, because it is free and it matters more than expected.
# Interlayer utilisation across the three axes, in PETG:
#
#     part            y (on its side)   z (flat)   x (built up in x)
#     thigh                  74%          136%           36%   <- best
#     shin                   13%  <-best   109%           40%
#     roll bracket          202%           78%  <-best   222%
#     chassis                76%            81%           35%   <- best
#
# The roll bracket goes from 202% to 78% for nothing but turning it on the bed.
# Get this wrong and a part that passes fails, with no other change.

def _parts():
    return {
        "thigh": dict(
            step="thigh.step", layer=(1, 0, 0),
            # Bolted to the hip servo horn: the four M2 on the 8 mm bolt circle.
            fix=lambda n: np.concatenate([
                near_axis(n, (8 * np.cos(a), 21, 8 * np.sin(a)), (0, 1, 0), 2.2)
                for a in np.deg2rad([45, 135, 225, 315])]),
            # The shin hangs off the knee servo, whose case bolts to the
            # inboard plate through these four. That is the whole load path.
            load=lambda n: np.concatenate([
                near_axis(n, (dx, -23.4, dz), (0, 1, 0), 2.2)
                for dx in (-8.5, 8.5) for dz in (-115.0, -79.8)]),
            at=(0, 0, -110), key="thigh", size=3.0,
            note="held at the hip horn, loaded through the knee servo bolts"),

        "shin": dict(
            step="shin.step", layer=(1, 0, 0),
            fix=lambda n: np.concatenate([
                near_axis(n, (8 * np.cos(a), 21, 8 * np.sin(a)), (0, 1, 0), 2.2)
                for a in np.deg2rad([45, 135, 225, 315])]),
            # The ankle-pitch bearing, now ON its axis.
            load=lambda n: near_axis(n, (0, 68, -110), (0, 1, 0), 11.5),
            at=(0, 68, -110), key="shin", size=3.0,
            note="held at the knee horn, loaded at the ankle-pitch bearing"),

        "ankle_yoke": dict(
            step="ankle_yoke.step", layer=(0, 1, 0),
            # Held at its ankle-pitch shaft, which is a real feature now.
            fix=lambda n: near_axis(n, (0, 58, 0), (0, 1, 0), 8.0),
            load=lambda n: np.concatenate([
                near_axis(n, (-56, dy, dz), (1, 0, 0), 2.2)
                for dy in (-4.0, 4.0) for dz in (-4.0, 4.0)]),
            at=(-56, 0, 0), key="ankle", size=3.0,
            note="held at the pitch shaft, loaded at the roll servo"),

        "roll_bracket": dict(
            step="roll_bracket.step", layer=(0, 0, 1),
            # Selected on solid material, not on holes: this part has none.
            # All four of its drawn features cut air (see docs/stress.md), so
            # the interfaces are taken where they physically have to be - the
            # tie end that meets the yoke, and the two patches of arm the
            # wheel servo must bolt through.
            # Held at the roll hub, which is now a real feature on the real
            # axis, and loaded where the wheel servo bolts through the arm.
            fix=lambda n: near_axis(n, (-43, 0, 0), (1, 0, 0), 8.5, half_len=3.0),
            load=lambda n: np.concatenate([
                in_box(n, (-36.5, 11, 11), (-31.5, 24.5, 23.5)),
                in_box(n, (-12.5, 11, 11), (-7.5, 24.5, 23.5))]),
            at=(0, 18, 0), key="rollbracket", size=2.5,
            note="held at the roll hub, loaded through the servo bolts"),

        "chassis": dict(
            step="chassis.step", layer=(1, 0, 0),
            # BOTH hips held, not one. The first version held one hip's four
            # bolts and hung the whole robot off them, which bent the chassis
            # 71 mm - a load case that cannot happen, because the other leg is
            # on the floor too. Held at both, the chassis is a box carrying its
            # payload, which is what it actually is.
            fix=lambda n: np.concatenate([
                near_axis(n, (dx, sgn * 38.1, dz), (0, 1, 0), 2.2, half_len=6.0)
                for sgn in (1, -1)
                for dx in (-17.0, 17.0) for dz in (5.0, 22.0)]),
            # Everything it carries hangs off the top plate and the shelves.
            # The top plate is the worst lever arm, so the payload goes there.
            load=lambda n: in_box(n, (-40, -36, 176), (57, 36, 181)),
            # Payload x SF3 down, plus the rated shove sideways. Not taken
            # from cad.loads: no joint carries this, it is the chassis's own
            # cargo, and the shove arrives at the torso directly.
            wrench=(np.array([0.0, 55.0, -24.6]), np.zeros(3)),
            at=(8.5, 0, 178.5), key=None, size=6.0,
            note="both hips held, payload x3 plus the rated shove on top"),
    }


# --- solving -----------------------------------------------------------------

def analyse(name, spec, wrench, size=None, verbose=True):
    """Mesh once, solve once per material. Returns a dict of results."""
    nodes, elems = fea.mesh_step(OUT / spec["step"], size=size or spec["size"])
    fea.check_ordering(nodes, elems)
    fixed = np.unique(spec["fix"](nodes))
    lnodes = np.unique(spec["load"](nodes))
    if verbose:
        print(f"{name:<13} {len(nodes):>6} nodes  {len(elems):>6} elems  "
              f"{len(fixed):>5} held  {len(lnodes):>5} loaded")
    if len(fixed) < 4 or len(lnodes) < 4:
        raise RuntimeError(f"{name}: boundary selection found almost nothing "
                           f"({len(fixed)} held, {len(lnodes)} loaded) - the "
                           f"feature coordinates do not match the STEP")
    if np.array_equal(fixed, lnodes):
        raise RuntimeError(f"{name}: the held nodes and the loaded nodes are "
                           f"the same set. Nothing can move, and the solve "
                           f"returns a confident 0.00 MPa")

    force, torque = wrench
    # The wrench is measured about the joint; move it to the centroid of the
    # nodes actually carrying it, or the moment is applied about the wrong point.
    c = nodes[lnodes].mean(axis=0)
    torque = torque + np.cross(np.asarray(spec["at"], float) - c, force)

    # ONE solve per part, not one per material.
    #
    # In linear elasticity with prescribed tractions the stress field does not
    # depend on E at all, and depends on Poisson's ratio only weakly. Measured
    # on this part, the peak away from the supports across the whole plausible
    # range of nu:
    #
    #     nu     0.33   0.35   0.38   0.40    0.42
    #     MPa   24.27  22.84  22.85  132.82  25.79
    #
    # Every value agrees within about +/-6% except nu = 0.40, which is not
    # physics: 0.42 is MORE incompressible and behaves perfectly, so it is not
    # volumetric locking. It is a spurious near-null-space mode that satisfies
    # equilibrium to 1e-8 and still poisons the stress tail. Solving once at a
    # representative nu is both more robust and five times faster.
    NU_REF = 0.35
    f = fea.distribute(nodes, lnodes, force, torque)
    u_ref, s_ref = fea.solve(nodes, elems, 1000.0, NU_REF, fixed, [(lnodes, f)])
    vm = fea.von_mises(s_ref)
    vm_raw, vm_p, kept = fea.report(vm, nodes, fixed)

    out = {}
    for mat, E, nu, s_xy, s_z, rho, note in MATERIALS:
        il = fea.interlayer(s_ref, spec["layer"])
        _, il_p, _ = fea.report(il, nodes, fixed)
        k = knockdown(mat)
        # Displacement scales exactly as 1/E, so the reference solve at
        # E = 1000 MPa rescales without re-solving.
        u = u_ref * (1000.0 / E)
        out[mat] = dict(u=u, vm=vm, il=il, nodes=nodes, elems=elems,
                        fixed=fixed, defl=float(np.abs(u).max()),
                        vm_raw=vm_raw, vm_p=vm_p, il_p=il_p, kept=kept,
                        allow=s_xy * k, allow_il=s_z * k,
                        util=vm_p / (s_xy * k), util_il=il_p / (s_z * k),
                        field=vm / (s_xy * k), rho=rho, note=note)
    return out


# --- rendering ---------------------------------------------------------------

def surface(elems):
    """Boundary triangles: the corner faces that belong to exactly one tet."""
    faces = np.vstack([elems[:, [0, 2, 1]], elems[:, [0, 1, 3]],
                       elems[:, [1, 2, 3]], elems[:, [0, 3, 2]]])
    key = np.sort(faces, axis=1)
    _, idx, cnt = np.unique(key, axis=0, return_index=True, return_counts=True)
    return faces[idx[cnt == 1]]


def ramp(t):
    """0 blue -> 0.5 green -> 0.8 yellow -> 1.0 red -> above 1.0 magenta.
    The break at 1.0 is deliberate: anything magenta is over the allowable."""
    t = np.clip(t, 0.0, 1.4)
    stops = [(0.00, (40, 70, 170)), (0.35, (40, 160, 190)),
             (0.55, (60, 180, 90)), (0.80, (235, 205, 60)),
             (1.00, (215, 55, 40)), (1.40, (225, 60, 220))]
    out = np.zeros((len(t), 3))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        m = (t >= t0) & (t <= t1)
        f = ((t[m] - t0) / (t1 - t0))[:, None]
        out[m] = np.array(c0) * (1 - f) + np.array(c1) * f
    return out.astype(np.uint8)


def render(nodes, tris, field, size, az=35.0, el=22.0, deform=None, scale=0.0):
    """Painter's-algorithm surface render, orthographic, coloured per node."""
    W, H = size
    pts = nodes + (deform * scale if deform is not None else 0.0)
    a, e = np.deg2rad(az), np.deg2rad(el)
    right = np.array([np.cos(a), -np.sin(a), 0.0])
    up = np.array([-np.sin(a) * np.sin(e), -np.cos(a) * np.sin(e), np.cos(e)])
    view = np.cross(right, up)

    c = pts.mean(axis=0)
    uv = np.stack([(pts - c) @ right, -((pts - c) @ up)], axis=1)
    depth = (pts - c) @ view

    span = max(uv.max(axis=0) - uv.min(axis=0))
    k = 0.86 * min(W, H) / max(span, 1e-9)
    px = uv * k + np.array([W / 2, H / 2]) - uv.mean(axis=0) * k

    im = Image.new("RGB", (W, H), (26, 27, 30))
    dr = ImageDraw.Draw(im)
    order = np.argsort(depth[tris].mean(axis=1))
    cols = ramp(field)
    # Flat shading off the facet normal, so the form reads instead of looking
    # like a coloured blob.
    n = np.cross(pts[tris[:, 1]] - pts[tris[:, 0]], pts[tris[:, 2]] - pts[tris[:, 0]])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    lit = 0.55 + 0.45 * np.clip(n @ (view * 0.5 + up * 0.6 + right * 0.3), 0, 1)

    for i in order:
        t = tris[i]
        col = cols[t].mean(axis=0) * lit[i]
        dr.polygon([tuple(px[j]) for j in t], fill=tuple(col.astype(int)))
    return im


def _font(sz):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(p, sz)
        except OSError:
            pass
    return ImageFont.load_default()


def sheet(name, spec, res, wrench, stamp):
    PW, PH, PAD, HEAD = 380, 430, 14, 118
    tris = surface(res["PETG"]["elems"])
    nodes = res["PETG"]["nodes"]
    W = PAD + len(MATERIALS) * (PW + PAD)
    H = HEAD + PH + 108
    im = Image.new("RGB", (W, H), (18, 19, 22))
    dr = ImageDraw.Draw(im)
    f18, f13, f11 = _font(19), _font(13), _font(11)

    f, t = wrench
    dr.text((PAD, 14), f"{name}", font=f18, fill=(240, 240, 245))
    dr.text((PAD, 42),
            f"design load {np.linalg.norm(f):.0f} N, {np.linalg.norm(t):.2f} N.m"
            f"   -   {spec['note']}", font=f13, fill=(160, 165, 175))
    dr.text((PAD, 62),
            "colour = von Mises / allowable.  yellow is 80%, red is at the "
            "limit, magenta is over it.", font=f13, fill=(160, 165, 175))
    dr.text((PAD, 82),
            "peaks quoted away from the supports: stress AT a fixed boundary "
            "is singular and never converges.  undeformed shape.",
            font=f13, fill=(130, 134, 142))

    for i, (mat, *_rest) in enumerate(MATERIALS):
        r = res[mat]
        x = PAD + i * (PW + PAD)
        panel = render(nodes, tris, r["field"], (PW, PH),
                       deform=r["u"], scale=0.0)
        im.paste(panel, (x, HEAD))
        ok = r["util"] < 1.0 and r["util_il"] < 1.0
        dr.rectangle([x, HEAD, x + PW - 1, HEAD + PH - 1],
                     outline=(70, 200, 110) if ok else (215, 70, 55), width=2)

        y = HEAD + PH + 6
        dr.text((x + 4, y), f"{mat}", font=f13, fill=(240, 240, 245))
        dr.text((x + 4, y + 18),
                f"peak {r['vm_p']:.1f} MPa  of {r['allow']:.0f} allowed",
                font=f11, fill=(190, 195, 205))
        dr.text((x + 4, y + 33),
                f"in-plane   {r['util']*100:>5.0f}%   "
                f"{'OK' if r['util'] < 1 else 'OVER'}",
                font=f11, fill=(120, 210, 140) if r["util"] < 1 else (235, 90, 75))
        dr.text((x + 4, y + 48),
                f"interlayer {r['util_il']*100:>5.0f}%   "
                f"{'OK' if r['util_il'] < 1 else 'OVER'}",
                font=f11,
                fill=(120, 210, 140) if r["util_il"] < 1 else (235, 90, 75))
        warn = "" if r["kept"] > 0.5 else \
            f"  (only {r['kept']*100:.0f}% clear of supports)"
        dr.text((x + 4, y + 63),
                f"deflects {r['defl']:.2f} mm{warn}",
                font=f11, fill=(150, 155, 165))
        dr.text((x + 4, y + 78), r["note"], font=f11, fill=(120, 125, 135))

    out = OUT / f"stress_{name}_{stamp}.png"
    im.save(out)
    return out


def main(only=None, size=None):
    OUT.mkdir(exist_ok=True)
    # Re-export first. This reads STEP files off disk, so without it an edited
    # part is analysed in its previous shape and the result looks like the fix
    # did nothing - which is exactly what happened once, and it is a confusing
    # thing to debug because every other number moves and one does not.
    import cad.robot
    cad.robot.export_all()
    print("measuring loads from the sim...")
    design = loads.design_loads()
    parts = _parts()
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")

    written, table = [], []
    for name, spec in parts.items():
        if only and name != only:
            continue
        if spec["key"] is None:
            f, t, gov = (*spec["wrench"], "its own payload")
        else:
            f, t, gov = design[spec["key"]]
        res = analyse(name, spec, (f, t), size=size)
        written.append(sheet(name, spec, res, (f, t), stamp))
        for mat, *_ in MATERIALS:
            r = res[mat]
            table.append((name, mat, r["vm_p"], r["allow"], r["util"],
                          r["util_il"], r["defl"]))
        print(f"{'':13} governed by {gov}")

    print(f"\n{'part':<14}{'material':<12}{'MPa':>7}{'allow':>7}"
          f"{'in-plane':>10}{'interlayer':>12}{'deflect mm':>12}")
    for row in table:
        n, mat, vm, al, u, ui, dfl = row
        flag = "  <-- OVER" if max(u, ui) >= 1.0 else ""
        print(f"{n:<14}{mat:<12}{vm:>7.1f}{al:>7.0f}{u*100:>9.0f}%"
              f"{ui*100:>11.0f}%{dfl:>12.2f}{flag}")
    for w in written:
        print(f"\nwrote {w}")
    return written


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    main(only=args[0] if args else None)
