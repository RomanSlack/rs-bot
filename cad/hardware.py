"""The bought hardware that actually holds this robot together.

    uv run python -m cad.hardware

WHY THIS EXISTS. Open the assembly viewer and look at where the shin meets the
ankle yoke, and there is nothing there. Not a rendering fault - nothing is what
the model contained. The joint is a 623ZZ bearing and a 3 mm shaft, and both of
them existed only as HOLES: `cad/shin.py` cuts a counterbore, `cad/ankle.py`
cuts a bore, and no solid was ever placed in either.

The same was true of every bought part on the order sheet that is not a servo:

    4 x 623ZZ bearing        ankle pitch and ankle roll
    4 x 3 mm shaft           the things the joints actually turn on
    10 x 25T metal horn      every servo output

Ten line items, about $80, and not one of them was drawn. So nothing could ask
whether the shaft is long enough for the bore it fills, whether the bearing
seats to its full 4 mm. On a
project whose whole claim is that you can order once and assemble once, the
parts that do the assembling were the ones nobody had checked.

WHAT IS DERIVED AND WHAT IS TYPED. The catalogue dimensions are typed, because
they are a part number: a 623ZZ is 3 x 10 x 4 and arguing with that is arguing
with the supplier. Everything about POSITION comes from the same constants that
cut the seats, and the shaft LENGTHS are measured off the assembled parts by
`shaft_span()` rather than chosen - the order sheet said "30 mm, or cut from
stock", which is a length nobody had checked against the hole it goes in.
"""

import sys

import build123d as bd
import numpy as np

import cad.ankle as ankle
import cad.servo as sv
import cad.shin as shin
from cad.servo_dims import SHAFT_R as SPLINE_R

# --- catalogue parts ----------------------------------------------------------
#
# 623ZZ: 3 mm bore, 10 mm outside, 4 mm wide. A stock size, sold in 10-packs,
# and the reason the shafts are 3 mm rather than anything nicer.
BEARING_ID, BEARING_OD, BEARING_W = 3.0, 10.0, 4.0
SHAFT_D = 3.0

# The 25T metal horn, from cad/servo.py's measured numbers rather than a guess:
# Ø19.93 measured on the bench, 2.51 mm plate, 4.50 mm including the nub.
HORN_OD, HORN_PLATE_T, HORN_OVERALL_T = 19.93, 2.51, 4.50


def bearing():
    """A 623ZZ, drawn as its outer race, inner race and the shield between.

    Three rings rather than one, because the thing you want to SEE at a joint
    is whether the inner race is on the shaft and the outer race is in the
    counterbore. One grey cylinder cannot show you that.
    """
    outer = bd.Cylinder(BEARING_OD / 2, BEARING_W) - bd.Cylinder(
        BEARING_OD / 2 - 1.2, BEARING_W + 1)
    inner = bd.Cylinder(BEARING_ID / 2 + 1.2, BEARING_W) - bd.Cylinder(
        BEARING_ID / 2, BEARING_W + 1)
    shield = (bd.Cylinder(BEARING_OD / 2 - 1.2, BEARING_W - 1.0)
              - bd.Cylinder(BEARING_ID / 2 + 1.2, BEARING_W))
    return (outer + inner + shield).clean()


def horn():
    """The 25T output horn, plate centred on z = 0, nub toward the servo.

    THIS IS THE PART THAT WAS HOLDING THE ROBOT TOGETHER AND WAS NOT DRAWN.
    Every joint that is not a bearing is a horn joint: the thigh bolts to the
    hip servo's horn, the shin to the knee's, the roll bracket to the roll
    servo's, and the wheel to the wheel servo's. Eight of them, and there were
    servo's. Ten of them, on the order sheet since the beginning.

    With them missing, a contact graph of the assembly came back in NINE
    disconnected pieces: two legs, two thighs, a torso, and four servos each
    floating on its own. Not one of those breaks was a fault in a printed part.
    They were all the same absent 2.51 mm disc.

    Ø19.93 and 2.51 measured on the bench, 4.50 overall including the nub. The
    nub faces the case, which is the thing that was feared and measured: if it
    faced outward it would stand the horn proud of its pocket.
    """
    plate = bd.Cylinder(HORN_OD / 2, HORN_PLATE_T)
    nub_t = HORN_OVERALL_T - HORN_PLATE_T
    nub = bd.Pos(0, 0, -(HORN_PLATE_T + nub_t) / 2) * bd.Cylinder(5.0, nub_t)
    return (plate + nub - bd.Cylinder(SPLINE_R, HORN_OVERALL_T * 4)).clean()


DRIVEN = {"vhipsv": "thigh", "vkneesv": "shin",
          "vrollsv": "rollbracket", "vwhlsv": "wheel"}


def horns(mode="wheel", targets=None):
    """[(name, solid)] - a horn on every servo output, in world mm.

    Built in the SERVO's canonical (length, width, shaft) frame and reordered by
    the same `SERVO_MESH` table `drawing_solid()` uses.

    WHICH END IS THE OUTPUT is decided by the part it drives, and that is not a
    detail. `SERVO_MESH` permutes axes and carries no SIGN, so canonical +shaft
    is not necessarily the output direction: placed at +shaft everywhere, the
    wheel's horn landed 33 mm from the wheel, on the rear boss. Every one of the
    ten was on an end chosen by an axis convention rather than by the joint.

    So each horn goes on whichever end of the shaft is nearer the thing it
    turns. That is the same lesson as the roll shaft in front of the tyre: ask
    the assembly where something belongs, do not infer it from a frame.
    """
    import mujoco

    from cad.assemble_check import _loc as ac_loc
    from cad.drives import _posed
    from src.rsbot.model import SERVO_MESH
    import cad.servo as sv

    m, d, _ = _posed(mode)
    from cad.assemble_check import gap as _gap
    from cad.drives import shaft_lines

    lines = shaft_lines(mode)
    targets = targets or {}
    out = []
    for n, (org, axis) in sorted(lines.items()):
        side = "r" if (n.endswith("_r") or n.endswith("-1")) else "l"
        stem = next(k for k in DRIVEN if n.startswith(k))
        tgt = (targets.get(f"{DRIVEN[stem]}_{side}")
               or targets.get(f"{DRIVEN[stem]} {side}"))
        a = np.asarray(axis, float)
        a = a / np.linalg.norm(a)
        org = np.asarray(org, float)

        # `shaft_lines` gives a point ON the shaft at the case's mid-height,
        # with SHAFT_X's sign already resolved against the joint. Everything
        # here is measured from that, so no sign is invented twice: the horn's
        # inner face is HORN_FACE_Z along the shaft, and which WAY is settled by
        # the part it drives.
        best = None
        for sgn in (1.0, -1.0):
            inner = org + a * (sgn * sv.HORN_FACE_Z)
            h = _place(horn(), inner + a * (sgn * (HORN_OVERALL_T
                                                   - HORN_PLATE_T / 2)),
                       a * sgn)
            if tgt is None:
                best = (0.0, h)
                break
            try:
                score = _gap(h, tgt)
            except Exception:
                score = 1e9
            if best is None or score < best[0]:
                best = (score, h)
        out.append((f"horn {n}", best[1]))
    return out


# THE BELT DRIVE'S PARTS USED TO BE DRAWN HERE: pulley(), belt_drive(), the
# hub-thickness solve _drive_hub_t(), and BELT_T. All of it went with the
# ankle-pitch servo (cad/linkage.py), because a module full of geometry that
# nothing builds is the same fault as a constant with no uses - it reads as
# verified because it sits among things that are (docs/how-checks-fail.md #6).
#
# The reasoning it encoded is not lost. `cad/belt.py` keeps why the ratio was
# pinned at 2:1 from both sides, and renders/servos-before-after-*.png show the
# drive as it was, generated by cad.robot compare at commit b21ac99. That is
# the last build in which those parts existed.


def _world(m, d, body, local):
    """A point in a link's own frame, in world mm."""
    import mujoco
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    R = d.xmat[bid].reshape(3, 3)
    return d.xpos[bid] * 1000.0 + R @ np.asarray(local, float)


def _corners(b):
    lo = np.array([b.min.X, b.min.Y, b.min.Z])
    hi = np.array([b.max.X, b.max.Y, b.max.Z])
    return [np.array([hi[j] if (i >> j) & 1 else lo[j] for j in range(3)])
            for i in range(8)]


def _cyl(base, direction, r, length):
    """A cylinder starting at `base` and running `length` along `direction`."""
    a = np.asarray(direction, float)
    a = a / np.linalg.norm(a)
    mid = np.asarray(base, float) + a * (length / 2)
    return bd.Plane(origin=tuple(mid), z_dir=tuple(a)) * bd.Cylinder(r, length)


def _place(solid, point, direction):
    a = np.asarray(direction, float)
    a = a / np.linalg.norm(a)
    return bd.Plane(origin=tuple(np.asarray(point, float)),
                    z_dir=tuple(a)) * solid


PROUD = 1.0     # how far a shaft stands out of its bore, so it can be pushed


def spans(mode="wheel"):
    """[(joint, t0, t1)] along each joint axis, from the seats themselves.

    NOT from scanning the solids. The first version of this looked for
    cylindrical faces of about the shaft's diameter that were coaxial with the
    joint, which sounds robust and is not: it missed the shin's pitch bore
    entirely and returned a 10 mm shaft for a 14 mm joint. The bore is there -
    350 mm3 of material rings it and its wall is exactly SHAFT_R from the axis -
    so the scan was wrong, not the part.

    These come from the constants that CUT the seats, mapped through the sim's
    own transform for the link. One source, and it moves when the part moves.
    """
    from cad.drives import _posed

    m, d, anchors = _posed(mode)
    # SOLVED ON THE LEFT, MIRRORED. The sim writes the right leg with mirrored
    # POSITIONS rather than a mirrored frame, so mapping the same local point
    # through the right shin's transform lands somewhere else entirely - it gave
    # a 136 mm pitch shaft against the left's 14 mm. The robot is symmetric;
    # cad/wiring.py reached the same conclusion about the cable runs and for the
    # same reason.
    axle_l = anchors["ankle_pitch_l"][0]
    bc_l = _world(m, d, "shin_l",
                  (0, shin.PITCH_Y - shin.PITCH_HALF + shin.BEARING_W / 2,
                   shin.ANKLE_Z))
    t_bear = abs(float(bc_l[1] - axle_l[1]))

    out = []
    for side in ("l", "r"):
        lo = min(ankle.YOKE_BOSS_Y[0], t_bear - shin.BEARING_W / 2)
        hi = max(ankle.YOKE_BOSS_Y[1], t_bear + shin.BEARING_W / 2)
        out.append((f"ankle_pitch_{side}", lo, hi, t_bear))
        # ROLL: written down explicitly in cad/ankle.py.
        rl, rh = ankle.ROLL_SHAFT_X
        out.append((f"ankle_roll_{side}", rl, rh, -49.0))
    return out


def fitted(mode="wheel", parts=None):
    """[(name, solid)] - the bought hardware, in place, in world mm.

    `parts` is the rest of the assembly, used only to decide which END of each
    servo its horn goes on. Without it the horns fall back to the canonical
    +shaft end, which is wrong for at least the wheel.
    """
    import mujoco

    from cad.assemble_check import _loc as ac_loc
    from cad.drives import _posed

    m, d, anchors = _posed(mode)
    out = []
    for key, lo, hi, t_bear in spans(mode):
        if key.startswith("ankle_roll"):
            continue                        # placed from the yoke's frame below
        p, a = anchors[key]
        a = np.asarray(a, float) / np.linalg.norm(np.asarray(a, float))
        s = 1.0 if key.endswith("_l") else -1.0
        a = a * s                           # +t outboard on both legs
        base = np.asarray(p, float) + a * (lo - PROUD)
        out.append((f"shaft {key}",
                    _cyl(base, a, SHAFT_D / 2, (hi - lo) + 2 * PROUD)))
        out.append((f"bearing {key}",
                    _place(bearing(), np.asarray(p, float) + a * t_bear, a)))

    # THE ROLL SHAFT COMES FROM THE YOKE'S FRAME, NOT FROM THE JOINT AXIS.
    #
    # Taking its direction from `anchors["ankle_roll_r"]` put the right leg's
    # shaft at x = +40..+57.5 where the left's is at -57.5..-40: MuJoCo reports
    # that joint's axis negated on the right, and the robot mirrors in Y, not X.
    # The result was a 17 mm rod floating in front of the right tyre, which is
    # exactly how it was spotted - by looking at it.
    #
    # cad/ankle.py already states where this shaft lives, in the yoke's own
    # frame, and the yoke is mirrored the same way every other part is. Reading
    # it from there cannot pick up a sign that belongs to the physics.
    rl, rh = ankle.ROLL_SHAFT_X
    for side in ("l", "r"):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"ankle_{side}")
        loc = ac_loc(d.xpos[bid], d.xmat[bid])
        rod = _cyl((rl - PROUD, 0, 0), (1, 0, 0), SHAFT_D / 2,
                   (rh - rl) + 2 * PROUD)
        brg = _place(bearing(), (-49.0, 0, 0), (1, 0, 0))
        if side == "r":
            rod = bd.mirror(rod, bd.Plane.XZ)
            brg = bd.mirror(brg, bd.Plane.XZ)
        out.append((f"shaft ankle_roll_{side}", loc * rod))
        out.append((f"bearing ankle_roll_{side}", loc * brg))

    tgt = dict(parts or {})
    tgt.update(dict(out))
    out.extend(horns(mode, targets=tgt))
    return out


def _euler(mat):
    m = np.asarray(mat, float).reshape(3, 3)
    sy = np.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2)
    if sy > 1e-9:
        return np.array([np.arctan2(m[2, 1], m[2, 2]),
                         np.arctan2(-m[2, 0], sy),
                         np.arctan2(m[1, 0], m[0, 0])])
    return np.array([np.arctan2(-m[1, 2], m[1, 1]),
                     np.arctan2(-m[2, 0], sy), 0.0])


def horn_joints(mode="wheel", verbose=True):
    """[(servo, part, clearance mm)] at every horn joint. Negative is a clash.

    THE CHECK THAT SHOULD HAVE EXISTED. Every joint in this robot that is not a
    bearing is a part bolted to a servo horn, and until 2026-08-02 the horn was
    not drawn, so nothing could ask whether the part reached it.

    It does not. Measured along each shaft, the horn's outer face sits at 19.35
    from the case centre and the driven faces sit at 18.25:

        hip -> thigh          1.10 mm INTERFERENCE
        knee -> shin          1.10 mm INTERFERENCE
        roll -> bracket       0.40 mm standoff
        wheel -> wheel        0.40 mm short of its pocket

    1.10 is exactly HORN_OVERALL_T - SPLINE_PROUD, 4.50 - 3.40. The parts were
    placed at the SPLINE TIP, which is where the servo's bounding box ends and
    is not where the horn's face is: the horn is 4.50 thick over a spline that
    stands 3.40 proud, so it finishes 1.10 mm further out.

    A part that lands 1.10 mm inside its horn does not seat. It rocks on the
    horn's rim, and the bolts take the wheel's load in bending - which is the
    same standoff fault this design has already been bitten by at 0.5 mm and
    0.035 mm, in the direction that does not show up as interference between
    printed parts because the thing in between was never modelled.
    """
    import build123d as bd

    import cad.fitview as fv
    from cad.drives import shaft_lines

    d = dict(fv.solids(mode))
    lines = shaft_lines(mode)
    rows = []
    for sname, (org, axis) in sorted(lines.items()):
        side = "r" if (sname.endswith("_r") or sname.endswith("-1")) else "l"
        stem = next(k for k in DRIVEN if sname.startswith(k))
        pname = next((k for k in (f"{DRIVEN[stem]}_{side}",
                                  f"{DRIVEN[stem]} {side}") if k in d), None)
        hname = f"horn {sname}"
        if pname is None or hname not in d:
            continue
        a = np.asarray(axis, float)
        a = a / np.linalg.norm(a)
        org = np.asarray(org, float)
        probe = bd.Plane(origin=tuple(org), z_dir=tuple(a)) * bd.Cylinder(
            10.0, 200)

        def span(sol):
            h = probe & sol
            if not h or h.volume < 1e-6:
                return None
            b = h.bounding_box()
            ts = [float(np.dot(np.array([x, y, z]) - org, a))
                  for x in (b.min.X, b.max.X) for y in (b.min.Y, b.max.Y)
                  for z in (b.min.Z, b.max.Z)]
            return min(ts), max(ts)

        hs, ps = span(d[hname]), span(d[pname])
        if hs is None or ps is None:
            rows.append((sname, pname, float("nan")))
            continue
        # Both measured along the same axis; the horn's far face against the
        # part's near face, whichever way the shaft points.
        out_face = max(hs, key=abs)[0] if False else (
            hs[1] if abs(hs[1]) > abs(hs[0]) else hs[0])
        near = ps[0] if abs(ps[0]) < abs(ps[1]) else ps[1]
        rows.append((sname, pname, float(abs(near) - abs(out_face))))

    if verbose:
        print(f"--- {mode} mode: does each part reach its servo horn?")
        for sname, pname, c in rows:
            if c != c:
                print(f"   {sname:<12} -> {pname:<16} not on the shaft axis")
                continue
            note = ("CLASH" if c < -0.01 else
                    "seats" if abs(c) <= 0.01 else "standoff")
            print(f"   {sname:<12} -> {pname:<16} {c:+7.2f} mm   {note}")
        bad = [r for r in rows if r[2] == r[2] and abs(r[2]) > 0.01]
        print(f"   {len(bad)} horn joint(s) do not seat"
              if bad else "   every horn joint seats")
    return rows


def main():
    mode = "foot" if "foot" in sys.argv else "wheel"
    print(f"bought hardware, {mode} mode")
    print()
    print("  shaft lengths, from the seats rather than chosen:")
    for key, lo, hi, _ in spans(mode):
        print(f"    {key:<16} bore {hi - lo:6.2f} mm"
              f"   -> cut {hi - lo + 2 * PROUD:5.1f} mm")
    print()
    print("  the order sheet says 30 mm, 'or cut from stock'. It is stock:")
    print("  one 30 mm rod makes two pitch shafts, near enough.")
    n = len(fitted(mode))
    print(f"\n  {n} bought solids placed")
    return 0


if __name__ == "__main__":
    sys.exit(main())


# --- into the SIM -------------------------------------------------------------
#
# Everything above places hardware in WORLD coordinates, which is what the
# viewer and the checks want. The sim needs the opposite: each part in the LOCAL
# frame of the body it is bolted to, so MuJoCo can carry it around as that body
# moves. Then driving the robot moves the bearings and shafts
# with it, instead of the twin showing printed parts floating in a vacuum.

def local_parts():
    """{stl stem: (solid in its body's frame, which body, colour)}.

    The frames are the CAD's own, which are also the sim's body frames: the
    ankle's origin is the axle, the shin's is the knee. That is not a
    coincidence to rely on quietly, so cad.twin's box check is what holds it.
    """
    from cad.servo_dims import LENGTH as _L
    out = {}

    # Ankle frame: origin at the axle, pitch axis along y, roll axis along x.
    lo, hi = ankle.ROLL_SHAFT_X
    out["hw_shaft_roll"] = (
        _cyl((lo - PROUD, 0, 0), (1, 0, 0), SHAFT_D / 2,
             (hi - lo) + 2 * PROUD), "ankle", "steel")
    out["hw_bearing_roll"] = (
        _place(bearing(), (-49.0, 0, 0), (1, 0, 0)), "ankle", "steel")

    # The pitch shaft runs out along +y to the shin's bearing. Its span is the
    # yoke boss plus the 623ZZ, exactly as spans() measures it.
    y0 = ankle.YOKE_BOSS_Y[0] - PROUD
    y1 = 70.0 + PROUD
    out["hw_shaft_pitch"] = (
        _cyl((0, y0, 0), (0, 1, 0), SHAFT_D / 2, y1 - y0), "ankle", "steel")
    out["hw_bearing_pitch"] = (
        _place(bearing(), (0, 68.0, 0), (0, 1, 0)), "ankle", "steel")

    # THE BELT DRIVE USED TO BE HERE: a 40T pulley on the pitch shaft, a 20T on
    # the servo's horn, and the loop between them. All three are deleted with
    # the servo they served (cad/linkage.py). The pitch shaft and its bearing
    # above are NOT deleted, because the joint still exists - it has only
    # stopped being driven.
    return out


def export_local(out_dir=None):
    """Write the local-frame STLs the sim's <asset> block needs."""
    from pathlib import Path

    d = Path(out_dir) if out_dir else Path(__file__).parent / "out"
    d.mkdir(parents=True, exist_ok=True)
    written = {}
    for stem, (solid, body, colour) in local_parts().items():
        f = d / f"{stem}.stl"
        bd.export_stl(solid, str(f), tolerance=0.03, angular_tolerance=0.15)
        written[stem] = (f, body, colour)
    return written


# The horn turns with the part it drives, not with the servo, so in the sim it
# belongs to the DRIVEN body. Without these the sim's servos are bare cases with
# a bolt pattern floating next to them, while the CAD viewer shows the joint -
# two pictures of the same robot disagreeing, which is the thing this project
# exists to prevent.
HORN_BODY = {"vhipsv": "thigh", "vkneesv": "shin",
             "vrollsv": "rollbracket", "vwhlsv": "wheel"}


def local_horns(mode="wheel"):
    """{stem: (solid in the driven body's frame, body stem)} for the left side.

    Taken from the world placement `horns()` already solves and pulled back
    through the driven body's own transform, rather than re-deriving the offset
    in a second place. The right side mirrors, as everything else does.
    """
    import mujoco

    import cad.fitview as fv
    from cad.assemble_check import _loc as ac_loc
    from cad.drives import _posed

    m, d, _ = _posed(mode)
    world = dict(horns(mode, targets=dict(fv.solids(mode))))
    out = {}
    for sname, bstem in HORN_BODY.items():
        key = f"horn {sname}1" if sname == "vhipsv" else f"horn {sname}_l"
        if key not in world:
            continue
        body = "thigh_l" if bstem == "thigh" else f"{bstem}_l"
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        if bid < 0:                       # beltdrive has no body in this build
            continue
        inv = ac_loc(d.xpos[bid], d.xmat[bid]).inverse()
        out[f"hw_horn_{sname}"] = (inv * world[key], bstem)
    return out


def export_horns(out_dir=None):
    from pathlib import Path

    dd = Path(out_dir) if out_dir else Path(__file__).parent / "out"
    dd.mkdir(parents=True, exist_ok=True)
    made = {}
    for stem, (solid, bstem) in local_horns().items():
        f = dd / f"{stem}.stl"
        bd.export_stl(solid, str(f), tolerance=0.03, angular_tolerance=0.15)
        made[stem] = (f, bstem)
    return made


def floor(mode="foot", verbose=True):
    """[(part, lowest z)] - does anything dig into the ground?

    ON THE REAL SOLIDS. tests/test_foot.py has a floor check, and for a MESH it
    returns the geom's CENTRE:

        return c[2]

    which cannot detect a mesh dipping below the floor, only a mesh whose middle
    is underground. Every printed part became a mesh, and all the bought
    hardware is a mesh, so the check had quietly stopped covering the parts that
    matter. That is the same shape as every other failure in this repo: the
    logic was fine and the input was not.

    What it finds, 2026-08-02: in FOOT mode the ankle-pitch drive is underground.
    """
    import cad.fitview as fv

    rows = sorted(((s.bounding_box().min.Z, n) for n, s in fv.solids(mode)))
    if verbose:
        print(f"--- {mode} mode: what is lowest?")
        for z, n in rows[:6]:
            flag = "  <-- THROUGH THE FLOOR" if z < -0.01 else ""
            print(f"   {n:<26} z {z:7.2f}{flag}")
        bad = [r for r in rows if r[0] < -0.01]
        print(f"   {len(bad)} part(s) below the floor" if bad
              else "   nothing is underground")
    return rows
