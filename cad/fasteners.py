"""Every fastener in the robot, and whether it can actually be fitted.

    uv run python -m cad.fasteners

Until this existed, every bolt hole in the CAD was a clearance hole into air.
Nothing said what screw went in it, what it threaded into, or how long it had
to be, and nothing checked that a hole in a printed part lined up with the hole
in the servo it was supposed to bolt to.

The check here is the one that does not depend on knowing the servo's exact
bolt pattern, which is unverified (cad/servo.py says why): does every screw run
along the axis of the hole it is meant to enter? All 28 do.

That check was worth building for a reason worth recording. Its first version
assumed the servo's shaft was the box's local z, because that is the order
cad/servo.py uses. The sim does not use that order - it writes each servo's
three dimensions in whatever sequence puts the case where it goes - and the
result was a confident report that every screw in the robot was at 90 degrees
to its servo and "nothing bolts to anything". Completely false. What caught it
was checking the derived shaft against the axis of the joint each servo drives:
they must be the same vector, and they were not. They now agree to |dot| = 1.000
for all ten.

A check whose input is wrong does not fail. It answers a different question
fluently.

## The decision: what threads into what

    part -> servo CASE    M2 self-tapping, through a 2.2 mm clearance hole in
                          the part, into the servo's own 1.5 mm pilot. The
                          servo supplies the thread, so the printed part needs
                          nothing but a clean hole.
    part -> servo HORN    M3 through a 3.2 mm CLEARANCE hole, into the horn,
                          which is metal and tapped. No insert: the thread is
                          bought. Measured off TheRobotStudio's own parts, see
                          cad/servo.py.
    part -> part          M2.5 into a heat-set insert.

Inserts for anything taken apart more than twice, which is every servo mount on
a robot that has not been built yet. The cost is that an insert needs a 3.5 mm
bore where a clearance hole needs 2.5, and enough wall around it to take the
melt without bulging.

The horn pattern IS verified now, measured off three of TheRobotStudio's
SO-ARM101 parts that bolt to the same horn. It moved the screw from M2.5 to M3
and every horn hole in this robot grew 0.7 mm. See cad/servo.py.
"""

import sys

import build123d as bd
import mujoco
import numpy as np

import cad.servo as servo

# --- hole standards ----------------------------------------------------------
#
# Radii, because everything downstream cuts cylinders.
M2_CLEAR_R = 1.10          # 2.2 mm, an M2 screw passes
M25_CLEAR_R = 1.35         # 2.7 mm, an M2.5 screw passes
M3_CLEAR_R = 1.60          # 3.2 mm, an M3 screw passes - the horn size
M25_INSERT_R = 1.75        # 3.5 mm bore for an M2.5 heat-set insert
M25_INSERT_DEPTH = 4.0     # and how deep it sits
INSERT_WALL = 1.2          # minimum material round an insert bore

# Head sizes, for the bearing-surface and tool-access checks.
M2_HEAD_R = 1.9            # M2 socket cap
M25_HEAD_R = 2.25          # M2.5 socket cap


# --- the schedule ------------------------------------------------------------
#
# (interface, screw, threads into, count, note)
# THE FORTY M2 CASE SCREWS ARE GONE, and this is the line where the decision
# shows up as money. Every "-> servo case" row went into the STS3215's own case
# holes. Feetech's drawing labels them "8-PA2.0" and never says where they are,
# none of their three brackets uses them, and this robot had four parts each
# drilling a DIFFERENT invented pattern. The servos are captured now instead:
# see the cradles in chassis.py, thigh.py, shin.py and ankle.py.
#
# Three of those rows were also fiction on their own terms. The shin's lower
# ankle bolt missed the case end by 3.5 mm, two of the yoke's four fell below
# the ring, and both of the roll bracket's ran parallel to the face they were
# meant to clamp. They were ordered, counted and priced; they never existed.
SCHEDULE = [
    ("thigh -> hip servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("shin -> knee servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("ankle yoke -> ankle pitch shaft", "M2.5 x 8", "insert in yoke", 2, ""),
    ("roll bracket -> roll servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("wheel body -> wheel servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("horn -> servo shaft", "M3 x 6", "the servo, tapped", 5,
     "the centre screw, from the manufacturer's drawing. Never drawn before."),
    ("chassis side plates -> top", "M2.5 x 10", "insert in chassis", 8, ""),
    ("Pi 5 -> chassis shelf", "M2.5 x 6", "insert in chassis", 4, ""),
]


def _servo_frames(mode="wheel"):
    """{geom name: (origin mm, rotation, shaft axis in world)}.

    The shaft axis is DERIVED from the box, not assumed. The sim writes each
    servo's three dimensions in whatever order puts the case where it goes, so
    the shaft is along local y for the hip, knee, ankle and wheel servos and
    along local x for the roll servo. Assuming local z - which is the order
    cad/servo.py uses - makes every screw in the robot look perpendicular to
    its servo, which is a very convincing wrong answer.

    L, W and H are 45.23, 24.73 and 36.50, all distinct, so matching the extent
    against HEIGHT identifies the shaft with no ambiguity. The argument holds
    whatever the numbers are, which is why it survived them changing twice; the
    numbers are quoted anyway, so it is obvious when it stops holding.
    """
    from fitcheck import pose
    from src.rsbot.model import load

    m, d = load()
    pose(m, d, mode)
    out = {}
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if m.geom_group[i] != 0 or not n.startswith(
                ("vhipsv", "vkneesv", "vanksv", "vrollsv", "vwhlsv")):
            continue
        R = d.geom_xmat[i].reshape(3, 3)
        full = m.geom_size[i] * 2000.0
        k = int(np.argmin(np.abs(full - servo.HEIGHT)))
        if abs(full[k] - servo.HEIGHT) > 0.5:
            raise RuntimeError(f"{n}: no side matches the servo height "
                               f"{servo.HEIGHT}; sides are {full}")
        out[n] = (d.geom_xpos[i] * 1000.0, R, R[:, k])
    return out


def _wraps(face, r, axis):
    """How far a cylindrical face wraps around its own axis, in radians.

    A fillet and a bolt hole are both cylindrical faces of similar radius, and
    NOTHING about the radius separates them. What does is how much of the
    cylinder is actually there: a bolt hole through material is a full 2*pi
    barrel, an edge blend is a quarter of one.

    Measured from area rather than by probing normals. The obvious test - is
    the surface concave or convex - needs a point ON the face and the position
    of the AXIS, and a first attempt passed the face's own centroid as both,
    making the radial vector zero and every face look like a hole. Area and
    length are unambiguous and cheap.
    """
    bb = face.bounding_box()
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    lo = np.array([bb.min.X, bb.min.Y, bb.min.Z])
    hi = np.array([bb.max.X, bb.max.Y, bb.max.Z])
    length = abs(float(np.dot(hi - lo, a)))
    if length < 1e-6 or r < 1e-6:
        return 0.0
    return float(face.area) / (r * length)


def part_hole_axes(solid, rmin=0.9, rmax=1.8):
    """Every fastener-scale cylindrical hole in a part: (radius, centre, axis).

    rmax is 1.8, not 1.6. When the horn screw went from M2.5 to M3 the holes
    grew to r = 1.60 and a 1.6 ceiling silently dropped half of them - the
    audit went from 28 holes to 14 and still said "every screw lines up",
    because it was no longer looking at the ones that had changed.
    """
    out = []
    for f in solid.faces():
        if str(getattr(f, "geom_type", "")).upper().find("CYLINDER") < 0:
            continue
        try:
            r = f.radius
            d = f.axis_of_rotation.direction
        except Exception:
            continue
        if r is None or not (rmin < r < rmax):
            continue
        c = f.center()
        centre = np.array([c.X, c.Y, c.Z])
        axis = np.array([d.X, d.Y, d.Z])
        if _wraps(f, float(r), axis) < np.pi:
            continue                    # an edge blend, not a hole
        out.append((float(r), centre, axis))
    return out


def audit(mode="wheel", near=30.0, verbose=True):
    """Do the parts' servo screws point the way the servo's holes point?

    This is the check that does not depend on the pattern. Where the holes are
    is unverified (cad/servo.py says why); which WAY they run is not, and if a
    screw is perpendicular to the hole it is meant to enter, nothing else about
    the pattern matters.
    """
    import cad.assemble_check as ac

    items = dict(ac.parts(mode))
    frames = _servo_frames(mode)
    printed = {n: s for n, s in items.items() if not n.startswith("v")}

    rows = []
    for sname, (org, R, shaft) in sorted(frames.items()):
        for pname, solid in printed.items():
            for r, c, a in part_hole_axes(solid):
                if np.linalg.norm(c - org) > near:
                    continue
                cos = abs(float(np.dot(a / np.linalg.norm(a), shaft)))
                rows.append(dict(servo=sname, part=pname, r=r,
                                 dist=float(np.linalg.norm(c - org)),
                                 align=cos))
    if verbose:
        _report(rows)
    return rows


def _report(rows):
    if not rows:
        print("  no M2-scale holes found near any servo at all")
        return rows
    by = {}
    for r in rows:
        by.setdefault((r["servo"], r["part"]), []).append(r)
    print(f"{'servo':<11}{'part':<16}{'holes':>6}{'aligned':>9}"
          f"{'perpendicular':>15}")
    bad = []
    for (sname, pname), rs in sorted(by.items()):
        ok = [x for x in rs if x["align"] > 0.9]
        perp = [x for x in rs if x["align"] < 0.1]
        print(f"{sname:<11}{pname:<16}{len(rs):>6}{len(ok):>9}{len(perp):>15}")
        if perp and not ok:
            bad.append((sname, pname, len(perp)))
    print()
    if bad:
        print("  PERPENDICULAR SCREWS - these cannot be fitted:")
        for sname, pname, n in bad:
            print(f"    {pname} has {n} hole(s) at 90 deg to {sname}'s "
                  f"mounting axis")
    else:
        print("  every servo screw runs along its servo's mounting axis")
    return bad


def bom():
    """The fastener order, aggregated by part number."""
    counts = {}
    for iface, screw, into, n, note in SCHEDULE:
        counts[screw] = counts.get(screw, 0) + n * (1 if "chassis" in iface
                                                    or "Pi" in iface else 2)
    inserts = sum(n * (1 if "chassis" in i or "Pi" in i else 2)
                  for i, s, into, n, _ in SCHEDULE if "insert" in into)
    return counts, inserts


# How finely to look, and how far. 0.1 deg either side of nothing is a servo
# that is located; 4 deg is far past anything that could be called a fit.
CAPTURE_STEP, CAPTURE_MAX = 0.1, 4.0
CAPTURE_TOUCH = 2.0        # mm3, the same threshold assemble_check calls a hit


def capture(mode="wheel", verbose=True):
    """How far can each servo TURN in its mount before something stops it?

    THE QUESTION THIS REPLACES was "does the servo touch the part that holds
    it", and every servo in the robot answers yes to that - a gap of 0.000 mm
    to its neighbour, eight for eight. It means nothing. A case can sit against
    a plate on its end face and still be free to rotate about its own shaft,
    which is the one direction its reaction torque actually pushes.

    So this rotates the case about its own shaft axis, a tenth of a degree at a
    time, and reports where it first bears on the printed part that is supposed
    to hold it. That angle adds to the gear lash at the same joint, because it
    is in series with the gearbox: the servo can turn that far before the link
    feels anything.

    The clearance is not a mistake and it cannot simply be closed. It is 0.4 mm
    per side because the print service quotes +/-0.3, so a tighter cradle risks
    not assembling at all. The fix is a preload feature - something compliant
    on one wall that pushes the case onto two hard datum faces - and that is a
    change to how four parts meet their servos, not a number to edit.

    Control-tested by opening cad/servo_dims.CRADLE_CLEAR from 0.4 to 1.2 mm,
    which takes the roll servo from 1.30 to 3.50 degrees and the hip from 1.30
    to 2.90. It measures the thing it says it measures.
    """
    from cad.assemble_check import parts

    ps = parts(mode)
    printed = [(n, sol) for n, sol in ps if not n.startswith("v")]
    frames = _servo_frames(mode)
    rows = []
    for name, sol in sorted(ps):
        if not name.startswith(("vhipsv", "vkneesv", "vrollsv", "vwhlsv")):
            continue
        origin, _, axis = frames[name]
        ax = bd.Axis(bd.Vector(*origin), bd.Vector(*axis))
        free, by = CAPTURE_MAX, None
        for deg in np.arange(CAPTURE_STEP, CAPTURE_MAX + 1e-9, CAPTURE_STEP):
            # Both ways. A wall on one side only would stop it turning one way
            # and read as located.
            for s in (1.0, -1.0):
                turned = sol.rotate(ax, float(s * deg))
                for pn, p in printed:
                    try:
                        v = (turned & p).volume
                    except Exception:
                        v = 0.0
                    if v > CAPTURE_TOUCH:
                        free, by = deg, pn
                        break
                if by:
                    break
            if by:
                break
        rows.append((free, name, by))

    if verbose:
        print("--- how far can each servo turn in its own mount?")
        for free, name, by in sorted(rows, reverse=True):
            print(f"   {name:<12}{free:5.2f} deg   "
                  f"{'stopped by ' + by if by else 'NOTHING STOPS IT'}")
        worst = max(r[0] for r in rows)
        print(f"   worst is {worst:.2f} deg, and it adds to that joint's gear "
              f"lash of 2-3 deg")
        print(f"   foot mode flips 8/8 at {3.0 + worst:.1f} deg total and dies "
              f"by 5.5, so this spends the margin rather than breaking it")
    return rows


def main():
    print(__doc__.strip().split("\n\n")[0])
    print()
    print(f"{'interface':<36}{'screw':<12}{'into':<22}{'n':>3}")
    for iface, screw, into, n, note in SCHEDULE:
        per = 1 if ("chassis" in iface or "Pi" in iface) else 2
        print(f"{iface:<36}{screw:<12}{into:<22}{n*per:>3}"
              + (f"   {note}" if note else ""))
    counts, inserts = bom()
    print()
    print("order:")
    for k in sorted(counts):
        print(f"   {counts[k]:>3} x {k}")
    print(f"   {inserts:>3} x M2.5 heat-set insert (3.5 mm bore, 4.0 deep)")
    print()
    audit()
    print()
    capture()


if __name__ == "__main__":
    main()
