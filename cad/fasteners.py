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
SCHEDULE = [
    ("thigh -> hip servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("thigh -> knee servo case", "M2 x 10", "servo pilot", 4, ""),
    ("shin -> knee servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("shin -> ankle servo case", "M2 x 10", "servo pilot", 4, ""),
    ("ankle yoke -> ankle pitch shaft", "M2.5 x 8", "insert in yoke", 2, ""),
    ("ankle yoke -> roll servo case", "M2 x 10", "servo pilot", 4, ""),
    ("roll bracket -> roll servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("roll bracket -> wheel servo case", "M2 x 10", "servo pilot", 4, ""),
    ("wheel body -> wheel servo horn", "M3 x 8", "the horn, tapped", 4, ""),
    ("chassis -> hip servo case", "M2 x 10", "servo pilot", 8,
     "both hips, 4 each"),
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

    L, W and H are 45.4, 24.8 and 39.6, all distinct, so matching the extent
    against HEIGHT identifies the shaft with no ambiguity.
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
        out.append((float(r), np.array([c.X, c.Y, c.Z]),
                    np.array([d.X, d.Y, d.Z])))
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


if __name__ == "__main__":
    main()
