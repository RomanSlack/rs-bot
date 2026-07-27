"""Can a driver actually reach every screw?

    uv run python -m cad.toolaccess

Nothing checked this. Three servos live within 45 mm of one axle and the ankle
yoke, roll bracket and wheel all crowd the same 60 mm, so "there is a hole
there" and "you can put a screw in it" are different claims and only the first
one had ever been tested.

The check is geometric, like the rest of the fit work: take every M2-scale hole
in every printed part, stand a driver on it, and intersect the driver with the
assembled robot. A screw can be fitted if the driver clears from at least ONE
end of its hole - you would naturally come in from whichever side is open, and
requiring both would fail almost everything for no reason.

The driver is a real one, not a ray. A zero-width line finds its way through
gaps that a 3 mm shaft and a 20 mm handle do not, and the handle is usually
what actually stops you.
"""

import sys

import build123d as bd
import numpy as np

import cad.fasteners as fasteners

# A precision driver for M2/M2.5 socket caps. Generous rather than optimistic:
# if it does not fit, a stubby might, and that is worth knowing separately.
SHAFT_R = 1.6            # 3.2 mm shaft
SHAFT_L = 45.0
HANDLE_R = 9.0           # 18 mm handle
HANDLE_L = 40.0

# A short hex key, the fallback when the driver will not go. An L-key needs
# swing room too, which this does not model - it is the optimistic case.
STUBBY_SHAFT_R = 1.0
STUBBY_SHAFT_L = 18.0
STUBBY_HANDLE_R = 0.0    # none

TOUCH = 1.0              # mm3, below this is numerical noise


def _align_z(axis):
    """Euler angles (deg) taking +z onto `axis`."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    v = np.cross([0.0, 0.0, 1.0], a)
    c = float(a[2])
    if np.linalg.norm(v) < 1e-9:
        return (0.0, 0.0, 0.0) if c > 0 else (180.0, 0.0, 0.0)
    v = v / np.linalg.norm(v)
    t = np.arccos(np.clip(c, -1, 1))
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * (K @ K)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        return (np.degrees(np.arctan2(R[2, 1], R[2, 2])),
                np.degrees(np.arctan2(-R[2, 0], sy)),
                np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    return (np.degrees(np.arctan2(-R[1, 2], R[1, 1])),
            np.degrees(np.arctan2(-R[2, 0], sy)), 0.0)


def driver(at, axis, shaft_r=SHAFT_R, shaft_l=SHAFT_L,
           handle_r=HANDLE_R, handle_l=HANDLE_L):
    """The tool, standing on `at` and reaching away along `axis`."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    loc = bd.Location(bd.Vector(*at)) * bd.Rotation(*_align_z(a))
    tool = loc * bd.Pos(0, 0, shaft_l / 2) * bd.Cylinder(shaft_r, shaft_l)
    if handle_r > 0:
        tool += (loc * bd.Pos(0, 0, shaft_l + handle_l / 2)
                 * bd.Cylinder(handle_r, handle_l))
    return tool


def _ends(face_bbox, centre, axis):
    """How far the hole runs each way along its own axis, from the centre."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    lo = np.array([face_bbox.min.X, face_bbox.min.Y, face_bbox.min.Z])
    hi = np.array([face_bbox.max.X, face_bbox.max.Y, face_bbox.max.Z])
    ts = []
    for i in range(8):
        c = np.array([hi[j] if (i >> j) & 1 else lo[j] for j in range(3)])
        ts.append(float(np.dot(c - centre, a)))
    return min(ts), max(ts)


def holes(mode="wheel"):
    """Every M2-scale hole in every printed part, in world coordinates."""
    import cad.assemble_check as ac

    out = []
    for pname, solid in ac.parts(mode):
        if pname.startswith("v"):
            continue
        for f in solid.faces():
            if str(getattr(f, "geom_type", "")).upper().find("CYLINDER") < 0:
                continue
            try:
                r = f.radius
                d = f.axis_of_rotation.direction
            except Exception:
                continue
            if r is None or not (0.9 < r < 1.6):
                continue
            c = f.center()
            centre = np.array([c.X, c.Y, c.Z])
            axis = np.array([d.X, d.Y, d.Z])
            t0, t1 = _ends(f.bounding_box(), centre, axis)
            out.append(dict(part=pname, r=float(r), centre=centre,
                            axis=axis / np.linalg.norm(axis), t0=t0, t1=t1))
    return out


# The order the robot goes together in. A screw only has to clear the parts
# that are ALREADY ON when it is fitted, which is the difference between a
# check that answers "can this be built" and one that answers "can this be
# serviced". Both are worth knowing and they give very different answers: the
# thigh's knee-servo bolts are buried by the shin and the ankle servo on the
# finished robot, and completely open at the moment you actually fit them.
BUILD_ORDER = ["torso", "vhipsv", "thigh", "vkneesv", "shin", "vanksv",
               "ankle", "vrollsv", "rollbracket", "vwhlsv", "wheel"]


def _stage(name):
    """Where a solid joins the build. Sides are ignored, they go on together."""
    stem = name.rstrip("_lr").rstrip("-1").rstrip("1")
    for i, k in enumerate(BUILD_ORDER):
        if stem == k or name.startswith(k):
            return i
    return len(BUILD_ORDER)


def is_horn_screw(h, frames, near=12.0):
    """A screw on a servo's horn pattern rather than on its case.

    Horn joints are sub-assembled: you bolt the part to the horn on the bench,
    then push the horn onto the spline and fix it with one central screw. So
    the four pattern screws are never fitted on the robot and it is wrong to
    ask whether a driver reaches them there - the roll bracket's four are
    buried in the ankle yoke on the assembled machine, and completely open on
    the bench.

    Which moves the question rather than answering it: the CENTRAL screw is
    then the one that has to be reachable, and it does not exist anywhere in
    this CAD. See docs/road-to-order.md.
    """
    for org, R, shaft in frames.values():
        d = h["centre"] - org
        along = float(np.dot(d, shaft))
        perp = float(np.linalg.norm(d - along * shaft))
        if perp < near and abs(along) < 40.0 and abs(
                float(np.dot(h["axis"], shaft))) > 0.9:
            return True
    return False


def check(mode="wheel", stubby=False, assembled=False, verbose=True):
    """`assembled=False` asks whether it can be BUILT: each screw sees only the
    parts already fitted. `assembled=True` asks whether it can be SERVICED on
    the finished robot."""
    import cad.assemble_check as ac

    solids = ac.parts(mode)
    kw = (dict(shaft_r=STUBBY_SHAFT_R, shaft_l=STUBBY_SHAFT_L, handle_r=0.0,
               handle_l=0.0) if stubby else {})

    frames = fasteners._servo_frames(mode)
    rows = []
    for h in holes(mode):
        limit = len(BUILD_ORDER) if assembled else _stage(h["part"])
        present = [(n, s) for n, s in solids if _stage(n) <= limit]
        best, best_by = None, {}
        for sgn, t in ((1.0, h["t1"]), (-1.0, h["t0"])):
            tool = driver(h["centre"] + h["axis"] * t, h["axis"] * sgn, **kw)
            hit, by = 0.0, {}
            for pname, solid in present:
                try:
                    i = tool & solid
                    v = i.volume if i else 0.0
                except Exception:
                    v = 0.0
                if v > TOUCH:
                    by[pname] = v
                hit += v
            if best is None or hit < best:
                best, best_by = hit, by
        rows.append(dict(**h, blocked=best, by=best_by,
                         horn=is_horn_screw(h, frames)))

    if verbose:
        _report(rows, stubby, assembled)
    return rows


def _report(rows, stubby, assembled=False):
    tool = "stubby key" if stubby else "driver"
    when = "on the finished robot" if assembled else "at the moment it is fitted"
    print(f"  screws checked {when}\n")
    by = {}
    for r in rows:
        by.setdefault(r["part"], []).append(r)
    print(f"{'part':<16}{'holes':>7}{'reachable':>11}{'blocked':>9}"
          f"   worst obstruction mm3")
    bad = []
    for pname in sorted(by):
        rs = by[pname]
        ok = [r for r in rs if r["blocked"] <= TOUCH or r["horn"]]
        no = [r for r in rs if r["blocked"] > TOUCH and not r["horn"]]
        worst = max((r["blocked"] for r in rs), default=0.0)
        print(f"{pname:<16}{len(rs):>7}{len(ok):>11}{len(no):>9}   {worst:9.0f}")
        bad += [(pname, r) for r in no]
    print()
    if bad:
        print(f"  {len(bad)} screw(s) the {tool} cannot reach:")
        for pname, r in sorted(bad, key=lambda x: -x[1]["blocked"])[:12]:
            c = r["centre"]
            print(f"    {pname:<14} r={r['r']:.2f} at "
                  f"({c[0]:7.1f} {c[1]:7.1f} {c[2]:7.1f})  "
                  f"{r['blocked']:8.0f} mm3 in the way")
    else:
        print(f"  every screw is reachable with the {tool}")
    return bad


def main():
    stubby = "--stubby" in sys.argv
    print(f"driver: {2*SHAFT_R:.1f} mm shaft x {SHAFT_L:.0f}, "
          f"{2*HANDLE_R:.0f} mm handle x {HANDLE_L:.0f}"
          if not stubby else
          f"stubby: {2*STUBBY_SHAFT_R:.1f} mm key x {STUBBY_SHAFT_L:.0f}")
    print()
    check(stubby=stubby, assembled="--assembled" in sys.argv)


if __name__ == "__main__":
    main()
