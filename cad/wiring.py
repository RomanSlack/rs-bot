"""Where do the cables go?

    uv run python -m cad.wiring

There was not one cable, channel, tie point or strain relief anywhere in this
project. Ten daisy-chained bus servos, a cable across every joint, and one of
those joints turns 90 degrees.

That makes wiring the item most likely to force a REPRINT rather than a
rethink, which is why it is worth modelling before ordering rather than after:

  - a bundle needs volume, and this robot's spare volume is already spoken for.
    cad/envelope.py found exactly ONE legal position for the ankle-pitch
    bearing. A cable has to live in whatever that left.
  - the ankle roll joint turns 90 degrees between wheel mode and foot mode. The
    two connector ports it runs between MOVE RELATIVE TO EACH OTHER, so the run
    needs enough slack for the longer pose and nowhere to snag in the shorter
    one. That is the same swept-envelope question the flip already needed,
    asked about a flexible object.
  - strain relief bolts to something, and mounting features are exactly what
    gets discovered missing after the parts arrive.

The connector position is measured, not guessed: six pins at x = -0.55,
z = -16.90 on the rear face opposite the shaft, in two 3-pin groups at
y = +/-5.2, on a 2.40 mm pitch. See cad/servo.py.

What this does NOT do is route a cable properly. It runs each segment as the
straight line between ports, which is the SHORTEST possible path - so anything
it reports as blocked is blocked for every real routing too, and anything it
reports as clear still has to be checked once a real path is drawn. It is a
lower bound on trouble, not a clean bill.
"""

import sys

import build123d as bd
import numpy as np

import cad.servo as servo
from cad.fasteners import _servo_frames

# A 3-pin bus lead with sheath. Two of them share most runs (in and out), so
# the bundle is wider than one cable wherever the chain passes through.
LEAD_R = 1.4              # 2.8 mm single lead
BUNDLE_R = 2.2            # 4.4 mm where two run together
MIN_BEND_R = 8.0          # do not kink a 28 AWG silicone bundle tighter

# Local port positions on any servo: two 3-pin groups on the rear face.
PORT_LOCAL = [np.array([-0.55, -5.2, servo.Z_MIN]),
              np.array([-0.55, 5.2, servo.Z_MIN])]

# The daisy chain, in order from the bus adapter outward.
CHAIN_L = ["vhipsv1", "vkneesv_l", "vanksv_l", "vrollsv_l", "vwhlsv_l"]
CHAIN_R = ["vhipsv-1", "vkneesv_r", "vanksv_r", "vrollsv_r", "vwhlsv_r"]

TOUCH = 1.0               # mm3


def ports(mode="wheel"):
    """{servo: [world port positions]}."""
    out = {}
    for name, (org, R, shaft) in _servo_frames(mode).items():
        out[name] = [org + R @ p for p in PORT_LOCAL]
    return out


def _nearest(a_pts, b_pts):
    """The shortest port-to-port pairing between two servos."""
    best = None
    for a in a_pts:
        for b in b_pts:
            d = float(np.linalg.norm(a - b))
            if best is None or d < best[0]:
                best = (d, a, b)
    return best


def runs(mode="wheel"):
    """Every cable segment, as (label, start, end, length)."""
    p = ports(mode)
    out = []
    for chain in (CHAIN_L, CHAIN_R):
        for a, b in zip(chain, chain[1:]):
            if a not in p or b not in p:
                continue
            d, pa, pb = _nearest(p[a], p[b])
            out.append((f"{a} -> {b}", pa, pb, d))
    return out


def tube(a, b, r=BUNDLE_R):
    """The cable as a swept volume: the straight line between two ports."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    v = b - a
    L = float(np.linalg.norm(v))
    if L < 1e-6:
        return None
    from cad.toolaccess import _align_z
    loc = bd.Location(bd.Vector(*a)) * bd.Rotation(*_align_z(v / L))
    return loc * bd.Pos(0, 0, L / 2) * bd.Cylinder(r, L)


def check(mode="wheel", verbose=True):
    """Does the shortest possible cable path fit?"""
    import cad.assemble_check as ac

    solids = ac.parts(mode)
    rows = []
    for label, a, b, L in runs(mode):
        t = tube(a, b)
        # A cable necessarily leaves its own connector, so the two servos at
        # the ends of a run are not obstructions - counting them makes every
        # run look blocked and says nothing.
        ends = set(label.split(" -> "))
        hit, by = 0.0, {}
        if t is not None:
            for pname, solid in solids:
                if pname in ends:
                    continue
                try:
                    i = t & solid
                    v = i.volume if i else 0.0
                except Exception:
                    v = 0.0
                if v > TOUCH:
                    by[pname] = v
                hit += v
        rows.append(dict(label=label, length=L, blocked=hit, by=by))

    if verbose:
        print(f"--- {mode} mode, straight-line runs, {2*BUNDLE_R:.1f} mm bundle")
        for r in rows:
            flag = "  BLOCKED" if r["blocked"] > TOUCH else ""
            print(f"   {r['label']:<28}{r['length']:7.1f} mm"
                  f"{r['blocked']:9.0f} mm3{flag}")
            if r["by"]:
                worst = sorted(r["by"].items(), key=lambda kv: -kv[1])[:3]
                print(f"      through: "
                      + ", ".join(f"{k} {v:.0f}" for k, v in worst))
        n = sum(1 for r in rows if r["blocked"] > TOUCH)
        print(f"   {n} of {len(rows)} runs pass through solid material")
    return rows


def slack(verbose=True):
    """How much the runs stretch between wheel mode and foot mode.

    The ankle roll joint turns 90 degrees, so the ports either side of it move
    relative to each other. Whatever that difference is has to be present as
    slack in BOTH poses, plus a bend radius, or the cable goes tight in one and
    loops loose enough to snag in the other.
    """
    w = {r[0]: r[3] for r in runs("wheel")}
    f = {r[0]: r[3] for r in runs("foot")}
    rows = []
    for k in w:
        if k in f:
            rows.append((k, w[k], f[k], f[k] - w[k]))
    rows.sort(key=lambda r: -abs(r[3]))
    if verbose:
        print("cable length, wheel mode vs foot mode")
        print(f"   {'run':<28}{'wheel':>8}{'foot':>8}{'change':>9}")
        for k, a, b, d in rows:
            note = "   <-- crosses the roll joint" if abs(d) > 1.0 else ""
            print(f"   {k:<28}{a:8.1f}{b:8.1f}{d:+9.1f}{note}")
        worst = max((abs(r[3]) for r in rows), default=0.0)
        print()
        print(f"   worst change {worst:.1f} mm, so a service loop of at least "
              f"{worst:.0f} mm plus a {MIN_BEND_R:.0f} mm bend radius")
    return rows


def main():
    for mode in ("wheel", "foot"):
        check(mode)
        print()
    slack()


if __name__ == "__main__":
    main()
