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

# The daisy chain, in order from the bus adapter outward. FOUR servos a leg,
# not five: the ankle-pitch servo is gone (cad/linkage.py), so the run that used
# to go knee -> ankle -> roll now goes knee -> roll and crosses the leg in one
# hop instead of two. That is a longer single run over a joint that moves, not
# simply one fewer cable, which is why this is a routing question and not
# bookkeeping.
CHAIN_L = ["vhipsv1", "vkneesv_l", "vrollsv_l", "vwhlsv_l"]
CHAIN_R = ["vhipsv-1", "vkneesv_r", "vrollsv_r", "vwhlsv_r"]

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
    """Every cable segment, as (label, start, end, length).

    The RIGHT chain is the left one MIRRORED, not solved independently.

    Choosing each side's port pairing by "whichever two are closest" looks
    harmless and is not: on a mirrored assembly the nearest pair on the right
    can be a different pair from the left, and it was. The left thigh's run sat
    at y = -10.0 in its own frame and the right's at y = -0.4, so a channel cut
    into the left part and mirrored onto the right did not line up with the
    right's cable at all - the left cleared and the right stayed blocked.

    The robot is symmetric and its wiring is too. One choice, mirrored.
    """
    p = ports(mode)
    out = []
    for a, b in zip(CHAIN_L, CHAIN_L[1:]):
        if a not in p or b not in p:
            continue
        d, pa, pb = _nearest(p[a], p[b])
        out.append((f"{a} -> {b}", pa, pb, d))
    mirror = np.array([1.0, -1.0, 1.0])
    for (label, pa, pb, d), (a, b) in zip(list(out), zip(CHAIN_R, CHAIN_R[1:])):
        out.append((f"{a} -> {b}", pa * mirror, pb * mirror, d))
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


# --- channels that follow the cable, instead of being retyped -----------------
#
# Every part with a cable channel used to carry hardcoded endpoints, and every
# one of them went stale the moment a servo moved. It happened three times in a
# day: putting the hip and wheel servos on their own joints moved their ports,
# and then correcting the case height from the STEP's 39.6 to the drawing's 36.5
# moved ALL TEN of them by 1.15 mm. Eight of eight runs ended up passing through
# solid material, none by more than 29 mm3, all of them invisible without
# running this file.
#
# So the parts ask where the cable is rather than remembering. The sim is loaded
# once and cached: the answer only changes when the model does.

from functools import lru_cache

import mujoco


@lru_cache(maxsize=None)
def _frames(mode):
    from fitcheck import pose
    from src.rsbot.model import load
    m, d = load()
    pose(m, d, mode)
    mujoco.mj_forward(m, d)
    out = {}
    for b in range(m.nbody):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        out[n] = (d.xmat[b].reshape(3, 3).copy(), d.xpos[b] * 1000.0)
    return out


@lru_cache(maxsize=None)
def local_run(label, body, mode="wheel"):
    """(start, end) of one cable run, in one body's own frame, millimetres.

    `label` is as cad/wiring.py names them, e.g. "vkneesv_l -> vanksv_l".
    Returns None if that run does not exist, so a part can cut what it has.
    """
    R, p = _frames(mode)[body]
    for lab, a, b, _L in runs(mode):
        if lab != label:
            continue
        return (tuple(R.T @ (np.asarray(a) - p)),
                tuple(R.T @ (np.asarray(b) - p)))
    return None


def channel(part, label, body, r=None):
    """Subtract the cable's space from `part`, in BOTH poses.

    Both, always. The run moves between wheel mode and foot mode wherever it
    crosses a joint, and a channel cut for one pose is the wrong channel half
    the time - which is how the shin ended up with 4 mm3 of foot-mode
    interference after its wheel-mode channel was cut and checked.
    """
    r = BUNDLE_R + 0.8 if r is None else r
    for mode in ("wheel", "foot"):
        seg = local_run(label, body, mode)
        if seg is not None:
            part -= tube(seg[0], seg[1], r=r)
    return part
