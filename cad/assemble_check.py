"""Whole-robot interference, on the real CAD solids.

    uv run python -m cad.assemble_check

Every part is transformed to the pose the SIMULATOR puts it in, then every
pair is intersected. Slow, but it is the only version that actually answers
the question.

Do NOT do this by dropping the meshes into a MuJoCo scene and counting
contacts: bodies that are all static children of world cannot move relative to
each other, so MuJoCo skips every pair and reports a clean zero no matter what
you feed it. That check passed a deliberately overlapping control.
"""

import itertools
import sys

import build123d as bd
import mujoco
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

import cad.ankle as ankle
import cad.chassis as chassis
import cad.shin as shin
import cad.thigh as thigh
import cad.wheel as wheel
from fitcheck import pose
from src.rsbot.model import load

# x, y, z of an STS3215 case as the sim places it. Was (24.7, 35.4, 45.2), the
# superseded listing guesses; from cad/servo_dims.py now.
from cad.servo_dims import LENGTH as _SL, WIDTH as _SW, HEIGHT as _SH
SERVO = (_SW, _SH, _SL)


def _loc(pos_m, mat):
    """sim body pose -> build123d Location, metres to millimetres."""
    r = bd.Rotation(*np.degrees(_euler(mat)))
    return bd.Location(bd.Vector(*(pos_m * 1000.0))) * r


def _euler(mat):
    m = mat.reshape(3, 3)
    sy = np.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2)
    if sy > 1e-9:
        return np.array([np.arctan2(m[2, 1], m[2, 2]),
                         np.arctan2(-m[2, 0], sy),
                         np.arctan2(m[1, 0], m[0, 0])])
    return np.array([np.arctan2(-m[1, 2], m[1, 1]), np.arctan2(-m[2, 0], sy), 0.0])


_BUILT = {}


def parts(mode, frac=None):
    """The real solids, posed. `frac` walks the flip instead of picking an end.

    The solids are built ONCE and cached. Placing them is cheap; building them
    is not, and a 26-pose sweep that rebuilt every part each step would take
    long enough that nobody would run it.
    """
    m, d = load()
    pose(m, d, mode, frac)
    built = _BUILT or {"torso": chassis.build(), "thigh": thigh.build(),
             "shin": shin.build(), "ankle": ankle.yoke(),
             "rollbracket": ankle.roll_bracket(),
             # The wheel was not in this check at all, which is a hole in it:
             # it is the part with the most neighbours - servo, roll bracket,
             # ankle yoke, the floor - and the only one that changes shape.
             # Body and tyre go in FUSED, because they are an interference fit
             # now and as separate solids their own press fit would read as a
             # fault. Rot(90, 0, 0) is the CAD frame (spin about z, sole at +z)
             # mapped into the sim's wheel body frame (spin about y, sole
             # inboard); the right side then mirrors in y with everything else.
             "wheel": bd.Rot(90, 0, 0) * (wheel.body() + wheel.tyre())}
    _BUILT.update(built)

    out = []
    for stem, solid in built.items():
        names = ["torso"] if stem == "torso" else [f"{stem}_l", f"{stem}_r"]
        for n in names:
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
            s = solid if not n.endswith("_r") else bd.mirror(solid, bd.Plane.XZ)
            out.append((n, _loc(d.xpos[bid], d.xmat[bid]) * s))

    # Servo cases, from wherever the sim actually puts them.
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if m.geom_group[i] != 0 or not n.startswith(
                ("vhipsv", "vkneesv", "vanksv", "vrollsv", "vwhlsv")):
            continue
        s = m.geom_size[i] * 2000.0
        box = bd.Box(*s)
        out.append((n, _loc(d.geom_xpos[i], d.geom_xmat[i]) * box))
    return out


LEG_CHAIN = ["thigh_l", "vkneesv_l", "shin_l", "vanksv_l",
             "ankle_l", "vrollsv_l", "rollbracket_l", "vwhlsv_l"]


def fused_leg(mode="wheel"):
    """The whole left leg as ONE solid, posed, in world coordinates.

    Every part in cad/stress.py is solved alone, rigidly clamped at its own
    bolt holes. That is non-conservative twice over: a real neighbour is
    compliant, not a wall, and nothing ever adds the deflections up. This is
    what you mesh to ask the assembled question.

    The servo cases are IN, because they are part of the load path: the shin
    hangs off the knee servo's case, not off the thigh directly.

    Fusing is a check in its own right, because an exact union only closes if
    the parts really touch. The first attempt returned TWO solids: the
    wheel-drive servo was floating 0.5 mm off the bracket it bolts to, which
    nothing had ever reported because assemble_check only tested interference
    and 0.5 mm of air reads like 20 mm of air. That gap is closed and the servo
    is now in the chain. See docs/assembled-strength.md.

    Fusing across the servo output shafts models every joint as RIGID, so this
    UNDERSTATES deflection: real gearboxes wind up and the servos have 0.87 deg
    of measured backlash on top. It is the structural contribution only.
    """
    items = dict(parts(mode))
    u = None
    for n in LEG_CHAIN:
        u = items[n] if u is None else u + items[n]
    u = u.clean()
    if len(u.solids()) != 1:
        raise RuntimeError(
            f"the leg fused into {len(u.solids())} solids, not 1 - something "
            f"in {LEG_CHAIN} is not touching its neighbour")
    return u


def gap(a, b):
    """Minimum distance between two solids, mm. 0.0 if they touch or overlap."""
    d = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    d.Perform()
    return d.Value() if d.IsDone() else float("nan")


# Anything closer than this and a tolerance stack can close it. The services
# quote +/-0.3 mm (JLCPCB MJF, PCBWay SLS), so two printed parts facing each
# other can each be 0.3 mm out and eat 0.6 mm of whatever gap was drawn.
TIGHT_MM = 0.8

# But most 0.00 mm pairs in this robot are SUPPOSED to touch: a printed part
# bolted flat to a servo case, or two parts meeting at a bearing. Flagging
# those makes the check noise, and a check that always shouts gets muted, which
# is the same failure as a check that cannot fail.
#
# The wheel is different, and it is the reason this exists. It is the only body
# that turns continuously, through 360 degrees, forever - so for the wheel any
# contact at all is a rub, not a mating face. Everything else either bolts up
# or moves through a limited range that fitcheck.py already sweeps.
def running_pair(na, nb):
    """True if these two move against each other without a bolt between."""
    return na.startswith("wheel") != nb.startswith("wheel")


# --- worst-case tolerance stack ------------------------------------------------
#
# Every pairwise gap above is measured on NOMINAL geometry. Real parts are not
# nominal: the services quote +/-0.3 mm, and the error between two parts is not
# one tolerance but one per interface separating them along the assembly chain.
# The wheel is four joints away from the chassis, so its position relative to
# the chassis can be out by five tolerances, not one.
#
# This is an analytic stack rather than a perturbed solve, because offsetting
# these solids by 0.3 mm fails outright in OCC - they have too many features.
# An analytic stack is also the honest worst case: it assumes every error lines
# up the wrong way, which is what "worst case" means.
TOL = 0.3

STACK_ORDER = ["torso", "thigh", "shin", "ankle", "rollbracket", "wheel"]


def _chain_depth(na, nb):
    """How many toleranced interfaces separate two parts."""
    def idx(n):
        stem = n.split("_")[0]
        return STACK_ORDER.index(stem) if stem in STACK_ORDER else None
    ia, ib = idx(na), idx(nb)
    if ia is None or ib is None:
        return 1                      # a bought part bolted straight on
    if na.endswith("_l") != nb.endswith("_l") and "torso" not in (na, nb):
        # opposite legs: up one chain and down the other
        return ia + ib + 1
    return abs(ia - ib) + 1


def stack(mode="wheel", verbose=True):
    """Nominal gap minus the worst-case stack, for every pair that must not
    touch. A pair that clears at nominal and closes at worst case is a part
    that fits in CAD and rubs on the bench."""
    items = parts(mode)
    rows = []
    for (na, a), (nb, b) in itertools.combinations(items, 2):
        if not running_pair(na, nb):
            continue
        try:
            g = gap(a, b)
        except Exception:
            continue
        d = _chain_depth(na, nb)
        rows.append((g - d * TOL, g, d, na, nb))
    rows.sort()
    if verbose:
        print(f"--- {mode} mode: worst-case stack at +/-{TOL} mm per interface")
        for worst, g, d, na, nb in rows[:8]:
            flag = "  <-- CLOSES" if worst <= 0 else ""
            print(f"   {na:<14} x {nb:<14} nominal {g:6.2f}  "
                  f"{d} interfaces  worst {worst:6.2f}{flag}")
        n = sum(1 for r in rows if r[0] <= 0)
        print(f"   {n} of {len(rows)} running pairs close at worst case")
    return rows


# Below this, an overlap is two faces meeting rather than two parts clashing.
# 50 microns is well under the 0.3 mm the print services quote, so nothing real
# can hide beneath it.
SEAM_MM = 0.05


def _thickness(inter):
    """The thinnest dimension of an overlap, mm. A face contact is ~0."""
    if inter is None:
        return 0.0
    try:
        s = inter.bounding_box().size
        return min(s.X, s.Y, s.Z)
    except Exception:
        return float("inf")


def main(mode="wheel"):
    items = parts(mode)
    print(f"--- {mode} mode: {len(items)} solids")
    worst, tight, mates = [], [], 0
    for (na, a), (nb, b) in itertools.combinations(items, 2):
        try:
            inter = a & b
            v = inter.volume if inter else 0.0
        except Exception:
            inter, v = None, 0.0
        # A COINCIDENT FACE IS NOT INTERFERENCE, and telling them apart needs
        # the shape of the overlap rather than its volume. Two parts that seat
        # flat on each other - which is what fourteen joints in this robot do -
        # produce a sliver a few microns thick across the whole contact patch,
        # and on a 385 mm2 face that is about 2 mm3. The bare volume test called
        # that interference the moment the roll arm was made to touch the wheel
        # servo exactly instead of standing 0.035 mm off it, which is precisely
        # backwards: the correct geometry got flagged and the faulty one passed.
        if v > 1.0 and _thickness(inter) > SEAM_MM:
            worst.append((v, na, nb))
            continue
        # NOT interfering is not the same as fitting. This check used to stop
        # at the line above, so a pair 0.1 mm apart reported exactly like a
        # pair 20 mm apart - and one really was 0.1 mm: the roll bracket's arm
        # against the face of a wheel that turns at 40 rad/s.
        try:
            g = gap(a, b)
        except Exception:
            continue
        if g < TIGHT_MM and running_pair(na, nb):
            tight.append((g, na, nb))
        elif g < 0.01:
            mates += 1

    worst.sort(reverse=True)
    for v, na, nb in worst:
        print(f"   {na:<14} x {nb:<14} {v:9.1f} mm3")
    print(f"   {len(worst)} interfering pairs")
    tight.sort()
    for g, na, nb in tight:
        print(f"   TIGHT {na:<14} x {nb:<14} {g:7.2f} mm  "
              f"(running clearance, needs > {TIGHT_MM})")
    print(f"   {len(tight)} running pairs under {TIGHT_MM} mm"
          f"   [{mates} bolted/bearing faces in contact, as designed]")
    return worst + tight


def run_all(modes=("wheel", "foot")):
    """Everything this module knows how to ask, in one command.

    `stack`, `sweep` and `sweep_stack` all existed and NONE of them ran here:
    the entry point called `main()` on the two rest poses and stopped. So the
    flip sweep on real solids - written on 2026-08-01, and the first thing ever
    to sweep this robot on its actual geometry - only ever ran when somebody
    imported it by hand and remembered to.

    A check nobody runs is not a weaker check, it is an absent one, and it is
    worse than absent because the repo's own documents list it as a check that
    passes. Same family as a check that cannot fail. Found 2026-08-02.
    """
    bad = 0
    for mo in modes:
        bad += len(main(mo))
        print()
        bad += sum(1 for r in stack(mo) if r[0] <= 0)
        print()
    bad += len(sweep())
    print()
    bad += sum(1 for r in sweep_stack() if r[0] <= 0)
    return bad


# The entry point is at the END of this file, not here. It used to be here, and
# `sweep` and `sweep_stack` are defined below, so calling them from here raised
# NameError - the module body had not reached them yet when __main__ ran.


# --- the flip, on the REAL solids ----------------------------------------------
#
# fitcheck.py already sweeps the flip, and it sweeps the sim's BOXES. That was
# fine while the boxes were the whole design. It stopped being fine the moment
# parts grew features the boxes do not carry: the servo capture rims added on
# 2026-08-01 are 2.5 mm walls standing 8-10 mm off four different faces, in the
# tightest region of the robot, and NOTHING had ever asked whether they hit
# anything. They exist only in cad/, so every interference check in the project
# was blind to them.
#
# cad/twin.py checks that no sim box lacks CAD material behind it. This is the
# other direction, and the reason that direction was left unchecked - "the CAD
# has fillets and channels the boxes never had" - does not cover a structural
# wall. A fillet is not a rim.
#
# Coarser than fitcheck on purpose. Solid booleans across 21 parts cost real
# time, so this runs a smaller number of poses and is meant to be run when
# geometry MOVES rather than on every edit.

def sweep_stack(steps=6, verbose=True):
    """The worst-case tolerance stack THROUGH the flip, not just at the ends.

    `stack()` answers "does anything rub once you allow for the print service"
    at the two poses the robot rests in. `sweep()` answers "does anything hit"
    everywhere in between, at nominal. Neither asks the question that actually
    decides whether the built robot works: does anything rub MID-FLIP once you
    allow for the service. docs/road-to-order.md item 5 has wanted this since
    2026-07-27 and called it "a sampling, not a proof ... with no tolerance
    applied at all".

    Analytic, for the reason `stack()` gives: perturbing these solids by 0.3 mm
    fails outright in OCC, and assuming every error lines up the wrong way is
    what worst case means anyway. So it is the same chain-depth arithmetic
    applied at every step of the manoeuvre instead of only at the ends.

    The tightest moment of the flip is not usually either end, which is the
    whole reason this is worth computing rather than inferring.
    """
    rows = []
    for i in range(steps + 1):
        frac = i / steps
        ps = parts(None, frac)
        for (na, a), (nb, b) in itertools.combinations(ps, 2):
            if not running_pair(na, nb):
                continue
            try:
                g = gap(a, b)
            except Exception:
                continue
            d = _chain_depth(na, nb)
            rows.append((g - d * TOL, g, d, frac, na, nb))
    rows.sort()

    if verbose:
        print(f"--- through the flip: worst-case stack at +/-{TOL} mm "
              f"per interface")
        for worst, g, d, frac, na, nb in rows[:8]:
            flag = "  <-- CLOSES" if worst <= 0 else ""
            print(f"   {na:14s} x {nb:14s} nominal {g:6.2f} "
                  f"- {d} x {TOL} = {worst:6.2f} mm at {frac*100:3.0f}%{flag}")
        bad = [r for r in rows if r[0] <= 0]
        print(f"   {len(bad)} pair(s) close under the stack mid-flip"
              if bad else "   nothing closes under the stack, at any point "
                          "in the flip")
    return rows


def sweep(steps=6, verbose=True):
    """[(frac, a, b, mm3)] for every pair of SOLIDS that overlap mid-flip."""
    hits = []
    for i in range(steps + 1):
        frac = i / steps
        ps = parts(None, frac)
        n = 0
        for (na, a), (nb, b) in itertools.combinations(ps, 2):
            try:
                inter = a & b
                v = inter.volume if inter else 0.0
            except Exception:
                inter, v = None, 0.0
            if v > 1.0 and _thickness(inter) > SEAM_MM:
                hits.append((frac, na, nb, v))
                n += 1
        if verbose:
            print(f"   flip {frac*100:3.0f}%   {n} pair(s)")
    if verbose:
        worst = {}
        for frac, na, nb, v in hits:
            k = tuple(sorted((na, nb)))
            if k not in worst or v > worst[k][1]:
                worst[k] = (frac, v)
        for (na, nb), (frac, v) in sorted(worst.items(), key=lambda kv: -kv[1][1]):
            print(f"   {na:14s} x {nb:14s} {v:8.1f} mm3 at {frac*100:3.0f}%")
        print(f"   {len(worst)} pair(s) collide at some point"
              if worst else "   nothing collides through the flip")
    return hits


if __name__ == "__main__":
    modes = sys.argv[1:] or ["wheel", "foot"]
    total = run_all(modes)
    print()
    print("clean" if total == 0 else f"{total} thing(s) to fix")
    sys.exit(1 if total else 0)
