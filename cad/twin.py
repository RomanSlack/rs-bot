"""Does the sim describe the same robot the CAD does?

    uv run python -m cad.twin

This is the check the whole project is for. rs-bot's goal is a digital twin
exact enough to build from once, and the twin is TWO models pretending to be
one: `src/rsbot/model.py` builds the robot out of boxes for the physics, and
`cad/` builds the same robot out of real solids for the printer. Nothing has
ever forced them to agree.

They had drifted, and the drift was invisible because each half is internally
consistent. The sim balances. The parts fit. Neither notices that they are
describing different robots.

WHAT IT MEASURES. Every group-0 visual box in the sim that stands in for
PRINTED structure gets intersected with the CAD solid for its link. A box that
is not backed by CAD material is the sim claiming structure that will not be
printed, which is the dangerous direction: the balancer is tuned against it,
the clearance checks route around it, and it does not exist.

WHAT IT FOUND FIRST TIME:

    vshinarm    0% backed, and that is the whole finding. An aft member from
                a routing cad/shin.py ABANDONED - "the aft version ended up
                13 mm THROUGH the floor. Forward, the same tilt lifts it." The
                CAD went forward. The sim never followed and had been carrying
                the dead member ever since, where fitcheck dutifully routed the
                wheel servo around a piece of structure that will never exist.

Everything else came back 90-100% backed, which is the useful half of the
result: the drift was one member, not a general rot.

The reverse direction is not checked and is deliberately not treated as an
error: the CAD has fillets, ribs, channels and bolt holes the boxes never had,
and demanding the boxes cover them would be demanding the sim stop being a
simplification.
"""

import sys

import build123d as bd
import mujoco
import numpy as np

import cad.ankle as ankle
import cad.chassis as chassis
import cad.shin as shin
import cad.thigh as thigh

# link -> the CAD solid that link is supposed to be
PART = {
    "thigh": thigh.build,
    "shin": shin.build,
    "ankle": ankle.yoke,
    "rollbracket": ankle.roll_bracket,
    "torso": chassis.build,
}

# How much of a stand-in box has to be real CAD material before it counts.
# Not 100%: these are deliberately coarse boxes and a rounded or ribbed part
# will always leave a few percent hanging in air at the corners.
BACKED = 0.60

# Boxes that are DELIBERATELY not solid CAD material, with the reason. Named
# one by one rather than handled by loosening BACKED, because "the threshold is
# generous enough to cover it" is how a real divergence gets waved through.
SIMPLIFIED = {
    "vpistand": "one 80 x 52 x 4 slab standing in for four Ø6 x 4 posts, so "
                "about 3% of it is ever material",
    # Declared rather than handled by loosening BACKED, which is the whole
    # argument of this dict. It fell from 60.7% to 50.5% on 2026-08-02 when the
    # post was shortened from 6.0 to 4.5 mm to free web section for the roll
    # bracket. The bearing pocket is a fixed 4.0 mm deep, so shortening the post
    # raised the fraction of it that is hole from 67% to 89%. The bearing still
    # gets its full width and the same 1 mm radial wall; what changed is only
    # how well a solid BOX describes a carrier that is mostly pocket.
    "vankpost": "a 4.5 mm bearing carrier with a 4.0 mm bearing pocket through "
                "it, so about half of it is hole by construction",
}


# --- the other direction: CAD constants that RESTATE a bought part -----------
#
# The check above asks whether the sim claims structure the CAD does not build.
# This one asks the opposite and narrower question: where cad/ has written down
# the box of a BOUGHT part, does that box still agree with the sim's?
#
# It exists because a bought part's size is not a design choice, it is a
# measurement, and every one of these constants is a second copy of it. The
# repo's own rule says a number in two files means one of them is already
# wrong. Three of them were:
#
#   envelope.WHEEL_SERVO   12.500 mm in x, which is SHAFT_X. It still centred
#                          the case on the wheel axis. The sim was corrected on
#                          2026-08-01 to put the SHAFT there; the envelope was
#                          not, and the envelope is what decides which bands
#                          the ankle-pitch bearing may sit in.
#   envelope.ROLL_SERVO     3.500 mm in z, from centring the case on a literal
#                          16.0 where the sim derives 12.5 from the shaft.
#   ankle.ROLL_SV           3.100 mm in x, the STEP's 39.6 shaft span that the
#                          manufacturer's drawing superseded with 36.5.
#
# None of them could interfere with anything, which is exactly why they lasted:
# assemble_check and fitcheck run on the sim's boxes and the real solids, and
# never look at these. They are design-time constraints, so being wrong makes
# the NEXT part wrong rather than this one.
#
# Each entry is (label, the CAD constant, the sim geom it copies, the frame both
# are measured in). A frame is either ("joint", name) - the world position
# relative to that joint's anchor - or ("body", name), the geom's own offset
# within that body, which is how a constant written in a link's frame is stated.
BOUGHT = [
    ("envelope.WHEEL_SERVO", "WHEEL_SERVO", "vwhlsv_l", ("joint", "ankle_roll_l")),
    ("envelope.ROLL_SERVO", "ROLL_SERVO", "vrollsv_l", ("joint", "ankle_roll_l")),
    ("ankle.ROLL_SV", "ROLL_SV", "vrollsv_l", ("joint", "ankle_roll_l")),
    # ANKLE_SERVO_SHIN used to be here. It was the entry that proved the value
    # of not excusing a constant: it had been left uncovered on the grounds
    # that the shin frame was not pinned against the sim's shin body, and the
    # first line of the measurement disproved that - x and y matched to a
    # thousandth of a millimetre, and only z had drifted, by 6.8 mm.
    #
    # There is nothing to compare now: that servo is deleted, so there is no
    # sim box for it. The constant is gone from cad/envelope.py too, rather
    # than left sitting among live ones with nothing reading it.
]

# These are copies of one measurement, so they should agree exactly. 0.01 mm is
# formatting noise, not tolerance: anything real shows up far above it. The old
# 24.8-vs-24.73 case width lands at 0.035 and must fail, which is the point.
BOX_TOL = 0.01

# Named rather than silently absent, because an unchecked constant that nobody
# lists reads as a checked one. Empty is the goal; when it is not, the entry
# says what would have to be true to cover it, so the excuse can be argued with.
NOT_COVERED = {}


def bought(verbose=True):
    """[(label, worst mm, ok)] for every CAD constant that restates a sim box."""
    import cad.ankle as _ankle
    import cad.envelope as _env
    from cad.drives import _posed

    m, d, anchors = _posed("wheel")
    src = {"WHEEL_SERVO": _env, "ROLL_SERVO": _env, "ROLL_SV": _ankle,
           "ANKLE_SERVO_SHIN": _env}

    rows = []
    for label, attr, geom, (kind, ref) in BOUGHT:
        sim = None
        for i in range(m.ngeom):
            if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "") != geom:
                continue
            if kind == "joint":
                c = d.geom_xpos[i] * 1000.0 - anchors[ref][0]
            else:
                bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ref)
                if m.geom_bodyid[i] != bid:
                    raise RuntimeError(
                        f"{geom} is not on {ref}, so the frame is wrong")
                c = m.geom_pos[i] * 1000.0
            h = m.geom_size[i] * 1000.0
            sim = np.array([c - h, c + h]).T
        if sim is None:
            rows.append((label, float("nan"), False))
            continue
        cad = np.asarray(getattr(src[attr], attr), float)
        worst = float(np.max(np.abs(cad - sim)))
        rows.append((label, worst, worst <= BOX_TOL))

    if verbose:
        print("--- does every CAD copy of a bought part still match the sim?")
        for label, worst, ok in rows:
            if worst != worst:
                print(f"   {label:24s}   no such geom in the sim")
                continue
            print(f"   {label:24s} worst {worst:7.3f} mm   "
                  f"{'ok' if ok else 'DRIFTED'}")
        for label, why in NOT_COVERED.items():
            print(f"   {label:24s} not covered: {why}")
        bad = [r for r in rows if not r[2]]
        print(f"   {len(bad)} constant(s) no longer describe the sim's part"
              if bad else "   every bought-part box agrees")
    return rows


# --- and does the sim WEIGH what the CAD says --------------------------------
#
# `SEG_INERTIA` in src/rsbot/model.py is pasted from `cad.inertia --emit`, with
# the regenerate command written in the comment directly above it. Nobody re-ran
# it, so every gram added to a part in cad/ stayed in cad/.
#
# Note it is SEG_INERTIA and not SEG_MASS. `SEG_MASS` sits right above it, looks
# authoritative, carries a comment reading "Derived, not estimated", and is
# referenced NOWHERE - dead, and stale in its own right. It cost an hour of
# chasing the wrong constant, which is what a dead lookalike does.
#
# Found 2026-08-02: the sim weighed 1874.8 g against the CAD's 1907 g. 29 of
# those 32 grams are the four servo cradles added on 2026-08-01, whose cost the
# status report that day recorded exactly - "mass went 1875 g to 1904 g" - into
# a document rather than into the model. The sim was still the pre-cradle robot.
#
# Why it matters more than 1.7% suggests: the balancer is tuned against this,
# and the missing mass is not spread evenly. It is on the legs, at the ankle and
# the roll bracket, furthest from the pitch axis and worst for inertia.
#
# Per body rather than in total, because two errors in opposite directions
# cancel in a total and a total is what everybody reads.
MASS_TOL = 0.002       # kg. 2 g per body, about a fastener's worth.


def masses(verbose=True):
    """[(body, sim kg, cad kg, ok)] for every body the CAD derives a mass for."""
    import mujoco as _mj

    import cad.masses as _masses
    from src.rsbot.model import load

    m, _ = load()
    cad = {b: tot / 1000.0 for b, (_, _, _, tot) in _masses.table().items()}
    rows = []
    for body, kg in sorted(cad.items()):
        bid = _mj.mj_name2id(m, _mj.mjtObj.mjOBJ_BODY, body)
        sim = float(m.body_mass[bid]) if bid >= 0 else float("nan")
        rows.append((body, sim, kg, abs(sim - kg) <= MASS_TOL))

    if verbose:
        print("--- does the sim weigh what the CAD says?")
        for body, sim, kg, ok in rows:
            print(f"   {body:14s} sim {sim*1000:7.1f} g   cad {kg*1000:7.1f} g   "
                  f"{(sim-kg)*1000:+7.1f} g   {'ok' if ok else 'DRIFTED'}")
        ssum = sum(r[1] for r in rows)
        csum = sum(r[2] for r in rows)
        print(f"   {'total':14s} sim {ssum*1000:7.1f} g   cad {csum*1000:7.1f} g   "
              f"{(ssum-csum)*1000:+7.1f} g")
        bad = [r for r in rows if not r[3]]
        print(f"   {len(bad)} body(s) do not weigh what the CAD builds"
              if bad else "   every body weighs what the CAD builds")
    return rows


def _boxes(m, body, side="l"):
    """Every printed stand-in box on one body, in that body's frame."""
    from src.rsbot.model import PRINTED_VIS_RE

    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    out = []
    for i in range(m.ngeom):
        if m.geom_bodyid[i] != bid or m.geom_group[i] != 0:
            continue
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if not PRINTED_VIS_RE.match(n):
            continue
        if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_BOX:
            out.append((n, None))        # cylinders etc, reported and skipped
            continue
        p = m.geom_pos[i] * 1000.0
        s = m.geom_size[i] * 1000.0
        out.append((n, bd.Pos(*p) * bd.Box(*(s * 2))))
    return out


def check(verbose=True):
    """[(link, geom, backed fraction, ok)] for every printed stand-in."""
    from src.rsbot.model import load

    m, _ = load()
    rows = []
    for link, build in PART.items():
        body = "torso" if link == "torso" else f"{link}_l"
        solid = build()
        for name, box in _boxes(m, body):
            if box is None:
                rows.append((link, name, float("nan"), True))
                continue
            v = box.volume
            inside = (box & solid).volume if v else 0.0
            frac = inside / v if v else 1.0
            base = name.rsplit("_", 1)[0] if name.endswith(("_l", "_r")) else name
            rows.append((link, name, frac,
                         frac >= BACKED or base in SIMPLIFIED))
    if verbose:
        print("--- is every box in the sim backed by real CAD material?")
        for link, name, frac, ok in rows:
            if frac != frac:
                print(f"   {link:12s} {name:18s}   not a box, skipped")
                continue
            base = name.rsplit("_", 1)[0] if name.endswith(("_l", "_r")) else name
            note = ("simplified on purpose" if base in SIMPLIFIED and frac < BACKED
                    else ("ok" if ok else "NOT IN THE CAD"))
            print(f"   {link:12s} {name:18s} {frac*100:6.1f}% backed   {note}")
        bad = [r for r in rows if not r[3]]
        print(f"   {len(bad)} stand-in(s) the CAD does not build"
              if bad else "   the sim and the CAD agree")
    return rows


def main():
    bad = sum(1 for r in check() if not r[3])
    print()
    bad += sum(1 for r in bought() if not r[2])
    print()
    bad += sum(1 for r in masses() if not r[3])
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
