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
}


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
    return 1 if any(not r[3] for r in check()) else 0


if __name__ == "__main__":
    sys.exit(main())
