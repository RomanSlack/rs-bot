"""Can these parts actually be printed?

    uv run python -m cad.printability

Everything so far has asked whether the parts are strong enough and whether
they fit together. Neither question notices that a part needs support material
through a bearing bore, or that a 1.4 mm web is thinner than two perimeters, or
that it does not fit on the bed. Those are the things that turn a correct design
into a failed print, and they are cheap to check.

Three checks, all off the exported STL in the part's chosen print orientation:

    overhangs        downward-facing area steeper than the printer can bridge
    thin walls       features narrower than a sensible number of perimeters
    bed footprint    the obvious one, and the one nobody checks until it fails

A note on what this cannot see: it works on triangles, so it finds overhanging
SURFACE, not whether that surface is self-supporting because something else is
underneath it. A steep face 2 mm above the bed is fine and this will flag it.
Treat the overhang number as "how much support material", not as a pass/fail.
"""

import sys
from pathlib import Path

import numpy as np

from cad import stress

OUT = Path(__file__).parent / "out"

NOZZLE = 0.4
PERIMETERS = 3
MIN_WALL = NOZZLE * 2 * PERIMETERS / 2      # 1.2 mm: three perimeters a side
OVERHANG_DEG = 45.0                          # steeper than this needs support
BED = (250.0, 210.0, 210.0)                  # a Prusa-class bed


def _stl(path):
    """(vertices, triangles) from a binary STL."""
    raw = path.read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    f = rec[:, :48].copy().view(np.float32).reshape(n, 4, 3)
    return f[:, 0], f[:, 1:]                 # normals, triangle vertices


def open_edges(tris, places=4):
    """How many edges are not shared by exactly two triangles.

    A closed surface has every edge used twice, so a watertight mesh satisfies
    edges == 1.5 * triangles exactly. Anything else is a hole, a crack or a
    duplicated facet, and it is the first thing a print service's own checker
    rejects - which costs a fortnight here, since we do not have the printer.

    Nothing checked this. It came up on 2026-08-02 because a chunk of geometry
    looked like an artifact and there was no way to answer "is that a real hole"
    except by eye. Rounding the coordinates is what makes the comparison work:
    STL stores float32 per triangle with no shared-vertex table, so the same
    corner arrives with slightly different bits from each face that meets there.
    """
    d = {}
    for t in np.round(tris, places):
        p = [tuple(v) for v in t]
        for i in range(3):
            a, b = p[i], p[(i + 1) % 3]
            k = (a, b) if a < b else (b, a)
            d[k] = d.get(k, 0) + 1
    return sum(1 for c in d.values() if c != 2)


def _rotate_to(layer):
    """Rotation taking the part's `layer` normal onto +z, i.e. into the
    orientation it is actually printed in."""
    n = np.asarray(layer, float)
    n = n / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    if np.allclose(n, z):
        return np.eye(3)
    v = np.cross(n, z)
    s, c = np.linalg.norm(v), float(np.dot(n, z))
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def overhang(normals, tris, R):
    """(overhanging area, total area) in the printed orientation."""
    n = normals @ R.T
    a = tris @ np.transpose(R)
    area = 0.5 * np.linalg.norm(np.cross(a[:, 1] - a[:, 0], a[:, 2] - a[:, 0]),
                                axis=1)
    nz = n[:, 2] / np.maximum(np.linalg.norm(n, axis=1), 1e-12)
    # Downward-facing, and steeper than the overhang angle from horizontal.
    steep = nz < -np.cos(np.deg2rad(90 - OVERHANG_DEG))
    return float(area[steep].sum()), float(area.sum())


def footprint(tris, R):
    v = tris.reshape(-1, 3) @ np.transpose(R)
    return v.max(axis=0) - v.min(axis=0)


def thin_features(part_boxes):
    """Named box members thinner than MIN_WALL in any direction."""
    bad = []
    for name, spec in part_boxes:
        d = [spec[i][1] - spec[i][0] for i in range(3)]
        if min(d) < MIN_WALL:
            bad.append((name, min(d)))
    return bad


def boxes_of(mod, names):
    return [(n, getattr(mod, n)) for n in names if hasattr(mod, n)]


PARTS = {
    "thigh": ("thigh.stl", []),
    "shin": ("shin.stl", ["SPINE", "STANDOFF", "FARM", "FPOST", "CROSS",
                          "DROP", "BACK"]),
    "ankle_yoke": ("ankle_yoke.stl", ["POST", "YOKE_CROSS", "YOKE_FWD"]),
    "roll_bracket": ("roll_bracket.stl", ["ROLL_ARM"]),
    "chassis": ("chassis.stl", []),
    # Both halves of the wheel. The tyre is the one part here that is not
    # rigid, and it still has to be printable: it carries the tread grooves and
    # the bead recess, and if either goes under the minimum wall it cannot be
    # ordered no matter how good the geometry is.
    "wheel_body": ("wheel_body.stl", []),
    "wheel_tyre": ("wheel_tyre.stl", []),
    # The parallelogram, cad/linkage.py. Two rods and an idler bar; the three
    # mount features are printed as part of the chassis, the shin and the yoke
    # and are checked with those. Small parts, but the eyes are the thinnest
    # sections in the robot and a rod eye printed on its side needs support
    # through the bore it turns on.
    "lk_rod1": ("lk_rod1.stl", []),
    "lk_rod2": ("lk_rod2.stl", []),
    "lk_idler": ("lk_idler.stl", []),
}
MODULES = {"shin": "cad.shin", "ankle_yoke": "cad.ankle",
           "roll_bracket": "cad.ankle"}
# Print orientation for parts that have no FEA entry to take it from. The tyre
# lies flat on its sole: that face is the one that must come out clean.
LAYER = {"wheel_tyre": (0, 0, 1)}
LAYER.update(__import__("cad.linkage", fromlist=["LAYER"]).LAYER)


def main():
    import importlib

    import cad.linkage
    import cad.robot
    cad.robot.export_all()
    cad.linkage.export_stls()

    specs = stress._parts()
    print(f"nozzle {NOZZLE} mm, {PERIMETERS} perimeters -> {MIN_WALL:.1f} mm "
          f"minimum wall; overhang limit {OVERHANG_DEG:.0f} deg\n")
    print(f"{'part':<14}{'footprint mm':>22}{'fits bed':>10}"
          f"{'overhang':>10}{'open edges':>12}{'thin features':>16}")
    leaks = 0
    for name, (stl, boxnames) in PARTS.items():
        R = _rotate_to(specs[name]["layer"] if name in specs else LAYER[name])
        normals, tris = _stl(OUT / stl)
        over, total = overhang(normals, tris, R)
        fp = footprint(tris, R)
        fits = all(fp[i] < BED[i] for i in range(3))
        holes = open_edges(tris)
        leaks += holes
        thin = []
        if name in MODULES:
            mod = importlib.import_module(MODULES[name])
            thin = thin_features(boxes_of(mod, boxnames))
        print(f"{name:<14}{fp[0]:>7.0f}{fp[1]:>7.0f}{fp[2]:>7.0f}"
              f"{'yes' if fits else 'NO':>10}{over/total*100:>9.0f}%"
              f"{holes if holes else 'closed':>12}"
              f"{(', '.join(f'{n} {d:.1f}mm' for n, d in thin) or 'none'):>16}")
    if leaks:
        print(f"\n  {leaks} open edge(s): a print service will reject this")
    return 1 if leaks else 0


if __name__ == "__main__":
    sys.exit(main())
