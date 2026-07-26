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

import cad.ankle as ankle
import cad.chassis as chassis
import cad.shin as shin
import cad.thigh as thigh
from fitcheck import pose
from src.rsbot.model import load

SERVO = (24.7, 35.4, 45.2)     # x, y, z of an STS3215 case as the sim places it


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


def parts(mode):
    m, d = load()
    pose(m, d, mode)
    built = {"torso": chassis.build(), "thigh": thigh.build(),
             "shin": shin.build(), "ankle": ankle.yoke(),
             "rollbracket": ankle.roll_bracket()}

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


def main(mode="wheel"):
    items = parts(mode)
    print(f"--- {mode} mode: {len(items)} solids")
    worst = []
    for (na, a), (nb, b) in itertools.combinations(items, 2):
        try:
            inter = a & b
            v = inter.volume if inter else 0.0
        except Exception:
            v = 0.0
        if v > 1.0:
            worst.append((v, na, nb))
    worst.sort(reverse=True)
    for v, na, nb in worst:
        print(f"   {na:<14} x {nb:<14} {v:9.1f} mm3")
    print(f"   {len(worst)} interfering pairs")
    return worst


if __name__ == "__main__":
    modes = sys.argv[1:] or ["wheel", "foot"]
    total = sum(len(main(mo)) for mo in modes)
    print()
    print("clean" if total == 0 else f"{total} pairs to fix")
