"""Do the real parts actually fit?  uv run python fitcheck.py

MuJoCo will not tell you this: geoms in the same body never collide, and the
visual build is non-colliding by design. So this does its own oriented-box
overlap test (separating axis) over every pair of visual parts, in both wheel
and foot mode, and reports interpenetration in millimetres.

Cylinders are treated as their bounding box, which is conservative: it can
report a near-miss as a touch, never the reverse.
"""

import itertools
import math

import mujoco
import numpy as np

from src.rsbot.model import (ROLL_FOOT, ROLL_WHEEL, WHEEL_HALF_W, WHEEL_R,
                             ankle_pitch_level, leg_ik, load)

SKIP_PAIRS = {("vhub", "vtire")}          # hub sits inside the tyre by design


def _obb(m, d, i):
    """Centre, half-extents, rotation for geom i as an oriented box."""
    sz = m.geom_size[i].copy()
    t = m.geom_type[i]
    if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        sz = np.array([sz[0], sz[0], sz[1]])
    elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
        sz = np.array([sz[0], sz[0], sz[1] + sz[0]])
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        sz = np.array([sz[0]] * 3)
    return d.geom_xpos[i].copy(), sz, d.geom_xmat[i].reshape(3, 3)


def penetration(a, b):
    """Overlap depth along the least-separated axis, 0 if disjoint (SAT)."""
    (ca, ea, Ra), (cb, eb, Rb) = a, b
    t = cb - ca
    best = math.inf
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            c = np.cross(Ra[:, i], Rb[:, j])
            if np.linalg.norm(c) > 1e-9:
                axes.append(c / np.linalg.norm(c))
    for ax in axes:
        ra = sum(ea[k] * abs(np.dot(ax, Ra[:, k])) for k in range(3))
        rb = sum(eb[k] * abs(np.dot(ax, Rb[:, k])) for k in range(3))
        gap = abs(np.dot(t, ax)) - (ra + rb)
        if gap > 0:
            return 0.0
        best = min(best, -gap)
    return best


def pose(m, d, mode):
    if mode == "foot":
        h, roll, z = 0.195, ROLL_FOOT, 0.195 + WHEEL_HALF_W
    else:
        h, roll, z = 0.207, ROLL_WHEEL, 0.207 + WHEEL_R
    hip, knee = leg_ik(h)
    want = {"hip": hip, "knee": knee,
            "ankle_pitch": ankle_pitch_level(hip, knee), "ankle_roll": roll}
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = z, 1.0
    for j in range(m.njnt):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        if n and n != "root":
            d.qpos[m.jnt_qposadr[j]] = want.get(n.rsplit("_", 1)[0], 0.0)
    mujoco.mj_forward(m, d)


def audit(mode, verbose=True):
    m, d = load()
    pose(m, d, mode)
    # Robot parts only. The floor is not a part, and the scale-reference human
    # is a deliberately overlapping stack of primitives.
    vis = [i for i in range(m.ngeom)
           if m.geom_group[i] == 0
           and (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
           not in ("floor",)
           and not (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
           .startswith("h_")]
    name = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)

    hits = []
    for i, j in itertools.combinations(vis, 2):
        na, nb = name(i), name(j)
        stem = tuple(sorted((na.rsplit("_", 1)[0], nb.rsplit("_", 1)[0])))
        if stem in SKIP_PAIRS:
            continue
        p = penetration(_obb(m, d, i), _obb(m, d, j))
        if p > 1e-4:
            hits.append((p, na, nb))
    hits.sort(reverse=True)
    if verbose:
        print(f"--- {mode} mode: {len(hits)} interpenetrating pairs")
        for p, na, nb in hits:
            print(f"   {na:<14} x {nb:<14} {p*1000:6.1f} mm")
    return hits


if __name__ == "__main__":
    total = 0
    for mode in ("wheel", "foot"):
        total += len(audit(mode))
        print()
    print("clean" if total == 0 else f"{total} overlapping pairs to fix")
