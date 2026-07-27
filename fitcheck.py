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
                             ankle_pitch_level, axle_height, leg_ik, load)

# Pairs that legitimately share material: a hub inside its tyre, and parts
# that bolt to each other through a lap joint.
SKIP_PAIRS = {("vhub", "vtire"), ("vankpost", "vrollsv")}

# Anything else may not overlap by more than a print tolerance.
TOL = 0.0005


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


def pose(m, d, mode, frac=None):
    """Pose the robot. `mode` picks an end state; `frac` in [0,1] instead walks
    the flip, which is where parts actually sweep past each other.
    """
    if frac is not None:
        h = 0.207 + (0.195 - 0.207) * frac
        roll = ROLL_WHEEL + (ROLL_FOOT - ROLL_WHEEL) * frac
        z = h + axle_height(roll)
    elif mode == "foot":
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


def gap(a, b):
    """Separation between two oriented boxes, 0 if they touch or overlap.

    Max over the separating axes, which is a lower bound on true distance:
    it can under-report a gap, never invent one.
    """
    (ca, ea, Ra), (cb, eb, Rb) = a, b
    t = cb - ca
    worst = -math.inf
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            c = np.cross(Ra[:, i], Rb[:, j])
            if np.linalg.norm(c) > 1e-9:
                axes.append(c / np.linalg.norm(c))
    for ax in axes:
        ra = sum(ea[k] * abs(np.dot(ax, Ra[:, k])) for k in range(3))
        rb = sum(eb[k] * abs(np.dot(ax, Rb[:, k])) for k in range(3))
        worst = max(worst, abs(np.dot(t, ax)) - (ra + rb))
    return max(0.0, worst)


def connectivity(mode, touch=0.004, verbose=True):
    """Is the robot one connected object, or a cloud of floating parts?

    Overlap-free is not the same as assembled. Two parts that miss each other
    by 30 mm pass a penetration check and still look like they are hovering.
    """
    m, d = load()
    pose(m, d, mode)
    name = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
    vis = [i for i in range(m.ngeom)
           if m.geom_group[i] == 0 and (name(i) or "") != "floor"
           and not (name(i) or "").startswith("h_")]

    parent = {i: i for i in vis}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    nearest = {i: (math.inf, None) for i in vis}
    for i, j in itertools.combinations(vis, 2):
        g = gap(_obb(m, d, i), _obb(m, d, j))
        for a, b in ((i, j), (j, i)):
            if g < nearest[a][0]:
                nearest[a] = (g, b)
        if g <= touch:
            parent[find(i)] = find(j)

    groups = {}
    for i in vis:
        groups.setdefault(find(i), []).append(i)
    if verbose:
        print(f"--- {mode} mode: {len(groups)} connected group(s)")
        if len(groups) > 1:
            big = max(groups.values(), key=len)
            for g in sorted(groups.values(), key=len):
                if g is big:
                    continue
                for i in g:
                    dist, other = nearest[i]
                    print(f"   FLOATING {name(i):<14} nearest {name(other):<14} "
                          f"{dist*1000:5.1f} mm away")
    return groups


def audit(mode, verbose=True, frac=None):
    m, d = load()
    pose(m, d, mode, frac)
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
        # A shaft lives INSIDE bores, and a box model has no bores - every
        # carrier a shaft passes through reads as a solid block. These pairs
        # are not unchecked: cad/assemble_check.py intersects the real solids,
        # where the bores exist, and covers exactly these pairs. Nothing else
        # is exempted, and no printed part is.
        if "shaft" in na or "shaft" in nb:
            continue
        # Geoms on the SAME body cannot move relative to each other, so shared
        # material between two PRINTED pieces is a lap joint, and MuJoCo skips
        # these for the same reason.
        #
        # But a bought part is not something you can lap-joint into: a bracket
        # cannot occupy the same space as a servo case. Skipping same-body
        # pairs blindly hid the ankle yoke sitting 2597 mm3 inside the roll
        # servo.
        bought = ("vhipsv", "vkneesv", "vanksv", "vrollsv", "vwhlsv",
                  "vtire", "vhub", "vpi", "vbatt", "vdriver")
        involves_bought = na.startswith(bought) or nb.startswith(bought)
        if m.geom_bodyid[i] == m.geom_bodyid[j] and not involves_bought:
            continue
        stem = tuple(sorted((na.rsplit("_", 1)[0], nb.rsplit("_", 1)[0])))
        if stem in SKIP_PAIRS:
            continue
        p = penetration(_obb(m, d, i), _obb(m, d, j))
        if p > TOL:
            hits.append((p, na, nb))
    hits.sort(reverse=True)
    if verbose:
        label = mode if frac is None else f"flip {frac*100:3.0f}%"
        print(f"--- {label}: {len(hits)} interpenetrating pairs")
        for p, na, nb in hits:
            print(f"   {na:<14} x {nb:<14} {p*1000:6.1f} mm")
    return hits


def sweep_audit(steps=25, verbose=True):
    """Walk the whole flip, not just its end points."""
    worst = {}
    for i in range(steps + 1):
        for _, na, nb in audit(None, verbose=False, frac=i / steps):
            pass
    for i in range(steps + 1):
        f = i / steps
        for p, na, nb in audit(None, verbose=False, frac=f):
            key = tuple(sorted((na, nb)))
            if p > worst.get(key, (0,))[0]:
                worst[key] = (p, f)
    if verbose:
        print(f"--- through the flip ({steps + 1} poses): "
              f"{len(worst)} pairs collide at some point")
        for (a, b), (p, f) in sorted(worst.items(), key=lambda kv: -kv[1][0]):
            print(f"   {a:<14} x {b:<14} {p*1000:6.1f} mm at {f*100:3.0f}%")
    return worst


if __name__ == "__main__":
    total = 0
    for mode in ("wheel", "foot"):
        total += len(audit(mode))
        print()
    print("no interpenetration" if total == 0
          else f"{total} overlapping pairs to fix")
    print()
    for mode in ("wheel", "foot"):
        connectivity(mode)
        print()
    sweep_audit()
