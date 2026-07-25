"""Sole sweep clearance check.  uv run python clearance.py

The rocker sole swings ~130 deg from retracted to deployed. Before any of that
gets printed, check the swept path is actually free:

  * does the sole dip below its own final deployed height mid-sweep (i.e.
    strike the ground and kick the robot), or does it descend cleanly?
  * does it clip the shin?
  * does it clip the wheel it is supposed to cradle?

Pure kinematics: pose the model, mj_forward, measure. No dynamics.
"""

import numpy as np
import mujoco

from src.rsbot.model import load, leg_ik, sole_angle, WHEEL_R, SHIN_L

SWEEP = np.linspace(0.0, 2.40, 241)
SHIN_R = 0.014
SOLE_HALF_W = 0.016
WHEEL_HALF_W = 0.012


def _ids(m):
    def g(kind, name):
        return mujoco.mj_name2id(m, getattr(mujoco.mjtObj, kind), name)
    return dict(
        sole=g("mjOBJ_GEOM", "sole_l"),
        shin=g("mjOBJ_BODY", "shin_l"),
        wheel=g("mjOBJ_BODY", "wheel_l"),
        ankle=g("mjOBJ_JOINT", "ankle_l"),
        torso=g("mjOBJ_BODY", "torso"),
    )


def sole_corners(m, d, gid):
    c = d.geom_xpos[gid]
    R = d.geom_xmat[gid].reshape(3, 3)
    s = m.geom_size[gid]
    signs = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)
    return c + (signs * s) @ R.T


def seg_dist(pts, a, b):
    """Min distance from each point to segment ab."""
    ab = b - a
    t = np.clip(((pts - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def sweep(height=0.207, verbose=True, sole=None):
    m, d = load(**(sole or {}))
    i = _ids(m)
    hip, knee = leg_ik(height)

    # Pose the leg, keep the wheel on the ground.
    d.qpos[2] = height + WHEEL_R
    for side, base in (("l", 7), ("r", 11)):
        d.qpos[base + 0] = hip
        d.qpos[base + 1] = knee

    rows = []
    for phi in SWEEP:
        # phi is the sole's WORLD angle; convert to the shin-relative encoder.
        th = sole_angle(hip, knee, phi)
        d.qpos[7 + 3] = th   # ankle_l
        d.qpos[11 + 3] = th  # ankle_r
        mujoco.mj_forward(m, d)

        pts = sole_corners(m, d, i["sole"])
        ground = pts[:, 2].min()

        # Shin capsule: body origin down to the ankle/wheel axle.
        sa = d.xpos[i["shin"]]
        sR = d.xmat[i["shin"]].reshape(3, 3)
        sb = sa + sR @ np.array([0, 0, -SHIN_L])
        shin = seg_dist(pts, sa, sb).min() - SHIN_R

        # Wheel: radial distance from the axle axis, for points that overlap
        # the wheel in y (the sole is wider than the wheel).
        wc = d.xpos[i["wheel"]]
        axis = sR @ np.array([0, 1, 0])
        rel = pts - wc
        along = rel @ axis
        radial = np.linalg.norm(rel - along[:, None] * axis, axis=1)
        overlap = np.abs(along) < (WHEEL_HALF_W + SOLE_HALF_W)
        wheel = (radial[overlap] - WHEEL_R).min() if overlap.any() else np.inf

        rows.append((phi, ground, shin, wheel))

    a = np.array(rows)
    # Sole bottom at phi=0, measured with the wheel still on the ground. It is
    # negative (below the floor), which is the point: the robot rises by that
    # much and the wheel ends up that far off the ground.
    deployed_z = a[0, 1]
    lift = -deployed_z

    if verbose:
        print(f"stance height {height*1000:.0f} mm  (hip {hip:+.3f}, knee {knee:+.3f})")
        print(f"  deployed sole face   : {-deployed_z*1000:+.1f} mm below floor "
              f"-> wheel lifts {lift*1000:+.1f} mm")
        print(f"  min shin clearance   : {a[:,2].min()*1000:+.1f} mm "
              f"at sole angle {a[a[:,2].argmin(),0]:.2f} rad")
        print(f"  min wheel clearance  : {a[:,3].min()*1000:+.1f} mm "
              f"at sole angle {a[a[:,3].argmin(),0]:.2f} rad")

        # The kick test: sweeping retracted -> deployed, does the sole ever get
        # lower than where it ends up?
        lowest = a[:, 1].min()
        print(f"  lowest point in sweep: {lowest*1000:+.1f} mm "
              f"at sole angle {a[a[:,1].argmin(),0]:.2f} rad")
        if lowest < deployed_z - 1e-6:
            print(f"  !! sole dips {(deployed_z-lowest)*1000:.1f} mm BELOW its "
                  f"deployed height mid-sweep -- it will strike the ground")
        else:
            print("  sole descends monotonically to contact, no mid-sweep strike")

        print(f"  support polygon      : {2*m.geom_size[i['sole']][0]*1000:.0f} mm "
              f"fore/aft x 120 mm lateral")
    return a


def digging_corner(height=0.207):
    """Which end of the plate leads on the way down, and by how much."""
    m, d = load()
    i = _ids(m)
    hip, knee = leg_ik(height)
    d.qpos[2] = height + WHEEL_R
    d.qpos[7], d.qpos[8] = hip, knee
    d.qpos[11], d.qpos[12] = hip, knee

    worst_phi = 0.63
    d.qpos[10] = d.qpos[14] = sole_angle(hip, knee, worst_phi)
    mujoco.mj_forward(m, d)

    R = d.geom_xmat[i["sole"]].reshape(3, 3)
    s = m.geom_size[i["sole"]]
    lowest, best = None, None
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = d.geom_xpos[i["sole"]] + R @ np.array([sx * s[0], 0, sz * s[2]])
            if lowest is None or p[2] < lowest:
                lowest, best = p[2], (sx, sz)
    end = "TRAILING (aft)" if best[0] < 0 else "LEADING (fore)"
    print(f"deepest corner at sole angle {worst_phi:.2f} rad is the {end} end, "
          f"{-lowest*1000:.1f} mm below floor")
    return best


def trade(face_r=0.055, com_z=0.281):
    """Sole half-length vs. corner overshoot vs. static tilt margin."""
    print(f"\nhalf-len  corner_r  overshoot  polygon  tilt margin")
    for L in (0.040, 0.030, 0.025, 0.020, 0.015, 0.010):
        rc = np.hypot(face_r, L)
        print(f"  {L*1000:4.0f} mm  {rc*1000:5.1f} mm   {(rc-face_r)*1000:5.1f} mm  "
              f"{2*L*1000:5.0f} mm  {np.degrees(np.arctan2(L, com_z)):6.1f} deg")


if __name__ == "__main__":
    for h in (0.207, 0.170, 0.140):
        sweep(h)
        print()
    digging_corner()
    trade()
