"""Does each servo actually drive the joint it is meant to?

    uv run python -m cad.drives

Every other check in this repo asks whether parts FIT. This one asks whether
the machine is connected: a servo's output shaft has to lie ON the axis of the
joint it turns, or it is not driving that joint at all.

WHY IT WAS WORTH WRITING. The twin had two servos that were not on their joints
and nothing noticed for eleven weeks:

    hip     10.20 mm off, which is exactly SHAFT_INSET. The servo was placed
            with its END on the hip axis rather than its SHAFT, so the whole
            case sat one inset too high.
    wheel   12.50 mm off, which is exactly SHAFT_X. The box was centred on the
            wheel axis, and the output shaft is not at the centre of the box.

Both are the same mistake in different clothes: positioning a servo by its case
instead of by its output. Neither shows up as interference, neither breaks the
sim - MuJoCo's joints are defined independently of the visual servo boxes, so
the robot walks perfectly well with its motors floating next to the joints they
turn. It is only wrong against REALITY, which is the thing this project is
trying to be right about.

THE ANKLE PITCH USED TO BE THE INTERESTING CASE and it is not here any more.
It was belt driven, because the wheel already owns the space its servo would
need, so the right answer for that one joint was not zero but the belt's centre
distance - and this file asserted it against the belt geometry rather than
switching the check off for that joint, because a check with an exception in it
is a check people stop believing.

That servo is deleted (cad/linkage.py) and the exception went with it. Every
drive in this robot is now direct, which is a simpler claim and a stronger one.
"""

import sys

import mujoco
import numpy as np

import cad.servo as servo

# servo geom -> the joint it turns, and how it turns it.
#   "direct"  the horn bolts to the driven link, so the shaft IS the axis
#
# All of them are direct now. The "belt" mode this used to carry, for the
# ankle-pitch servo's 2:1 drive, went with that servo.
DRIVES = {
    "vhipsv1": ("hip_l", "direct"),
    "vhipsv-1": ("hip_r", "direct"),
    "vkneesv_l": ("knee_l", "direct"),
    "vkneesv_r": ("knee_r", "direct"),
    "vrollsv_l": ("ankle_roll_l", "direct"),
    "vrollsv_r": ("ankle_roll_r", "direct"),
    "vwhlsv_l": ("wheel_l", "direct"),
    "vwhlsv_r": ("wheel_r", "direct"),
}

TOL = 1.0          # mm. A shaft is either on its axis or it is not.
BELT_TOL = 2.0     # the belt centre distance, which is a real dimension


def _shaft_line(m, d, gid):
    """Where a servo's output shaft actually is, in world mm.

    Derived from the box rather than assumed, for the reason cad/servo.py
    spells out: the sim writes each servo's three dimensions in whatever order
    puts the case where it goes, so neither the shaft nor the length is on a
    fixed local axis. L, W and H are all distinct, so matching extents against
    them is unambiguous.

    The shaft sits SHAFT_X off the case centre ALONG THE LENGTH, and which way
    is not recorded anywhere, so both are tried and the nearer one wins. That is
    deliberately generous: this check is looking for servos that are off by an
    inset or a half-case, not for a sign error.
    """
    R = d.geom_xmat[gid].reshape(3, 3)
    p = d.geom_xpos[gid] * 1000.0
    full = m.geom_size[gid] * 2000.0
    k = int(np.argmin(np.abs(full - servo.HEIGHT)))     # along the shaft
    ell = int(np.argmin(np.abs(full - servo.LENGTH)))   # along the length
    return [(p + R[:, ell] * (s * servo.SHAFT_X), R[:, k]) for s in (1, -1)]


def _perp(p, anchor, axis):
    """Distance from a point to the line through `anchor` along `axis`, mm."""
    v = np.asarray(p, float) - np.asarray(anchor, float)
    return float(np.linalg.norm(v - np.dot(v, axis) * axis))


def _posed(mode):
    """The model at `mode`, plus every joint's world anchor and axis."""
    from fitcheck import pose
    from src.rsbot.model import load

    m, d = load()
    pose(m, d, mode)
    mujoco.mj_forward(m, d)
    anchors = {}
    for j in range(m.njnt):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        anchors[n] = (d.xanchor[j] * 1000.0, d.xaxis[j])
    return m, d, anchors


def shaft_lines(mode="wheel", posed=None):
    """{servo geom: (a point on its output shaft, the shaft direction)}, world mm.

    Exported because this is the only correct answer in the repo to "where is
    this servo's output", and it is worth having exactly one. cad/toolaccess.py
    had its own, taken from the servo's CASE centre, which is SHAFT_X = 12.5 mm
    away from the shaft - far enough to throw half of a four-screw horn pattern
    outside its own horn. Import this instead of measuring from a case again.

    `_shaft_line` offers two candidates, because which way SHAFT_X points along
    the case is recorded nowhere in the sim. The JOINT settles it: a servo's
    shaft lies on the axis it turns, or a belt's centre distance from it, and
    the two candidates are 25 mm apart, so the nearer one wins outright.
    """
    m, d, anchors = posed or _posed(mode)
    out = {}
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if n not in DRIVES or DRIVES[n][0] not in anchors:
            continue
        anchor, axis = anchors[DRIVES[n][0]]
        out[n] = min(_shaft_line(m, d, i),
                     key=lambda ca: _perp(ca[0], anchor, axis))
    return out


def check(mode="wheel", verbose=True):
    """[(servo, joint, kind, offset mm, ok)] for every driven joint."""
    posed = _posed(mode)
    m, d, anchors = posed
    lines = shaft_lines(mode, posed)

    rows = []
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if n not in DRIVES:
            continue
        joint, kind = DRIVES[n]
        if joint not in anchors:
            rows.append((n, joint, kind, float("nan"), False))
            continue
        anchor, axis = anchors[joint]
        best = _perp(lines[n][0], anchor, axis)
        if kind == "direct":
            ok = best < TOL
        else:
            from cad.belt import C
            ok = abs(best - C) < BELT_TOL
        rows.append((n, joint, kind, best, ok))

    if verbose:
        print(f"--- {mode} mode: is every servo on the joint it drives?")
        for n, joint, kind, off, ok in rows:
            want = "0" if kind == "direct" else f"{_belt_c():.2f} (belt)"
            print(f"   {n:12s} -> {joint:16s} {kind:7s} "
                  f"offset {off:7.2f} mm   want {want:14s} "
                  f"{'ok' if ok else 'OFF AXIS'}")
        bad = [r for r in rows if not r[4]]
        print(f"   {len(bad)} servo(s) not driving their joint"
              if bad else "   every servo is on its joint")
    return rows


def _belt_c():
    from cad.belt import C
    return C


def main():
    bad = 0
    for mode in ("wheel", "foot"):
        bad += sum(1 for r in check(mode) if not r[4])
        print()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
