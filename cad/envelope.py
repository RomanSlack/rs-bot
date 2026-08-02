"""The volumes a new part is not allowed to occupy.

    uv run python -m cad.envelope          # where the ankle-pitch bearing can go

Everything in this robot that has gone wrong geometrically went wrong the same
way: someone reasoned about one clearance, satisfied it, and walked into
another. The shin's ankle-pitch bore is 53 mm off its own axis. The roll
bracket's four holes cut air. Its two members lapped by 0.5 mm because they
were dodging a post that is itself in the wrong place.

So the constraints are written down as SOLIDS rather than as prose, and a
candidate part is intersected against them. A rule you can only apply by hand
is a rule you will forget.

Frames: `shin` has its origin at the knee with the ankle axis at z = -110;
`ankle` has its origin at the axle. Millimetres, left leg. The right leg
mirrors in y.

This is a DESIGN AID and it is deliberately conservative: roll and pitch are
swept independently, so the swept volumes include combinations the robot never
actually reaches (the two are correlated during a flip). A part clear here is
certainly clear; a part with a small overlap here may still be fine on the real
trajectory, and `fitcheck.py` and `cad/assemble_check.py` are what settle that,
because they sweep the manoeuvre the robot actually performs.

The two wheel shapes are the whole difficulty. Upright it is a 24 mm rim swept
through 80 mm vertically; flat it is an 80 mm platter swept horizontally. A
part that does not rotate with the wheel has to miss BOTH, and they overlap
almost nowhere, so the free space is much smaller than either suggests.
"""

import build123d as bd
import numpy as np

WHEEL_R = 40.0
WHEEL_HALF_W = 12.0
ANKLE_Z = -110.0          # ankle axis in the shin frame

# Ankle pitch actually used, plus margin. Everything below the pitch joint -
# wheel, wheel servo, roll servo - swings in the SHIN's frame as this changes,
# so a shin part has to clear the swept volume, not the neutral pose. The
# measured range through a full flip is +19.5 .. +29.4 deg.
PITCH_RANGE = (-20.0, 45.0)

# Bought-part envelopes, from src/rsbot/model.py. (x0,x1),(y0,y1),(z0,z1) in
# the ANKLE frame unless noted.
# WAS 45.2, 24.7, 35.4 - the original product-listing guesses, superseded twice
# and never updated here. This file is what solved which bands the ankle-pitch
# bearing is allowed to sit in, against a servo 1.1 mm short in its shaft axis.
from cad.servo_dims import LENGTH as SERVO_L, WIDTH as SERVO_W, HEIGHT as SERVO_H
HL, HW, HH = SERVO_L / 2, SERVO_W / 2, SERVO_H / 2

# Wheel-drive servo: outboard, bolted to the roll bracket, so it turns with
# the wheel in ROLL but shares the ankle's pitch.
WHEEL_SERVO = ((-HL, HL), (14.0, 14.0 + 2 * HH), (-HW, HW))
# Roll servo: on the yoke, aft and lifted.
ROLL_SERVO = ((-(58.0 + 2 * HH), -58.0), (-HW, HW), (16.0 - HL, 16.0 + HL))
# Ankle-pitch servo: on the SHIN, high and outboard. Given in the shin frame.
ANKLE_SERVO_SHIN = ((-HW, HW), (34.0, 34.0 + 2 * HH), (-62.6, -17.4))


def _box(spec):
    (x0, x1), (y0, y1), (z0, z1) = spec
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def wheel_upright(frame="ankle"):
    """The disc spinning about y: radius 40, half-width 12."""
    z = 0.0 if frame == "ankle" else ANKLE_Z
    return bd.Pos(0, 0, z) * bd.Rot(90, 0, 0) * bd.Cylinder(WHEEL_R, 2 * WHEEL_HALF_W)


def wheel_flat(frame="ankle"):
    """The same disc laid over: an 80 mm platter, axis vertical."""
    z = 0.0 if frame == "ankle" else ANKLE_Z
    return bd.Pos(0, 0, z) * bd.Cylinder(WHEEL_R, 2 * WHEEL_HALF_W)


def wheel_swept(frame="ankle", steps=9):
    """Both shapes and everything between. The flip passes through every roll
    angle, so a part that clears the two END poses can still be hit halfway -
    which is exactly how the wheel-drive servo came to sweep 20.7 mm through
    the shin at 64% of the manoeuvre."""
    z = 0.0 if frame == "ankle" else ANKLE_Z
    out = None
    for a in np.linspace(0, 90, steps):
        w = (bd.Pos(0, 0, z) * bd.Rot(float(a), 0, 0) * bd.Rot(90, 0, 0)
             * bd.Cylinder(WHEEL_R, 2 * WHEEL_HALF_W))
        out = w if out is None else out + w
    return out


def _swept_roll(solid, steps=9):
    """A solid that turns with the roll bracket, swept through the flip."""
    out = None
    for a in np.linspace(0, 90, steps):
        w = bd.Rot(float(a), 0, 0) * solid
        out = w if out is None else out + w
    return out


def obstacles(frame="ankle", swept=True, rolls_with_wheel=False):
    """{name: solid} everything a new part must miss, in `frame`.

    `rolls_with_wheel` is for the roll bracket, which is the wheel's PARENT.
    The wheel only spins about y relative to it, so in its frame the wheel is
    always the upright disc - checking it against the flat shape as well is
    checking it against a pose it can never be in, and reports a collision
    that cannot happen.
    """
    z = 0.0 if frame == "ankle" else ANKLE_Z
    shift = bd.Pos(0, 0, z)
    if rolls_with_wheel:
        wheel = wheel_upright(frame)
    else:
        wheel = wheel_swept(frame) if swept else (wheel_upright(frame)
                                                 + wheel_flat(frame))
    out = {"wheel": wheel, "roll servo": shift * _box(ROLL_SERVO)}
    if not rolls_with_wheel:
        # The wheel-drive servo bolts to the roll bracket, so it is not an
        # obstacle to the bracket - it is part of it. To everything else it is
        # an obstacle that MOVES: it turns with the bracket through the flip,
        # and its far corner swings out to sqrt(49.4^2 + 12.35^2) = 50.9 mm,
        # 1.5 mm further than its static outboard face. Treating it as a fixed
        # box put the ankle-pitch boss at y = 50 and fitcheck found it being
        # clipped 0.9 mm at 16% of the manoeuvre.
        out["wheel servo"] = shift * _swept_roll(_box(WHEEL_SERVO))
    if frame == "shin":
        # Everything below the pitch joint swings in the shin's frame as the
        # ankle pitches, so sweep it. Only roll was being swept, and fitcheck
        # found the shin clipping the wheel servo by 2.4 mm at 64% of the flip
        # - the servo moves in x with PITCH, which rotation about x cannot show.
        out = {k: _swept_pitch(v, z) for k, v in out.items()}
        out["ankle servo"] = _box(ANKLE_SERVO_SHIN)   # on the shin, fixed
    return out


def _swept_pitch(solid, z, steps=7):
    """Swept through the ankle-pitch range, about the axis at z."""
    lo, hi = PITCH_RANGE
    out = None
    for a in np.linspace(lo, hi, steps):
        w = bd.Pos(0, 0, z) * bd.Rot(0, float(a), 0) * bd.Pos(0, 0, -z) * solid
        out = w if out is None else out + w
    return out


def check(part, frame="ankle", name="part", swept=True, quiet=False,
          rolls_with_wheel=False):
    """Intersect `part` against every obstacle. Returns {obstacle: mm3}."""
    hits = {}
    for label, solid in obstacles(frame, swept, rolls_with_wheel).items():
        v = (part & solid).volume
        if v > 1.0:
            hits[label] = v
    if not quiet:
        if hits:
            for label, v in sorted(hits.items(), key=lambda kv: -kv[1]):
                print(f"  {name:<22} x {label:<14} {v:9.1f} mm3")
        else:
            print(f"  {name:<22} clear")
    return hits


def pitch_axis_free_y(step=1.0, half=6.0):
    """Where on the ankle-pitch axis can a bearing actually sit?

    The axis is the line x = 0, z = ANKLE_Z, varying y. A bearing there is a
    short cylinder ON that line, so this walks y and asks what it hits.
    """
    rows = []
    for y in np.arange(-80.0, 80.0 + step, step):
        probe = (bd.Pos(0, y, 0) * bd.Rot(90, 0, 0)
                 * bd.Cylinder(5.0, 2 * half))
        hits = check(probe, frame="ankle", quiet=True)
        rows.append((float(y), hits))
    return rows


def _bands(rows):
    """Contiguous runs of clear y."""
    out, start = [], None
    for y, hits in rows:
        if not hits and start is None:
            start = y
        elif hits and start is not None:
            out.append((start, y))
            start = None
    if start is not None:
        out.append((start, rows[-1][0]))
    return out


if __name__ == "__main__":
    print("Ankle-pitch bearing: which y on the axis is free?\n")
    rows = pitch_axis_free_y()
    print(f"{'y mm':>7}  what it is inside")
    shown = None
    for y, hits in rows:
        key = tuple(sorted(hits)) or ("clear",)
        if key != shown:
            print(f"{y:>7.0f}  {', '.join(key)}")
            shown = key
    print("\nusable bands (a 10 mm bearing, on the axis):")
    for a, b in _bands(rows):
        print(f"   y = {a:+.0f} .. {b:+.0f} mm    ({b - a:.0f} mm wide)")
    print("\nThe leg already reaches y = 69 at the ankle servo, so an outboard")
    print("band costs no extra width. Inboard is limited by the other leg:")
    print("the legs are 120 mm apart, so local y = -60 is the centreline.")
