"""The STS3215, measured rather than assumed.

    uv run python -m cad.servo

Every servo dimension in this project was previously a number I made up from a
product listing: a 45.2 x 24.7 x 35.4 box with a bolt circle at radius 8. Four
of the five printed parts bolt to a servo, so those numbers set every mounting
face in the robot, and being wrong about them means every part is wrong at the
one place it has to be right.

These come from TheRobotStudio's SO-ARM100 STEP model of the same servo
(vendor/refs/STS3215_03a.step, Apache-2.0), read with OCC rather than typed in.

What changed:

    case height      35.4 -> 39.60 mm   the 4.2 mm is ALONG THE SHAFT, which
                                        is exactly where the horn clearances are
    horn bolts       r = 8.0 at 45 deg  -> a 9.9 x 10.0 mm rectangle, so the
                                        holes were 0.7 mm out of position
    horn screw       M2 (2.2 clear)     -> 2.5 mm clearance
    shaft inset      10.0 (guessed)     -> 10.20 mm from the end. Nearly right.

The frame the STEP arrives in: x along the 45.4 mm length, y across the 24.8 mm
width, z along the OUTPUT SHAFT. Shaft axis at x = +12.5, y = 0. Output end at
z = +20.2, rear idler boss at z = -18.0.
"""

from pathlib import Path

import build123d as bd

STEP = Path(__file__).parents[1] / "vendor" / "refs" / "STS3215_03a.step"

# --- measured off the STEP ---------------------------------------------------
LENGTH = 45.40           # x
WIDTH = 24.80            # y
HEIGHT = 39.60           # z, ALONG the output shaft
Z_MIN, Z_MAX = -19.40, 20.20

SHAFT_X = 12.50          # shaft axis, from the case centre
SHAFT_INSET = LENGTH / 2 - SHAFT_X       # 10.20 mm from the near end
SHAFT_R = 2.95           # 25T spline, 5.9 mm across
HORN_FACE_Z = 18.70      # where the horn seats
IDLER_Z = -18.00         # rear pivot boss
IDLER_R = 3.00

# --- the case mounting holes ---------------------------------------------------
#
# WHAT IS CERTAIN:
#
# Every mounting-scale hole in this servo runs along Z, the OUTPUT SHAFT AXIS.
# There is not a single hole on the +/-12.40 side faces, or on either end. A
# sweep of every cylindrical face under r = 1.6 finds them all on Z and none on
# X or Y.
#
# The printed parts agree with that: cad/fasteners.py checks the axis of every
# M2-scale hole near a servo against that servo's shaft, and all 28 line up.
#
# (A warning for whoever checks this next, because it cost an hour. The SIM
# does not store servo boxes in this file's axis order. It writes the three
# dimensions in whatever order puts the case where it goes, so the shaft is
# along the box's local y for the hip, knee, ankle and wheel servos and along
# local x for the roll servo. Assume local z, as this file uses, and every
# screw in the robot reads as perpendicular to its servo - a complete and very
# convincing false alarm. Derive the shaft by matching the box side against
# HEIGHT; L, W and H are all distinct so it is unambiguous.)
#
# The exact pattern is LESS certain, and is not needed to know the above. The
# r = 0.75 holes sit at y = +/-10.25 and z = +/-15.15, which reads like a
# through-hole along z appearing where it crosses the top and bottom walls, but
# their x values do not pair up between the two walls (-19.55 and 4.95 at the
# bottom, -15.75 and 4.95 at the top). A third-party visual CAD model is not a
# dimensioned drawing, and reverse-engineering fastener detail out of one is
# how this project got the horn wrong in the first place.
#
# So: treat the pattern as UNVERIFIED and get a real servo or the
# manufacturer's drawing. It is the top validation-order item.
CASE_MOUNT_AXIS = "z"            # certain
CASE_MOUNT_VERIFIED = False      # the positions are not
CASE_SCREW_PILOT_R = 0.75        # 1.5 mm, an M2 self-tapper
CASE_FACE_Y = 12.40
# Best reading of the candidates, all on the z axis at y = +/-10.25.
CASE_MOUNTS = [(-19.55, -10.25), (4.95, -10.25),
               (-15.75, 10.25), (4.95, 10.25)]

# --- the horn bolt pattern, NOT measured ---------------------------------------
#
# READ THIS BEFORE TRUSTING THE NUMBERS BELOW.
#
# The shaft position and the horn seating face ARE verified against the STEP:
# the 25T spline sits at x = 12.50 +/- 1.64, y = 0, running z = 18.60..20.20,
# and the face it seats on is the plane at z = 18.70. Those are solid.
#
# The bolt pattern is not, and cannot be, because the horn is a separate bought
# part and this STEP is the servo without it. There is no hole anywhere in this
# model that corresponds to these.
#
# There IS a decoy, and it is a good one. The case has four r = 1.25 screws at
# each end, on a 9.9 x 9.9 mm rectangle - almost exactly the "9.9 x 10.0" this
# project believes the horn pattern to be. They are NOT it: they appear at both
# z ends, which a horn pattern cannot, and they are centred on x = 11.25 rather
# than on the shaft at 12.50. Anyone re-deriving the horn pattern from this file
# will find them and think they have found the horn.
#
# Four of the five printed parts bolt to a horn, so this is the single most
# load-bearing assumption left in the robot. It is a validation-order item:
# measure a real horn before committing the structure. See docs/road-to-order.md.
HORN_DX, HORN_DY = 4.95, 5.00            # ASSUMED, about the shaft
HORN_SCREW_R = 1.25                      # ASSUMED, 2.5 mm clearance
HORN_VERIFIED = False

# The case screws, measured, kept so the decoy above is checkable rather than
# just described.
CASE_END_SCREWS = [(6.30, -4.95), (6.30, 4.95), (16.20, -4.95), (16.20, 4.95)]
CASE_END_Z = (-16.65, 17.45)

MASS_G = 55.0            # from the BOM; the STEP has no material

# The old guesses, kept so the diff is legible from the code.
OLD = dict(length=45.2, width=24.7, height=35.4, horn_r=8.0, screw_r=1.1)


def solid():
    """The real servo, in its own frame. Use this for clearance checks: a box
    is 36.2 cm3 of bounding box round a 36.2 cm3 part, but it is the wrong
    36.2 cm3 near the horn."""
    return bd.import_step(str(STEP))


def envelope():
    """Bounding box, for the cheap checks that cannot afford the real solid."""
    return bd.Pos(0, 0, (Z_MIN + Z_MAX) / 2) * bd.Box(LENGTH, WIDTH, HEIGHT)


def horn_holes(depth=20.0, clearance=HORN_SCREW_R):
    """The four horn screws, as cutting cylinders in the servo's frame."""
    out = []
    for dx in (-HORN_DX, HORN_DX):
        for dy in (-HORN_DY, HORN_DY):
            out.append(bd.Pos(SHAFT_X + dx, dy, HORN_FACE_Z)
                       * bd.Cylinder(clearance, depth))
    return out


def case_mount_holes(r=CASE_SCREW_PILOT_R, length=HEIGHT):
    """The case mounting holes, as cutting cylinders in the servo frame.

    Along Z, the shaft axis, because that is where they are. Positions are the
    best reading and are not verified; the AXIS is.
    """
    return [bd.Pos(x, y, 0) * bd.Cylinder(r, length) for x, y in CASE_MOUNTS]


def main():
    p = solid()
    bb = p.bounding_box()
    print(f"STS3215, from {STEP.name}")
    print(f"  bounding box   {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm")
    print(f"  solid volume   {p.volume/1000:.1f} cm3")
    print(f"  shaft axis     x = {SHAFT_X:.2f}, {SHAFT_INSET:.2f} mm from the end")
    print(f"  output face    z = {HORN_FACE_Z:.2f}, tip at {Z_MAX:.2f}")
    print(f"  horn screws    4 x dia {2*HORN_SCREW_R:.1f} at "
          f"+/-{HORN_DX:.2f} x +/-{HORN_DY:.2f} about the shaft")
    print(f"  rear boss      z = {IDLER_Z:.2f}, dia {2*IDLER_R:.1f}")
    print()
    print("against what this project assumed before:")
    print(f"  height   {OLD['height']:.1f} -> {HEIGHT:.2f} mm  "
          f"({HEIGHT-OLD['height']:+.2f}, and it is along the shaft)")
    print(f"  horn     r={OLD['horn_r']:.1f} at 45 deg -> "
          f"+/-{HORN_DX:.2f} x +/-{HORN_DY:.2f}  "
          f"(holes {abs(OLD['horn_r']*0.7071-HORN_DX):.2f} mm out of position)")
    print(f"  screw    r={OLD['screw_r']:.2f} -> {HORN_SCREW_R:.2f} mm")


if __name__ == "__main__":
    main()
