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
# WHY THE HORN TRICK DOES NOT WORK HERE, which is the interesting part.
#
# The horn pattern was recovered by measuring parts that BOLT TO the horn: a
# mating dimension has to be encoded exactly by everyone who mates with it, so
# three independent parts agreeing to 0.00 mm is as good as a drawing.
#
# The case pattern has no such consensus, because in the reference design
# NOTHING BOLTS TO IT. TheRobotStudio's Base_motor_holder_SO101 is a cradle,
# 47.6 x 31.4 mm around a 45.4 x 24.8 mm servo, with a cavity the servo drops
# into: a servo-sized box can be placed inside it overlapping by ~300 mm3. Its
# own 4-hole patterns (+/-14.64 x +/-9.71 and +/-13.65 x +/-9.71, M2 clearance)
# match nothing on the servo - they are how the HOLDER bolts to the arm.
#
# So the case holes are not a mating dimension anywhere, nobody had to get them
# right, and there is nothing to cross-check against. That is also why Feetech
# not publishing a dimensioned drawing has never bitten anyone: the people
# using these servos in volume do not use those holes.
#
# WHICH IS A QUESTION FOR THIS ROBOT. rs-bot puts 40 M2 screws into case holes
# it cannot confirm exist, where the best-known reference design captures the
# servo in a cradle instead. Capturing needs no holes and no thread in a
# 55 g plastic case. See docs/road-to-order.md.
CASE_MOUNT_CONSENSUS = False     # no other design bolts here to compare with
CASE_MOUNT_AXIS = "z"            # certain
CASE_MOUNT_VERIFIED = False      # the positions are not
CASE_SCREW_PILOT_R = 0.75        # 1.5 mm, an M2 self-tapper
CASE_FACE_Y = 12.40
# Best reading of the candidates, all on the z axis at y = +/-10.25.
CASE_MOUNTS = [(-19.55, -10.25), (4.95, -10.25),
               (-15.75, 10.25), (4.95, 10.25)]

# --- the horn bolt pattern, MEASURED -------------------------------------------
#
# The horn is a separate bought part and is NOT in the servo STEP - that file
# holds one solid, the bare case. So the pattern cannot come from there, and
# for a long time it was simply assumed.
#
# It is now measured, from the other end: TheRobotStudio's own SO-ARM101 parts,
# which bolt to this exact horn on this exact servo. A part that mates with the
# horn carries the horn's pattern as its own clearance holes.
#
#     Rotation_Pitch_SO101   two patterns   r = 1.60   +/-4.95 x +/-4.95
#     Wrist_Roll_Pitch_SO101 two patterns   r = 1.60   +/-4.95 x +/-4.95
#     Upper_arm_SO101        two patterns   r = 1.50   +/-4.95 x +/-4.95
#
# Six independent patterns across three parts, all agreeing on position to
# 0.00 mm. Rotation_Pitch is vendored next to the servo (Apache-2.0, same
# source) so this is re-derivable rather than a claim.
#
# WHAT THAT CHANGED, and it is not the position:
#
#     spacing   4.95 x 5.00  ->  4.95 x 4.95    0.05 mm, immaterial
#     hole      r = 1.25     ->  r = 1.60       0.70 mm on diameter, and it is
#                                               the difference between M2.5 and
#                                               M3. Every horn hole in this
#                                               robot was too small for its
#                                               screw.
#
# A 3.2 mm hole in a printed part is an M3 CLEARANCE hole, so the screw passes
# through the plastic and threads into the horn, which is metal and tapped.
# That means no heat-set insert at a horn joint - the thread is bought.
#
# THE DECOY IS STILL THERE, and it is why this was believable for so long. The
# case carries four r = 1.25 screws at EACH end on a 9.9 x 9.9 rectangle, which
# is the right size and the wrong everything else: it appears at both z ends,
# which a horn pattern cannot, and it is centred on x = 11.25 rather than on
# the shaft at 12.50. The assumed values were almost exactly these.
HORN_DX, HORN_DY = 4.95, 4.95            # measured, about the shaft
HORN_SCREW_R = 1.60                      # 3.2 mm, M3 clearance
HORN_SCREW = "M3"
HORN_VERIFIED = True
HORN_REF = "vendor/refs/Rotation_Pitch_SO101.step"

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


# --- capturing a servo instead of only bolting to it ---------------------------

CRADLE_WALL = 2.0        # rim thickness; there is 3-4 mm free at every mount
CRADLE_CLEAR = 0.4       # per side, and it must EXCEED the print tolerance
CRADLE_DEPTH = 6.0       # how far the rim reaches along the shaft


def cradle(centre, depth=CRADLE_DEPTH, wall=CRADLE_WALL, clear=CRADLE_CLEAR,
           sign=1.0):
    """A rim that grips the servo case, in a part frame with the SHAFT ALONG Y.

    A servo's reaction is a couple about its shaft, and up to now this robot
    fed all of it into four M2 self-tapping screws in a 55 g bought plastic
    case. The ankle's are 12 mm apart, which makes its 1.63 N.m into 136 N per
    screw. A rim round the case takes the same couple in BEARING across the
    full 45.4 x 24.8 face instead, which is both a far better joint and the way
    TheRobotStudio mount this exact servo - their holder is a cradle and does
    not use the case holes at all.

    NOT WIRED IN, AND HERE IS WHY - this is the useful part.

    The intent was an additive change: drop a rim round each servo, keep the
    bolt holes, let the screws become retention. It does not work on this
    design. Tried on the shin's ankle servo at both ends and at depths of 6, 10
    and 16 mm, the rim touches the shin in NO configuration - it fuses as a
    second, floating solid every time.

    The reason is structural, not a bug. Every servo mount in this robot is a
    PLATE on the case's end face, perpendicular to the shaft, with no material
    anywhere around the case's perimeter. There is nothing for a rim to grow
    from. Capturing a servo here is not a feature you add, it is a change to
    how the part meets the servo, on four parts, with the packaging re-checked
    each time.

    So it is scoped rather than half-done. The helper stays because the
    geometry is right and the measurement behind it is right: there IS room,
    3-4 mm free at the hip, knee and ankle mounts, and the roll and wheel
    servos are already enclosed by structure on three or four faces.

    `centre` is the servo box centre in the part's frame; `sign` picks which
    way along y the rim reaches from that centre's near face.
    """
    cx, cy, cz = centre
    y0 = cy + sign * HEIGHT / 2.0
    ymid = y0 - sign * depth / 2.0
    outer = bd.Pos(cx, ymid, cz) * bd.Box(WIDTH + 2 * wall, depth,
                                          LENGTH + 2 * wall)
    inner = bd.Pos(cx, ymid, cz) * bd.Box(WIDTH + 2 * clear, depth + 2.0,
                                          LENGTH + 2 * clear)
    return outer - inner
