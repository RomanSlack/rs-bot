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

# --- MEASURED ON THE BENCH, 2026-08-01 -----------------------------------------
#
# Two C018 servos arrived and everything below was taken off the physical part
# with calipers. Up to here the file's "measured" meant measured off somebody
# else's STEP; from here it means measured off the servo.
#
# THE TWO LATERAL DIMENSIONS ARE CONFIRMED, and confirmed well:
#
#     length   45.40 modelled   45.35 measured   0.05 mm
#     width    24.80 modelled   24.77 measured   0.03 mm
#
# That level of agreement is what makes the third one worth taking seriously,
# because it rules out both a bad caliper and a misread datum.
#
#     shaft tip to shaft tip   39.60 modelled   37.24 measured   2.36 mm
#
# THE STEP IS PROBABLY THE WRONG VARIANT. vendor/refs/STS3215_03a.step comes
# from TheRobotStudio's SO-ARM100, and SO-ARM100/101 runs the 7.4 V servo. This
# project bought C018, the 12 V 1:345. Feetech sells at least five visually
# identical, non-interchangeable variants (docs/order-sheet.md), a different
# ratio means a different gear train, and the gear train is precisely what lives
# along the shaft axis. The two directions that agree are the two that do not
# contain a gearbox, which is what you would expect if this is a variant
# difference rather than a modelling error.
#
# WHICH END LOST THE 2.36 IS NOT YET KNOWN, and it is the whole question. If it
# came off the rear boss it costs nothing. If it came off the output end then
# HORN_FACE_Z moves, and HORN_FACE_Z is where every horn-mounted part in the
# robot sits. A rough reading of the spline standing about 1.0 mm proud of the
# boss face against a modelled 1.50 puts roughly 0.5 mm at the output end and
# the rest at the rear, which would be the good outcome, but the operator
# flagged that reading as shaky and it is a lead rather than a result.
#
# Settle it with two numbers: the diameter of the round boss the horn seats on
# (modelled 20.00) and the spline height above that boss face (modelled 1.50).
MEASURED_LENGTH = 45.35
MEASURED_WIDTH = 24.77
MEASURED_Z_SPAN = 37.24          # shaft tip to shaft tip, the full span
Z_SPAN_UNEXPLAINED = 2.36        # modelled minus measured, end unknown
CASE_Z_VERIFIED = False          # and this is why

# THE CASE MOUNTING HOLES: the consensus argument is confirmed, and retired.
#
# The C018 kit ships three bracket types and every one of them attaches through
# the HORN. Not one bolts to the case. That is the prediction above, made from
# TheRobotStudio's cradle and now checked against the manufacturer's own
# brackets, so CASE_MOUNT_CONSENSUS is not a guess any more.
#
# It also stops mattering. Consensus between mating parts was the instrument
# only because nobody had the servo. The servo is now on the desk, so the
# pattern gets measured directly and the question changes from "can this be
# recovered" to "pin the holes and read them". CASE_MOUNT_VERIFIED stays False
# until that is done, and 40 M2 screws still depend on it.
CASE_MOUNT_NO_BRACKET_BOLTS_TO_CASE = True     # checked, C018 kit, 3 bracket types

# THE HORN, and this is the one that changes a part.
#
# Everything in the kit is metal, the horn included, so the plan to drop from 48
# heat-set inserts to 16 survives: the M3 thread is bought, not printed.
#
# The screws measure 2.93 across the thread, which is an M3 major diameter. That
# is now the third independent agreement on M3 (six mating patterns, the
# reseller drawing, and the physical screw), so the M2.5 to M3 rework was right.
#
#     outer diameter    19.93        against a Ø20.00 pocket in cad/wheel.py
#     plate thickness    2.51
#     overall            4.50        plate plus the hub nub
#     nub                1.99        raised, and it faces the CROWN
#
# THE 4.5 / 3.6 / 2.5 SECTION ON THE RESELLER DRAWING IS NOW DECODED. 2.5 is the
# plate and 4.5 is the overall height including the nub. The fault this project
# was braced for was the other reading: a 4.5 mm seating flange against a 4.0 mm
# pocket, standing the horn 0.5 mm proud so the wheel could not seat flat. That
# fault does not exist. The nub points at the servo, the wheel lands on a flat
# 2.51 mm plate, and the joint seats.
#
# THE FAULT THAT DOES EXIST IS THE DIAMETER, and it is real. cad/wheel.py cut a
# Ø20.00 pocket for a Ø19.93 horn: 0.035 mm per side, from a print service
# quoting +/-0.3 mm. Most wheels would not have accepted the horn at all. Fixed
# there, not here, and the fix has to clear the servo's own Ø20 boss as well.
HORN_OD = 19.93                  # measured
HORN_PLATE_T = 2.51              # measured, the part that seats
HORN_OVERALL_T = 4.50            # measured, including the nub
HORN_NUB_T = HORN_OVERALL_T - HORN_PLATE_T
HORN_METAL = True                # so the M3 thread is bought
HORN_SCREW_MEASURED = 2.93       # major diameter, confirms M3
HORN_DIMS_VERIFIED = True        # off the part, unlike the pattern below
# Still not measured off the part: the bolt-circle spacing (modelled 4.95 x
# 4.95) and the nub diameter. The spacing has six independent patterns behind it
# and a drawing, so it is the best-supported number in the file; the nub
# diameter has nothing behind it and is a placeholder used only for drawing.
HORN_NUB_D_ASSUMED = 10.0

# --- THE MANUFACTURER'S DRAWING, 2026-08-01 ------------------------------------
#
# It exists. It was on the Amazon listing the whole time, which is the second
# time in a week this project has proved at length that a document could not be
# recovered and then found it on a shopping page. Look for the drawing first.
#
# It is the 12 V part (the servo in the same listing photo is labelled
# STS3215-12V), so unlike vendor/refs/STS3215_03a.step it is OUR variant.
#
# WHAT IT DIMENSIONS, and these supersede everything above them:
DWG_LENGTH = 45.23               # model carries 45.40, caliper read 45.35
DWG_WIDTH = 24.73                # model carries 24.80, caliper read 24.77
DWG_SHAFT_X = 12.50              # from the case centre. Model exact.
DWG_SPLINE_PROUD = 3.40          # spline above the case top face
DWG_BODY = 29.00                 # case body, top face to bottom face
DWG_REAR_BOSS_PROUD = 4.10       # rear boss below the bottom face
DWG_Z_SPAN = 36.50               # tip to tip, and 3.4 + 29 + 4.1 = 36.5 exactly
DWG_IDLER_D = 6.00               # model has IDLER_R = 3.00. Exact.
DWG_SPLINE = "25T"               # model has SHAFT_R = 2.95. Exact.
DWG_HORN_CENTRE_SCREW = "M3x6"   # the screw that appears nowhere in the CAD
DWG_CASE_SCREWS = 8              # "8-PA2.0". The model lists four.
DWG_CASE_SCREW = "PA2.0"         # a 2.0 mm self-tapper, so M2 was the right call
DWG_REAR_SCREWS = ("PA3.0x5", 2)  # a second fastener size the model does not have
DWG_CONNECTOR = "5264 / 2.54 mm / 3P"   # check against cad/wiring.py
#
# THE SHAFT-AXIS SPAN IS 36.50, NOT 39.60. The model is 3.1 mm too tall, which
# is worse than the 2.36 mm the caliper suggested, and it is now the
# manufacturer saying so rather than one shaky reading. The 3.4 / 29 / 4.1
# breakdown adds up exactly, so this is not a bounding box being compared
# against a case dimension: it is the real stack.
#
# LENGTH, WIDTH AND HEIGHT ARE DELIBERATELY NOT CHANGED YET, and that is a
# judgement call worth defending. They feed cad/chassis.py, cad/thigh.py, the
# axis check in cad/fasteners.py and the port positions in cad/wiring.py, so
# correcting them moves real geometry on four parts and wants the FEA and the
# full suite re-run against that change ALONE. Folding it into the same commit
# as the servo-capture work would make it impossible to tell which change broke
# what. Separate job, and the numbers to use are right here.
#
# The other reason to wait: solid() returns the STEP, which is the wrong variant
# and is OVERSIZED along z. Every clearance check that runs against it is
# therefore conservative, which is the safe direction to be wrong in. Shrinking
# the envelope to the drawing while the solid stays big would break that.
DWG_SUPERSEDES_STEP = True
CASE_Z_VERIFIED = False          # still, and now for a documented reason

# AND THE POSITIONS ARE STILL NOT PUBLISHED. The drawing labels the case holes
# "8-PA2.0" with a leader line and never dimensions where they are. Feetech's
# own metal brackets, all three types, attach through the HORN and none of them
# touches the case. So the manufacturer did not dimension the holes for exactly
# the reason this file predicted: nobody mates with them, so nobody was ever
# forced to say where they are. Three witnesses now agree - TheRobotStudio's
# cradle, Feetech's brackets, and Feetech's own drawing declining to answer.
#
# WHICH IS WHY THE ROBOT STOPS USING THEM. See the four parts: chassis, thigh,
# shin and ankle each invented a DIFFERENT pattern (17.0, 5 mm from the ends,
# 40.0 spacing, and +/-10.2), so it was never one unverified number, it was four
# mutually inconsistent ones drilled into the same part. Capture replaces them.
CASE_MOUNT_ABANDONED = True

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
    print()
    print("against the MANUFACTURER'S DRAWING (12 V part, so our variant):")
    print(f"  length   {LENGTH:.2f} model   {DWG_LENGTH:.2f} drawing"
          f"   {LENGTH - DWG_LENGTH:+.2f}")
    print(f"  width    {WIDTH:.2f} model   {DWG_WIDTH:.2f} drawing"
          f"   {WIDTH - DWG_WIDTH:+.2f}")
    print(f"  z span   {HEIGHT:.2f} model   {DWG_Z_SPAN:.2f} drawing"
          f"   {HEIGHT - DWG_Z_SPAN:+.2f}   <- the model is too tall")
    print(f"    and it breaks down exactly: {DWG_SPLINE_PROUD:.1f} spline"
          f" + {DWG_BODY:.1f} body + {DWG_REAR_BOSS_PROUD:.1f} rear boss"
          f" = {DWG_SPLINE_PROUD + DWG_BODY + DWG_REAR_BOSS_PROUD:.1f}")
    print()
    print(f"  case screws   {DWG_CASE_SCREWS} x {DWG_CASE_SCREW}, positions NOT"
          " dimensioned anywhere")
    print(f"  horn screw    {DWG_HORN_CENTRE_SCREW}, absent from the CAD")
    print(f"  rear screws   {DWG_REAR_SCREWS[1]} x {DWG_REAR_SCREWS[0]}")
    print(f"  connector     {DWG_CONNECTOR}")
    print(f"  case mounting abandoned: {CASE_MOUNT_ABANDONED}")


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


# --- the servo as a solid, drawn from the DRAWING ------------------------------
#
# solid() returns vendor/refs/STS3215_03a.step, which is somebody else's model
# of the 7.4 V part and is 3.1 mm too tall. This is ours, built from the
# dimensions the manufacturer publishes, and it is what the live sim shows.
#
# Visual only, deliberately. The sim's servo boxes stay exactly as they are:
# fitcheck measures interference from the oriented bounding boxes of the group-0
# geoms, and a mesh has no meaningful geom_size, so swapping the collision shape
# would silently start auditing something else. This changes what you see and
# nothing about what the robot does.
#
# WHAT IS NOT ON IT: the eight PA2.0 case holes. They are real, the drawing says
# so, and nobody publishes where. Drawing them would mean inventing a fifth
# pattern to sit alongside the four this project just deleted.

S_BOT = -DWG_Z_SPAN / 2                       # rear boss tip
S_BODY0 = S_BOT + DWG_REAR_BOSS_PROUD         # body starts
S_BODY1 = S_BODY0 + DWG_BODY                  # body ends
S_TIP = S_BODY1 + DWG_SPLINE_PROUD            # spline tip
PAD_D, PAD_H = 11.0, 1.5      # the pad the horn seats on; scaled off the
                              # drawing, not dimensioned on it
CONN_L, CONN_W, CONN_H = 8.0, 4.0, 2.2        # the two 5264 3P housings

_AXIS = {"L": 0, "W": 1, "S": 2}


def _place(order, v):
    """Reorder a canonical (length, width, shaft) triple onto the target axes."""
    p = [_AXIS[c] for c in order]
    return (v[p[0]], v[p[1]], v[p[2]])


def drawing_solid(order=("L", "W", "S")):
    """The STS3215 C018, with its dimensions on the requested axes.

    `order` names which canonical dimension lands on x, y and z: "L" length,
    "W" width, "S" the output shaft. The sim writes each servo's box in
    whatever order puts the case where it goes, so the mesh has to be built to
    match rather than rotated afterwards and hoped for. Baking the orientation
    in here means no euler in the MJCF and nothing to get backwards.
    """
    def box(size, pos):
        return bd.Pos(*_place(order, pos)) * bd.Box(*_place(order, size))

    def cyl(r, s0, s1, at_l=DWG_SHAFT_X):
        h = s1 - s0
        axis = order.index("S")
        rot = {0: bd.Rot(0, 90, 0), 1: bd.Rot(-90, 0, 0), 2: bd.Rot(0, 0, 0)}[axis]
        return bd.Pos(*_place(order, (at_l, 0.0, (s0 + s1) / 2))) * rot \
            * bd.Cylinder(r, h)

    p = box((DWG_LENGTH, DWG_WIDTH, DWG_BODY),
            (0.0, 0.0, (S_BODY0 + S_BODY1) / 2))
    p += cyl(PAD_D / 2, S_BODY1, S_BODY1 + PAD_H)      # horn seating pad
    p += cyl(SHAFT_R, S_BODY1, S_TIP)                  # 25T spline
    p += cyl(DWG_IDLER_D / 2, S_BOT, S_BODY0)          # rear pivot boss
    for w in (-5.2, 5.2):                              # the two bus connectors
        p += box((CONN_L, CONN_W, CONN_H),
                 (-0.55, w, S_BODY0 - CONN_H / 2))
    return p.clean()


# --- getting it into the sim, and the trap that cost an hour -------------------
#
# THE OBVIOUS APPROACH DOES NOT WORK. The sim writes each servo's box in
# whatever order puts the case where it goes - (W, S, L) for the hip, knee and
# ankle, (S, W, L) for the roll, (L, S, W) for the wheel - so the tempting fix
# is to export three STLs, one per order, and use no euler at all. Orientation
# baked into the geometry, nothing to get backwards.
#
# MuJoCo throws it away. The compiler canonicalises every mesh: vertices are
# translated to the centre of mass and rotated onto the principal axes of
# inertia, with the authored frame recovered afterwards through mesh_quat. Three
# STLs of the SAME SOLID therefore canonicalise identically, and all three
# geoms come out in one orientation. The check said so: all three reported
# extents of 24.73 x 36.52 x 45.35, whichever file they pointed at.
#
# So there is one mesh and the orientation lives in the MJCF, as euler.
#
# AND CHECKING IT HAS ITS OWN TRAP. geom_aabb is expressed in the GEOM's own
# frame, and euler rotates the geom relative to its BODY, so geom_aabb does not
# move when you change euler. A search over all 64 axis-aligned eulers found
# nothing, because every candidate reported identical numbers. Compare in the
# body frame - abs(geom_xmat) @ half - and the answers fall straight out.
# RADIANS, not degrees, and that is the third trap in ten lines. MuJoCo's MJCF
# defaults to degrees; this project's model does not - src/rsbot/model.py writes
# euler="1.5708 0 0" for the wheel cylinders, so it is running in radians. A
# search done under the default found "90 0 90", which fed to the real model is
# ninety RADIANS and lands the servo at an arbitrary angle. It does not error, it
# does not warn, it just quietly puts a bounding box 62 mm across where a 45 mm
# one belongs.
MESH_EULER = {"wsl": "1.5708 0 1.5708",   # hip, knee, ankle
              "swl": "0 1.5708 0",        # roll
              "lsw": "1.5708 0 0"}        # wheel


def export_mesh(out=None):
    """One STL, in this file's own frame: x length, y width, z shaft."""
    out = Path(out) if out else Path(__file__).parent / "out"
    out.mkdir(parents=True, exist_ok=True)
    f = out / "servo.stl"
    bd.export_stl(drawing_solid(("L", "W", "S")), str(f),
                  tolerance=0.02, angular_tolerance=0.1)
    return f
