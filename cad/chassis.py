"""The chassis.  uv run python cad/chassis.py

Origin at the hip axis, z up, millimetres. Two side plates carrying the hip
servos, tied by a top plate and two shelves that also mount the Pi and the
pack. Open front and back, as the sim has it.

Structurally the easy part: 11x margin in bending on 3 mm plates. What it does
have to get right is the hip mounting, because both hip servo cases bolt
straight to these plates and everything the robot weighs passes through them.
"""

import sys
from pathlib import Path

import build123d as bd

OUT = Path(__file__).parent / "out"
from cad.material import DENSITY as PRINT_DENSITY, INFILL  # PA6-CF, one source
M25_CLEAR = 1.35
from cad.fasteners import M25_INSERT_R, M25_INSERT_DEPTH  # insert bore, one source

X0, X1 = -36.5, 53.5          # 90 mm deep, centred on the sim's 8.5 offset
SIDE_Y, SIDE_T = 38.1, 3.0    # plate centre-line and thickness
INNER_Y = SIDE_Y - SIDE_T / 2
TOP_Z, SHELF1_Z, SHELF2_Z = 178.5, 20.0, 61.0
PLATE_T = 3.0

# THE HIP SERVO DOES NOT LIVE INSIDE THE CHASSIS, whatever this comment used to
# say. Measured off the sim: the case runs y = 35.4..75.0 and the side plate is
# at 36.6..39.6, so it passes THROUGH the plate's notch and 35 mm of it - most
# of the servo - sits outboard. Its output face is at y = 75.0, which is exactly
# where cad/thigh.py puts the inner face of the hip hub, so the drive itself was
# always right; only the description was wrong.
#
# That matters because the capture rim was built inboard on the strength of this
# sentence, and it gripped 1.2 mm of case. The rim belongs OUTBOARD.
from cad.servo import LENGTH as SERVO_L, WIDTH as SERVO_W, SHAFT_INSET

HIP_Z = 0.0                   # hip axis

# WHERE THE HIP SERVO ACTUALLY IS, which is not where this file used to assume.
#
# The servo's output shaft sits SHAFT_INSET from the near end of its case, so a
# servo whose horn is ON the hip axis runs from -10.2 to +35.2, not from 0 to
# 45.4. The sim had it at 0..45.4 - positioned by its case rather than by its
# output - which put the shaft 10.2 mm above the joint it drives. cad/drives.py
# now checks exactly that and the sim is fixed; this follows it.
#
# THE CONSEQUENCE IS REAL AND IS NOT RESOLVED HERE: 10.2 mm of servo now hangs
# below z = 0, and the side plates start at z = 0. Either the chassis grows
# downward to enclose it or the servo is exposed under the robot. That is a
# proportions decision, not bookkeeping, so it is flagged rather than taken.
HIP_SERVO_Z = (HIP_Z - SHAFT_INSET, HIP_Z - SHAFT_INSET + SERVO_L)

HIP_NOTCH_X = SERVO_W / 2 + 0.8
HIP_NOTCH_Z = (HIP_SERVO_Z[0] - 1.0, HIP_SERVO_Z[1] + 1.0)

# Capturing the hip servo instead of bolting into its case. The four M2 that
# used to go here were at +/-17.0, one of four different invented patterns this
# robot drilled into the same servo; nobody publishes where those holes are, so
# all of them are gone (cad/servo.py).
#
# THE CHASSIS WAS ALREADY MOST OF THE WAY THERE and nobody noticed. HIP_NOTCH_X
# is SERVO_W/2 + 0.8, so the lower shelf's notch already runs 0.8 mm off the
# servo's sides - it grips the case over the shelf's 3 mm thickness by accident
# of being sized to clear it. All this adds is depth: a rim on the inside of the
# side plate, running the length of the case.
from cad.servo_dims import CRADLE_CLEAR  # noqa: E402  one copy, see there
CRADLE_WALL = 2.5
CRADLE_DEPTH = 8.0            # inboard from the side plate, along the shaft
# The SERVO's span, not the NOTCH's. The notch runs -1.0..SERVO_L + 1.0 because
# a cutter has to overshoot to cut cleanly; a wall must not. Borrowing those
# limits hung the cradle 1 mm below the chassis floor at z = 0, as an
# unsupported sliver that was also partly floating in the notch's own opening.
# It exported an STL with inconsistent face orientation, which MuJoCo refuses to
# load: visible as broken geometry along the bottom of the chassis.
#
# A cut and a wall want different bounds. This is the wall's.
CRADLE_Z = HIP_SERVO_Z        # the full case, now that the plate reaches it

# The side plates start BELOW the hip axis, not at it. A servo whose horn is on
# the axis runs down to -10.2, and a rim has to have something to grow from, so
# the plate drops far enough to back the whole case. This is the proportions
# question resolved the cheap way: 12.7 mm of extra plate, about 1 g, rather
# than moving the hip axis and re-tuning a balancer against a new ride height.
SIDE_Z0 = HIP_SERVO_Z[0] - CRADLE_WALL

PI_HOLES = [(-35.0, -24.5), (-35.0, 24.5), (23.0, -24.5), (23.0, 24.5)]

# --- end panels ---------------------------------------------------------------
#
# The chassis was open front and back: a U-section, which is the worst shape
# there is in torsion, and docs/before-you-order.md has wanted a rear brace for
# a while. Closing the ends turns it into a box.
#
# They start at z = 45.4, not z = 0, and that is not a styling choice. The Pi
# runs x = -34.0..51.0 inside a 90 mm deep chassis, so it has 2.5 mm at each
# end and a 3 mm panel down to the floor would go straight through it. Above
# 45.4 the Pi is done (it tops out at 42.5) and the panel clears everything.
#
# The front panel is a FRAME, not a plate. The battery stands upright at
# y = +/-17, z = 62.5..167.5, and with both ends closed there is no other way
# to get it in or out. The window is sized to pass it with 2 mm to spare.
PANEL_T = 3.0
PANEL_Z0 = 45.4
BACK_BORDER = 9.0             # solid rim round the rear lightening window
FRONT_WIN_Y = 19.0            # battery is +/-17
FRONT_WIN_Z = (53.4, 172.0)   # battery is 62.5..167.5
CORNER_R = 6.0                # window corners, so it reads as designed


def _round_rect(y0, y1, z0, z1, x0, x1, r):
    """A rounded-corner window cutter, in the y-z plane.

    Built from two crossed boxes plus corner cylinders rather than by
    filleting afterwards: OCC refuses the fillet often enough on parts this
    busy that a silent fallback to sharp corners is a real risk, and a sharp
    re-entrant corner in a shear panel is exactly where it would crack.
    """
    cut = _plate((x0, x1), (y0 + r, y1 - r), (z0, z1))
    cut += _plate((x0, x1), (y0, y1), (z0 + r, z1 - r))
    for cy in (y0 + r, y1 - r):
        for cz in (z0 + r, z1 - r):
            cut += (bd.Pos((x0 + x1) / 2, cy, cz) * bd.Rot(0, 90, 0)
                    * bd.Cylinder(r, x1 - x0))
    return cut


def _plate(x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def build():
    part = None
    for sgn in (1, -1):
        y0, y1 = sgn * (SIDE_Y - SIDE_T / 2), sgn * (SIDE_Y + SIDE_T / 2)
        side = _plate((X0, X1), (min(y0, y1), max(y0, y1)),
                      (SIDE_Z0, TOP_Z + 1.5))
        side -= _plate((-HIP_NOTCH_X, HIP_NOTCH_X),
                       (min(y0, y1) - 1, max(y0, y1) + 1),
                       HIP_NOTCH_Z)
        # Hip servo capture, replacing the four M2 that went through this plate.
        # OUTBOARD of the plate, because that is where the servo is.
        outer = sgn * (SIDE_Y + SIDE_T / 2)
        cy = tuple(sorted((outer, outer + sgn * CRADLE_DEPTH)))
        xi = SERVO_W / 2 + CRADLE_CLEAR
        xo = xi + CRADLE_WALL
        for sx in (-1, 1):
            side += _plate(tuple(sorted((sx * xi, sx * xo))), cy, CRADLE_Z)
        # Seat the case along its length. Two walls with CRADLE_CLEAR each side
        # LOCATE nothing - the servo drops in and floats 0.4 mm on every face,
        # so the load path from the torso into the leg runs through two M3 in
        # air. cad/floating.py found it (status/2026-08-04); the fix is one
        # flush face, the way cad/thigh.py seats its knee servo on a plate. A
        # flush WALL cannot be a press fit (print tol is +/-0.3, see
        # CRADLE_CLEAR), so the datum is a face PERPENDICULAR to the walls: an
        # end wall at the servo's lower z face, in the 2.5 mm the side plate
        # already reaches below it (SIDE_Z0). The case bottoms on it at 0.000.
        #
        # Runs the full width to +/-xo, not +/-xi. Stopping at xi met the cradle
        # walls (which start at xi) along a single EDGE at z = HIP_SERVO_Z[0],
        # and an edge-only join is non-manifold - it left four open edges that a
        # print service rejects (cad/printability.py). To xo it shares a FACE
        # with each wall and the outer x = xo face runs continuous, the way the
        # walls already meet the side plate cleanly.
        side += _plate((-xo, xo), cy, (SIDE_Z0, HIP_SERVO_Z[0]))
        part = side if part is None else part + side

    for z, name in ((TOP_Z, "top"), (SHELF1_Z, "shelf1"), (SHELF2_Z, "shelf2")):
        shelf = _plate((X0, X1), (-INNER_Y, INNER_Y),
                       (z - PLATE_T / 2, z + PLATE_T / 2))
        if name == "shelf1":
            # At the hip servo's height, so it needs the same notch the side
            # plates do - the servo body passes right through this plane.
            shelf -= _plate((-HIP_NOTCH_X, HIP_NOTCH_X),
                            (-INNER_Y - 1, INNER_Y + 1),
                            (z - PLATE_T, z + PLATE_T))
        part += shelf

    # Pi standoffs with a heat-set insert in each top, NOT a clearance hole
    # through the shelf. The schedule bolts the Pi DOWN into the chassis
    # (M2.5 x 6, "insert in chassis"), so the thread lives in the standoff. The
    # old 2.7 mm through-hole matched no fastener - it was the clearance for a
    # screw coming the other way, which the bolt list does not have, and it left
    # cad/fasteners.inserts_seated red. The post is r = 3.0, so a 3.5 mm bore
    # leaves 1.25 mm of wall, just over INSERT_WALL; 4 mm deep bottoms on the
    # shelf.
    for px, py in PI_HOLES:
        part += bd.Pos(px + 8.5, py, SHELF1_Z + 3.5) * bd.Cylinder(3.0, 4.0)
        part -= (bd.Pos(px + 8.5, py, SHELF1_Z + 3.5)
                 * bd.Cylinder(M25_INSERT_R, M25_INSERT_DEPTH))

    # Close the two open ends. See the note by PANEL_T for why they start at
    # z = 45.4 and why the front one is a frame.
    ztop = TOP_Z + 1.5
    back = _plate((X0, X0 + PANEL_T), (-INNER_Y, INNER_Y), (PANEL_Z0, ztop))
    back -= _round_rect(-INNER_Y + BACK_BORDER, INNER_Y - BACK_BORDER,
                        PANEL_Z0 + BACK_BORDER, ztop - BACK_BORDER,
                        X0 - 1, X0 + PANEL_T + 1, CORNER_R)
    part += back

    front = _plate((X1 - PANEL_T, X1), (-INNER_Y, INNER_Y), (PANEL_Z0, ztop))
    front -= _round_rect(-FRONT_WIN_Y, FRONT_WIN_Y,
                         FRONT_WIN_Z[0], FRONT_WIN_Z[1],
                         X1 - PANEL_T - 1, X1 + 1, CORNER_R)
    part += front

    # Slots for a hook-and-loop strap over the pack.
    for sx in (-14.0, 31.0):
        part -= _plate((sx, sx + 4), (-20.0, -14.0),
                       (SHELF2_Z - 5, SHELF2_Z + 5))
        part -= _plate((sx, sx + 4), (14.0, 20.0),
                       (SHELF2_Z - 5, SHELF2_Z + 5))

    part = part.clean()

    # Soften the four long outside corners. Not only for looks: they are the
    # full-height edges of a shear box, and cad/shape.py explains why a fillet
    # that quietly did not happen is worse than none at all.
    from cad.shape import long_edges, soften
    sy = SIDE_Y + SIDE_T / 2
    part, r = soften(part,
                     long_edges(part, "z", 100.0,
                                at=[(X0, sy), (X0, -sy), (X1, sy), (X1, -sy)]),
                     what="chassis corners")
    build.corner_r = r

    # THE PARALLELOGRAM'S GROUND PIN LIVES ON THIS PART. Stage 1 of the linkage
    # is a rod from a pin on the chassis to a pin on the idler, and that first
    # pin needs a fin to press into: see cad/linkage_mounts.py, and
    # cad/linkage.py for what the mechanism is. One per leg.
    #
    # It is HERE and not a separate STEP file, which it was for a day. A fin
    # drawn on its own reads fine in a picture and is not fine: the sim's own
    # `test_every_body_is_one_rigid_piece` came back "torso is 3 loose pieces",
    # because the pin is fixed to the torso and the thing it presses into was
    # not part of the chassis.
    #
    # AFTER the fillet, deliberately. soften() picks the four full-height
    # outside corners by position, and a fin standing off the side plate adds
    # edges it would have to be taught to ignore.
    # IN THIS PART'S FRAME, which is not the frame torso_boss draws in.
    # cad/linkage_mounts.py works in LEG-LOCAL y, where zero is the leg's own
    # plane; the chassis's zero is the centreline, 60 mm inboard of that. Added
    # without the shift the fin lands on the robot's midline and its pin
    # engages nothing, which is what `assembled()` reported: 0.00 mm in torso.
    from cad.linkage_mounts import torso_boss
    from src.rsbot.model import LEG_Y

    leg_y = LEG_Y * 1000.0
    fin = torso_boss(leg_y, SIDE_Y + SIDE_T / 2, SIDE_Z0)
    part += bd.Pos(0, leg_y, 0) * fin
    part += bd.Pos(0, -leg_y, 0) * bd.mirror(fin, bd.Plane.XZ)
    return part.clean()


def main(export=True):
    p = build()
    g = p.volume / 1000.0 * PRINT_DENSITY * INFILL
    if export:
        OUT.mkdir(exist_ok=True)
        bd.export_step(p, str(OUT / "chassis.step"))
        bd.export_stl(p, str(OUT / "chassis.stl"),
                      tolerance=0.02, angular_tolerance=0.2)
    bb = p.bounding_box()
    print(f"bounding box   {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    print(f"solid volume   {p.volume/1000:.2f} cm3")
    print(f"printed @{INFILL:.0%}   {g:.1f} g")
    return g / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
