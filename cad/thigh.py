"""The thigh, as a real printable part.  uv run python cad/thigh.py

Hip axis at z = 0, knee axis at z = -110, millimetres, same frame as the sim.

The hard part is not the beam, it is the knee servo mount. The servo's case
occupies y = -20.4 .. 15 right where the shin's hub also wants to be, so the
thigh has to reach AROUND it to a plate on the inboard side. Bolting to the
outboard side would put the bracket straight through the shin hub.
"""

import sys
from pathlib import Path

import build123d as bd
import numpy as np

# --- cable channel -------------------------------------------------------------
#
# The bus lead from this part's servo to the next runs straight through here.
# cad/wiring.py found it, and the runs are modelled as the STRAIGHT LINE between
# measured connector ports - the shortest possible path - so anything it hits is
# hit by every real routing too.
#
# Cut at CABLE_CH_R, the 4.4 mm bundle plus room to lay it in without pinching.
# A relief where the path grazes the surface, a tunnel where it does not; either
# way it is the cable's space reserved, instead of discovered at the bench with
# the parts already printed.
# The endpoints are NOT stored here any more. They were, and they went stale
# three times in one day: moving the hip and wheel servos onto their own joints
# moved their connector ports, and correcting the case height from the STEP's
# 39.6 to the drawing's 36.5 then moved all ten of them by 1.15 mm. Eight of
# eight runs ended up passing through solid material and nothing but
# cad/wiring.py could see it.
#
# cad.wiring.channel() cuts the run where the run actually is, in both poses,
# every build. A number that has to be retyped whenever the robot moves is a
# number that will be wrong the next time the robot moves.
CABLE_CH_R = 3.0
# TWO PATHS, like the shin and the ankle already had. The thigh only ever cut
# one, and it went stale the moment the hip servo moved: putting that servo's
# shaft on the hip axis (cad/drives.py) dropped its connector port 9.6 mm, and
# the old channel missed the new line by enough to put 27 mm3 of thigh through
# the cable in wheel mode and 47 mm3 in foot mode.
#
# The far end is the knee servo's own port and does not move in this frame. The
# near end is on the TORSO, across the hip joint, so it swings between the poses
# and one channel can only ever be right for one of them.



OUT = Path(__file__).parent / "out"
from cad.material import DENSITY as PRINT_DENSITY, INFILL  # PA6-CF, one source

SPINE_Y = 21.0
SPY_OUT = 10.0                # half-thickness, 20 mm total
KNEE_Z = -110.0

HUB_R, HORN_BORE = 13.0, 4.0

# From cad/servo.py, measured off a real STEP model. These were 45.2/24.7/35.4
# and 10.0, invented from a product listing; the height was 4.2 mm short and it
# is the axis the horn sits on.
from cad.servo import LENGTH as SERVO_L, WIDTH as SERVO_W, HEIGHT as SERVO_H
from cad.servo import SHAFT_INSET, HORN_DX, HORN_DY, HORN_SCREW_R
SERVO_Y_HI = SPINE_Y - 6.0                    # servo's outboard face, y = 15
SERVO_Y_LO = SERVO_Y_HI - SERVO_H             # inboard face, y = -21.5
# That comment said -20.4 for as long as SERVO_H was 35.4. cad/servo.py
# re-exports cad/servo_dims.py, HEIGHT there was corrected to 36.50, and the
# face moved 1.1 mm without the comment following it. Nothing read the comment,
# so nothing broke - but src/rsbot/model.py's thigh boxes were being written off
# it, which put a box 0.5 mm inside the knee servo that the solids never had.
SERVO_Z_LO = KNEE_Z - SHAFT_INSET             # -120.0
SERVO_Z_HI = SERVO_Z_LO + SERVO_L             # -74.8

# 9 mm, not 6. FEA put the peak at 161% of PETG's allowable right here, on
# the plate the whole shin hangs off - not on the spine, which is what got
# widened for the hip's lateral moment. Section modulus goes as thickness
# squared, so 6 -> 9 is 2.25x on its own, and the ribs below do the rest.
PLATE_T = 9.0                 # inboard mounting plate thickness
# Two triangular ribs tying the plate back to the cross-member, in the x-z
# plane at either side. Depth at the corner is worth more than thickness
# everywhere: bending stiffness goes as depth cubed.
RIB_T = 4.0
RIB_RUN = 26.0                # how far the rib reaches down the plate

# --- capturing the knee servo instead of bolting into its case -----------------
#
# The four M2 that used to go here are gone. Not because they were badly placed
# but because NOBODY KNOWS WHERE THEY GO. Feetech's drawing labels the case holes
# "8-PA2.0" and never dimensions them, none of their three brackets touches the
# case, and the reference design cradles the servo instead. This robot had four
# parts each drilling a DIFFERENT invented pattern into the same case: 17.0 at
# the chassis, 5 mm from the ends here, 40.0 spacing on the shin, +/-10.2 at the
# ankle. At most one of those could have been right. See cad/servo.py.
#
# A rim round the case takes the servo's reaction couple in BEARING across the
# full 45.4 x 39.6 side faces, rather than as a force couple through four
# self-tappers in a 55 g plastic housing.
#
# WHY IT ATTACHES HERE AND NOT ON THE SHIN. cad/servo.cradle() fails on the shin
# because that mount is a bare bar with no material round the case perimeter, so
# a rim fuses as a floating solid. The thigh already has the inboard plate and
# the cross-member, so there is something to grow from. The plate does have to
# get wider: it was +/-10 against a servo that is 24.8 across, so it did not even
# reach the case sides it now has to grip.
CRADLE_CLEAR = 0.4            # per side, and it must EXCEED the print tolerance
CRADLE_WALL = 2.5
CRADLE_DEPTH = 10.0           # how far the rim reaches along the shaft
CRADLE_X = SERVO_W / 2 + CRADLE_CLEAR + CRADLE_WALL      # 15.3
#
# AXIAL RETENTION IS FREE HERE and that is worth writing down, because it is not
# free everywhere. The knee servo's horn drives the shin, so the servo cannot
# slide out along its own shaft without taking the shin with it. It is trapped
# between the cradle and its own output. No lid, no strap, no screws.


def _box(x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def _gusset(pts_xz, y0, t):
    """Triangular rib in the x-z plane, spanning y0 to y0 + t.

    Normalised by MEASUREMENT rather than by trusting the extrude direction.
    `bd.extrude` follows the face normal, and that normal flips with the
    polygon's winding order, so the identical call places the rib at
    y0..y0+t for one point order and y0+t..y0+2t for the other. The shin's rib
    landed 12 mm out of position that way, sitting inside the ankle servo, and
    nothing but cad/envelope.py would have noticed.
    """
    g = bd.extrude(bd.Plane.XZ * bd.Polygon(*pts_xz), amount=t)
    return bd.Pos(0, y0 - g.bounding_box().min.Y, 0) * g


def build():
    # Spine, running hip to just above the servo. Wider in y than the shin's:
    # lateral bending at the hip is the governing load in the whole leg.
    part = _box((-10, 10), (SPINE_Y - 6, SPINE_Y + SPY_OUT), (-66, -4))

    # Hip hub, centred ON the hip axis.
    part += bd.Pos(0, SPINE_Y, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(HUB_R, 12)
    part -= bd.Pos(0, SPINE_Y, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(HORN_BORE, 14)
    # Horn pocket, on the face that meets the hip servo. Same 1.10 mm clash as
    # the shin's, same cause: positioned at the spline tip, and the horn ends
    # 1.10 mm past it. See cad/servo.py HORN_PROUD.
    from cad.servo import HORN_POCKET_R as _HPR, HORN_POCKET_T as _HPT
    part -= (bd.Pos(0, SPINE_Y - 6 + _HPT / 2, 0) * bd.Rot(90, 0, 0)
             * bd.Cylinder(_HPR, _HPT))
    # The real horn pattern: a 9.9 x 10.0 mm rectangle, not a circle at r = 8.
    # Every one of these holes used to be 0.71 mm out of position.
    for dx in (-HORN_DX, HORN_DX):
        for dz in (-HORN_DY, HORN_DY):
            part -= (bd.Pos(dx, SPINE_Y, dz) * bd.Rot(90, 0, 0)
                     * bd.Cylinder(HORN_SCREW_R, 20))

    # Cross-member over the top of the servo, then down the inboard side.
    part += _box((-10, 10), (SERVO_Y_LO - PLATE_T, SPINE_Y + SPY_OUT),
                 (SERVO_Z_HI, SERVO_Z_HI + 12))
    # The inboard plate. Widened from +/-10 to CRADLE_X so the rim below has
    # something to grow from, and extended past the servo's lower end for the
    # same reason.
    cz0 = SERVO_Z_LO - CRADLE_CLEAR - CRADLE_WALL
    part += _box((-CRADLE_X, CRADLE_X), (SERVO_Y_LO - PLATE_T, SERVO_Y_LO),
                 (cz0, SERVO_Z_HI + 12))

    # Ribs from the cross-member down the plate, one at each edge in x.
    for x0 in (-10.0, 10.0):
        sgn = 1 if x0 > 0 else -1
        pts = ((x0, SERVO_Z_HI + 12), (x0, SERVO_Z_HI + 12 - RIB_RUN),
               (x0 - sgn * RIB_RUN, SERVO_Z_HI + 12))
        part += _gusset(pts, SERVO_Y_LO - PLATE_T, RIB_T)

    # The cradle: two side walls and a lower end wall, standing off the plate
    # along the shaft. The upper end needs nothing, because the cross-member
    # already spans the full depth of the servo there.
    xi = SERVO_W / 2 + CRADLE_CLEAR
    cy = (SERVO_Y_LO, SERVO_Y_LO + CRADLE_DEPTH)
    for sgn in (-1, 1):
        part += _box(tuple(sorted((sgn * xi, sgn * CRADLE_X))), cy,
                     (cz0, SERVO_Z_HI + CRADLE_CLEAR))
    part += _box((-CRADLE_X, CRADLE_X), cy, (cz0, SERVO_Z_LO - CRADLE_CLEAR))

    # Reserve the cable run. See CABLE_CH_R.
    from cad.wiring import channel as _channel
    part = _channel(part, "vhipsv1 -> vkneesv_l", "thigh_l", r=CABLE_CH_R)

    # NOT FILLETED, and the honest reason is narrower than it first looked.
    #
    # OCC would only take r = 1.0 on this part's verticals, which on a 133 mm
    # part is invisible. It also made the FEA worse - but it did not CAUSE the
    # problem, which is the correction worth recording. With the fillet the
    # reported peak was 135 MPa at a 3.0 mm mesh; with it removed, still 73.
    # The artifact is the part's own default mesh being too coarse for the load
    # direction the closed chassis produces, and the fillet only amplified it.
    #
    # So: no fillet, because a 1.0 mm one buys nothing visible and does make
    # the meshing harder on the part carrying the highest torque in the robot
    # (8.77 N.m at the hip). The mesh size is fixed separately, in
    # cad/stress.py, which is where the actual fault was.
    return part.clean()


def main(export=True):
    p = build()
    g = p.volume / 1000.0 * PRINT_DENSITY * INFILL
    if export:
        OUT.mkdir(exist_ok=True)
        bd.export_step(p, str(OUT / "thigh.step"))
        bd.export_stl(p, str(OUT / "thigh.stl"),
                      tolerance=0.01, angular_tolerance=0.1)
    bb = p.bounding_box()
    print(f"bounding box   {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    print(f"solid volume   {p.volume/1000:.2f} cm3")
    print(f"printed @{INFILL:.0%}   {g:.1f} g")
    return g / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
