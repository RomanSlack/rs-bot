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


OUT = Path(__file__).parent / "out"
PETG_SOLID, INFILL = 1.270, 0.60

SPINE_Y = 21.0
SPY_OUT = 10.0                # half-thickness, 20 mm total
KNEE_Z = -110.0

HUB_R, HORN_BORE = 13.0, 4.0
M2_CLEAR = 1.1

# From cad/servo.py, measured off a real STEP model. These were 45.2/24.7/35.4
# and 10.0, invented from a product listing; the height was 4.2 mm short and it
# is the axis the horn sits on.
from cad.servo import LENGTH as SERVO_L, WIDTH as SERVO_W, HEIGHT as SERVO_H
from cad.servo import SHAFT_INSET, HORN_DX, HORN_DY, HORN_SCREW_R
SERVO_Y_HI = SPINE_Y - 6.0                    # servo's outboard face, y = 15
SERVO_Y_LO = SERVO_Y_HI - SERVO_H             # inboard face, y = -20.4
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
    # The real horn pattern: a 9.9 x 10.0 mm rectangle, not a circle at r = 8.
    # Every one of these holes used to be 0.71 mm out of position.
    for dx in (-HORN_DX, HORN_DX):
        for dz in (-HORN_DY, HORN_DY):
            part -= (bd.Pos(dx, SPINE_Y, dz) * bd.Rot(90, 0, 0)
                     * bd.Cylinder(HORN_SCREW_R, 20))

    # Cross-member over the top of the servo, then down the inboard side.
    part += _box((-10, 10), (SERVO_Y_LO - PLATE_T, SPINE_Y + SPY_OUT),
                 (SERVO_Z_HI, SERVO_Z_HI + 12))
    part += _box((-10, 10), (SERVO_Y_LO - PLATE_T, SERVO_Y_LO),
                 (SERVO_Z_LO, SERVO_Z_HI + 12))

    # Ribs from the cross-member down the plate, one at each edge in x.
    for x0 in (-10.0, 10.0):
        sgn = 1 if x0 > 0 else -1
        pts = ((x0, SERVO_Z_HI + 12), (x0, SERVO_Z_HI + 12 - RIB_RUN),
               (x0 - sgn * RIB_RUN, SERVO_Z_HI + 12))
        part += _gusset(pts, SERVO_Y_LO - PLATE_T, RIB_T)

    # Servo mounting bolts: all four corners, through the inboard plate.
    for dx in (-8.5, 8.5):
        for dz in (SERVO_Z_LO + 5.0, SERVO_Z_HI - 5.0):
            part -= (bd.Pos(dx, SERVO_Y_LO - PLATE_T / 2, dz)
                     * bd.Rot(90, 0, 0) * bd.Cylinder(M2_CLEAR, 40))

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
    g = p.volume / 1000.0 * PETG_SOLID * INFILL
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
