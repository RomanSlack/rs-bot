"""The shin, as a real printable part.  uv run python cad/shin.py

Built from the dimensions the simulator already uses, so the CAD and the sim
describe the same object. Adds what a real part needs and a parametric sketch
does not: a hub to bolt onto the knee servo horn, a bearing bore for the ankle
pitch axis, and M2 clearance holes for the ankle servo.

Outputs STEP (to print or machine) and STL (as a MuJoCo asset), and reports
mass from the solid rather than from an estimate.

Frame matches the sim's shin body: origin at the knee, z negative downward,
ankle axis at z = -110. Millimetres.
"""

import sys
from pathlib import Path

import build123d as bd
import numpy as np

OUT = Path(__file__).parent / "out"

PETG_SOLID = 1.270            # g/cm3
# A print is not solid plastic. 4 perimeters and ~40% gyroid on a part this
# size lands near 60% of solid. Measure a real print and correct this.
INFILL = 0.60

# --- geometry, straight from the sim -----------------------------------------
SPINE_Y, SPY = 21.0, 6.0      # spine centre-line and half-thickness
ANKLE_Z = -110.0

SPINE = ((-10, 10), (SPINE_Y - SPY, SPINE_Y + SPY), (-55, 0))
# 14 mm deep, not 10. At 10 the arm peaked at 18.8 MPa, which is only 2.1x on
# a 3x design load and uncomfortably near PETG's ~20 MPa ACROSS layers.
ARM = ((-42, -10), (SPINE_Y - SPY, SPINE_Y + SPY), (-55, -41))
POST = ((-54, -42), (SPINE_Y - SPY, SPINE_Y + SPY), (-94, -48))
# Runs the length of the servo case so its mounting bolts can be far apart.
STANDOFF = ((-8, 8), (SPINE_Y + SPY, SPINE_Y + SPY + 7), (-62, -18))

# --- fasteners ---------------------------------------------------------------
M2_CLEAR = 1.1                # radius, clearance for M2
HUB_R = 13.0                  # hub outer radius
HORN_BORE = 4.0               # radius, clearance over the servo output boss
HORN_BOLTS = 8.0              # bolt circle radius; MUST sit outside HORN_BORE
                              # and inside HUB_R, or the holes cut thin air
BEARING_OD = 5.0              # radius, 623ZZ outer race (10 mm dia)
BEARING_W = 4.0               # 623ZZ width
SHAFT_R = 2.0                 # radius, clearance for the 3 mm shaft
SERVO_BOLTS = 40.0            # mounting bolt spacing along the servo case


def _box(spec):
    (x0, x1), (y0, y1), (z0, z1) = spec
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def build():
    part = _box(SPINE) + _box(ARM) + _box(POST) + _box(STANDOFF)

    # Hub at the knee, bolted to the servo horn. The knee axis is +y.
    part += bd.Pos(0, SPINE_Y, -14) * bd.Rot(90, 0, 0) * bd.Cylinder(HUB_R, 12)
    part -= bd.Pos(0, SPINE_Y, -14) * bd.Rot(90, 0, 0) * bd.Cylinder(HORN_BORE, 14)
    for i in range(4):
        ang = np.deg2rad(45 + 90 * i)
        part -= (bd.Pos(HORN_BOLTS * np.cos(ang), SPINE_Y,
                        -14 + HORN_BOLTS * np.sin(ang))
                 * bd.Rot(90, 0, 0) * bd.Cylinder(M2_CLEAR, 20))

    # Ankle pitch bearing: a 623ZZ pressed into a counterbore, with a
    # clearance hole through for the shaft. A plain hole would run the shaft
    # straight against printed plastic.
    part += bd.Pos(-48, SPINE_Y, -88) * bd.Rot(90, 0, 0) * bd.Cylinder(11, 12)
    part -= bd.Pos(-48, SPINE_Y, -88) * bd.Rot(90, 0, 0) * bd.Cylinder(SHAFT_R, 14)
    part -= (bd.Pos(-48, SPINE_Y + SPY - BEARING_W / 2, -88)
             * bd.Rot(90, 0, 0) * bd.Cylinder(BEARING_OD, BEARING_W))

    # Ankle-servo mounting. Spacing matters: the servo's 1.63 N.m reaction is
    # a force couple through these bolts, so 12 mm apart meant 136 N each.
    # At 40 mm it is 41 N.
    for z in (-40 - SERVO_BOLTS / 2, -40 + SERVO_BOLTS / 2):
        part -= (bd.Pos(0, SPINE_Y + SPY + 3.5, z) * bd.Rot(90, 0, 0)
                 * bd.Cylinder(M2_CLEAR, 20))

    # No lightening pocket. The first attempt cut a 10 mm slot clean through
    # a 20 mm spine, leaving a thin-necked keyhole, and the 0.73 mm stiffness
    # figure in docs/bom.md assumes a SOLID section. It saved about 2 g on a
    # 17 g part - not a trade worth making.

    return part.clean()


def main(export=True):
    p = build()
    solid_cm3 = p.volume / 1000.0
    solid_g = solid_cm3 * PETG_SOLID
    printed_g = solid_g * INFILL

    if export:
        OUT.mkdir(exist_ok=True)
        bd.export_step(p, str(OUT / "shin.step"))
        bd.export_stl(p, str(OUT / "shin.stl"),
                      tolerance=0.01, angular_tolerance=0.1)

    bb = p.bounding_box()
    print(f"bounding box   {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    print(f"solid volume   {solid_cm3:.2f} cm3")
    print(f"solid PETG     {solid_g:.1f} g")
    print(f"printed @{INFILL:.0%}   {printed_g:.1f} g   <- use this")
    c = p.center()
    print(f"centroid       ({c.X:+.1f}, {c.Y:+.1f}, {c.Z:+.1f}) mm")
    return printed_g / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
