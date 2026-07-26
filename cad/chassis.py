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
PETG_SOLID, INFILL = 1.270, 0.60
M2_CLEAR, M25_CLEAR = 1.1, 1.35

X0, X1 = -36.5, 53.5          # 90 mm deep, centred on the sim's 8.5 offset
SIDE_Y, SIDE_T = 38.1, 3.0    # plate centre-line and thickness
INNER_Y = SIDE_Y - SIDE_T / 2
TOP_Z, SHELF1_Z, SHELF2_Z = 178.5, 20.0, 61.0
PLATE_T = 3.0

HIP_Z = 0.0                   # hip axis
HIP_BOLTS_X = 17.0            # servo case, bolts either side of the axis
HIP_BOLTS_Z = 17.0

PI_HOLES = [(-35.0, -24.5), (-35.0, 24.5), (23.0, -24.5), (23.0, 24.5)]


def _plate(x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def build():
    part = None
    for sgn in (1, -1):
        y0, y1 = sgn * (SIDE_Y - SIDE_T / 2), sgn * (SIDE_Y + SIDE_T / 2)
        side = _plate((X0, X1), (min(y0, y1), max(y0, y1)), (0, TOP_Z + 1.5))
        # Hip servo mounting: four M2 through the plate, around the hip axis.
        for dx in (-HIP_BOLTS_X, HIP_BOLTS_X):
            for dz in (HIP_Z + 5.0, HIP_Z + 5.0 + HIP_BOLTS_Z):
                side -= (bd.Pos(dx, sgn * SIDE_Y, dz) * bd.Rot(90, 0, 0)
                         * bd.Cylinder(M2_CLEAR, 20))
        part = side if part is None else part + side

    for z, name in ((TOP_Z, "top"), (SHELF1_Z, "shelf1"), (SHELF2_Z, "shelf2")):
        part += _plate((X0, X1), (-INNER_Y, INNER_Y),
                       (z - PLATE_T / 2, z + PLATE_T / 2))

    # Pi standoffs and clearance holes on the lower shelf.
    for px, py in PI_HOLES:
        part += bd.Pos(px + 8.5, py, SHELF1_Z + 3.5) * bd.Cylinder(3.0, 4.0)
        part -= bd.Pos(px + 8.5, py, SHELF1_Z) * bd.Cylinder(M25_CLEAR, 20)

    # Slots for a hook-and-loop strap over the pack.
    for sx in (-14.0, 31.0):
        part -= _plate((sx, sx + 4), (-20.0, -14.0),
                       (SHELF2_Z - 5, SHELF2_Z + 5))
        part -= _plate((sx, sx + 4), (14.0, 20.0),
                       (SHELF2_Z - 5, SHELF2_Z + 5))

    return part.clean()


def main(export=True):
    p = build()
    g = p.volume / 1000.0 * PETG_SOLID * INFILL
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
