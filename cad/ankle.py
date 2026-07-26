"""Ankle yoke and roll bracket.  uv run python cad/ankle.py

Two small parts, both in the ankle body's frame with the origin at the AXLE.

  ankle yoke     carries the roll servo and the ankle-pitch bearing
  roll bracket   carries the wheel servo, and rolls 90 deg with it

They are small but they sit in the tightest region of the robot: three joints
inside 45 mm, two of which must clear a wheel that changes shape halfway
through the manoeuvre. See docs/porting-a-link.md for the clearance rules.
"""

import sys
from pathlib import Path

import build123d as bd
import numpy as np

OUT = Path(__file__).parent / "out"
PETG_SOLID, INFILL = 1.270, 0.60
M2_CLEAR = 1.1

# From the sim, millimetres, relative to the axle.
POST = ((-56, -44), (-6, 6), (-6, 6))          # bearing carrier, on the roll axis
                                               # between servo (ends -58) and
                                               # the flat wheel (needs |x|>40)
ROLL_SV = ((-93.4, -58.0), (-12.35, 12.35), (-6.6, 38.6))
ROLL_ARM = ((-44, 0), (12.1, 23.5), (12.35, 22.35))
ROLL_TIE = ((-56, -42), (7.1, 12.6), (5, 15))

ROLL_BEARING_OD = 5.0        # 623ZZ outer race radius
ROLL_BEARING_W = 4.0
ROLL_SHAFT_R = 2.0
WHEEL_SHAFT_R = 2.5


def _box(spec):
    (x0, x1), (y0, y1), (z0, z1) = spec
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def yoke():
    """Ankle yoke: bearing carrier plus the roll servo's mounting face."""
    part = _box(POST)

    # Roll bearing, on the roll axis (x), seated from the aft face.
    part -= bd.Pos(-50, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(ROLL_SHAFT_R, 30)
    part -= (bd.Pos(-46, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ROLL_BEARING_OD, ROLL_BEARING_W))

    # Face for the roll servo, and its four mounting bolts.
    part += _box(((-58, -54), (-6, 6), (-6, 6)))
    for dy in (-4.0, 4.0):
        for dz in (-4.0, 4.0):
            part -= (bd.Pos(-56, dy, dz) * bd.Rot(0, 90, 0)
                     * bd.Cylinder(M2_CLEAR, 20))
    return part.clean()


def roll_bracket():
    """Rolls with the wheel: carries the wheel servo and the wheel axle."""
    part = _box(ROLL_ARM) + _box(ROLL_TIE)

    # Wheel axle bore, on the spin axis (y), outboard where the servo sits.
    part -= bd.Pos(0, 18, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(WHEEL_SHAFT_R, 40)

    # Roll-side bearing bore, on the roll axis (x).
    part -= bd.Pos(-49, 7.88, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(ROLL_SHAFT_R, 30)

    # Wheel-servo mounting, two bolts through the arm.
    for dx in (-34.0, -10.0):
        part -= (bd.Pos(dx, 18, 0) * bd.Rot(90, 0, 0)
                 * bd.Cylinder(M2_CLEAR, 30))
    return part.clean()


def main(export=True):
    y, r = yoke(), roll_bracket()
    gy = y.volume / 1000.0 * PETG_SOLID * INFILL
    gr = r.volume / 1000.0 * PETG_SOLID * INFILL
    if export:
        OUT.mkdir(exist_ok=True)
        for name, solid in (("ankle_yoke", y), ("roll_bracket", r)):
            bd.export_step(solid, str(OUT / f"{name}.step"))
            bd.export_stl(solid, str(OUT / f"{name}.stl"),
                          tolerance=0.01, angular_tolerance=0.1)
    print(f"ankle yoke     {y.volume/1000:5.2f} cm3   {gy:5.1f} g printed")
    print(f"roll bracket   {r.volume/1000:5.2f} cm3   {gr:5.1f} g printed")
    return gy / 1000.0, gr / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
