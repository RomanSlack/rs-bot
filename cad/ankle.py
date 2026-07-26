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
# The arm reaches 4 mm further aft and the tie is deeper and wider, so the two
# actually LAP instead of grazing. They used to overlap in a box 2.00 x 0.50 x
# 2.65 mm - 2.65 mm3 - and the entire wheel load went through it. FEA put the
# part at 1068% of PETG's allowable, and at 111% in aluminium: no material
# fixes a 0.5 mm neck. The lap is now about 200 mm3, 75x more, plus a rib.
ROLL_ARM = ((-44, 0), (12.1, 23.5), (12.35, 22.35))
ROLL_TIE = ((-56, -42), (7.1, 14.5), (5, 15.5))
# The arm cannot grow aft and the tie cannot grow forward: aft it runs into the
# shin's post, forward it goes inside the upright wheel. So the lap grows in y
# and z instead - 2.0 x 5.9 x 5.65 mm rather than 2.00 x 0.50 x 2.65 - and a
# rib carries the load over a span instead of through the lap.
#
# That rib is the real fix. It bridges x = -44 to -28, overlapping the tie at
# one end and running well out along the arm at the other, so the joint is no
# longer a butt. A triangle is worth far more here than uniform thickening:
# bending stiffness goes as depth cubed, so depth at a corner is the cheapest
# strength on the part.
ROLL_RIB = ((-44, 12.35), (-44, 26.0), (-28, 12.35))
ROLL_RIB_Y = (12.1, 14.5)
# A tongue running aft from the arm to meet the tie along its whole length,
# filling the y down to the tie's own inboard face so the two together are a
# solid 7.5 x 17 mm section rather than a thin L. FEA had the peak on the
# inside of that L.
# threaded through the one gap available: INBOARD of the shin's post, which
# starts at y = 15, and outboard of the yoke's rotation sweep. It turns a 2 mm
# butt into a 12 mm lap, which is the difference between a hinge and a joint.
ROLL_TONGUE = ((-56, -44), (7.1, 14.6), (12.35, 22.35))

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


def _gusset(pts_xz, y0, t):
    """Triangular rib in the x-z plane, spanning y0 to y0 + t."""
    g = bd.extrude(bd.Plane.XZ * bd.Polygon(*pts_xz), amount=t)
    return bd.Pos(0, y0 + t, 0) * g


def roll_bracket():
    """Rolls with the wheel: carries the wheel servo and the wheel axle."""
    part = _box(ROLL_ARM) + _box(ROLL_TIE) + _box(ROLL_TONGUE)
    part += _gusset(ROLL_RIB, ROLL_RIB_Y[0], ROLL_RIB_Y[1] - ROLL_RIB_Y[0])

    # Wheel axle bore, on the spin axis (y), outboard where the servo sits.
    part -= bd.Pos(0, 18, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(WHEEL_SHAFT_R, 40)

    # Roll-side bearing bore, on the roll axis (x).
    part -= bd.Pos(-49, 7.88, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(ROLL_SHAFT_R, 30)

    # Wheel-servo mounting, two bolts through the arm.
    for dx in (-34.0, -10.0):
        part -= (bd.Pos(dx, 18, 0) * bd.Rot(90, 0, 0)
                 * bd.Cylinder(M2_CLEAR, 30))

    # Fillet the corner where arm, tie and tongue meet. FEA put every one of
    # the twelve hottest nodes inside an 3 x 3 x 3 mm cube right here, on a
    # sharp re-entrant corner - which in a linear-elastic solve is a
    # SINGULARITY: refine the mesh and the number just grows, so it was never
    # going to converge. A radius fixes the real part and the artefact at once,
    # and costs 14 mm3.
    # Narrow selection deliberately. Widening it to take in the tongue's edges
    # as well makes OCC refuse the whole operation, and a `try: ... except:
    # continue` around it swallows that and silently returns the SHARP part -
    # which reads as "the fillet did not help much" rather than "there is no
    # fillet". The volume check makes that loud.
    corner = [e for e in part.edges()
              if -47 <= e.center().X <= -39 and 10 <= e.center().Y <= 17
              and 10 <= e.center().Z <= 18]
    before = part.volume
    part = part.fillet(2.0, corner)
    assert part.volume > before + 5.0, "the corner fillet did not take"
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
