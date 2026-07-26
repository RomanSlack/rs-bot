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

OUT = Path(__file__).parent / "out"

PETG_SOLID = 1.270            # g/cm3
# A print is not solid plastic. 4 perimeters and ~40% gyroid on a part this
# size lands near 60% of solid. Measure a real print and correct this.
INFILL = 0.60

# --- geometry, straight from the sim -----------------------------------------
SPINE_Y, SPY = 21.0, 6.0      # spine centre-line and half-thickness
ANKLE_Z = -110.0

SPINE = ((-10, 10), (SPINE_Y - SPY, SPINE_Y + SPY), (-55, 0))
ARM = ((-42, -10), (SPINE_Y - SPY, SPINE_Y + SPY), (-53, -43))
POST = ((-54, -42), (SPINE_Y - SPY, SPINE_Y + SPY), (-94, -48))
STANDOFF = ((-8, 8), (SPINE_Y + SPY, SPINE_Y + SPY + 7), (-52, -28))

# --- fasteners ---------------------------------------------------------------
M2_CLEAR = 1.1                # radius, clearance for M2
HORN_BORE = 10.0              # radius, over the 25T servo horn
HORN_BOLTS = 7.5              # bolt circle radius on the horn
BEARING_BORE = 3.0            # radius, ankle pitch shaft


def _box(spec):
    (x0, x1), (y0, y1), (z0, z1) = spec
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def build():
    part = _box(SPINE) + _box(ARM) + _box(POST) + _box(STANDOFF)

    # Hub at the knee, bolted to the servo horn. The knee axis is +y.
    hub = bd.Pos(0, SPINE_Y, -14) * bd.Rot(90, 0, 0) * bd.Cylinder(16, 12)
    part += hub
    part -= bd.Pos(0, SPINE_Y, -14) * bd.Rot(90, 0, 0) * bd.Cylinder(HORN_BORE, 14)
    for ang in (0, 90, 180, 270):
        loc = bd.Rot(0, 0, ang) * bd.Pos(HORN_BOLTS, 0, 0)
        part -= (bd.Pos(loc.position.X, SPINE_Y, -14 + loc.position.Y)
                 * bd.Rot(90, 0, 0) * bd.Cylinder(M2_CLEAR, 20))

    # Ankle pitch bearing, on the same +y axis, at the bottom of the aft post.
    part += bd.Pos(-48, SPINE_Y, -88) * bd.Rot(90, 0, 0) * bd.Cylinder(11, 12)
    part -= bd.Pos(-48, SPINE_Y, -88) * bd.Rot(90, 0, 0) * bd.Cylinder(BEARING_BORE, 14)

    # Ankle-servo mounting, through the standoff.
    for z in (-46, -34):
        part -= (bd.Pos(0, SPINE_Y + SPY + 3.5, z) * bd.Rot(90, 0, 0)
                 * bd.Cylinder(M2_CLEAR, 20))

    # Lightening pocket down the spine, where there is no load path.
    part -= bd.Pos(0, SPINE_Y, -34) * bd.Box(10, 40, 30)

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
