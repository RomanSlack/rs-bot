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
POST = ((-56, -47), (-6, 6), (-6, 6))          # bearing carrier, on the roll axis
                                               # between servo (ends -58) and
                                               # the flat wheel (needs |x|>40)

# The ankle-pitch joint, which was previously not drawn at all: the shin had a
# bore 52.8 mm off the axis and the yoke had no matching feature, so assembled
# the two parts came no closer than 10.3 mm with nothing between them.
#
# The yoke reaches the axis OUTBOARD, threading aft of the wheel at x = -56..-46
# and then forward at y > 52, which is just past the wheel-drive servo's outer
# face at y = 49.4. See cad/envelope.py for why there is nowhere else.
PITCH_SHAFT_R = 2.0           # 3 mm shaft, clearance
# These two must OVERLAP, not merely touch. At (-46, 53) they met along a
# single edge - zero contact area - which is a hinge, not a joint. It does not
# show up as interference, it does not show up as disconnection (the solid is
# topologically one piece), and the mass is right. What caught it was the FEA
# refusing to converge: a part with a hinge in it has a rigid-body mode and no
# amount of constraint on one side fixes that.
YOKE_CROSS = ((-56, -46), (-6, 56), (-8, 8))
YOKE_FWD = ((-56, 0), (53, 63), (-8, 8))
YOKE_BOSS_Y = (53.0, 63.0)
ROLL_SV = ((-93.4, -58.0), (-12.35, 12.35), (-6.6, 38.6))
# The arm reaches 4 mm further aft and the tie is deeper and wider, so the two
# actually LAP instead of grazing. They used to overlap in a box 2.00 x 0.50 x
# 2.65 mm - 2.65 mm3 - and the entire wheel load went through it. FEA put the
# part at 1068% of PETG's allowable, and at 111% in aluminium: no material
# fixes a 0.5 mm neck. The lap is now about 200 mm3, 75x more, plus a rib.
ROLL_ARM = ((-44, 0), (12.1, 23.5), (12.35, 22.35))
ROLL_TIE = ((-56, -42), (7.1, 14.5), (5, 15.5))
# The tie, the tongue and the corner rib are all gone. They existed to crank
# the load from an outboard arm, round the shin's post, to a bearing the part
# never actually reached - three redesign rounds of dodging a post that was
# only there to carry a bore 52.8 mm off its own axis.
#
# With the shin now crossing outboard HIGH and descending at y = 60..74, that
# whole corridor is empty, and the bracket can be what it should always have
# been: a plate in the y-z plane running straight from the arm to the roll
# axis. x = -45..-41 is the one window forward of the yoke's post that is still
# outside the 40 mm wheel in both of its shapes.
ROLL_WEB_X = (-45.0, -41.0)
ROLL_WEB = ((0.0, 0.0), (0.0, 23.5), (-23.5, 22.35))   # (unused placeholder)
ROLL_HUB_R = 8.0

ROLL_BEARING_OD = 5.0        # 623ZZ outer race radius
ROLL_BEARING_W = 4.0
ROLL_SHAFT_R = 2.0
WHEEL_SHAFT_R = 2.5


def _box(spec):
    (x0, x1), (y0, y1), (z0, z1) = spec
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def yoke():
    """Ankle yoke: roll bearing, roll servo face, and the ankle-pitch shaft."""
    part = _box(POST) + _box(YOKE_CROSS) + _box(YOKE_FWD)

    # Pitch shaft boss, ON the pitch axis (x = 0, z = 0 in this frame).
    y0, y1 = YOKE_BOSS_Y
    part += (bd.Pos(0, (y0 + y1) / 2, 0) * bd.Rot(90, 0, 0)
             * bd.Cylinder(7.0, y1 - y0))
    part -= (bd.Pos(0, (y0 + y1) / 2, 0) * bd.Rot(90, 0, 0)
             * bd.Cylinder(PITCH_SHAFT_R, 2 * (y1 - y0)))

    # Roll bearing, on the roll axis (x), seated from the aft face. Moved
    # inboard to x = -50 so the post can end at -47 and leave the bracket a
    # window at x = -45..-41 to grip the same shaft - the only place forward of
    # this post that is still outside the 40 mm wheel.
    part -= bd.Pos(-50, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(ROLL_SHAFT_R, 40)
    part -= (bd.Pos(-52, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ROLL_BEARING_OD, ROLL_BEARING_W))

    # Face for the roll servo, and its four mounting bolts.
    part += _box(((-58, -54), (-6, 6), (-6, 6)))
    for dy in (-4.0, 4.0):
        for dz in (-4.0, 4.0):
            part -= (bd.Pos(-56, dy, dz) * bd.Rot(0, 90, 0)
                     * bd.Cylinder(M2_CLEAR, 20))
    return part.clean()


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


def roll_bracket():
    """Rolls with the wheel: carries the wheel servo and grips the roll shaft.

    A single web from the arm to the roll axis. The load path is a straight
    line now, where before it went through a 2.00 x 0.50 x 2.65 mm lap that
    FEA put at 1596% of PETG's allowable.
    """
    x0, x1 = ROLL_WEB_X
    part = _box(ROLL_ARM)

    # Web in the y-z plane: a triangle from the arm's inboard-lower corner down
    # to the roll axis, plus a hub round the axis itself.
    web = bd.extrude(bd.Plane.YZ * bd.Polygon(
        (0.0, 0.0), (23.5, 0.0), (23.5, 22.35), (0.0, 8.0)), amount=x1 - x0)
    part += bd.Pos(x0, 0, 0) * web   # extrudes +x, so anchor at x0
    part += (bd.Pos((x0 + x1) / 2, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ROLL_HUB_R, x1 - x0))

    # Roll shaft bore, on the axis, through the hub.
    part -= (bd.Pos((x0 + x1) / 2, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ROLL_SHAFT_R, 3 * (x1 - x0)))

    # Wheel axle bore and the wheel servo's mounting bolts, through the ARM
    # this time. All four of these used to be drawn at z = 0, on the wheel spin
    # axis, while the arm they pass through spans z = 12.35..22.35 - so every
    # one of them cut air and the part had no holes at all.
    az = (ROLL_ARM[2][0] + ROLL_ARM[2][1]) / 2
    part -= (bd.Pos(0, 18, az) * bd.Rot(90, 0, 0)
             * bd.Cylinder(WHEEL_SHAFT_R, 40))
    for dx in (-34.0, -10.0):
        part -= (bd.Pos(dx, 18, az) * bd.Rot(90, 0, 0)
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
