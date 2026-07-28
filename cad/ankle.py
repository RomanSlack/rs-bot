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

# --- cable channel, roll servo to wheel servo ----------------------------------
#
# This run crosses the ankle ROLL joint, so its far end is on a bracket that
# turns 90 degrees and the cable sweeps a long arc between the modes. Only the
# end anchored on this part stays put.
#
# So both extremes get a channel: the cable has to live somewhere in both poses,
# and reserving one of them reserves the wrong one half the time. cad/wiring.py
# measures the length change at 6.3 mm, which is the service loop this has to
# let it take up.
CABLE_CH_R = 3.0
CABLE_A = (-78.3, 5.2, -6.9)          # roll servo port, fixed on the yoke
CABLE_B_WHEEL = (-0.5, 28.1, -19.4)
CABLE_B_FOOT = (-0.6, 19.4, 28.1)



OUT = Path(__file__).parent / "out"
PETG_SOLID, INFILL = 1.270, 0.60
M2_CLEAR = 1.1

# From the sim, millimetres, relative to the axle.
# The roll drive, drawn for the first time. Layout along the roll axis:
#
#   x = -97.6..-58   roll servo, bolted to the yoke through its CASE screws at
#                    y = +/-10.2 - outside the coupler's radius, which the old
#                    4 bolts at +/-4 were not
#   x = -56.5..-52   coupler disc, bolted to the servo's 25T horn. Belongs to
#                    the ROLL BRACKET: this is what the servo actually turns
#   x = -51..-45     yoke's bearing carrier, taking the radial load off the
#                    servo's own output bearing
#   x = -45..-41     roll bracket's hub
#
# A 3 mm shaft runs -56.5..-41 through all of it. The servo drives one end, the
# yoke bearing supports the middle, the bracket grips both ends.
POST = ((-51, -45), (-6, 6), (-6, 6))          # bearing carrier, on the roll axis
COUPLER_X = (-56.5, -52.0)
COUPLER_R = 9.0
ROLL_SHAFT_X = (-56.5, -41.0)
SERVO_CASE_DY = 10.2          # the STS3215's own case screws
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
# Starts at x = -51, not -56: the roll bracket's coupler disc spins at radius 9
# about the axis over x = -56.5..-52, and the cross used to run straight
# through it - 734 mm3 of a joint that would simply not turn.
YOKE_CROSS = ((-51, -45), (-6, 59), (-8, 8))
YOKE_FWD = ((-56, 0), (56, 66), (-8, 8))
YOKE_BOSS_Y = (56.0, 66.0)
ROLL_SV = ((-93.4, -58.0), (-12.35, 12.35), (-6.6, 38.6))
# The arm reaches 4 mm further aft and the tie is deeper and wider, so the two
# actually LAP instead of grazing. They used to overlap in a box 2.00 x 0.50 x
# 2.65 mm - 2.65 mm3 - and the entire wheel load went through it. FEA put the
# part at 1068% of PETG's allowable, and at 111% in aluminium: no material
# fixes a 0.5 mm neck. The lap is now about 200 mm3, 75x more, plus a rib.
# z starts at 12.40, which is EXACTLY the wheel servo's face (the case is
# 24.8 deep in z, so half of it is 12.4). The arm seats flat on the servo, the
# way the other fourteen bolted faces in this robot do.
#
# It was 12.9. That came from clearing a 0.05 mm interference by moving 0.55,
# which left the servo floating 0.5 mm off the bracket it bolts to - visible to
# nobody, because assemble_check only tested for interference and 0.5 mm of air
# reads exactly like 20 mm of air. A servo on a 0.5 mm standoff puts its
# reaction couple into the bolts in BENDING with no friction preload, and this
# design has already been bitten by that once: the ankle servo's bolts were
# 12 mm apart, which turned its 1.63 N.m into 136 N per bolt.
#
# y starts at 12.9, not 12.1. The wheel face is at 12.0 and the arm plate runs
# parallel to it across the whole face, from the hub out to the rim, so 12.1
# was a 0.1 mm RUNNING clearance against a part that turns at 40 rad/s. That is
# a drawing clearance, not a manufacturing one: JLCPCB quote +/-0.3 mm on MJF
# and PCBWay the same on SLS, so a 0.1 mm gap is inside the tolerance band of
# both parts and the wheel would have rubbed on a good fraction of builds.
# Moved out 0.8 mm, keeping the plate thickness, so the gap is 0.9 mm and
# survives a worst-case stack on both sides.
ROLL_ARM = ((-44, 0), (12.9, 24.3), (12.4, 22.9))
ROLL_TIE = ((-56, -42), (7.1, 14.5), (5, 16.0))
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

    # Roll bearing seat and the shaft bore through it.
    part -= bd.Pos(-48, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(ROLL_SHAFT_R, 40)
    part -= (bd.Pos(-49, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ROLL_BEARING_OD, ROLL_BEARING_W))

    # Face the roll servo bolts to. It is a RING, not a plate: the bracket's
    # coupler disc turns inside it at radius 9, so a solid face would be hit by
    # the thing it is meant to hold still. Bolts go through the servo's own
    # case screws at y = +/-10.2, which is outside that radius.
    # Upper half only. In foot mode the floor is 12 mm below the axle, and a
    # symmetric ring puts its lower lugs 5 mm THROUGH it - which the
    # floor-clearance test caught and no interference check ever would. So the
    # servo is held from above and the sides, using the two case screws at
    # z = +5 rather than all four.
    part += (_box(((-58, -51), (-17, 17), (2, 17)))
             - bd.Pos(-56, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(COUPLER_R + 1.0, 20))
    for dy in (-SERVO_CASE_DY, SERVO_CASE_DY):
        for dz in (-10.2, 10.2):
            part -= (bd.Pos(-56, dy, dz) * bd.Rot(0, 90, 0)
                     * bd.Cylinder(M2_CLEAR, 20))
    # Tie the ring back to the bearing carrier, clear of the coupler. Upper
    # only, for the same reason.
    part += _box(((-52, -45), (-6, 6), (6, 16)))
    # Reserve the roll-to-wheel cable, in both poses.
    from cad.wiring import tube as _tube
    part -= _tube(CABLE_A, CABLE_B_WHEEL, r=CABLE_CH_R)
    part -= _tube(CABLE_A, CABLE_B_FOOT, r=CABLE_CH_R)

    part = part.clean()
    from cad.shape import long_edges, soften
    # x-edges only. The yoke's y-edges refuse at every radius OCC will try,
    # and so do all four of the roll bracket's - that part already carries the
    # one fillet it needs, on the re-entrant corner the FEA found hottest.
    part, yoke.corner_r = soften(part, long_edges(part, "x", 30.0),
                                 what="yoke edges")
    return part


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

    # Coupler disc: bolts to the servo's 25T horn, and is what the servo
    # actually turns. Joined to the hub by the 3 mm shaft, which is a bought
    # rod running the whole length.
    cx0, cx1 = COUPLER_X
    part += (bd.Pos((cx0 + cx1) / 2, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(COUPLER_R, cx1 - cx0))
    from cad.servo import HORN_DX, HORN_DY, HORN_SCREW_R
    for dy in (-HORN_DX, HORN_DX):
        for dz in (-HORN_DY, HORN_DY):
            part -= (bd.Pos((cx0 + cx1) / 2, dy, dz) * bd.Rot(0, 90, 0)
                     * bd.Cylinder(HORN_SCREW_R, 3 * (cx1 - cx0)))

    # Roll shaft bore, on the axis, through hub and coupler alike.
    part -= (bd.Pos(-48, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ROLL_SHAFT_R, 60))

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
