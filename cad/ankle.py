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
# x moved -0.5 -> -13.05 when the wheel servo was put on its own axis. Its
# case had been centred on the wheel axis, which left the output shaft 12.5 mm
# off the joint it drives (cad/drives.py), and the connector port moved with the
# case. The old channel then ran 38 mm3 of yoke straight through the cable.

# The ankle-servo lead, coming down the shin into the roll servo. It never
# needed a channel before, because before there was nothing here: it crossed
# open air where the cradle's +y wall now stands, and cutting that wall put
# 62 mm3 of yoke through the middle of it.
#
# Worth noting how it was caught. Not by fitcheck, not by assemble_check, both
# of which were clean - a cable is not a solid and neither of them models one.
# It was the wiring test, which asserts that the ONLY blocked run left in this
# robot is the roll-to-wheel one, by exactly the wheel envelope and nothing
# else. A test written to pin a known limit is what noticed a new one.
#
# Two paths again, for the same reason as everything else in this file: the
# yoke drops 28 mm in z between the modes, so the run's far end moves and a
# channel cut for one pose is the wrong channel half the time.



OUT = Path(__file__).parent / "out"
from cad.material import DENSITY as PRINT_DENSITY, INFILL  # PA6-CF, one source

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
from cad.servo import HORN_PROUD as _HORN_PROUD  # noqa: E402
ROLL_SV_X1 = -58.0          # the roll servo's output end; ROLL_SV asserts this

# SHORTENED from (-51, -45) to free 1.5 mm for the roll bracket's web.
#
# The post's job is to house the 623ZZ, which is 4.0 mm wide and sits at
# x = -49, so it needs -51..-47 and had -51..-45. The spare 1.5 mm was doing
# nothing, and the web next door was the most loaded part in the robot at 99%
# of PA6-CF. Measured: 4.0 mm web is 56.6 MPa, 5.5 mm is 31.4. The window
# between this post and the wheel at x = -40 is the whole budget.
POST = ((-51, -46.5), (-6, 6), (-6, 6))       # bearing carrier, on the roll axis
# The inner face is DERIVED from the horn, not typed. It read -56.5, which left
# the coupler standing 0.40 mm off the horn it bolts to - measured 2026-08-02 by
# cad.hardware.horn_joints(). The horn is 4.50 thick over a spline that stands
# 3.40 proud, so its outer face is HORN_PROUD past the servo's box end, and the
# box end here is ROLL_SV's near face.
COUPLER_X = (ROLL_SV_X1 + _HORN_PROUD, -52.0)
COUPLER_R = 9.0
ROLL_SHAFT_X = (-56.5, -41.0)   # between servo (ends -58) and the flat wheel,
                                # which needs |x| > 40

# Capturing the roll servo instead of bolting into its case. See build().
from cad.servo_dims import CRADLE_CLEAR  # noqa: E402  one copy, see there
CRADLE_WALL = 2.5
CRADLE_DEPTH = 10.0           # how far the rim reaches along the shaft
# How far up the walls run. They are carried by the ring, which stops at z = 17,
# so going much past that is a cantilever holding nothing. The grip only has to
# react a couple, and 15 mm of contact each side at a 25.5 mm arm does that.
CRADLE_Z_TOP = 19.0

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
# x ends at -46.5, not -45, for the same reason POST was shortened: the roll
# bracket's web moved out to -46.5 to get its section, and this bar was still
# reaching to -45. That put 477 mm3 of yoke inside the bracket, which
# assemble_check caught the moment the web moved.
YOKE_CROSS = ((-51, -46.5), (-6, 59), (-8, 8))
YOKE_FWD = ((-56, 0), (56, 66), (-8, 8))
YOKE_BOSS_Y = (56.0, 66.0)
# THIS BOX WAS WRONG, AND NOTHING HAD EVER CHECKED IT. It read
# ((-93.4, -58.0), (-12.35, 12.35), (-6.6, 38.6)): the old 35.4 x 24.7 x 45.2
# guesses, and 3.4 mm out of position in z. The sim's own vrollsv_l geom is
# 39.6 x 24.8 x 45.4 at (-77.8, 0, 12.5) in this frame, which is the box below.
#
# It only surfaced because the cradle is the first feature ever positioned FROM
# this constant rather than merely near it. assemble_check put 491 mm3 of new
# wall inside the servo, which is the check earning its keep: a box that is
# 3.4 mm off but never used for anything is invisible.
#
# AND THEN IT WAS WRONG AGAIN, 2026-08-02, for the same reason one layer up: it
# was rewritten to the SIM's 39.6 x 24.8 x 45.4, and the manufacturer's drawing
# had already superseded that with 36.5 x 24.73 x 45.23. 3.1 mm in x, plus
# 0.035 in y and 0.085 in z. The two that bit are the small ones, because they
# are on the faces the cradle is built from: the walls stood 0.035 mm off the
# case they are supposed to grip, which is the same standoff fault this design
# has now been bitten by at 0.5 mm, 0.035 mm and 0.005 mm.
#
# So it is no longer written down here at all. This IS envelope.ROLL_SERVO -
# the same servo, the same frame, stated twice - and cad.twin.bought() now
# fails if either drifts from the sim. One box, one place.
from cad.envelope import ROLL_SERVO as ROLL_SV  # noqa: E402
# The wheel-drive servo, in this same frame, for the cradle at the end of
# roll_bracket(). Imported for the same reason: it is one box and this is not
# where it lives.
from cad.envelope import WHEEL_SERVO as WHEEL_SV  # noqa: E402
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
# z0 is the wheel servo's own face, DERIVED. It was the literal 12.4, written
# when the case was believed to be 24.8 across; the drawing says 24.73, so the
# arm stood 0.035 mm off the servo it seats on. Tiny, and it is exactly the
# fault recorded above in a bigger size - a servo on a standoff puts its
# reaction couple into the joint in bending with no preload - and it was found
# by assemble_check's seated-face count dropping from 16 to 14, not by anything
# looking for it.
from cad.servo_dims import WIDTH as _SERVO_W  # noqa: E402
ROLL_ARM = ((-44, 0), (12.9, 24.3), (_SERVO_W / 2, 22.9))
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
# WIDENED to 5.5 mm, which took this part from 99% of PA6-CF to 55%. The plate
# carries the whole wheel load in bending and stress goes as 1/thickness, so
# the last 1.5 mm was worth 44 points of margin. Bounded by POST above and by
# the flat wheel at |x| = 40 below; the 1.0 mm to the wheel is the running
# clearance cad/assemble_check.py checks.
ROLL_WEB_X = (-46.5, -41.0)
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

    # Retain the ankle-pitch shaft: two M2.5 set screws in heat-set inserts,
    # radial from the boss's EXPOSED +x hemisphere (the -x half is buried in
    # YOKE_FWD) onto the 3 mm shaft. Two along the boss lock it against spinning.
    # One 3.5 mm bore each, boss surface (x = 7) in to the shaft AXIS (x = 0):
    # the insert seats in the outer 4 mm, the set-screw tip runs the rest onto
    # the shaft. It runs to 0, not to the shaft bore's edge at x = 2, because
    # stopping tangent to that r = 2 bore left the mesh non-manifold (2 open
    # edges); reaching past it merges the two cleanly. The mouth is tangent to
    # the r = 7 cylinder, so the insert is fully walled from x = 6.8 inward.
    # cad/fasteners.inserts_seated wanted these; it read yoke 2/0.
    from cad.fasteners import M25_INSERT_R
    for sy in (59.0, 63.0):
        part -= (bd.Pos(3.5, sy, 0) * bd.Rot(0, 90, 0)
                 * bd.Cylinder(M25_INSERT_R, 7.0))

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
    # CAPTURE, NOT BOLTS. The four holes that were here cut two: dz = -10.2 fell
    # below the ring, which only starts at z = 2, so it removed nothing. Same
    # shape of fault as the shin's missing ankle bolt. All of them are gone now
    # because the case hole positions are not published by anyone - see
    # cad/servo.py - so a rim grips the case instead.
    #
    # TWO WALLS, AND THE BOTTOM STAYS OPEN. The first attempt closed it into a
    # channel, which is the better section, and the floor killed it: with the
    # servo's real z range the bottom wall lands at -13.1, and in foot mode the
    # floor is 12 mm below the axle. It would have printed 1.1 mm underground.
    # Same constraint that made the ring upper-half only, one step further out.
    #
    # Two opposed walls are enough on their own. The servo's reaction is a
    # couple about its own shaft, which lies along x here, so it is reacted by
    # forces in the y-z plane, and 25.5 mm of separation in y provides them.
    #
    # ONE WALL IS THE DATUM, THE OTHER KEEPS ITS CLEARANCE. With CRADLE_CLEAR on
    # BOTH walls the servo dropped in and located nothing - it floated 0.4 mm
    # off the yoke on every face, which cad/floating.py found (status/2026-08-04)
    # and cad/fasteners.capture() had been reporting for weeks as 1.3 deg. The
    # perpendicular seats the thigh and roll bracket use are shut here: the
    # bottom face is the one the floor rules out (above), and the shaft face is
    # buried in the servo's own interference envelope, so a ring on it reads as
    # 828 mm3 of overlap in cad/assemble_check even though the real spline is
    # round and clears. That leaves the +y wall as the datum: flush to the case,
    # so the servo registers against it, while -y keeps CRADLE_CLEAR so it still
    # drops in. A flush wall is not a press fit - the servo slides in along its
    # shaft (x), not between the walls - so print tolerance does not jam it.
    (sv_x, sv_y, sv_z) = ROLL_SV
    cx = (sv_x[1] - CRADLE_DEPTH, sv_x[1])
    cz = (sv_z[0] - CRADLE_CLEAR, CRADLE_Z_TOP)
    for sgn, clear in ((+1, 0.0), (-1, CRADLE_CLEAR)):
        yi = sv_y[1] + clear
        yo = sv_y[1] + CRADLE_CLEAR + CRADLE_WALL
        part += _box((cx, tuple(sorted((sgn * yi, sgn * yo))), cz))
    # Tie the ring back to the bearing carrier, clear of the coupler. Upper
    # only, for the same reason.
    # Ends at -46.5 with POST and YOKE_CROSS, not -45. Three separate boxes
    # referenced the old web face and only two were moved with it; this one was
    # 45 mm3 inside the bracket until assemble_check said so.
    part += _box(((-52, -46.5), (-6, 6), (6, 16)))
    # Reserve the roll-to-wheel cable, in both poses.
    from cad.wiring import channel as _channel
    part = _channel(part, "vrollsv_l -> vwhlsv_l", "ankle_l", r=CABLE_CH_R)
    part = _channel(part, "vanksv_l -> vrollsv_l", "ankle_l", r=CABLE_CH_R)

    part = part.clean()
    from cad.shape import long_edges, soften
    # x-edges only. The yoke's y-edges refuse at every radius OCC will try,
    # and so do all four of the roll bracket's - that part already carries the
    # one fillet it needs, on the re-entrant corner the FEA found hottest.
    part, yoke.corner_r = soften(part, long_edges(part, "x", 30.0),
                                 what="yoke edges")
    # STAGE 2 OF THE PARALLELOGRAM ENDS HERE. The arm is this yoke's forward
    # member carried 30 mm further on, at the same y band and the same z, and
    # it is part of the yoke rather than a separate solid for the reason
    # cad/linkage_mounts.py gives: a pin fixed to this body needs something on
    # this body to press into.
    from cad.linkage_mounts import yoke_arm
    part += yoke_arm()

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
    # THE WHEEL SERVO'S TWO MOUNTING BOLTS ARE GONE, and removing them changes
    # nothing, which is the finding. They ran along Y at z = 17.65. The arm
    # seats on the servo's face at z = 12.40 and the case runs DOWN from there,
    # so those holes were parallel to the joint they were meant to clamp and
    # never entered the servo at all. They drilled the arm and stopped.
    #
    # That is the third mount in this robot with case screws that cut nothing:
    # the shin's lower ankle bolt missed the case end by 3.5 mm, two of the
    # yoke's four fell below the ring. cad/fasteners.py did not catch any of
    # them because it checks that a hole is PARALLEL to its servo's shaft, not
    # that it arrives at the servo.
    #
    # THE FIFTH CRADLE. It was left out on 2026-08-01 because the arm's frame
    # and envelope.WHEEL_SERVO's frame disagreed about where the case sits, and
    # building against a box that is wrong is how you get a fifth invented
    # pattern. That box is now derived from servo_dims about SHAFT_X and checked
    # against the sim by cad.twin.bought(), so there is something real to grip.
    #
    # WHY THIS ONE IS A C AND NOT A RIM. The four cradles built yesterday all
    # grew from a plate on the case's END face. This mount is different: the arm
    # lies across the servo's +z FACE, over x = -35.1..0, and that face is
    # already one half of an opposed pair. The couple is about the shaft, which
    # is y here, so it is reacted by forces in x-z - which means a wall under
    # the case at -z completes the pair on its own, with the arm doing the other
    # half. Two opposed faces, exactly the yoke's argument, and no new plate.
    #
    # An x-pair was the other option and it is worse: the aft wall is free, but
    # the forward one would sit at x = +10.1 where the arm ends at x = 0, so it
    # would mean growing the arm 13 mm outboard to have anything to hang from.
    # That is a proportions change to win a joint we can get for a web.
    (wx0, wx1), (wy0, wy1), (wz0, wz1) = WHEEL_SV
    cz0 = wz0 - CRADLE_CLEAR - CRADLE_WALL       # underside of the bottom wall
    cz1 = wz0 - CRADLE_CLEAR                     # the face that grips the case
    cx0 = wx0 - CRADLE_CLEAR - CRADLE_WALL       # outside of the aft web
    # y is clipped to the ARM's own span. The case runs on to y = 50 and the arm
    # stops at 24.3, and a wall reaching past the thing it grows from is the
    # floating solid cad.servo.cradle() spent a day failing to attach.
    cy = (max(wy0, ROLL_ARM[1][0]), ROLL_ARM[1][1])
    part += _box(((cx0, 0.0), cy, (cz0, cz1)))            # under the case
    part += _box(((cx0, wx0 - CRADLE_CLEAR), cy,
                  (cz0, ROLL_ARM[2][1])))                 # aft web, up to the arm

    # The roll-to-wheel lead crosses this part, and the channel for it is cut
    # HERE, last, after every piece of material that could stand in it.
    #
    # It used to be cut before the cradle above, and the cradle promptly filled
    # it back in: 156 mm3 of new wall through the run, 2 of 8 blocked where the
    # robot had been at 0 of 8 in both poses. Same shape of bug as the fillet
    # that ate a servo seat on 2026-08-01 - a feature subtracted before the
    # material that would have obstructed it exists is a feature subtracted from
    # nothing. Cut last, and the order stops mattering.
    from cad.wiring import channel as _channel
    part = _channel(part, "vrollsv_l -> vwhlsv_l", "rollbracket_l", r=CABLE_CH_R)
    return part.clean()


def main(export=True):
    y, r = yoke(), roll_bracket()
    gy = y.volume / 1000.0 * PRINT_DENSITY * INFILL
    gr = r.volume / 1000.0 * PRINT_DENSITY * INFILL
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
