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
CABLE_CH_R = 3.0
# Two paths, not one: the run pivots slightly at the knee end between wheel
# mode and foot mode, and the channel has to hold the cable in BOTH poses.
# Cutting only the wheel-mode line left 4 mm3 of foot-mode interference -
# small, but foot mode is the pose the robot stands in.
CABLE_A = (-4.8, 0.4, -5.0)
CABLE_A_FOOT = (-6.0, 0.4, -3.5)
CABLE_B = (-0.6, 48.6, -52.6)

# The OUTGOING lead, ankle servo to roll servo. It leaves the port at CABLE_B
# and drops away aft and down, and until the cradle went in it left through open
# air. The +x wall now stands in it, by 7 mm3. Same catch as on the yoke: fitcheck
# and assemble_check were both clean, because neither of them models a cable.
# The far end is on the yoke, which moves between the poses, so two paths again.
CABLE_C_WHEEL = (-76.06, 5.2, -89.96)
CABLE_C_FOOT = (-72.64, 5.2, -79.84)



OUT = Path(__file__).parent / "out"

PETG_SOLID = 1.270            # g/cm3
# A print is not solid plastic. 4 perimeters and ~40% gyroid on a part this
# size lands near 60% of solid. Measure a real print and correct this.
INFILL = 0.60

# --- geometry, straight from the sim -----------------------------------------
SPINE_Y, SPY = 21.0, 6.0      # spine centre-line and half-thickness
ANKLE_Z = -110.0

# The ankle-pitch bearing sits at (0, PITCH_Y, ANKLE_Z) - ON the axis, which
# the old bore was 52.8 mm away from. `cad/envelope.py` solves for where it is
# allowed to be, and there are exactly two bands: y = -80..-47 inboard and
# y = +56..+80 outboard. Everything between is inside the wheel in one of its
# two shapes, or inside the wheel-drive servo.
#
# Outboard wins. The leg already reaches y = 69 at the ankle-pitch servo, so
# this costs no width at all, and inboard would put the two legs' bearings
# 6 mm apart across the centreline.
# y = 68, not 67, and the band starts at 57 not 56: the wheel-drive servo
# TURNS with the roll bracket, and its far corner swings out to 50.9 mm,
# 1.5 mm past its static face. Treating it as a fixed box put this boss where
# fitcheck found it being clipped 0.9 mm at 16% of the flip.
PITCH_Y = 71.0                # bearing centre
PITCH_HALF = 5.0              # bearing block half-width in y

SPINE = ((-10, 10), (SPINE_Y - SPY, SPINE_Y + SPY), (-49, 0))

# The route to the bearing goes FORWARD, and both halves of that are forced.
#
# It cannot go down the middle: the wheel-drive servo turns with the roll
# bracket, so it sweeps an annulus 14..50.9 mm about the roll axis for
# |x| < 22.6, and that swallows everything below z = -55 near the centreline.
# So the shin has to leave at |x| > 22.6 - and further than that, because the
# servo PITCHES too: its corner swings out to sqrt(22.6^2 + 12.35^2) = 25.8 mm.
# Sweeping only roll put these members at x = 23 and fitcheck found them
# clipped 2.4 mm at 64% of the flip.
#
# It cannot go AFT either, which is what the first two attempts did. In foot
# mode the axle is 12 mm off the floor and the shin stands at 27.5 deg, so
# structure hanging aft at axle height swings down: the aft version ended up
# 13 mm THROUGH the floor. Forward, the same tilt lifts it.
FARM = ((10, 47), (SPINE_Y - SPY, SPINE_Y + SPY), (-49, -35))
FPOST = ((37, 47), (SPINE_Y - SPY, SPINE_Y + SPY), (-70, -48))
# 9 mm deep, and it cannot be more. Deepening it to 15 buys 116% -> 107% of
# PETG's allowable and then collides with the wheel-drive servo 5.3 mm deep at
# 64% of the flip: the servo sweeps that space and there is nowhere to put the
# extra section. The rib below is what carries this member instead.
CROSS = ((37, 47), (SPINE_Y - SPY, PITCH_Y + PITCH_HALF), (-70, -61))
DROP = ((37, 47), (PITCH_Y - PITCH_HALF, PITCH_Y + PITCH_HALF), (-114, -61))
BACK = ((0, 47), (PITCH_Y - PITCH_HALF, PITCH_Y + PITCH_HALF), (-114, -102))
# --- the ankle-pitch servo mount ----------------------------------------------
#
# THE OLD COMMENT HERE WAS WRONG, AND IT MATTERED. It read "runs the length of
# the ankle servo's case so its mounting bolts can be far apart: 1.63 N.m
# through bolts 12 mm apart is 136 N each, at 40 mm it is 41." The bolts were at
# z = -60 and -20. The servo case runs -56.5..-11.1 (its shaft is at -46.32,
# from cad/belt.py, inset 10.2). So the z = -60 bolt was 3.5 mm off the END of
# the servo, and it missed the standoff too, which only started at -56.
#
# It cut nothing. The shin has had ONE ankle-servo bolt, not two, and the whole
# 41 N argument above was computed for a pair that does not exist.
#
# Both are gone now anyway: the case holes are abandoned robot-wide because
# nobody knows where they are (cad/servo.py). What replaces them is a rim.
from cad.servo import WIDTH as SERVO_W  # noqa: E402
CRADLE_CLEAR = 0.4            # per side, and it must EXCEED the print tolerance
CRADLE_WALL = 2.5
CRADLE_DEPTH = 10.0           # how far the rim reaches along the shaft
CRADLE_X = SERVO_W / 2 + CRADLE_CLEAR + CRADLE_WALL      # 15.3
# The ankle servo's case, from the sim's vanksv_l geom: z = -33.2 +/- 22.7.
SERVO_Z = (-55.9, -10.5)

# SIDE WALLS ONLY, NO LOWER END WALL, and that is forced rather than lazy.
# The wheel-drive servo sweeps an annulus 14..50.9 mm about the roll axis at
# z = -110 for |x| < 22.6. The servo's own lower end sits at z = -56.5, which is
# 53.5 mm from that axis and clear. An end wall 2.9 mm below it would sit at
# 50.6 mm, INSIDE the swept annulus, and would be clipped during the flip.
#
# Two opposed walls still do the job. The servo's reaction is a couple about its
# own shaft, which lies along y here, so it is reacted by forces in x - which is
# exactly what a pair of walls 25.6 mm apart provides. The end walls were never
# the ones carrying it.
STANDOFF = ((-CRADLE_X, CRADLE_X), (SPINE_Y + SPY, SPINE_Y + SPY + 7),
            (-56, -12))

# --- fasteners ---------------------------------------------------------------
HUB_R = 13.0                  # hub outer radius
HORN_BORE = 4.0               # radius, clearance over the servo output boss
# The real horn pattern, from cad/servo.py: a 9.9 x 10.0 mm rectangle. It was
# a bolt circle at r = 8, so every hole was 0.71 mm out of position.
from cad.servo import HORN_DX, HORN_DY, HORN_SCREW_R
BEARING_OD = 5.0              # radius, 623ZZ outer race (10 mm dia)
BEARING_W = 4.0               # 623ZZ width
SHAFT_R = 2.0                 # radius, clearance for the 3 mm shaft


# Ribs at the two inside corners, in the x-z plane and across the full 12 mm
# width, so they lie IN the layer plane where the material is strong. The shin
# passed at 89% of PETG's allowable, which is passing with nothing to spare;
# a triangle at a corner is the cheapest way to buy that back, because bending
# stiffness goes as depth cubed and a rib is all depth.
RIB_SPINE = ((10, -35), (10, -18), (41, -35))      # spine to forward arm
# The two hottest places on the part, both re-entrant corners, found by asking
# the FEA rather than by eye: 80 MPa where the outboard crossing meets the
# descent, and 68 MPa where the forward column meets the crossing. Triangles
# in the y-z plane, across the full 10 mm width of those members.
# One rib, running the full width to the descent, so it ends ON something.
RIB_CROSS = ((27, -61), (27, -45), (66, -61))     # column to crossing
RIB_X = (37.0, 47.0)
# (the arm-to-post rib went with the post's lower half)


def _gusset_yz(pts_yz, x0, t):
    """Triangular rib in the y-z plane, spanning x0 to x0 + t. Same
    measure-don't-trust-the-normal treatment as _gusset."""
    g = bd.extrude(bd.Plane.YZ * bd.Polygon(*pts_yz), amount=t)
    return bd.Pos(x0 - g.bounding_box().min.X, 0, 0) * g


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


def _box(spec):
    (x0, x1), (y0, y1), (z0, z1) = spec
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def build():
    part = (_box(SPINE) + _box(STANDOFF) + _box(FARM) + _box(FPOST)
            + _box(CROSS) + _box(DROP) + _box(BACK))
    part += _gusset(RIB_SPINE, SPINE_Y - SPY, 2 * SPY)
    part += _gusset_yz(RIB_CROSS, RIB_X[0], RIB_X[1] - RIB_X[0])

    # Hub at the knee, bolted to the servo horn. It must be centred ON the
    # knee axis at z = 0, or the part does not pivot about the joint - it was
    # 14 mm below it, which no amount of stress margin would have saved.
    part += bd.Pos(0, SPINE_Y, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(HUB_R, 12)
    part -= bd.Pos(0, SPINE_Y, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(HORN_BORE, 14)
    for dx in (-HORN_DX, HORN_DX):
        for dz in (-HORN_DY, HORN_DY):
            part -= (bd.Pos(dx, SPINE_Y, dz) * bd.Rot(90, 0, 0)
                     * bd.Cylinder(HORN_SCREW_R, 20))

    # Ankle pitch bearing: a 623ZZ pressed into a counterbore, with a
    # clearance hole through for the shaft. A plain hole would run the shaft
    # straight against printed plastic.
    #
    # ON the axis this time. The old one was at (-48, 21, -88), which is
    # 52.8 mm from the line the sim rotates this joint about, so the part
    # could not have pivoted about its own joint - the same fault as the knee
    # hub, and a bigger instance of it.
    part -= (bd.Pos(0, PITCH_Y, ANKLE_Z) * bd.Rot(90, 0, 0)
             * bd.Cylinder(SHAFT_R, 4 * PITCH_HALF))
    part -= (bd.Pos(0, PITCH_Y - PITCH_HALF + BEARING_W / 2, ANKLE_Z)
             * bd.Rot(90, 0, 0) * bd.Cylinder(BEARING_OD, BEARING_W))

    # Ankle-servo capture. Two walls gripping the case sides, standing off the
    # standoff face, positioned from the SERVO rather than from a guess: its
    # shaft is on the belt centre distance and the case hangs off that.
    # Positioned from the SIM's box, not from the belt centre distance. Working
    # it out as SERVO_SHAFT_Z - SHAFT_INSET gives -56.52..-11.12; the sim's
    # vanksv_l geom is at z = -33.2 +/- 22.7, so -55.9..-10.5. The 0.62 mm
    # between those two is not academic: it put the upper end wall 0.22 mm
    # inside the servo, which assemble_check caught as 57.5 mm3. The sim is what
    # the interference checks measure against, so the sim is what the part is
    # built from.
    sz0, sz1 = SERVO_Z
    face_y = SPINE_Y + SPY + 7.0
    xi = SERVO_W / 2 + CRADLE_CLEAR
    for sgn in (-1, 1):
        part += _box(((min(sgn * xi, sgn * CRADLE_X),
                       max(sgn * xi, sgn * CRADLE_X)),
                      (face_y, face_y + CRADLE_DEPTH),
                      (sz0 - CRADLE_CLEAR, sz1 + CRADLE_CLEAR + CRADLE_WALL)))
    # Upper end wall. This end is 98 mm from the roll axis, so unlike the lower
    # one it is nowhere near the wheel-drive servo's swept annulus.
    part += _box(((-CRADLE_X, CRADLE_X),
                  (face_y, face_y + CRADLE_DEPTH),
                  (sz1 + CRADLE_CLEAR, sz1 + CRADLE_CLEAR + CRADLE_WALL)))

    # No lightening pocket. The first attempt cut a 10 mm slot clean through
    # a 20 mm spine, leaving a thin-necked keyhole, and the 0.73 mm stiffness
    # figure in docs/bom.md assumes a SOLID section. It saved about 2 g on a
    # 17 g part - not a trade worth making.

    # Reserve the cable run, in both poses. See CABLE_CH_R.
    from cad.wiring import tube as _tube
    part -= _tube(CABLE_A, CABLE_B, r=CABLE_CH_R)
    part -= _tube(CABLE_A_FOOT, CABLE_B, r=CABLE_CH_R)
    # And the outgoing lead, through the cradle wall that now stands in it.
    part -= _tube(CABLE_B, CABLE_C_WHEEL, r=CABLE_CH_R)
    part -= _tube(CABLE_B, CABLE_C_FOOT, r=CABLE_CH_R)

    part = part.clean()
    from cad.shape import long_edges, soften
    part, build.corner_r = soften(part, long_edges(part, "z", 40.0),
                                  radii=(1.5, 1.0, 0.8), what="shin verticals")

    # RELIEF, CUT AFTER THE FILLET, and it has to be after or it does nothing.
    #
    # Widening the standoff gave soften() new long z-edges to work on, and the
    # r = 1.0 blend where a cradle wall rises off the seating face is CONCAVE:
    # it adds material, 0.2 mm of it, straight into the space the servo has to
    # sit in. assemble_check found 3.4 mm3. Nobody would have seen it on a
    # drawing, and the result at the bench is a servo rocking on a fillet
    # instead of seating on a face - the same fault as the 0.5 mm standoff this
    # design has already been bitten by once, minus the bolts that used to
    # tighten it out.
    #
    # So the pocket is defined by the SERVO rather than by whatever the fillet
    # leaves behind: subtract the case, grown by the clearance in x and z, and
    # open-ended in +y so the seating face itself survives.
    sz0, sz1 = SERVO_Z
    part -= _box(((-SERVO_W / 2 - CRADLE_CLEAR, SERVO_W / 2 + CRADLE_CLEAR),
                  (SPINE_Y + SPY + 7.0, SPINE_Y + SPY + 7.0 + 2 * CRADLE_DEPTH),
                  (sz0 - CRADLE_CLEAR, sz1 + CRADLE_CLEAR)))
    return part


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
