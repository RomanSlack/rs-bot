"""The three linkage features that belong to OTHER parts.

    the fin    is chassis      the stub is shin      the arm is yoke

Split out of cad/linkage.py so those three modules can build them in, which is
what makes them features rather than loose pieces. They could not import
cad/linkage.py to do it: that pulls in cad/hardware.py, which imports cad/ankle
and cad/shin, so ankle importing linkage closes a circle.

WHY THEY ARE HERE AT ALL. They were drawn as separate solids for a day, on the
grounds that folding them in was the next job. It reads fine in a picture and
it is not fine: tests/test_foot.py's `test_every_body_is_one_rigid_piece` came
back with "torso is 3 loose pieces", because the PIN that rod 1 turns on is
fixed to the torso and the fin it presses into was not in the chassis. A robot
whose fin is a separate STEP file is a robot with a 12 g part and nothing to
bolt it to.

Nothing here knows where its host's other features are, by design. Whatever a
host needs to tell them - the chassis its plate position, the shin its face -
comes in as an argument, because the alternative is this file importing the
three modules that import it.
"""

import build123d as bd
import numpy as np

from cad.linkage_dims import (ARM_T, ARM_W, BEARING_ID, BEARING_OD, BEARING_W,
                              EYE_R, FIN_H, FIN_T, IDLER_W, PIN_D, SHIN_FACE_Y,
                              STUB_R, U1, U2, Y_ARM, Y_FIN, Y_IDLER, Y_ROD1,
                              Y_ROD2, pin_span)



def _pin_bore(x, z, carrier_y, rod_y, seat):
    """The hole a pin is pressed into. Same span as the pin: see
    cad/linkage.py's _pin, which is the part that goes in it."""
    lo, hi = pin_span(carrier_y, rod_y, seat)
    return (bd.Pos(x, (lo + hi) / 2 - carrier_y, z) * bd.Rot(90, 0, 0)
            * bd.Cylinder(PIN_D / 2, hi - lo))




def knee_stub(face_y=SHIN_FACE_Y):
    """The stub axle the idler turns on, coaxial with the knee.

    Drawn because of docs/how-checks-fail.md #13: the parts that do the
    assembling are the ones nobody checks, and an idler with no axle is a bar
    floating on the knee axis in a picture.

    It grows off the SHIN, not the thigh, for the dull reason that the shin is
    what is out there: probing the knee axis for material finds the knee servo
    to y = 15, the shin's hub to y = 32, and air past that. Which of the two
    neighbours carries it does not matter mechanically, because the idler turns
    against both.

    A fat printed boss rather than a 3 mm shaft. It reaches 27 mm past the last
    material and carries both rods' reaction, about 86 N; at 16 mm diameter that
    is 6 MPa and 0.03 mm of deflection, and 0.03 mm at this radius is 0.06 deg
    at the foot. On a 3 mm shaft it would be 30 times that. See error_budget()
    for why tenths of a millimetre out here are the whole argument.

    In the SHIN's frame, origin at the knee axis, which is that frame's origin.
    """
    y1 = Y_IDLER + IDLER_W / 2 + 1.0
    # The fat root stops where the bearing starts; past that it is the 3 mm
    # inner race diameter, or the root bores straight through the race it is
    # supposed to be carrying. That was 368 mm3 of idler.
    y_race = Y_IDLER - IDLER_W / 2 - 0.5
    root = (bd.Pos(0, (face_y + y_race) / 2, 0) * bd.Rot(90, 0, 0)
            * bd.Cylinder(STUB_R, abs(y_race - face_y)))
    shaft = (bd.Pos(0, (face_y + y1) / 2, 0) * bd.Rot(90, 0, 0)
             * bd.Cylinder(BEARING_ID / 2, abs(y1 - face_y)))
    return (root + shaft).clean()


# AT THE PIN'S OWN HEIGHT, not at z = 0. Rooted on the axis the arm's root
# straddles the ankle-pitch SHAFT BORE, which runs along y at x = 0, z = 0: it
# was attached by two 2 mm strips either side of the hole, half of its 48 mm2
# root on air. At z = +6 it is a straight bar and lands on solid yoke.
YOKE_ROOT = np.array([0.0, Y_ARM, float(U2[1]) - 1.0])



def yoke_arm():
    """From the yoke's cross member to the stage-2 pin, in the ANKLE frame.

    Barely a part: it is YOKE_FWD carried 30 mm further forward, at the same
    y band and the same z, ending in an eye. Every version of this that started
    anywhere else was longer and hit something. The wheel occupies everything
    within 41.8 mm of the axle at some roll angle and this whole member sits at
    y = 59, so it is clear of the wheel by inspection; what it is NOT clear of
    by inspection is the wheel-drive servo swinging through the flip, and that
    is what clearance() is for.
    """
    tip = np.array([U2[0], Y_ARM, U2[1]])
    v = tip - YOKE_ROOT
    length = float(np.linalg.norm(v))
    z_dir = bd.Vector(*(v / length))
    x_dir = bd.Vector(0, 1, 0).cross(z_dir)
    pl = bd.Plane(origin=bd.Vector(*((YOKE_ROOT + tip) / 2)),
                  x_dir=x_dir, z_dir=z_dir)
    arm = pl * bd.Box(ARM_T, ARM_W, length)
    arm += (bd.Pos(*tip) * bd.Rot(90, 0, 0) * bd.Cylinder(EYE_R, ARM_W))
    arm -= (bd.Pos(0, tip[1], 0)
            * _pin_bore(float(tip[0]), float(tip[2]), Y_ARM, Y_ROD2,
                        ARM_W / 2 + 1.0))
    return arm.clean()




def fin_z(plate_z0=None):
    """The fin's z centre. One copy, because cad/linkage.py's mounts() has to
    probe the same face this builds."""
    return max(-12.7 if plate_z0 is None else plate_z0,
               float(U1[1]) - FIN_H / 2) + FIN_H / 2


def torso_boss(leg_y, plate_y=None, plate_z0=None):
    """The stage-1 ground pin, on a fin off the OUTBOARD face of the side plate.

    Origin on the hip axis, leg-local y, so it places like every other part
    here. `leg_y` is how far the hip axis is from the robot's centreline, read
    off the sim by placed() rather than written down again.

    THIS IS THE WORST PART OF THE DESIGN and it is worth saying so plainly. The
    linkage lives at y = 52 and the chassis plate ends at y = -20, so this
    reaches 79 mm into free air to hold a pin, and a pin that moves is a foot
    that tilts: about 2 degrees per millimetre. As a 12 mm round boss it would
    deflect 0.7 mm at the factored rod force, which is 1.3 degrees, and the free
    play the whole mechanism is bought for is 0.1.

    So it is a FIN, 25 mm deep in the load direction, not a boss: 10 400 mm4
    against 1 700, which is 0.1 mm and 0.2 degrees. It should really be
    triangulated back to the plate over the plate's own 90 mm of length, and
    that is a chassis change rather than a part, so it is flagged and not taken.
    """
    # The chassis passes its own plate position in. Importing cad.chassis here
    # would close a circle, and it is the chassis that owns that number.
    y0 = (39.6 if plate_y is None else plate_y) - leg_y
    y1 = Y_FIN
    # SITTING ON THE PLATE, not hanging off the bottom of it. Centred on the
    # pin the fin runs z = -18.5 .. 6.5 and the side plate starts at -12.7, so
    # 5.8 mm of its root was over air - 23% of it, which mounts() measures. It
    # is pushed up to start where the plate does; the pin at z = -6 is still
    # comfortably inside.
    fin = (bd.Pos(float(U1[0]), (y0 + y1) / 2, fin_z(plate_z0))
           * bd.Box(FIN_T, abs(y1 - y0), FIN_H))
    fin += (bd.Pos(float(U1[0]), (y0 + y1) / 2, float(U1[1]))
            * bd.Rot(90, 0, 0) * bd.Cylinder(FIN_T / 2, abs(y1 - y0)))
    fin -= (bd.Pos(0, Y_FIN, 0)
            * _pin_bore(float(U1[0]), float(U1[1]), Y_FIN, Y_ROD1, 8.0))
    return fin.clean()
