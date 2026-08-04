"""The parallelogram's geometry, and nothing else.

Split out of cad/linkage.py for the same reason cad/servo_dims.py was split out
of cad/servo.py: so that src/rsbot/model.py can build the robot without pulling
in build123d, which is a dev-group dependency and has no business being required
to load a physics model.

It was required, briefly. Putting the linkage into the sim as real bodies meant
the model needed to know where a rod's pin sits and what its hinge is coupled
to, and the first version imported cad.linkage for it - which imports build123d
at module scope, so `load()` stopped working without the CAD kernel installed.
Nothing failed loudly; it just quietly moved a dev dependency into the physics.

These numbers are the ONLY copy. Import them; do not retype them.
"""

# Crank radius: every pin's offset from the axis it works about. It sets the rod
# force (torque / R), the swept width, and how much an error at a pin is worth
# at the foot, which goes as 1/R. Clearance chose it and not strength: 34 mm
# grazes the shin, 26 works, 30 is the middle of the window.
R = 30.0
DZ = 6.0                      # the idler's two pins, either side of the offset

# The stack across y, leg-local, outboard positive. Each part gets its own slice
# because at a pivot two parts meet, and two parts in one y band are not a
# joint, they are the same lump of plastic. See cad/linkage.py for what sets
# each one; the short version is the wheel at one end and the shin's ankle
# carrier at the other.
Y_FIN = 31.0
Y_ROD1 = 38.0
Y_IDLER = 48.0
Y_ROD2 = 54.5
Y_ARM = 61.0

# Both offsets forward, split in z. Stage 1 sits low because the thigh leans
# back and stage 2 sits high because the shin leans forward, which keeps both
# transmission angles above 69 degrees rather than one of them at 57. The other
# way round would do that too and is wrong for two other reasons: it crosses the
# rods below the knee, and it puts stage 2's pin BELOW an axle that is 12 mm off
# the floor. Given as (x, z) in the TORSO frame, the only frame they are
# constant in.
U1 = (+R, -DZ)
U2 = (+R, +DZ)

# --- sections ----------------------------------------------------------------
#
# The rods are two-force members at 39 N, sized by buckling and not by stress,
# and by a long way: 18x. 8 x 6 is the smallest section that still prints with
# four perimeters and does not look like a wire.
ROD_W = 6.0                   # along y
ROD_T = 8.0                   # in the plane of motion
PIN_D = 3.0                   # the 3 mm shaft everything else in this robot uses
EYE_R = 5.0                   # rod eye outer radius, 3.5 mm of wall on the pin
IDLER_T = 8.0                 # the idler bar's in-plane depth
IDLER_W = 6.0
IDLER_HUB_R = 8.0             # over a 623ZZ, 10 mm outside
# 6 and not 8, and the root's height is what sets it. YOKE_FWD spans z = -8..8
# and the ankle-pitch shaft bore runs along y at z = 0 with r = 2, so the arm
# has to fit between +2 and +8: six millimetres, no more. At 8 it either
# straddled the bore (half its root on air) or overhung the yoke's top face by
# 2 mm. The cost is section - 39 N at 30 mm through 6 x 6 is 32 MPa, 57% of
# PA6-CF, where 8 x 6 would have been 32% - and it is the honest trade, because
# the alternative was 25% of the root sitting on nothing.
ARM_T = 6.0
ARM_W = 6.0
# The chassis fin. FIN_H is the dimension that matters: it reaches 79 mm off the
# side plate to hold stage 1's ground pin, and a pin that moves is a foot that
# tilts at 2 deg/mm, so it is a deep fin and not a round boss.
FIN_T = 8.0                   # thickness in x
FIN_H = 25.0                  # depth in z

# name, offset, the CARRIER's plane, the ROD's plane, how deep it seats, host.
# Both ends of every pin are DERIVED from those two planes: sized by hand they
# engaged 3 mm of a 6 mm idler and none at all of the fin.
#
# THE IDLER'S TWO ARE RECESSED, 0.5 mm short of its faces rather than 1 mm
# proud of them. Proud, pin 1's far end reached y = 52 and rod 2's plane starts
# at 51.5, so it pushed 3.5 mm3 into a rod that TURNS on it - a running pair
# needing the 0.8 mm assemble_check enforces, not an interference. Recessed it
# clears by 1.0 and still engages 5.5 mm of the idler's 6. The fin and the arm
# stay proud: there is nothing outboard of either to hit.
PINS = (("fin", U1, Y_FIN, Y_ROD1, 8.0, "thigh"),
        ("idler1", U1, Y_IDLER, Y_ROD1, IDLER_W / 2 - 0.5, "idler"),
        ("idler2", U2, Y_IDLER, Y_ROD2, IDLER_W / 2 - 0.5, "idler"),
        ("arm", U2, Y_ARM, Y_ROD2, ARM_W / 2 + 1.0, "ankle"))

BEARING_OD, BEARING_ID, BEARING_W = 10.0, 3.0, 4.0     # 623ZZ, the idler's

# The shin's outboard face at the knee axis, leg-local: the HUB's, which is
# cad/shin.py's SPINE_Y + SPY = 27.
#
# IT WAS 32, AND 32 IS THE WRONG FACE. It came from probing the knee axis with
# a radius-5 cylinder against the assembled parts and reading off where the
# shin stopped blocking - and what was blocking at 27..32 was the ANKLE SERVO'S
# CRADLE WALL, four millimetres off the axis, not the hub. Measuring something
# adjacent to the thing you want is docs/how-checks-fail.md #14.
#
# What it cost: the stub axle landed 5 mm clear of the hub and touched the shin
# only through that cradle wall, by THIRTEEN cubic millimetres. The part fused,
# every connectivity check said one solid, and it would have stayed true right
# up until the cradle came out - which is already on the list, because it holds
# a servo that no longer exists. A joint held together by a part scheduled for
# deletion.
SHIN_FACE_Y = 27.0
STUB_R = 8.0                  # 16 mm boss, sized by stiffness not strength


def stub_span():
    """(near, far) in leg-local y for the idler's stub axle on the shin."""
    return SHIN_FACE_Y, Y_IDLER + IDLER_W / 2 + 1.0


def pin_span(carrier_y, rod_y, seat):
    """(near, far) in leg-local y: `seat` into the carrier, 2 mm past the rod."""
    out = 1.0 if rod_y > carrier_y else -1.0
    a, b = carrier_y - out * seat, rod_y + out * (ROD_W / 2 + 2.0)
    return min(a, b), max(a, b)


def sim_bought(leg_y, sgn):
    """[(name, host body, pos, half-length, radius)] for the pins and bearing.

    Primitives rather than meshes, because a pin IS a cylinder and a mesh of one
    is a file to keep in step. The sim showed rods and an idler held by nothing
    at all until these went in - which is not the same fault as a floating part,
    the bodies were correctly parented, but it is the same picture.
    """
    out = []
    for name, u, plane, rod, seat, host in PINS:
        lo, hi = pin_span(plane, rod, seat)
        mid = (lo + hi) / 2
        # The idler's own body frame has its origin at Y_IDLER already; the
        # others hang off a joint, so the plane is part of the offset.
        y = (mid - Y_IDLER) if host == "idler" else mid
        body = {"thigh": "torso", "idler": "lkidler", "ankle": "ankle"}[host]
        if body == "torso":
            y += leg_y
        out.append((f"lkpin_{name}", body, (u[0], sgn * y, u[1]),
                    (hi - lo) / 2, PIN_D / 2))
    out.append(("lkbrg_idler", "shin", (0.0, sgn * Y_IDLER, 0.0),
                BEARING_W / 2, BEARING_OD / 2))
    return out


# --- the linkage as sim bodies ---------------------------------------------
#
# name, parent body, offset in the PARENT's frame, and the joints its hinge is
# coupled to. Each hinge is pinned by a fixed tendon summing itself and these,
# constrained to zero:
#
#     rod 1   absolute angle = the thigh's        rod1  - hip        = 0
#     idler   absolute angle = the torso's        idler + hip + knee = 0
#     rod 2   absolute angle = the shin's         rod2  - hip - knee = 0
#
# A 3D `connect` at each pin would model the pin literally and remove three
# degrees of freedom where a planar loop needs two, so every pin would add a
# redundant row for the solver to fight over. The mechanism is an EXACT
# parallelogram, so the joint relation is not an approximation of the pin, it is
# what the pin achieves.
#
# The cost of that choice, stated plainly: these bodies FOLLOW, they do not
# DRIVE, so nothing in the physics would notice if a coupling were wrong.
# cad.linkage.sim_matches_cad() is the check that would, and it caught one the
# first time it ran.
SIM_BODIES = (
    ("lkrod1", "torso", lambda leg_y, sgn: (U1[0], sgn * (leg_y + Y_ROD1), U1[1]),
     {"hip": -1.0}),
    ("lkidler", "shin", lambda leg_y, sgn: (0.0, sgn * Y_IDLER, 0.0),
     {"hip": 1.0, "knee": 1.0}),
    ("lkrod2", "lkidler",
     lambda leg_y, sgn: (U2[0], sgn * (Y_ROD2 - Y_IDLER), U2[1]),
     {"hip": -1.0, "knee": -1.0}),
)

# The mesh each sim body wears, and the cad/linkage.py solid it comes from.
SIM_MESH = {"lkrod1": "lk_rod1", "lkidler": "lk_idler", "lkrod2": "lk_rod2"}


def sim_stance(stance):
    """{body: hinge angle} at a given {joint: angle} pose.

    DERIVED from the couplings rather than written down beside them: each hinge
    is whatever makes its tendon zero. Written down, the two would eventually
    disagree, and the robot would start every hand-written pose with its rods a
    degree out of assembly.

    Anything that poses the model by writing qpos has to call this. mj_forward
    does NOT solve equality constraints, so a pose that sets hip and knee and
    stops leaves the linkage hinges wherever they were - at zero, if qpos was
    cleared first, which is the rods hanging straight down off their pins. That
    is how fitcheck.pose() had them for its first hour.
    """
    return {name: -sum(c * stance[j] for j, c in coef.items())
            for name, _, _, coef in SIM_BODIES}
