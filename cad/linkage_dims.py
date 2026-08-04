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
