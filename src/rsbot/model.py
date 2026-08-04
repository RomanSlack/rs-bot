"""Model loading and shared geometry."""

import math
import re
from pathlib import Path

import mujoco
import numpy as np

XML = Path(__file__).parent / "model" / "rsbot.xml"
CAD_OUT = Path(__file__).parents[2] / "cad" / "out"

# From cad/wheel_dims.py, which is the ONLY copy, for the same reason the servo
# dimensions come from cad/servo_dims.py: a bought-or-printed part's size is a
# measurement, not a modelling choice, and a second copy is a second place to be
# wrong. Both files carried 40.0 and 12.0 and agreed; the third number did not.
from cad.wheel_dims import R as _WR, HALF_W as _WHW, TYRE_T as _WTT

WHEEL_R = _WR / 1000.0
WHEEL_HALF_W = _WHW / 1000.0
THIGH_L = 0.110
SHIN_L = 0.110
# Half the track: how far each hip axis sits from the centreline. It was the
# literal 0.060 inside _leg() and nowhere else, until cad/linkage.py needed it
# too and the choice was between importing it and typing it again.
LEG_Y = 0.060

# Ankle roll: 0 is wheel mode, pi/2 lays the disc flat so it becomes the foot.
ROLL_WHEEL = 0.0
ROLL_FOOT = math.pi / 2


def axle_height(roll):
    """Floor-to-axle height at a given roll, i.e. the tilted disc's low point.

    Rises 1.76 mm at 16.7 deg as the rim corner leads, then falls monotonically
    to WHEEL_HALF_W. That 1.76 mm is this mechanism's entire dig; the flat
    plate it replaces dug 11-28 mm and went airborne doing it.
    """
    return WHEEL_R * math.cos(roll) + WHEEL_HALF_W * math.sin(roll)


def ankle_pitch_level(hip, knee, bias=0.0):
    """Ankle pitch holding the foot face level with the floor.

    Hip, knee and ankle pitch all turn about +y and compound, so the face is
    level when they sum to zero. `bias` tips the foot on purpose, which is the
    ankle flex available in foot mode.
    """
    return bias - (hip + knee)


def leg_ik(height, shift=0.0):
    """Hip/knee placing the ankle `height` below and `shift` ahead of the hip.

    shift=0 is the symmetric pose, hip = q, knee = -2q, which keeps the ankle
    directly under the hip so leg length is the only thing that changes.
    """
    r = math.hypot(shift, height)
    c = min(max(r / (THIGH_L + SHIN_L), -1.0), 1.0)
    phi = math.acos(c)
    psi = math.atan2(-shift, height)
    return psi + phi, -2.0 * phi


def make_ctrl(hip, knee, aroll, wheel, wheel_r=None):
    """Assemble the 8-actuator command.

    Legs share the joint angles; the wheels can differ, which is how the robot
    steers - there is no steering joint, only a speed difference.

    EIGHT, not ten. There is no ankle-pitch command because there is no ankle
    pitch servo: the parallelogram holds that angle mechanically. The joint is
    still in qpos, because the joint still exists.
    """
    if wheel_r is None:
        wheel_r = wheel
    return np.array([hip, knee, aroll, wheel,
                     hip, knee, aroll, wheel_r])


# --- Real part dimensions, metres --------------------------------------------
#
# Visual-only geometry built from the actual parts, so the render shows what
# would really be bolted together. It carries no mass and no collision: mass
# lives on the simple collision shapes, which are moved to geom group 4 and
# hidden. Nothing here changes the dynamics. See docs/bom.md.

# STS3215, MEASURED off TheRobotStudio's STEP model rather than taken from a
# product listing. See cad/servo.py. The third figure is along the OUTPUT
# SHAFT and was 4.2 mm short, which is exactly where the horn clearances are.
# From cad/servo_dims.py, which is the ONLY copy. This line used to be the
# STEP's 45.4 x 24.8 x 39.6, and the STEP is a model of the 7.4 V servo: 3.1 mm
# too tall along the shaft, which is the axis every servo mounting face in this
# robot is positioned from. Visible in the twin as a servo floating off the
# plate it bolts to.
from cad.servo_dims import LENGTH as _SL, WIDTH as _SW, HEIGHT as _SH
from cad.servo_dims import CRADLE_CLEAR  # the 0.4 mm the thigh's cradle is cut with
SERVO = (_SL / 1000.0, _SW / 1000.0, _SH / 1000.0)
SERVO_HORN_R = 0.0100                 # 25T output horn
# The output shaft is NOT at the centre of the case: it sits 12.5 mm off along
# the length (cad/servo.py, confirmed by the manufacturer's drawing). A servo
# positioned by its case is therefore a servo whose shaft is 12.5 mm from where
# you think it is, which is how the hip and the wheel drive both ended up off
# their own joints. Position servos by SHAFT, not by box. cad/drives.py checks.
SERVO_SHAFT_X = 0.0125
PI5 = (0.085, 0.056, 0.017)
BATT_3S = (0.105, 0.034, 0.024)       # 2200 mAh 3S pack
DRIVER = (0.050, 0.030, 0.010)        # TTL bus adapter
# WAS 0.008, and the CAD builds 3.0 mm. Nothing read it, which is why a 5 mm
# error sat in the file that describes this robot to the physics: a constant
# with no uses is a wrong answer waiting for whoever trusts it next, and it
# reads as verified because it sits beside numbers that are. See cad/wheel_dims.
WHEEL_TIRE_T = _WTT / 1000.0          # tread thickness

C_SERVO = "0.13 0.13 0.15 1"
C_HORN = "0.72 0.73 0.76 1"
C_PRINT = "0.88 0.45 0.13 1"          # printed PETG
C_TIRE = "0.09 0.09 0.10 1"
C_HUB = "0.55 0.56 0.60 1"
C_PCB = "0.05 0.33 0.17 1"
C_BATT = "0.16 0.16 0.38 1"
C_PLATE = "0.62 0.64 0.68 0.30"   # translucent, so the internals show
# The bought hardware, coloured as the material rather than by role: chrome
# steel shafts and races, anodised pulleys, a black rubber belt.
C_STEEL = "0.84 0.86 0.88 1"
C_ALU = "0.62 0.65 0.68 1"
# The parallelogram, in its own colour, because it is the newest thing on the
# robot and the one a picture is most useful for. cad/robot.py imports this
# rather than keeping a second copy.
C_LINK = "0.20 0.55 0.85 1"
C_BELT = "0.07 0.07 0.09 1"


# SIX decimal places, not five, and it is not cosmetic. These strings are
# METRES, so `:.5f` quantises the whole sim to 0.01 mm, and cad/ is not
# quantised at all. A CAD face derived exactly from a servo dimension then
# lands INSIDE the sim's box for that servo.
#
# It cost a red test for a week. The roll arm seats on the wheel servo at
# WIDTH/2 = 12.365 mm, exact; the sim rounded the servo's half-width to
# 0.01237 m and put its face at 12.370. That 0.005 mm, spread over the
# 379 mm2 the arm covers, is 1.9 mm3 of "the bracket is eating a bought part",
# which is exactly what a seating check should scream about, and it was right
# to. The part was never wrong.
#
# The other direction is the one that matters more, and nothing had noticed it:
# LENGTH rounds DOWN, 45.23 to 45.22, so every clearance in this robot that
# involves the side of a servo case has been checked against a case 0.01 mm
# thinner than the real one. Small, but it is the dangerous sign - a sim box
# smaller than the part it stands for is a gap that exists in the model and not
# on the bench. At 1e-6 m both come out exact.
def _v(name, gtype, size, pos, rgba, euler=None):
    """A visual-only geom: no mass, no collision, hidden group for the solids."""
    e = f' euler="{euler}"' if euler else ""
    return (f'<geom name="{name}" type="{gtype}" size="{size}" pos="{pos}"'
            f'{e} rgba="{rgba}" contype="0" conaffinity="0" mass="0" group="0"/>')


# --- Leg construction, with optional gear backlash ----------------------------
#
# Backlash is modelled physically, not as a command deadband: each actuated
# joint gets a second "lash" joint in series, free to float within +/- lash/2.
# The servo drives and senses the proximal side; the link hangs off the distal
# side and can move without the motor knowing. That floating deadzone in the
# kinematic chain is what actually hurts a balancer, and a deadband on the
# command would not reproduce it.
#
# The link bodies keep their canonical names (thigh_l, shin_l, wheel_l, ...)
# so sensors, contact excludes and lookups work either way.

# A whisker of damping on each linkage hinge. The hinges are pinned by exact
# tendon equalities so they have nothing to do dynamically, and this is only
# here to stop the solver ringing on a body with 7 g of inertia.
LINKAGE_DAMPING = 1e-4

GEAR_MASS = 0.002        # gearbox-side inertia stub, one per lashed joint
LASH_DAMPING = 0.001
LASH_ARMATURE = 0.0001
LASH_FRICTION = 0.0005

# Derived, not estimated: run `uv run python -m cad.masses`. Each body is its
# printed structure (volume x PETG x infill) plus the servos whose CASES bolt
# to it plus bought parts. The shin uses its real CAD mass from cad/shin.py.
#
# The old numbers put 100 g in the shin on the assumption the wheel servo lived
# there; it is really on the roll bracket, 110 mm further out.
SEG_MASS = {"thigh": 0.0894, "shin": 0.0889, "ankle": 0.0693,
            "rollbracket": 0.0600, "wheel": 0.060}

# Mass was derived; this is where that mass SITS and how it is spread out.
#
# Until this existed, each link's inertia came from the simple capsule or box
# carrying its mass - a 28 mm capsule standing in for an L-shaped part with a
# servo bolted to one side. The mass was right and the distribution was a
# guess, and a balancer is sensitive to the distribution: the ankle's centre of
# mass turned out to be 75 mm from where the stand-in put it, because the roll
# servo hangs well aft of the axle.
#
# Composed from the real B-rep solid plus the servo and bought-part boxes at
# the positions this file already places them, combined about the composite
# centre of mass rather than added as scalars.
#
# link -> (mass kg, com m, (ixx iyy izz ixy ixz iyz) kg.m2),
# LEFT side and torso; the right side mirrors in y.
# Regenerate with: uv run python -m cad.inertia --emit
SEG_INERTIA = {
    "thigh": (0.096075, (-0.000001, -0.001186, -0.082131),
              (1.151560e-04, 9.302291e-05, 3.237157e-05, -9.038478e-10, 1.720578e-09, -2.839567e-05)),
    "shin": (0.034371, (+0.020614, +0.036036, -0.045906),
              (5.398202e-05, 5.187562e-05, 2.674926e-05, -5.590930e-06, 1.379589e-05, 1.825922e-05)),
    "ankle": (0.069571, (-0.067645, +0.008224, +0.010396),
              (4.277212e-05, 4.459982e-05, 6.513480e-05, -2.456065e-05, 5.072140e-06, 5.636587e-06)),
    "rollbracket": (0.062637, (-0.014798, +0.029617, +0.001139),
              (1.292240e-05, 1.785285e-05, 2.230819e-05, -2.779284e-06, 8.089292e-07, 8.200901e-07)),
    "torso": (1.100703, (+0.008180, +0.000000, +0.082687),
              (4.123392e-03, 3.851683e-03, 1.440641e-03, -1.810980e-20, -3.976111e-05, -2.727584e-20)),
    "wheel": (0.054782, (+0.000000, -0.000683, -0.000000),
              (3.247476e-05, 5.851804e-05, 3.247476e-05, -5.598628e-17, 1.773249e-16, 3.053642e-15)),
    "lkrod1": (0.006801, (-0.000000, -0.000000, -0.055000),
              (8.050659e-06, 8.068349e-06, 5.849861e-08, -2.630324e-22, 7.188831e-23, -2.851857e-22)),
    "lkidler": (0.003790, (+0.016648, -0.000000, -0.000000),
              (1.113201e-07, 6.033217e-07, 5.147389e-07, -2.432764e-23, -1.573994e-22, 7.514599e-24)),
    "lkrod2": (0.006510, (-0.000000, -0.000000, -0.055000),
              (8.036603e-06, 8.054887e-06, 5.734404e-08, -2.659411e-22, 1.327761e-22, -2.824800e-22)),
}
_RANGE = {"hip": "-0.60 1.40", "knee": "-2.00 0.05",
          "ankle_pitch": "-1.60 1.60", "ankle_roll": "-0.10 1.75"}


# Servo half-extents. On a real bus servo the output shaft is PERPENDICULAR to
# the long axis and sits near one end of the top face, so a servo driving a
# joint occupies the plane perpendicular to that joint's axis. Getting this
# backwards is what made the first pass interpenetrate.
HL, HW, HH = SERVO[0] / 2, SERVO[1] / 2, SERVO[2] / 2
SHAFT_INSET = 0.0102         # shaft centre from the near end (measured)


SPINE_Y = 0.021      # spine centre-line, outboard of the 24 mm wide wheel
                     # and clear of the tie that crosses the wheel plane aft
SPY = 0.006          # spine half-thickness
ANK_Y = 0.032        # ankle structure runs outboard of the shin's, so the
                     # two never touch as the ankle pitches between them



# Visual geoms that stand in for printed structure. When the CAD meshes are
# switched on these come out and the real part goes in; the servo, wheel and
# electronics boxes stay, because those are bought parts and a box is all they
# ever were.
# Which visual geoms are stand-ins for PRINTED structure, and so have to come
# out when the real mesh goes in.
#
# THIS WAS A LIST OF EXACT NAMES AND IT HAD GONE STALE THREE SEPARATE WAYS.
#
#   1. Suffixes. The side plates and lower shelf come out of the build split by
#      the hip notch, as "vside1f-0.036" and "vshelf10.012", so an exact match
#      on "vside1" never fired at all.
#   2. Additions. Every redesign of the shin, ankle and roll bracket added
#      members - vshinfarm, vshinfpost, vshincross, vshindrop, vshinback,
#      vankring, vanktie, vyokecross, vyokefwd, vyokeboss, vrollweb, vrollhub,
#      vrollcoup, vrollshaft - and not one was added here.
#   3. Deletions. vshinpost, vankface and vrolltie are still listed and no
#      longer exist, which is harmless and is also why nobody looked: the list
#      LOOKED maintained.
#
# The result was 17 boxes drawn INSIDE the real meshes, in the same group, which
# is exactly the duplicate geometry you see in the twin. It is also invisible
# from the code: nothing errors when a name in this list matches nothing.
#
# So it is a pattern now, with the alternatives spelled out rather than left as
# a loose prefix, because "vroll" and "vank" would otherwise swallow vrollsv and
# vrollsv - the SERVOS, which are bought parts and must stay. _check_no_standins()
# below turns the whole thing into a check that can fail.
PRINTED_VIS_RE = re.compile(
    r"^v(thigh|shin\w*|ank(ring|tie|stand|post|face)|yoke\w*"
    r"|roll(web|hub|coup|shaft|arm|tie)|side[-\d]|top|shelf\d|pistand)")
MESH_STEM = {"thigh": "thigh", "shin": "shin", "ankle": "ankle_yoke",
             "rollbracket": "roll_bracket", "torso": "chassis"}

# The wheel is two parts in two materials and needs a rotation the others do
# not: its CAD spins about z with the sole at +z, the sim body spins about y
# with the sole INBOARD. +90 deg about x on the left, -90 on the right - which
# also says the two wheels are the same part flipped over, not a mirrored pair.
# It is kept separate from MESH_STEM because that machinery assumes one mesh
# per link, no rotation, and mirrored by a negative y scale. None of those hold
# here, and forcing it would have shown the wheel in the wrong mode.
# The servo, drawn from the manufacturer's drawing rather than left as a grey
# box. Which STL a servo gets is decided by the order its box writes its own
# dimensions in, because the sim stores each one in whatever order puts the case
# where it goes. cad/servo.py builds one mesh per order, so the orientation is
# baked into the geometry and there is no euler here to get backwards.
#
# The euler comes from cad/servo.py, which explains at length why there is one
# mesh and not three: MuJoCo canonicalises mesh vertices onto principal axes, so
# an orientation baked into the STL does not survive the compiler.
SERVO_MESH = {"vhipsv": "wsl", "vkneesv": "wsl",
              "vrollsv": "swl", "vwhlsv": "lsw"}

MESH_MULTI = {"wheel": [("wheel_body", C_HUB), ("wheel_tyre", C_TIRE)]}
MESH_EULER = {"wheel": {"l": "1.5708 0 0", "r": "-1.5708 0 0"}}


def _inertial(link, sgn):
    """The <inertial> for one link, mirrored in y for the right side.

    Mirroring flips the sign of the centre of mass in y AND of the Ixy and Iyz
    products of inertia. Missing the products is invisible in any symmetric
    pose and shows up only as a slow drift in a turn, which is the sort of bug
    that gets blamed on the yaw gain for a week.
    """
    mass, com, I = SEG_INERTIA[link]
    ixx, iyy, izz, ixy, ixz, iyz = I
    if sgn < 0:
        com = (com[0], -com[1], com[2])
        ixy, iyz = -ixy, -iyz
    return (f'<inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" '
            f'mass="{mass:.6f}" fullinertia="{ixx:.6e} {iyy:.6e} {izz:.6e} '
            f'{ixy:.6e} {ixz:.6e} {iyz:.6e}"/>')



def _servo_meshes(geoms):
    """Grey servo boxes out, the real STS3215 in. Position is left untouched.

    cad.servo is imported HERE rather than at the top of the file on purpose.
    It pulls in build123d, which is a dev-group dependency, and the sim has to
    stay loadable without a CAD kernel installed. This path only runs when
    meshes=True, which only serve.py asks for.
    """
    import cad.servo as _servo
    out = []
    for g in geoms:
        m = re.search(r'name="(v[a-z]+sv)[_-]?\w*"', g)
        key = SERVO_MESH.get(m.group(1)) if m else None
        if key is None:
            out.append(g)
            continue
        out.append(re.sub(r'type="box" size="[^"]*"',
                          f'type="mesh" mesh="cad_servo" '
                          f'euler="{_servo.MESH_EULER[key]}"', g))
    return out


def _swap_meshes(vis, link, side, sgn):
    """Printed stand-in boxes out, the real part in."""
    if link in MESH_MULTI:
        # The wheel: drop the tyre/hub cylinders, put both real solids in.
        kept = [g for g in vis
                if f'name="vtire_{side}"' not in g
                and f'name="vhub_{side}"' not in g]
        e = MESH_EULER[link][side]
        return kept + [
            f'<geom name="cad_{stem}_{side}" type="mesh" '
            f'mesh="cad_{stem}_{side}" euler="{e}" rgba="{rgba}" '
            f'contype="0" conaffinity="0" mass="0" group="0"/>'
            for stem, rgba in MESH_MULTI[link]]
    kept = [g for g in vis if not _is_standin(g)]
    return kept + [_mesh_geom(link, side, sgn)]


def _is_standin(geom_xml):
    m = re.search(r'name="([^"]+)"', geom_xml)
    return bool(m) and bool(PRINTED_VIS_RE.match(m.group(1)))


def _mesh_geom(link, side, sgn):
    """The real printed part as a visual mesh, in place of the boxes.

    Visual only, and only when asked for. The primitive build stays the
    default because `fitcheck.py` measures interference from oriented bounding
    boxes of the group-0 geoms, and a mesh has no meaningful geom_size - it
    would silently start auditing the wrong shape.
    """
    stem = MESH_STEM[link]
    return (f'<geom name="cad_{link}_{side}" type="mesh" '
            f'mesh="cad_{stem}_{side}" rgba="{C_PRINT}" '
            f'contype="0" conaffinity="0" mass="0" group="0"/>')


def _link_geoms(link, side, sgn, meshes=False):
    """Collision shape (group 4, carries the mass) plus the visual build.

    Each link is a SPINE plate running from its own joint to the child joint,
    with the child's servo bolted flush against it. The spine is what makes
    this an assembly rather than a cloud of parts.

    Everything is routed OUTBOARD of the wheel plane and stops short of the
    axle, because the wheel changes shape: upright it is a 24 mm rim swept
    through 80 mm vertically, flat it is an 80 mm platter swept horizontally.
    A part has to miss both.
    """
    col, vis = [_inertial(link, sgn)], []
    y = sgn * SPINE_Y
    off = HL - SHAFT_INSET

    if link == "thigh":
        col.append(f'<geom class="leg" name="thigh_{side}" fromto="0 0 0  0 0 -0.110" '
                   f'group="4"/>')
        # FIVE BOXES, and it used to be one. The thigh does not run straight
        # down: it reaches AROUND the knee servo to a plate on the INBOARD
        # side, because the outboard band is where the shin's hub has to be.
        # One box could not say that, so it said the outboard spine ran to
        # z = -102 - and the real part's outboard material stops at -74.
        # cad.twin measured the box at 71.4% backed and passed it; what finally
        # showed it was the idler's stub, a boss at the knee that the box model
        # had colliding with a thigh 22 mm away on the real solids.
        #
        # 16 mm wide in y, not 12: with no hip roll joint, the STRUCTURE
        # carries the whole lateral moment at the hip (9.5 N.m at the design
        # load), and at 12 mm that is only a 2.0x margin. See cad/thigh.py.
        vis.append(_v(f"vthigh_{side}", "box", f"0.010 0.008 0.035",
                      f"0 {sgn*0.023:.6f} -0.039", C_PRINT))
        # The inboard end, which is the half the single box had on the wrong
        # side of the leg. It is a FORK, not a plate: a back wall with two
        # cheeks that the knee servo sits between. Drawn as a plate first, and
        # that plate reported 10 mm inside the servo while the real solids
        # intersect at 0 mm3 - the y profile is continuous across the pocket, so
        # a slice's bounding box cannot tell a fork from a slab. The x profile
        # can, and does: material at |x| = 12.5..15.3 with air between.
        # Both are hung off the servo's own inboard face rather than measured,
        # so they abut it by construction. Probing the solid put that face at
        # -21.0 and the wall 0.5 mm inside the servo; cad/thigh.py says
        # SERVO_Y_LO = SERVO_Y_HI - SERVO_H with the comment "y = -20.4", and
        # that comment is stale - cad/servo.py re-exports servo_dims, whose
        # HEIGHT was corrected to 36.50, so the face has been at -21.5 since.
        wall = SPINE_Y - SPY - 2 * HH          # -0.0215, the servo's inboard face
        # The cross-member OVER the servo, which is what ties the two sides
        # together. Left out of the first version, and tests/test_foot.py's
        # test_every_body_is_one_rigid_piece came straight back with "thigh_l is
        # 2 loose pieces": the spine stops at z = -74 and the fork starts at
        # -63, so without this they are a leg in two halves. It is the one check
        # that asks whether the parts on a body touch EACH OTHER rather than
        # whether the robot is connected overall, and it is why it exists.
        z_hi = -0.110 + off + HL               # -0.07497, the servo's top
        vis.append(_v(f"vthighcross_{side}", "box", "0.010 0.03075 0.006",
                      f"0 {sgn*0.00025:.6f} {z_hi + 0.006:.6f}", C_PRINT))
        vis.append(_v(f"vthighfork_{side}", "box", "0.0153 0.0045 0.03025",
                      f"0 {sgn*(wall - 0.0045):.6f} -0.09325", C_PRINT))
        # The cheeks, reaching past the servo to the knee axis, so they have to
        # hug it: inner face at HW + CRADLE_CLEAR, which is the same 0.4 mm the
        # cradle is drawn with.
        c_in = HW + CRADLE_CLEAR / 1000.0            # 0.012765
        c_out = c_in + 0.0025                        # cad/thigh.py CRADLE_WALL
        for k, s in ((0, -1), (1, 1)):
            vis.append(_v(f"vthighcheek{k}_{side}", "box",
                          f"{(c_out - c_in)/2:.6f} 0.00525 0.0245",
                          f"{s*(c_in + c_out)/2:.6f} "
                          f"{sgn*(wall + 0.00525):.6f} -0.099", C_PRINT))
        # INBOARD. The thigh wraps over and down the inboard side to reach it,
        # because the outboard band is where the shin's hub has to be.
        vis.append(_v(f"vkneesv_{side}", "box", f"{HW:.6f} {HH:.6f} {HL:.6f}",
                      f"0 {-sgn*(HH - SPINE_Y + SPY):.6f} {-0.110+off:.6f}",
                      C_SERVO))
    elif link == "shin":
        col.append(f'<geom class="leg" name="shin_{side}" fromto="0 0 0  0 0 -0.094" '
                   f'group="4"/>')
        # Stops 55 mm above the axle. Below that it is inside the volume the
        # wheel-drive servo SWEEPS as the ankle rolls: that servo turns with
        # the roll bracket and carves an annulus 14-50 mm from the roll axis,
        # for |x| < 23 mm. Clearing the two end poses is not enough.
        vis.append(_v(f"vshin_{side}", "box", f"0.010 {SPY} 0.0245",
                      f"0 {y:.6f} -0.0245", C_PRINT))
        # NO ANKLE-PITCH SERVO STANDOFF EITHER. The mount outlived the servo
        # by a day; cad/shin.py deletes it, and cad.twin caught this box the
        # moment it did - 0.0% backed, NOT IN THE CAD, and the shin 8.8 g heavy.
        # That is the check working in the direction it was built for.
        # NO ANKLE-PITCH SERVO. It is deleted, along with the belt drive it
        # needed, and a passive parallelogram holds hip + knee + ankle = 0
        # instead. cad/linkage.py draws the mechanism and checks that it fits;
        # docs/deleting-the-ankle-pitch-servo.md is why. The joint remains, it
        # is simply not driven.
        # NO AFT MEMBER. There used to be a "vshinarm" here, stepping aft to
        # reach the ankle bearing, and cad/shin.py ABANDONED that route: "it
        # cannot go AFT either, which is what the first two attempts did. In
        # foot mode the axle is 12 mm off the floor and the shin stands at 27.5
        # deg, so structure hanging aft at axle height swings down: the aft
        # version ended up 13 mm THROUGH the floor."
        #
        # The CAD went forward. The sim did not follow, and carried the dead
        # member until cad/twin.py measured it at 0% backed by CAD material.
        # The forward route is the vshinfarm/fpost/cross/drop/back chain below.
        # The route to the ankle-pitch bearing at (0, +/-68, -110) goes
        # FORWARD, and both halves of that are forced. It cannot go down the
        # middle, because the wheel-drive servo turns with the roll bracket and
        # sweeps an annulus 14..50.9 mm about the roll axis for |x| < 22.6. It
        # cannot go aft either: in foot mode the axle is 12 mm off the floor
        # and the shin stands at 27.5 deg, so structure hanging aft at axle
        # height swings 13 mm THROUGH the floor. Forward, the same tilt lifts
        # it. See cad/envelope.py, which solves for the bearing's two legal
        # bands: y = -80..-47 and y = +57..+80.
        vis.append(_v(f"vshinfarm_{side}", "box", f"0.0185 {SPY} 0.007",
                      f"0.0285 {y:.6f} -0.042", C_PRINT))
        vis.append(_v(f"vshinfpost_{side}", "box", f"0.005 {SPY} 0.011",
                      f"0.042 {y:.6f} -0.059", C_PRINT))
        vis.append(_v(f"vshincross_{side}", "box", "0.005 0.0315 0.0045",
                      f"0.042 {sgn*0.0465:.6f} -0.0655", C_PRINT))
        vis.append(_v(f"vshindrop_{side}", "box", "0.005 0.005 0.0265",
                      f"0.042 {sgn*0.071:.6f} -0.0875", C_PRINT))
        vis.append(_v(f"vshinback_{side}", "box", "0.0215 0.005 0.006",
                      f"0.0215 {sgn*0.071:.6f} -0.108", C_PRINT))
    elif link == "ankle":
        col.append(f'<geom class="ankle" name="ankle_{side}" '
                   f'size="{HW:.6f} {HH:.6f} {HL:.6f}" pos="0 0 {HL:.6f}" '
                   f'group="4"/>')
        # Yoke reaching AFT along the roll axis to a bearing clear of the
        # wheel disc, then up to meet the shin.
        # Bearing carrier, aft on the ROLL AXIS. Two clearances fall out of
        # that: it is inside the hole of the wheel-servo annulus (radius under
        # 14 mm, so the sweep misses it), and at |x| > 40 mm it is outside both
        # the upright and the flat wheel.
        # Between the roll servo and the wheel, not inside either: the servo
        # case ends at x = -58, and |x| > 40 keeps it clear of the flat wheel.
        # -51..-46.5, shortened with cad/ankle.py's POST to free 1.5 mm for the
        # roll bracket's web. It read -51..-45 here after the CAD moved, and
        # cad.twin caught it at 37.9% backed: a sim box bigger than the part it
        # stands for is the dangerous direction, because clearance then gets
        # checked against material that will not be printed.
        vis.append(_v(f"vankpost_{side}", "box", "0.00225 0.006 0.006",
                      "-0.04875 0 0", C_PRINT))
        # Ring the roll servo bolts to, with the coupler turning inside it.
        vis.append(_v(f"vankring_{side}", "box", "0.0035 0.017 0.002",
                      "-0.0545 0 0.015", C_PRINT))
        # -52..-46.5, moved with the post for the same reason.
        vis.append(_v(f"vanktie_{side}", "box", "0.00275 0.006 0.005",
                      "-0.04925 0 0.011", C_PRINT))
        # Mounting face the roll servo bolts to, bridging it to the carrier.
        # Kept within 8.5 mm of the roll axis: the roll bracket's tie sweeps
        # an annulus 8.7-19.6 mm out, so anything reaching into that band gets
        # hit partway through the flip.

        # The ankle-pitch joint, which was not drawn at all until now: the
        # shin had a bore 52.8 mm off the axis and the yoke had no matching
        # feature, so the two parts came no closer than 10.3 mm.
        vis.append(_v(f"vyokecross_{side}", "box", "0.003 0.0325 0.008",
                      f"-0.048 {sgn*0.0265:.6f} 0", C_PRINT))
        vis.append(_v(f"vyokefwd_{side}", "box", "0.028 0.005 0.008",
                      f"-0.028 {sgn*0.061:.6f} 0", C_PRINT))
        vis.append(_v(f"vyokeboss_{side}", "cylinder", "0.007 0.005",
                      f"0 {sgn*0.061:.6f} 0", C_PRINT, euler="1.5708 0 0"))
        vis.append(_v(f"vrollsv_{side}", "box", f"{HH:.6f} {HW:.6f} {HL:.6f}",
                      # Lifted off the roll axis so it clears the floor in foot mode,
                      # where the axle is only 12 mm up. Safe despite the larger
                      # radius because |x| > 23 mm puts it outside the wheel-servo sweep.
                      f"{-(0.058+HH):.6f} 0 0.0125", C_SERVO))
    elif link == "rollbracket":
        # Wheel-drive servo and bearing block, OUTBOARD. The 90 deg roll maps
        # +y onto +z, so outboard becomes directly above the flat wheel, which
        # is the only place a support for a vertical shaft can live.
        # x = -SERVO_SHAFT_X, not 0. Centring the CASE on the wheel axis put
        # the SHAFT 12.5 mm off it, and the wheel bolts straight to the horn.
        # Negative, so the case sits back under the arm at x = -44..0 rather
        # than reaching forward past it.
        vis.append(_v(f"vwhlsv_{side}", "box", f"{HL:.6f} {HH:.6f} {HW:.6f}",
                      f"{-SERVO_SHAFT_X:.6f} {sgn*(0.0135+HH):.6f} 0", C_SERVO))
        # The printed arm the wheel servo bolts to. It was dropped when the
        # tie and tongue were replaced by the web, which left the bracket as
        # two loose pieces in the sim - caught by the one-rigid-piece test.
        vis.append(_v(f"vrollarm_{side}", "box", "0.022 0.0057 0.005",
                      f"-0.022 {sgn*0.0178:.6f} 0.01735", C_PRINT))
        # Web straight from the arm to the roll axis, at x = -45..-41: the one
        # window forward of the yoke's post that is still outside the 40 mm
        # wheel. The tie, tongue and corner rib it replaces existed only to
        # crank round the shin's old post.
        vis.append(_v(f"vrollweb_{side}", "box", "0.002 0.012 0.011",
                      f"-0.043 {sgn*0.012:.6f} 0.011", C_PRINT))
        vis.append(_v(f"vrollhub_{side}", "cylinder", "0.008 0.002",
                      "-0.043 0 0", C_PRINT, euler="0 1.5708 0"))
        # Coupler disc bolted to the roll servo's horn - what the servo turns -
        # and the 3 mm shaft joining it to the hub.
        vis.append(_v(f"vrollcoup_{side}", "cylinder", "0.009 0.00225",
                      "-0.05425 0 0", C_PRINT, euler="0 1.5708 0"))
        vis.append(_v(f"vrollshaft_{side}", "cylinder", "0.0015 0.00775",
                      "-0.04875 0 0", C_HORN, euler="0 1.5708 0"))

    elif link == "wheel":
        col.append(f'<geom class="wheel" name="wheel_{side}" zaxis="0 1 0" '
                   f'group="4"/>')
        vis += [_v(f"vtire_{side}", "cylinder", "0.040 0.012", "0 0 0", C_TIRE,
                   euler="1.5708 0 0"),
                _v(f"vhub_{side}", "cylinder", "0.024 0.0115", "0 0 0", C_HUB,
                   euler="1.5708 0 0")]
    if meshes and (link in MESH_STEM or link in MESH_MULTI):
        vis = _swap_meshes(vis, link, side, sgn)
    if meshes:
        vis = _servo_meshes(vis)
        vis += _hw_geoms(link, side)
    return col + vis


# joint, link body, offset from the parent
CHAIN = [("hip", "thigh", None), ("knee", "shin", "0 0 -0.110"),
         ("ankle_pitch", "ankle", "0 0 -0.110"),
         ("ankle_roll", "rollbracket", "0 0 0"), ("wheel", "wheel", "0 0 0")]


def _linkage_bodies(side, meshes, parent):
    """The parallelogram's bodies that hang off `parent`, as MJCF.

    Geometry and couplings come from cad/linkage.py, which owns them. This only
    turns them into XML: a second copy of where a rod's pin sits is a second
    place to be wrong about it, and this file has been that second place before.
    """
    import cad.linkage_dims as lk

    sgn = 1 if side == "l" else -1
    out = ""
    close = ""
    for name, par, offset, _ in lk.SIM_BODIES:
        if par != parent:
            continue
        x, y, z = [v / 1000.0 for v in offset(LEG_Y * 1000.0, sgn)]
        mass, com, I = SEG_INERTIA[name]
        ixx, iyy, izz, ixy, ixz, iyz = I
        geom = _linkage_bought(side, name)
        if meshes:
            stem = lk.SIM_MESH[name]
            geom += (f'<geom name="{stem}_{side}" type="mesh" '
                     f'mesh="{stem}_{side}" rgba="{C_LINK}" contype="0" '
                     f'conaffinity="0" mass="0" group="0"/>')
        else:
            geom += _linkage_prims(name, side)
        out += (f'<body name="{name}_{side}" pos="{x:.6f} {y:.6f} {z:.6f}">'
                f'<joint name="{name}_{side}" axis="0 1 0" limited="false" '
                f'damping="{LINKAGE_DAMPING}"/>'
                f'<inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" '
                f'mass="{mass:.6f}" fullinertia="{ixx:.6e} {iyy:.6e} {izz:.6e} '
                f'{ixy:.6e} {ixz:.6e} {iyz:.6e}"/>{geom}')
        # anything parented to THIS body nests inside it
        out += _linkage_bodies(side, meshes, name)
        close += "</body>"
    return out + close


def _linkage_prims(name, side):
    """The rods and the idler as BOXES, for the plain build.

    Every other part in this robot has a primitive stand-in as well as a mesh,
    and the linkage did not: with meshes off, rod 1 and rod 2 had no geometry at
    all and the idler was two pins hanging in space. The sim's own
    `test_robot_is_one_assembly_not_a_cloud_of_parts` counted NINE groups.

    They are also what cad/twin.py checks - every group-0 box has to have CAD
    material behind it - so a stand-in is not decoration, it is the thing that
    gets compared.
    """
    import cad.linkage_dims as lk

    half = {"lkrod1": (lk.ROD_T, lk.ROD_W, THIGH_L * 1000.0),
            "lkrod2": (lk.ROD_T, lk.ROD_W, SHIN_L * 1000.0)}
    if name in half:
        t, w, L = half[name]
        return _v(f"vlk{name[2:]}_{side}", "box",
                  f"{t/2000.0:.6f} {w/2000.0:.6f} {L/2000.0:.6f}",
                  f"0 0 {-L/2000.0:.6f}", C_LINK)
    # The idler is a vee, so it gets one box per arm, each turned to lie along
    # it. A single bounding box would be mostly air, and air is exactly what
    # cad/twin.py measures a box against.
    out = ""
    for i, u in enumerate((lk.U1, lk.U2)):
        L = math.hypot(u[0], u[1])
        ang = math.atan2(u[1], u[0])
        out += _v(f"vlkidler{i}_{side}", "box",
                  f"{L/2000.0:.6f} {lk.IDLER_W/2000.0:.6f} "
                  f"{lk.IDLER_T/2000.0:.6f}",
                  f"{u[0]/2000.0:.6f} 0 {u[1]/2000.0:.6f}", C_LINK,
                  euler=f"0 {-ang:.6f} 0")
    return out


def _linkage_host_prims(link, side, sgn):
    """The linkage's MOUNT features, as primitives, for the plain build.

    The stub on the shin and the arm on the yoke are real material on those
    parts (cad/linkage_mounts.py, built in by cad/shin.py and cad/ankle.py), and
    the pins that press into them are geoms here. Without stand-ins the sim's
    own `test_every_body_is_one_rigid_piece` reports "shin_l is 2 pieces": the
    bearing and the pin, floating beside a part that is really there.
    """
    import cad.linkage_dims as lk

    if link == "shin":
        lo, hi = lk.stub_span()
        return _v(f"vlkstub_{side}", "cylinder",
                  f"{lk.STUB_R/1000.0:.6f} {(hi-lo)/2000.0:.6f}",
                  f"0 {sgn*(lo+hi)/2000.0:.6f} 0", C_LINK,
                  euler="1.5708 0 0")
    if link == "ankle":
        L = math.hypot(lk.U2[0], lk.U2[1])
        ang = math.atan2(lk.U2[1], lk.U2[0])
        return _v(f"vlkarm_{side}", "box",
                  f"{L/2000.0:.6f} {lk.ARM_W/2000.0:.6f} "
                  f"{lk.ARM_T/2000.0:.6f}",
                  f"{lk.U2[0]/2000.0:.6f} {sgn*lk.Y_ARM/1000.0:.6f} "
                  f"{lk.U2[1]/2000.0:.6f}", C_LINK,
                  euler=f"0 {-ang:.6f} 0")
    return ""


def _linkage_bought(side, link):
    """The linkage's pins and its idler bearing, as geoms on `link`.

    Bought parts, drawn as the cylinders they are. Without them the sim shows
    rods and an idler held by nothing: not floating - the bodies are parented
    and constrained - but the same picture, and the same fault
    docs/how-checks-fail.md #13 is about.
    """
    import cad.linkage_dims as lk

    sgn = 1 if side == "l" else -1
    out = ""
    for name, host, pos, half, r in lk.sim_bought(LEG_Y * 1000.0, sgn):
        if host != link:
            continue
        x, y, z = [v / 1000.0 for v in pos]
        out += (f'<geom name="{name}_{side}" type="cylinder" '
                f'size="{r/1000.0:.6f} {half/1000.0:.6f}" '
                f'pos="{x:.6f} {y:.6f} {z:.6f}" euler="1.5708 0 0" '
                f'rgba="{C_STEEL}" contype="0" conaffinity="0" mass="0" '
                f'group="0"/>')
    return out


def _linkage_tendons(side):
    """One fixed tendon per linkage hinge, each constrained to zero."""
    import cad.linkage_dims as lk

    out = []
    for name, _, _, coef in lk.SIM_BODIES:
        legs = "".join(
            f'      <joint joint="{j}_{side}" coef="{c:g}"/>\n'
            for j, c in coef.items())
        out.append(f'    <fixed name="{name}_{side}">\n'
                   f'      <joint joint="{name}_{side}" coef="1"/>\n'
                   f'{legs}    </fixed>')
    return "\n".join(out)


def _leg(side, backlash, meshes=False):
    """One leg: hip pitch, knee pitch, ankle pitch, ankle ROLL, wheel.

    The roll bracket is its own body because the wheel-drive servo bolts to it
    and must NOT spin with the wheel.
    """
    sgn = 1 if side == "l" else -1
    y = sgn * LEG_Y
    b = backlash / 2.0
    lash_attrs = (f'damping="{LASH_DAMPING}" armature="{LASH_ARMATURE}" '
                  f'frictionloss="{LASH_FRICTION}"')
    stub = f'<inertial pos="0 0 0" mass="{GEAR_MASS}" diaginertia="1e-6 1e-6 1e-6"/>'

    opens, closes = [], []
    depth = 3
    for joint, link, pos in CHAIN:
        pos = pos or f"0 {y} 0"
        axis = ("1 0 0" if side == "l" else "-1 0 0") if joint == "ankle_roll" else "0 1 0"
        jn = f"{joint}_{side}"
        rng = f' range="{_RANGE[joint]}"' if joint in _RANGE else ""
        ind = "  " * depth
        joint_xml = f'<joint name="{jn}" axis="{axis}"{rng}/>'
        body = f"{link}_{side}"

        # A PARALLELOGRAM HAS NO GEARBOX AT THE ANKLE, so it gets no lash
        # joint. That is the whole mechanical argument for it: the ankle's free
        # play does not exist to be compensated, because there is no reduction
        # in that path - only links and pin joints. What play it does have is
        # pin clearance, which cad/linkage.py budgets at 0.10 deg on bearings
        # against the 2-3 deg of gear lash it replaces.
        lashed = backlash > 0 and joint != "ankle_pitch"
        if lashed:
            opens.append(
                f'{ind}<body name="{jn}_drv" pos="{pos}">\n{ind}  {joint_xml}\n'
                f'{ind}  {stub}\n'
                f'{ind}  <body name="{body}" pos="0 0 0">\n'
                f'{ind}    <joint name="{jn}_lash" axis="{axis}" '
                f'range="{-b:.6f} {b:.6f}" {lash_attrs}/>\n')
            closes.append(f"{ind}  </body>\n{ind}</body>\n")
            gind = "  " * (depth + 2)
            depth += 2
        else:
            opens.append(f'{ind}<body name="{body}" pos="{pos}">\n{ind}  {joint_xml}\n')
            closes.append(f"{ind}</body>\n")
            gind = "  " * (depth + 1)
            depth += 1

        for g in _link_geoms(link, side, sgn, meshes):
            opens[-1] += gind + g + "\n"
        # The idler rides on the knee, so it is a child of the SHIN, and rod 2
        # is a child of the idler. They go in here rather than at the torso
        # level because that is where they physically hang.
        opens[-1] += _linkage_bought(side, link)
        if not meshes:
            opens[-1] += _linkage_host_prims(link, side, sgn)
        opens[-1] += _linkage_bodies(side, meshes, link)

    # rod 1 hangs off the TORSO, and so does the pin it hangs on. _leg's return
    # is substituted inside the torso body, so both land in the right frame.
    return ("".join(opens) + "".join(reversed(closes))
            + _linkage_bought(side, "torso")
            + _linkage_bodies(side, meshes, "torso"))


def _hip_servos():
    """Hip servo cases bolt to the torso, not the thigh."""
    g = []
    for sgn in (1, -1):
        # Flush inboard of the thigh spine, and reaching back to the chassis.
        # z = SERVO_SHAFT_X, not HL. It was HL, which put the BOTTOM of the
        # case on the hip axis and the shaft 10.2 mm - one SHAFT_INSET - above
        # it. The servo was not driving the joint it is bolted to.
        g.append(_v(f"vhipsv{sgn}", "box", f"{HW:.6f} {HH:.6f} {HL:.6f}",
                    f"0 {sgn*(0.060+SPINE_Y-SPY-HH):.6f} {SERVO_SHAFT_X:.6f}",
                    C_SERVO))
    return g


def _torso_visual(meshes=False):
    """Chassis and the parts inside it, drawn at real size.

    Two things this makes obvious that a plain box did not: the 3S pack is
    105 mm long and will not lie flat in a 90 mm bay, so it stands upright;
    and once the Pi and the pack are in, the bay is essentially full.
    """
    x = 0.0085
    g = []
    # Each side plate is fore and aft segments plus a bridge above, not one
    # slab: the hip servo lives INSIDE the chassis and its output boss passes
    # out through the plate to reach the thigh hub, so there is a notch there.
    # Modelled as one slab it reads as a 4.2 mm interpenetration, which is not
    # a fault - it is a hole nobody had drawn.
    # The parallelogram's ground fin, one per leg. Real material on the
    # chassis (cad/chassis.py builds it in), so it needs a stand-in here or the
    # pin that presses into it is a cylinder floating beside the side plate -
    # which is what "torso is 3 loose pieces" was.
    import cad.linkage_dims as _lk
    fin_y0, fin_y1 = 0.0396, (_lk.Y_FIN + LEG_Y * 1000.0) / 1000.0
    for sgn in (1, -1):
        g.append(_v(f"vlinkfin{sgn}", "box",
                    f"{_lk.FIN_T/2000.0:.6f} {(fin_y1-fin_y0)/2:.6f} "
                    f"{_lk.FIN_H/2000.0:.6f}",
                    f"{_lk.U1[0]/1000.0:.6f} {sgn*(fin_y0+fin_y1)/2:.6f} "
                    f"{_lk.U1[1]/1000.0:.6f}", C_LINK))

    notch_x, notch_z = 0.0124, 0.0454
    for sgn in (1, -1):
        for x0, x1 in ((-0.0365, -notch_x), (notch_x, 0.0535)):
            g.append(_v(f"vside{sgn}f{x0:.3f}", "box",
                        f"{(x1-x0)/2:.6f} 0.0015 {notch_z/2:.6f}",
                        f"{(x0+x1)/2:.6f} {sgn*0.0381:.4f} {notch_z/2:.6f}",
                        C_PLATE))
        g.append(_v(f"vside{sgn}", "box", "0.045 0.0015 0.0673",
                    f"{x} {sgn*0.0381:.4f} {notch_z+0.0673:.6f}", C_PLATE))
    # The RS-BOT decal, on the outside of each side plate. Cosmetic only: no
    # mass, no collision, and NOT in PRINTED_VIS, so the mesh swap leaves it
    # alone and it survives meshes=True. 60 x 60 mm, sitting 0.2 mm proud so it
    # does not z-fight with the plate it is stuck to. Generated by cad/decal.py.
    for sgn in ((1, -1) if _has_decal() else ()):
        g.append(f'<geom name="vdecal{sgn}" type="box" '
                 f'size="0.030 0.0002 0.030" '
                 f'pos="{x} {sgn*0.0397:.4f} 0.120" '
                 f'material="{"decal_r" if sgn > 0 else "decal"}" '
                 f'contype="0" conaffinity="0" mass="0" group="0"/>')
    g.append(_v("vtop", "box", "0.045 0.0366 0.0015", f"{x} 0 0.1785", C_PLATE))
        # Shelf 1 is at the hip servo's height, so it is two segments with a gap
    # where the servo passes - not a narrowed slab. Narrowing it cleared the
    # servo and left the shelf 2 mm short of the side plates, which the
    # one-rigid-piece test called out as a torso in four loose parts.
    for x0, x1 in ((-0.0365, -0.0124), (0.0124, 0.0535)):
        g.append(_v(f"vshelf1{x0:.3f}", "box",
                    f"{(x1-x0)/2:.6f} 0.0366 0.0015",
                    f"{(x0+x1)/2:.6f} 0 0.0200", C_PLATE))
    # Sits ON 4 mm standoffs above the shelf, not on the shelf itself: the
    # PCB needs clearance underneath for its through-hole legs.
    g.append(_v("vpistand", "box", "0.040 0.026 0.002", f"{x} 0 0.0235", C_PLATE))
    g.append(_v("vpi", "box", f"{PI5[0]/2} {PI5[1]/2} {PI5[2]/2}",
                f"{x} 0 0.0340", C_PCB))
    # Stood on end: 105 mm will not fit across a 90 mm bay.
    g.append(_v("vshelf2", "box", "0.045 0.0366 0.0015", f"{x} 0 0.0610", C_PLATE))
    g.append(_v("vbatt", "box", f"{BATT_3S[2]/2} {BATT_3S[1]/2} {BATT_3S[0]/2}",
                f"{x} 0 0.115", C_BATT))
    # Above the Pi now that the Pi sits on standoffs, still bolted to the
    # side plate.
    g.append(_v("vdriver", "box", "0.025 0.010 0.005", f"{x} 0 0.0545", C_PCB))
    if meshes:
        g = _swap_meshes(g, "torso", "l", 1)
    g += _hip_servos()
    if meshes:
        g = _servo_meshes(g)
    return "\n      ".join(g)



def _has_decal():
    """The decal is a GENERATED artifact and cad/out/ is gitignored, so a fresh
    clone does not have it until `python -m cad.decal` is run.

    Both the assets AND the geoms that reference them have to be conditional on
    this. Only the assets were, which meant a clean checkout loaded a geom
    pointing at a material that did not exist and the model refused to build:
    "material 'decal_r' not found in geom 18". A cosmetic sticker must never be
    able to stop the robot loading.
    """
    return ((CAD_OUT / "decal.png").exists()
            and (CAD_OUT / "decal_r.png").exists())


def _decal_assets():
    """<texture>/<material> for the side decal, if it has been generated."""
    f = CAD_OUT / "decal.png"
    fr = CAD_OUT / "decal_r.png"
    if not _has_decal():
        return ""
    # type="cube", not "2d". A 2d texture on a BOX tiles in spatial units and
    # smears the artwork across the whole part; a cube texture from a single
    # image puts the whole image on each face, which is what a sticker is.
    return "\n    ".join(
        f'<texture name="{n}" type="cube" file="{p}"/>\n    '
        f'<material name="{n}" texture="{n}" texuniform="false" '
        f'texrepeat="1 1" specular="0.25" shininess="0.35"/>'
        for n, p in (("decal", f), ("decal_r", fr)))


# link -> the hardware meshes that ride on it. From cad/hardware.py, which
# writes them in each body's own frame; see local_parts() there.
#
# WHY THIS IS IN THE SIM AND NOT ONLY IN THE CAD VIEWER. The bearings, shafts,
# pulleys and belt are what actually hold this robot together, and until they
# were here the twin showed printed parts floating with nothing between them.
# A digital twin you cannot see the joints of is not one.
HW_MESHES = {
    # The pitch SHAFT and its bearing stay: the joint still exists. The 40T
    # pulley, the 20T and the belt have gone with the servo that needed them.
    "ankle": [("hw_shaft_roll", C_STEEL), ("hw_bearing_roll", C_STEEL),
              ("hw_shaft_pitch", C_STEEL), ("hw_bearing_pitch", C_STEEL)],
    "shin": [("hw_horn_vkneesv", C_STEEL)],
    "thigh": [("hw_horn_vhipsv", C_STEEL)],
    "rollbracket": [("hw_horn_vrollsv", C_STEEL)],
    "wheel": [("hw_horn_vwhlsv", C_STEEL)],
}

def _hw_assets():
    """<mesh> entries for the bought hardware, mirrored for the right side."""
    out = []
    for stems in HW_MESHES.values():
        for stem, _ in stems:
            f = CAD_OUT / f"{stem}.stl"
            if not f.exists():
                raise FileNotFoundError(
                    f"{f} is missing - run `uv run python -m cad.hardware "
                    f"--export` once before loading with meshes=True")
            for side, sgn in (("l", 1), ("r", -1)):
                out.append(f'<mesh name="{stem}_{side}" file="{f}" '
                           f'scale="0.001 {0.001 * sgn} 0.001"/>')
    return "\n    ".join(out)


# THE MESH BUILD USED TO CARRY 2 MORE DOF THAN THE PLAIN ONE. The belt's 20T
# pulley was its own hinged body, geared to the ankle by an equality, and it
# only existed when meshes=True - so the twin's degree-of-freedom count depended
# on a display flag, which is the kind of split this project keeps getting
# bitten by. It went with the belt.
def _parallelogram(side):
    """hip + knee + ankle_pitch = 0, enforced as a constraint.

    That sum IS the foot's orientation relative to the torso, which is what
    src/rsbot/model.py's ankle_pitch_level() commands a servo to hold. A
    four-bar linkage holds it mechanically instead, so this models the linkage
    as what it is: a rigid kinematic constraint, not an actuator.
    """
    return (f'    <fixed name="par_{side}">\n'
            f'      <joint joint="hip_{side}" coef="1"/>\n'
            f'      <joint joint="knee_{side}" coef="1"/>\n'
            f'      <joint joint="ankle_pitch_{side}" coef="1"/>\n'
            f'    </fixed>')


def _constraints():
    """The tendons that hold the parallelogram, and their equalities.

    FOUR per leg, not one. `par` is the one that matters mechanically - it is
    the ankle, held at -(hip + knee) - and the other three pin the linkage's own
    bodies to the angles the mechanism puts them at. Those three change no
    physics; they are what makes the rods and the idler appear where they
    really are instead of swinging free on their hinges.
    """
    import cad.linkage_dims as lk

    names = ["par"] + [b[0] for b in lk.SIM_BODIES]
    tendons = "\n".join(_parallelogram(s) + "\n" + _linkage_tendons(s)
                        for s in ("l", "r"))
    eqs = "\n".join(f'    <tendon tendon1="{n}_{s}"/>'
                    for s in ("l", "r") for n in names)
    return (f"\n  <tendon>\n{tendons}\n  </tendon>\n"
            f"  <equality>\n{eqs}\n  </equality>")


def _linkage_assets():
    """<mesh> entries for the parallelogram, mirrored for the right side."""
    import cad.linkage_dims as lk

    out = []
    for stem in lk.SIM_MESH.values():
        f = CAD_OUT / f"{stem}.stl"
        if not f.exists():
            raise FileNotFoundError(
                f"{f} is missing - run `uv run python -m cad.linkage` once "
                f"before loading with meshes=True")
        for side, sgn in (("l", 1), ("r", -1)):
            out.append(f'<mesh name="{stem}_{side}" file="{f}" '
                       f'scale="0.001 {0.001 * sgn} 0.001"/>')
    return "\n    ".join(out)


def _hw_geoms(link, side):
    """The hardware geoms for one link, visual only."""
    return [f'<geom name="{stem}_{side}" type="mesh" mesh="{stem}_{side}" '
            f'rgba="{rgba}" contype="0" conaffinity="0" mass="0" group="0"/>'
            for stem, rgba in HW_MESHES.get(link, [])]


def _mesh_assets(meshes):
    """<mesh> entries for the CAD parts. Right-hand parts are the same STL with
    a negative y scale, which is how a mirrored part is expressed in MJCF."""
    if not meshes:
        return ""
    out = []
    for link, stem in MESH_STEM.items():
        sides = ("l",) if link == "torso" else ("l", "r")
        for side in sides:
            sgn = 1 if side == "l" else -1
            f = CAD_OUT / f"{stem}.stl"
            if not f.exists():
                raise FileNotFoundError(
                    f"{f} is missing - run `uv run python -m cad.robot` once to "
                    f"export the STLs before loading the model with meshes=True")
            out.append(f'<mesh name="cad_{stem}_{side}" file="{f}" '
                       f'scale="0.001 {0.001 * sgn} 0.001"/>')
    f = CAD_OUT / "servo.stl"
    if not f.exists():
        raise FileNotFoundError(
            f"{f} is missing - run `uv run python -m cad.robot` once to "
            f"export the STLs before loading the model with meshes=True")
    out.append(f'<mesh name="cad_servo" file="{f}" scale="0.001 0.001 0.001"/>')
    for link, parts in MESH_MULTI.items():
        for stem, _rgba in parts:
            f = CAD_OUT / f"{stem}.stl"
            if not f.exists():
                raise FileNotFoundError(
                    f"{f} is missing - run `uv run python -m cad.robot` once to "
                    f"export the STLs before loading the model with meshes=True")
            for side in ("l", "r"):
                # NOT mirrored: same part, flipped by the euler below.
                out.append(f'<mesh name="cad_{stem}_{side}" file="{f}" '
                           f'scale="0.001 0.001 0.001"/>')
    return "\n    ".join(out)


def _excludes():
    """Contact excludes for every link pair inside a leg.

    MuJoCo only auto-excludes DIRECT parent-child pairs. The ankle body already
    made shin and wheel grandparent/grandchild, and switching backlash on adds
    a drive body at every joint, which breaks the remaining pairs too. Without
    these the leg collides with itself: 14 contacts at rest and qacc of 2e4.
    Listing them all is cheap and does not depend on the chain's shape.
    """
    links = ["thigh", "shin", "ankle", "rollbracket", "wheel"]
    out = []
    for s in ("l", "r"):
        out.append(f'    <exclude body1="torso" body2="thigh_{s}"/>')
        for i, a in enumerate(links):
            for b in links[i + 1:]:
                out.append(f'    <exclude body1="{a}_{s}" body2="{b}_{s}"/>')
    return "  <contact>\n" + "\n".join(out) + "\n  </contact>"


def _com_offset(m, d):
    """Standing CoM offset from the wheel axle. Looks the torso up by name:
    the scale-reference human is also a top-level body, so index 1 is not
    safe."""
    mujoco.mj_forward(m, d)
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")]
    return d.subtree_com[torso][0] - axle[0]


STANCE = {"hip": 0.35, "knee": -0.70, "ankle_pitch": 0.35,
          "ankle_roll": 0.0, "wheel": 0.0}


def _stance():
    """STANCE plus the linkage hinges, which are DERIVED from it.

    A linkage hinge's stance angle is whatever makes its tendon zero, so it is
    computed from the couplings rather than written down beside them. Written
    down, the two would eventually disagree and the robot would start the sim
    with its rods a degree out of assembly.
    """
    import cad.linkage_dims as lk

    out = dict(STANCE)
    out.update(lk.sim_stance(STANCE))
    return out


def _write_stance(m):
    """Rewrite keyframe 0 by joint NAME.

    The lash joints interleave into the joint ordering, so a hand-written
    qpos vector silently scrambles across the wrong joints the moment
    backlash is switched on. Addressing by name is the only safe way.
    """
    d = mujoco.MjData(m)
    d.qpos[:] = 0.0
    d.qpos[0:3] = [0.0, 0.0, 0.247]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        if not name or name == "root":
            continue
        key = name.rsplit("_", 1)[0] if name.endswith(("_l", "_r")) else None
        d.qpos[m.jnt_qposadr[i]] = 0.0 if key is None else _stance().get(key, 0.0)
    m.key_qpos[0] = d.qpos

    ctrl = np.zeros(m.nu)
    for i in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        key = {"apit": "ankle_pitch", "aroll": "ankle_roll"}.get(
            name.rsplit("_", 1)[0], name.rsplit("_", 1)[0])
        ctrl[i] = STANCE.get(key, 0.0)
    m.key_ctrl[0] = ctrl


def load(trim=None, backlash=0.0, meshes=False):
    """Return (model, data) reset to the stance keyframe.

    `backlash` is total gear lash per joint in radians, split +/- either side.
    Zero builds the rigid model with no extra bodies, so joint indices are
    unchanged. See docs/backlash.md.

    THE PARALLELOGRAM IS NOT OPTIONAL any more and there is no flag for it. It
    used to be `parallel=True`, a proposal sitting beside the belt-driven servo
    it was competing with, and keeping both meant keeping two robots: two
    actuator counts, two ctrl layouts, two mass budgets. The comparison is
    settled and recorded in docs/deleting-the-ankle-pitch-servo.md, and the
    before/after picture is built from the CAD by `cad.robot compare`, so
    nothing needs the old one to still be loadable.

    `trim` is the fore/aft offset of the torso mass, solved for rather than
    hardcoded: build once, measure how far the standing CoM sits from the axle,
    shift the torso to null it. Linear, so one pass is exact. Any residual
    offset shows up as a permanent standing lean.
    """
    def build(x_trim, meshes):
        mass, com, I = SEG_INERTIA["torso"]
        ixx, iyy, izz, ixy, ixz, iyz = I
        inertial = (f'<inertial pos="{x_trim:.6f} {com[1]:.6f} {com[2]:.6f}" '
                    f'mass="{mass:.6f}" fullinertia="{ixx:.6e} {iyy:.6e} '
                    f'{izz:.6e} {ixy:.6e} {ixz:.6e} {iyz:.6e}"/>')
        return (XML.read_text()
                .replace("<!--LEGS-->", _leg("l", backlash, meshes)
                         + _leg("r", backlash, meshes))
                .replace("<!--EXCLUDES-->", _excludes() + _constraints())
                .replace("<!--MESHES-->", _decal_assets() + "\n    "
                          + _mesh_assets(meshes)
                          + ("\n    " + _hw_assets()
                             + "\n    " + _linkage_assets() if meshes else ""))
                .replace("<!--TORSO_INERTIAL-->", inertial)
                .replace("<!--TORSO_VIS-->", _torso_visual(meshes)))

    nominal = SEG_INERTIA["torso"][1][0]
    if trim is None:
        # Probe on the primitive build regardless of `meshes`: the meshes are
        # visual and massless, so they cannot move the centre of mass, and
        # meshing is much the slower of the two to load.
        probe = mujoco.MjModel.from_xml_string(build(nominal, False))
        _write_stance(probe)
        pd = mujoco.MjData(probe)
        mujoco.mj_resetDataKeyframe(probe, pd, 0)
        t = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, "torso")
        trim = nominal - _com_offset(probe, pd) * (probe.body_subtreemass[t]
                                                   / probe.body_mass[t])

    m = mujoco.MjModel.from_xml_string(build(trim, meshes))
    _write_stance(m)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d
