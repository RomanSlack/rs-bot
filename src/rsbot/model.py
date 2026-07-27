"""Model loading and shared geometry."""

import math
from pathlib import Path

import mujoco
import numpy as np

XML = Path(__file__).parent / "model" / "rsbot.xml"
CAD_OUT = Path(__file__).parents[2] / "cad" / "out"

WHEEL_R = 0.040
WHEEL_HALF_W = 0.012
THIGH_L = 0.110
SHIN_L = 0.110

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


def make_ctrl(hip, knee, apitch, aroll, wheel, wheel_r=None):
    """Assemble the 10-actuator command.

    Legs share the joint angles; the wheels can differ, which is how the robot
    steers - there is no steering joint, only a speed difference.
    """
    if wheel_r is None:
        wheel_r = wheel
    return np.array([hip, knee, apitch, aroll, wheel,
                     hip, knee, apitch, aroll, wheel_r])


# --- Real part dimensions, metres --------------------------------------------
#
# Visual-only geometry built from the actual parts, so the render shows what
# would really be bolted together. It carries no mass and no collision: mass
# lives on the simple collision shapes, which are moved to geom group 4 and
# hidden. Nothing here changes the dynamics. See docs/bom.md.

# STS3215, MEASURED off TheRobotStudio's STEP model rather than taken from a
# product listing. See cad/servo.py. The third figure is along the OUTPUT
# SHAFT and was 4.2 mm short, which is exactly where the horn clearances are.
SERVO = (0.0454, 0.0248, 0.0396)      # STS3215 body, L x W x H(shaft)
SERVO_HORN_R = 0.0100                 # 25T output horn
PI5 = (0.085, 0.056, 0.017)
BATT_3S = (0.105, 0.034, 0.024)       # 2200 mAh 3S pack
DRIVER = (0.050, 0.030, 0.010)        # TTL bus adapter
WHEEL_TIRE_T = 0.008                  # tyre wall thickness

C_SERVO = "0.13 0.13 0.15 1"
C_HORN = "0.72 0.73 0.76 1"
C_PRINT = "0.88 0.45 0.13 1"          # printed PETG
C_TIRE = "0.09 0.09 0.10 1"
C_HUB = "0.55 0.56 0.60 1"
C_PCB = "0.05 0.33 0.17 1"
C_BATT = "0.16 0.16 0.38 1"
C_PLATE = "0.62 0.64 0.68 0.30"   # translucent, so the internals show


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
    "thigh": (0.090187, (+0.000000, -0.000805, -0.080357),
              (1.129334e-04, 8.998957e-05, 3.107577e-05, -1.247795e-20, -4.154540e-20, -2.777948e-05)),
    "shin": (0.125175, (+0.005662, +0.056758, -0.051407),
              (1.866848e-04, 1.471591e-04, 9.130730e-05, 8.984832e-06, 9.993500e-06, 5.610963e-05)),
    "ankle": (0.069119, (-0.069563, +0.008099, +0.010346),
              (4.134228e-05, 4.105159e-05, 5.993474e-05, -2.194376e-05, 5.212387e-06, 5.791592e-06)),
    "rollbracket": (0.060970, (-0.003051, +0.031401, +0.001212),
              (1.369960e-05, 2.022070e-05, 2.581655e-05, -3.746565e-06, 1.553582e-06, 9.696260e-07)),
    "torso": (1.059162, (+0.007697, +0.000000, +0.085038),
              (3.658079e-03, 3.452785e-03, 1.281970e-03, -7.275958e-21, -5.294932e-05, 0.000000e+00)),
    "wheel": (0.054044, (+0.000000, -0.000836, -0.000000),
              (3.235652e-05, 5.846958e-05, 3.235656e-05, -5.620039e-17, 1.773251e-16, 3.057042e-15)),
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
PRINTED_VIS = ("vthigh", "vshin", "vankstand", "vshinarm", "vshinpost",
               "vankpost", "vankface", "vrollarm", "vrolltie",
               "vside1", "vside-1", "vtop", "vshelf1", "vshelf2", "vpistand")
MESH_STEM = {"thigh": "thigh", "shin": "shin", "ankle": "ankle_yoke",
             "rollbracket": "roll_bracket", "torso": "chassis"}


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



def _swap_meshes(vis, link, side, sgn):
    """Printed stand-in boxes out, the real part in."""
    kept = [g for g in vis
            if not any(f'name="{p}_{side}"' in g or f'name="{p}"' in g
                       for p in PRINTED_VIS)]
    return kept + [_mesh_geom(link, side, sgn)]


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
    outb = sgn * (SPINE_Y + SPY + HH)     # servo flush outboard of the spine

    if link == "thigh":
        col.append(f'<geom class="leg" name="thigh_{side}" fromto="0 0 0  0 0 -0.110" '
                   f'group="4"/>')
        # Stops short of the knee: the shin swings 40 deg there and would
        # otherwise scissor into it. The knee servo bridges the gap.
        # 16 mm wide in y, not 12: with no hip roll joint, the STRUCTURE
        # carries the whole lateral moment at the hip (9.5 N.m at the design
        # load), and at 12 mm that is only a 2.0x margin. See cad/thigh.py.
        vis.append(_v(f"vthigh_{side}", "box", f"0.010 0.008 0.049",
                      f"0 {sgn*0.023:.5f} -0.053", C_PRINT))
        # INBOARD. The thigh wraps over and down the inboard side to reach it,
        # because the outboard band is where the shin's hub has to be.
        vis.append(_v(f"vkneesv_{side}", "box", f"{HW:.5f} {HH:.5f} {HL:.5f}",
                      f"0 {-sgn*(HH - SPINE_Y + SPY):.5f} {-0.110+off:.5f}",
                      C_SERVO))
    elif link == "shin":
        col.append(f'<geom class="leg" name="shin_{side}" fromto="0 0 0  0 0 -0.094" '
                   f'group="4"/>')
        # Stops 55 mm above the axle. Below that it is inside the volume the
        # wheel-drive servo SWEEPS as the ankle rolls: that servo turns with
        # the roll bracket and carves an annulus 14-50 mm from the roll axis,
        # for |x| < 23 mm. Clearing the two end poses is not enough.
        vis.append(_v(f"vshin_{side}", "box", f"0.010 {SPY} 0.0245",
                      f"0 {y:.5f} -0.0245", C_PRINT))
        # Ankle-pitch servo. It cannot be coaxial with its own joint, because
        # the wheel already owns that axle, so it sits high on the shin and
        # drives down through a belt that is not drawn.
        # Stood off 7 mm further outboard than the spine face. At the flush
        # position its inner corner sits 49 mm from the roll axis, just inside
        # the wheel-servo sweep, and clips it at mid-flip by 2.4 mm.
        vis.append(_v(f"vankstand_{side}", "box", f"0.008 0.0035 0.022",
                      f"0 {sgn*(SPINE_Y+SPY+0.0035):.5f} {-0.110+0.0768:.5f}",
                      C_PRINT))
        vis.append(_v(f"vanksv_{side}", "box", f"{HW:.5f} {HH:.5f} {HL:.5f}",
                      f"0 {outb+sgn*0.007:.5f} {-0.110+0.0768:.5f}", C_SERVO))
        # The SHIN carries the ankle bearing, so the member reaching down to
        # the ankle axis belongs here. It steps aft high up, where the radius
        # from the roll axis already clears the servo sweep, then drops at
        # |x| > 32 mm, which also clears the flat wheel.
        vis.append(_v(f"vshinarm_{side}", "box", f"0.016 {SPY} 0.007",
                      f"-0.026 {y:.5f} -0.048", C_PRINT))
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
                      f"0.0285 {y:.5f} -0.042", C_PRINT))
        vis.append(_v(f"vshinfpost_{side}", "box", f"0.005 {SPY} 0.011",
                      f"0.042 {y:.5f} -0.059", C_PRINT))
        vis.append(_v(f"vshincross_{side}", "box", "0.005 0.0315 0.0045",
                      f"0.042 {sgn*0.0465:.5f} -0.0655", C_PRINT))
        vis.append(_v(f"vshindrop_{side}", "box", "0.005 0.005 0.0265",
                      f"0.042 {sgn*0.071:.5f} -0.0875", C_PRINT))
        vis.append(_v(f"vshinback_{side}", "box", "0.0215 0.005 0.006",
                      f"0.0215 {sgn*0.071:.5f} -0.108", C_PRINT))
    elif link == "ankle":
        col.append(f'<geom class="ankle" name="ankle_{side}" '
                   f'size="{HW:.5f} {HH:.5f} {HL:.5f}" pos="0 0 {HL:.5f}" '
                   f'group="4"/>')
        # Yoke reaching AFT along the roll axis to a bearing clear of the
        # wheel disc, then up to meet the shin.
        # Bearing carrier, aft on the ROLL AXIS. Two clearances fall out of
        # that: it is inside the hole of the wheel-servo annulus (radius under
        # 14 mm, so the sweep misses it), and at |x| > 40 mm it is outside both
        # the upright and the flat wheel.
        # Between the roll servo and the wheel, not inside either: the servo
        # case ends at x = -58, and |x| > 40 keeps it clear of the flat wheel.
        vis.append(_v(f"vankpost_{side}", "box", "0.003 0.006 0.006",
                      "-0.048 0 0", C_PRINT))
        # Ring the roll servo bolts to, with the coupler turning inside it.
        vis.append(_v(f"vankring_{side}", "box", "0.0035 0.017 0.002",
                      "-0.0545 0 0.015", C_PRINT))
        vis.append(_v(f"vanktie_{side}", "box", "0.0035 0.006 0.005",
                      "-0.0485 0 0.011", C_PRINT))
        # Mounting face the roll servo bolts to, bridging it to the carrier.
        # Kept within 8.5 mm of the roll axis: the roll bracket's tie sweeps
        # an annulus 8.7-19.6 mm out, so anything reaching into that band gets
        # hit partway through the flip.

        # The ankle-pitch joint, which was not drawn at all until now: the
        # shin had a bore 52.8 mm off the axis and the yoke had no matching
        # feature, so the two parts came no closer than 10.3 mm.
        vis.append(_v(f"vyokecross_{side}", "box", "0.003 0.0325 0.008",
                      f"-0.048 {sgn*0.0265:.5f} 0", C_PRINT))
        vis.append(_v(f"vyokefwd_{side}", "box", "0.028 0.005 0.008",
                      f"-0.028 {sgn*0.061:.5f} 0", C_PRINT))
        vis.append(_v(f"vyokeboss_{side}", "cylinder", "0.007 0.005",
                      f"0 {sgn*0.061:.5f} 0", C_PRINT, euler="1.5708 0 0"))
        vis.append(_v(f"vrollsv_{side}", "box", f"{HH:.5f} {HW:.5f} {HL:.5f}",
                      # Lifted off the roll axis so it clears the floor in foot mode,
                      # where the axle is only 12 mm up. Safe despite the larger
                      # radius because |x| > 23 mm puts it outside the wheel-servo sweep.
                      f"{-(0.058+HH):.5f} 0 0.0125", C_SERVO))
    elif link == "rollbracket":
        # Wheel-drive servo and bearing block, OUTBOARD. The 90 deg roll maps
        # +y onto +z, so outboard becomes directly above the flat wheel, which
        # is the only place a support for a vertical shaft can live.
        vis.append(_v(f"vwhlsv_{side}", "box", f"{HL:.5f} {HH:.5f} {HW:.5f}",
                      f"0 {sgn*(0.0135+HH):.5f} 0", C_SERVO))
        # The printed arm the wheel servo bolts to. It was dropped when the
        # tie and tongue were replaced by the web, which left the bracket as
        # two loose pieces in the sim - caught by the one-rigid-piece test.
        vis.append(_v(f"vrollarm_{side}", "box", "0.022 0.0057 0.005",
                      f"-0.022 {sgn*0.0178:.5f} 0.01735", C_PRINT))
        # Web straight from the arm to the roll axis, at x = -45..-41: the one
        # window forward of the yoke's post that is still outside the 40 mm
        # wheel. The tie, tongue and corner rib it replaces existed only to
        # crank round the shin's old post.
        vis.append(_v(f"vrollweb_{side}", "box", "0.002 0.012 0.011",
                      f"-0.043 {sgn*0.012:.5f} 0.011", C_PRINT))
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
    if meshes and link in MESH_STEM:
        vis = _swap_meshes(vis, link, side, sgn)
    return col + vis


# joint, link body, offset from the parent
CHAIN = [("hip", "thigh", None), ("knee", "shin", "0 0 -0.110"),
         ("ankle_pitch", "ankle", "0 0 -0.110"),
         ("ankle_roll", "rollbracket", "0 0 0"), ("wheel", "wheel", "0 0 0")]


def _leg(side, backlash, meshes=False):
    """One leg: hip pitch, knee pitch, ankle pitch, ankle ROLL, wheel.

    The roll bracket is its own body because the wheel-drive servo bolts to it
    and must NOT spin with the wheel.
    """
    sgn = 1 if side == "l" else -1
    y = sgn * 0.060
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

        if backlash > 0:
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

    return "".join(opens) + "".join(reversed(closes))


def _hip_servos():
    """Hip servo cases bolt to the torso, not the thigh."""
    g = []
    for sgn in (1, -1):
        # Flush inboard of the thigh spine, and reaching back to the chassis.
        g.append(_v(f"vhipsv{sgn}", "box", f"{HW:.5f} {HH:.5f} {HL:.5f}",
                    f"0 {sgn*(0.060+SPINE_Y-SPY-HH):.5f} {HL:.5f}", C_SERVO))
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
    notch_x, notch_z = 0.0124, 0.0454
    for sgn in (1, -1):
        for x0, x1 in ((-0.0365, -notch_x), (notch_x, 0.0535)):
            g.append(_v(f"vside{sgn}f{x0:.3f}", "box",
                        f"{(x1-x0)/2:.5f} 0.0015 {notch_z/2:.5f}",
                        f"{(x0+x1)/2:.5f} {sgn*0.0381:.4f} {notch_z/2:.5f}",
                        C_PLATE))
        g.append(_v(f"vside{sgn}", "box", "0.045 0.0015 0.0673",
                    f"{x} {sgn*0.0381:.4f} {notch_z+0.0673:.5f}", C_PLATE))
    g.append(_v("vtop", "box", "0.045 0.0366 0.0015", f"{x} 0 0.1785", C_PLATE))
        # Shelf 1 is at the hip servo's height, so it is two segments with a gap
    # where the servo passes - not a narrowed slab. Narrowing it cleared the
    # servo and left the shelf 2 mm short of the side plates, which the
    # one-rigid-piece test called out as a torso in four loose parts.
    for x0, x1 in ((-0.0365, -0.0124), (0.0124, 0.0535)):
        g.append(_v(f"vshelf1{x0:.3f}", "box",
                    f"{(x1-x0)/2:.5f} 0.0366 0.0015",
                    f"{(x0+x1)/2:.5f} 0 0.0200", C_PLATE))
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
    return "\n      ".join(g)



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
        d.qpos[m.jnt_qposadr[i]] = 0.0 if key is None else STANCE.get(key, 0.0)
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
                .replace("<!--EXCLUDES-->", _excludes())
                .replace("<!--MESHES-->", _mesh_assets(meshes))
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
