"""The passive ankle parallelogram, as a mechanism rather than a constraint.

    uv run python -m cad.linkage

docs/deleting-the-ankle-pitch-servo.md proved this as a CONSTRAINT: a fixed
tendon holding hip + knee + ankle_pitch = 0, which stands foot mode 10/10 at
the 2-3 degrees of gear lash where the servo and its ankle loop manage 2 or 3.
What that could not settle is everything a tendon does not have: where the
links go, whether they fit, how hard they are pushed, and what the PIN PLAY
does to the foot angle the whole idea is bought for.

This is that half. The tendon is one line; the mechanism is five pivots.

WHAT IT REPLACES, all of which comes out with the ankle servo:

    the servo itself, 2 of them                          ~ 2 x 55 g
    a 2:1 GT2 belt drive, 15 mm wide, 188 mm long
    a 40T driven pulley that is 2.23 mm THROUGH THE FLOOR in foot mode
    a 20T pulley on a 4.40 mm hub, which is not a catalogue part
    the ankle-pitch bearing forced outboard, which is what makes this
    robot 306 mm wide

THE TOPOLOGY IS TWO PARALLELOGRAMS IN SERIES, not one, and that is forced.
Holding the foot parallel to the TORSO across two joints needs the torso's
attitude carried down past the knee first. So:

    stage 1   torso -> thigh -> idler      holds the IDLER level
    stage 2   idler -> shin  -> ankle      holds the FOOT level

The idler is one bar on the knee axis, free to turn on both neighbours, with a
pin at each end. Stage 1 pins the bar's angle in the torso frame; stage 2 hands
that angle on to the yoke. Because both stages are exact parallelograms, the
foot angle is hip + knee + ankle = 0 by construction rather than by fit, which
is the point: a linkage with unequal links would only approximate it.

WHERE IT LIVES, and this is the whole packaging answer: OUTBOARD, in the band
the ankle-pitch bearing already occupies. It was swept rather than chosen, and
the inboard corridor between the legs was tried first and rejected: the rods fit
there, but the yoke has no material inboard of y = -6 and the route to it is
blocked by the roll bracket above and by the floor below. The one straight arm
that clears the bracket sits 1 mm off the ground in foot mode.

Outboard it is easy, because the yoke is ALREADY out there: YOKE_FWD spans
y = 56 .. 66 from x = -56 to 0, so stage 2's arm is that member carried 30 mm
further forward rather than a new limb. corridor() sweeps a whole linkage in one
plane, over squat and flip, against the real solids:

    offset      y = 48      y = 52      y = 56
    +26 mm       clear       clear     127 mm3
    +30 mm       clear       clear     139 mm3
    +34 mm     22 mm3      20 mm3      147 mm3

+34 grazes the shin's ankle carrier and +26 has no more room than +30, so 30 is
the middle of the only window there is. The parts do not all end up in one plane
- see the stack below - but that scan is what said there was a window at all.

That function is committed for a reason. status/2026-08-03 published a corridor
table for this linkage from a script nobody kept, and it does not reproduce.

THE TWO PINS ARE SPLIT IN Z, by 6 mm either side, and that is not cosmetic.
With one shared offset both rods would want the same pin, and two eyes cannot
share one pin span. Splitting them also improves both transmission angles at
once, because the thigh leans back and the shin leans forward: worst-case sin
goes from 0.84 to 0.93. Which pin goes high matters too, and the obvious way
round is wrong: stage 1 low and stage 2 high keeps the rods apart AND keeps
stage 2's pin above the axle, which is 12 mm off the floor in foot mode. The
other way round crossed the rods for 950 mm3 and put the yoke arm 1 mm off the
ground.
"""

import itertools
import sys

import build123d as bd
import mujoco
import numpy as np

from cad.hardware import BEARING_ID, BEARING_OD, BEARING_W
from cad.wheel_dims import HALF_W as WHEEL_HALF_W, R as WHEEL_R

# --- the mechanism -------------------------------------------------------------
#
# All of it in the left leg's local frame, millimetres, x forward and z up, the
# same convention as cad/shin.py and cad/ankle.py. The right leg mirrors in y.

THIGH_L = 110.0               # hip to knee
SHIN_L = 110.0                # knee to ankle

# Crank radius: the offset of every pin from the axis it works about. It sets
# the rod force (torque / R), the swept width, and how much a given error at a
# pin is worth at the foot, which goes as 1/R. Clearance chose it and not
# strength: 34 mm grazes the shin, 26 works, 30 is the middle of the window.
R = 30.0
DZ = 6.0                      # the idler's two pins, either side of the offset

# The linkage plane, leg-local y. Positive is OUTBOARD.
#
# The wheel is what decides it, and it is worth writing down as arithmetic
# rather than as a preference. In foot mode the wheel is an 80 mm platter 24 mm
# thick lying about the axle, so every point within hypot(40, 12) = 41.8 mm of
# the axle is inside it at SOME roll angle. Anything hung off the yoke has to be
# further out than that in one axis or another, and y is the cheap axis: a part
# at |y| > 41.8 is clear of the wheel whatever its x and z do. That, and the
# wheel-drive servo's swept corner at 51.5, is what sets the stack below.
FLAT_WHEEL_R = float(np.hypot(WHEEL_R, WHEEL_HALF_W))

# A STACK, not a plane, and the first version of this file had a plane. Six
# parts meet at five pivots and no two of them can share a y band, or they are
# not a joint, they are the same lump of plastic. The clearance check said so
# four separate ways, at 430 to 950 mm3 a time.
#
# So each part gets its own slice and every pin crosses one gap. Ordered by
# what constrains each one, inboard to outboard:
#
#   31.0  torso fin   reaches out from the chassis plate at y = -20. Every
#                     millimetre of it is cantilever, and a pin that moves is a
#                     foot that tilts, so it stops at the first slice it can.
#   38.0  rod 1       free air: the thigh ends at 31 and the knee servo at 15.
#   48.0  idler       on the knee axis. Not 45: the shin carries material out
#                     to y = 44 within 24 mm of the knee, which a probe along
#                     the axis alone does not see, and the clearance check
#                     found the bar grazing it by 103 mm3 at full squat.
#   54.5  rod 2       must clear the flat wheel, hypot(40,12) = 41.8, and the
#                     wheel-drive servo's swept corner at 51.5.
#   61.0  yoke arm    inside YOKE_FWD's own band, y = 56 .. 66, so stage 2's
#                     arm is an existing member carried forward and not a new
#                     one. Its outboard face has to stay under 66, where the
#                     shin's ankle-bearing carrier starts.
#
# The steps are not equal and there is no reason they should be. Each slice is
# where its own part can be; 6 mm is only the minimum, which is what stops two
# 6 mm-wide parts sharing a band.
Y_FIN = 31.0
Y_ROD1 = 38.0
Y_IDLER = 48.0
Y_ROD2 = 54.5
Y_ARM = 61.0

# Both offsets forward, split in z, for the reasons in the module docstring.
# Given as (x, z) in the TORSO frame, which is the only frame they are constant
# in. Stage 1 sits low because the thigh leans back and stage 2 sits high
# because the shin leans forward, which keeps both transmission angles above
# 69 degrees rather than one of them at 57. The other way round would do that
# too and is wrong for two other reasons: it crosses the rods below the knee,
# and it puts stage 2's pin BELOW an axle that is 12 mm off the floor.
U1 = np.array([+R, -DZ])
U2 = np.array([+R, +DZ])

# --- sections ------------------------------------------------------------------
#
# The rods are in pure tension and compression (a two-force member cannot be
# anything else), at 38.7 N. They are sized by buckling, not by stress, and by a
# long way - 18x, see loads() - so 8 x 6 is the smallest section that still
# prints with four perimeters and does not look like a wire.
ROD_W = 6.0                   # along y
ROD_T = 8.0                   # in the plane of motion
PIN_D = 3.0                   # the 3 mm shaft everything else in this robot uses
EYE_R = 5.0                   # rod eye outer radius, 3.5 mm of wall on the pin
IDLER_T = 8.0                 # the idler bar's in-plane depth
IDLER_W = 6.0
IDLER_HUB_R = 8.0             # over a 623ZZ, 10 mm outside
PIN_REACH = 18.0              # pin: its own slice, across the gap, through
                              # the eye and 2 past. Sized on the widest gap

# The shin's outboard face at the knee axis, leg-local. MEASURED off the built
# solid rather than read off cad/shin.py's constants: the face that matters is
# wherever the part actually ends near the axis, and the shin builds that out of
# a hub, a spine and a set of gussets. Probing the axis: knee servo to y = 15,
# shin to y = 32, clear past that.
SHIN_FACE_Y = 32.0
STUB_R = 8.0                  # 16 mm boss, sized by stiffness not strength

# Pin play, radial, per joint. A 3 mm shaft in a 3.1 mm printed bore is the
# cheap version and it is the one that has to be defended, because free play at
# a pin is exactly the thing the linkage is being bought to remove.
# error_budget() answers it. A 623ZZ instead is 0.01 mm and costs $0.40.
PLAY = 0.10
BEARING_PLAY = 0.01

# What the print service quotes, the same number cad/assemble_check.py stacks.
TOL = 0.3


def _rot(a):
    """Rotation by `a` about +y, acting on (x, z). Positive pitches nose-down,
    which is the sim's sign: hip = +0.35 rad swings the knee AFT."""
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, s], [-s, c]])


def _dir(a):
    """Unit vector from a joint to its child at absolute angle `a`."""
    return _rot(a) @ np.array([0.0, -1.0])


# --- does it close? ------------------------------------------------------------

def foot_angle(hip, knee, dims=None):
    """The foot's absolute pitch, solved from the LINKAGE rather than assumed.

    This is the check with teeth, and the reason it is not a tautology: it does
    not compute where the pins are from the ankle angle, it computes the ankle
    angle from where the pins are. Feed it perturbed dimensions and it returns
    a foot that is not level, which is what error_budget() needs.

    `dims` overrides any of the nominal geometry:

        a       torso pin, from the hip axis          (x, z)
        b1      idler pin 1, from the knee axis       (x, z), in the idler frame
        b2      idler pin 2, from the knee axis       (x, z), in the idler frame
        dpin    yoke pin, from the ankle axis         (x, z), in the yoke frame
        l1, l2  the two rod lengths

    Returns (foot_angle, idler_angle) in radians. Both are 0 at nominal, for
    every pose, which is what "exact parallelogram" means.
    """
    g = dict(a=U1, b1=U1, b2=U2, dpin=U2, l1=THIGH_L, l2=SHIN_L)
    g.update(dims or {})

    hip_p = np.zeros(2)
    knee_p = hip_p + THIGH_L * _dir(hip)
    ankle_p = knee_p + SHIN_L * _dir(hip + knee)
    A = hip_p + np.asarray(g["a"], float)

    # Stage 1: the rod from A to the idler's first pin fixes the idler's angle.
    iota = _solve(lambda i: np.linalg.norm(
        A - (knee_p + _rot(i) @ g["b1"])) - g["l1"])
    if iota is None:
        return None, None
    B2 = knee_p + _rot(iota) @ g["b2"]

    # Stage 2: the rod from there to the yoke's pin fixes the foot's angle.
    foot = _solve(lambda f: np.linalg.norm(
        B2 - (ankle_p + _rot(f) @ g["dpin"])) - g["l2"])
    return foot, iota


def _solve(f, span=0.9, steps=181):
    """The root of `f` nearest zero, by bracket and bisect.

    Deliberately not a Newton step from zero. A four-bar has two assembly
    branches and Newton will happily walk onto the other one, which returns a
    number that is a real solution of the equation and a lie about the
    mechanism: the anti-parallelogram, with the foot upside down.
    """
    xs = np.linspace(-span, span, steps)
    vs = np.array([f(x) for x in xs])
    roots = []
    for i in range(len(xs) - 1):
        if vs[i] == 0.0:
            roots.append(xs[i])
        elif vs[i] * vs[i + 1] < 0:
            lo, hi = xs[i], xs[i + 1]
            for _ in range(60):
                mid = (lo + hi) / 2
                if f(lo) * f(mid) <= 0:
                    hi = mid
                else:
                    lo = mid
            roots.append((lo + hi) / 2)
    if not roots:
        return None
    return min(roots, key=abs)


# The squat band the robot actually uses: cruise_height 0.207 and stand_height
# 0.195 in src/rsbot/transition.py, measured over a full drive-flip-stand-
# unflip cycle at 3 deg of lash as hip 19.2 .. 27.6 deg. Swept wider than that
# on purpose, because a band measured on one controller is not a mechanical
# limit.
HEIGHT_BAND = (0.185, 0.215)


def _band(n=9):
    from src.rsbot.model import leg_ik
    return [leg_ik(h) for h in np.linspace(*HEIGHT_BAND, n)]


def closes(verbose=True):
    """Nominal geometry, over the band. The foot must be level everywhere.

    Control test built in: it also solves a deliberately WRONG linkage, one rod
    1 mm long, and reports the tilt that produces. If that comes back zero the
    solver is not reading the dimensions it is given and nothing below it means
    anything.
    """
    worst = 0.0
    for hip, knee in _band():
        f, _ = foot_angle(hip, knee)
        worst = max(worst, abs(np.degrees(f)))
    ctrl = max(abs(np.degrees(foot_angle(hip, knee, {"l2": SHIN_L + 1.0})[0]))
               for hip, knee in _band())
    if verbose:
        print(f"--- closure over the squat band {HEIGHT_BAND[0]*1000:.0f}"
              f"..{HEIGHT_BAND[1]*1000:.0f} mm")
        print(f"   nominal foot tilt        {worst:.6f} deg  (exact by "
              f"construction)")
        print(f"   control, rod 2 +1.00 mm  {ctrl:.3f} deg  "
              f"{'ok' if ctrl > 0.1 else 'THE SOLVER IS NOT READING dims'}")
    return worst, ctrl


# --- what the play actually costs ----------------------------------------------

_PINS = {"a": U1, "b1": U1, "b2": U2, "dpin": U2}


def _worst_over_signs(e, tol, band):
    """Worst (spread, offset) in degrees over every worst-case sign pattern.

    `spread` is how much the foot angle MOVES across the squat band, `offset`
    is the mean it moves about. They are different faults and the split is the
    point of this function: see error_budget().

    Conservative on purpose: every pin takes its full error in x AND in z at
    once, which is a circle's worth of freedom treated as a square, and nothing
    is correlated with anything. This is a bound, not a distribution.
    """
    worst_spread, worst_offset = (0.0, None), (0.0, None)
    for signs in itertools.product((-1, 1), repeat=2 * len(_PINS) + 2):
        dims = {}
        for i, (k, base) in enumerate(_PINS.items()):
            dims[k] = base + e * np.array([signs[2 * i], signs[2 * i + 1]])
        dims["l1"] = THIGH_L + tol * signs[-2]
        dims["l2"] = SHIN_L + tol * signs[-1]
        fs = [foot_angle(hip, knee, dims)[0] for hip, knee in band]
        fs = [f for f in fs if f is not None]
        if not fs:
            continue
        spread = max(fs) - min(fs)
        offset = (max(fs) + min(fs)) / 2
        worst_spread = max(worst_spread, (spread, signs), key=lambda o: o[0])
        worst_offset = max(worst_offset, (abs(offset), signs),
                           key=lambda o: o[0])
    return np.degrees(worst_spread[0]), np.degrees(worst_offset[0])


def error_budget(radial=PLAY, tol=TOL, verbose=True):
    """What the foot angle is actually worth, split three ways.

    THIS IS THE NUMBER THE WHOLE IDEA STANDS ON, and it is not one number. The
    linkage is bought to get rid of 2-3 degrees of gear lash at the ankle, and
    it pays with four pin joints and six printed dimensions. Lumping those
    together answers the wrong question, because they do not behave alike:

      FREE PLAY, from pin clearance. Reversible, instant, and unbounded in
      direction: this is the same animal as gear lash and it is what has to
      beat 2-3 degrees.

      BUILD OFFSET, from print tolerance. A fixed tilt, per leg, decided once
      when the parts come out of the machine. Not lash. It can be taken out
      with one adjustment per leg IF a rod length is adjustable, and cannot be
      touched at all if both rods are printed to length.

      BAND SPREAD, what is left of the build error after that adjustment. A
      four-bar with mis-placed pins is no longer an exact parallelogram, so its
      foot angle drifts as the leg squats and no single adjustment nulls it.
      docs/deleting-the-ankle-pitch-servo.md asks for exactly this number and
      could not have it before the geometry existed.
    """
    band = _band(5)
    play_spread, play_off = _worst_over_signs(radial, 0.0, band)
    tol_spread, tol_off = _worst_over_signs(tol, tol, band)
    if verbose:
        print("--- foot angle error budget, worst case over the squat band")
        print(f"   free play    {radial:.2f} mm at each of {len(_PINS)} pins"
              f"          {play_off:5.2f} deg")
        print(f"      compare   gear lash at the ankle servo          "
              f"2-3    deg   (docs/backlash.md)")
        print(f"   build offset print +/-{tol:.2f} mm, fixed per leg   "
              f"{tol_off:5.2f} deg   "
              f"{'ADJUSTABLE ROD REQUIRED' if tol_off > 1.0 else 'ok'}")
        print(f"   band spread  what one adjustment cannot remove  "
              f"{tol_spread:5.2f} deg")
    return play_off, tol_off, tol_spread


# --- what it is pushed with ----------------------------------------------------

def transmission(verbose=True):
    """sin of the angle between each rod and the link it works against.

    A parallelogram is not singular in the middle of its travel, but it goes
    singular when a rod lines up with its link, and the rod force runs as
    1/sin of that angle. Both offsets are horizontal in the torso frame and the
    leg is never near straight or near folded, so this stays high. It is
    checked rather than asserted because it is the one thing that would make
    the rod forces below wrong.
    """
    rows = []
    for hip, knee in _band():
        for name, u, ang in (("rod 1 / thigh", U1, hip),
                             ("rod 2 / shin", U2, hip + knee)):
            link = _dir(ang)
            s = abs(u[0] * link[1] - u[1] * link[0]) / np.linalg.norm(u)
            rows.append((s, name, ang))
    rows.sort()
    if verbose:
        print("--- transmission angle over the band")
        for name in ("rod 1 / thigh", "rod 2 / shin"):
            ss = [r[0] for r in rows if r[1] == name]
            print(f"   {name:<15} sin {min(ss):.3f} .. {max(ss):.3f}  "
                  f"({np.degrees(np.arcsin(min(ss))):.0f} deg at worst)")
    return rows[0][0]


# PA6-CF, in-plane. The same modulus cad/stress.py solves with; imported there
# from a table of literature values, which is still what it is, and the
# validation order is what turns it into a measurement.
E_PA6CF = 6000.0              # MPa


def loads(torque_nm=None, verbose=True):
    """Rod force, and how far it is from buckling.

    `torque_nm` defaults to the LIVE survey, which takes a couple of minutes.
    cad/belt.py's 1.68 N.m is a literal that has been in that file since before
    cad/loads.py could produce the number; it is 1.08 today.
    """
    if torque_nm is None:
        from cad.loads import design_pitch_torque
        torque_nm, case = design_pitch_torque("ankle")
    else:
        case = "given"
    s_min = transmission(verbose=False)
    f = torque_nm * 1000.0 / (R * s_min)
    # Euler, pinned both ends, about the weak axis, which is the y one.
    i_weak = ROD_T * ROD_W ** 3 / 12.0
    p_cr = np.pi ** 2 * E_PA6CF * i_weak / SHIN_L ** 2
    if verbose:
        print("--- rod force")
        print(f"   design ankle torque      {torque_nm:.3f} N.m  ({case})")
        print(f"   rod force at R = {R:.0f} mm   {f:.1f} N  "
              f"(worst transmission, sin {s_min:.3f})")
        print(f"   Euler buckling, {ROD_T:.0f} x {ROD_W:.0f} mm, "
              f"{SHIN_L:.0f} mm  {p_cr:.0f} N   {p_cr/f:.1f}x")
    return f, p_cr


# --- the parts -----------------------------------------------------------------

def _beam(length, t, w, eye_r=EYE_R, bore=PIN_D):
    """A rod along -z from the origin, with an eye at each end."""
    body = bd.Pos(0, 0, -length / 2) * bd.Box(t, w, length)
    for z in (0.0, -length):
        body += bd.Pos(0, 0, z) * bd.Rot(90, 0, 0) * bd.Cylinder(eye_r, w)
    for z in (0.0, -length):
        body -= (bd.Pos(0, 0, z) * bd.Rot(90, 0, 0)
                 * bd.Cylinder(bore / 2, w * 4))
    return body.clean()


def rod1():
    """Hip to idler, parallel to the thigh. Origin at the torso pin."""
    return _beam(THIGH_L, ROD_T, ROD_W)


def rod2():
    """Idler to yoke, parallel to the shin. Origin at the idler pin."""
    return _beam(SHIN_L, ROD_T, ROD_W)


def _pin_at(x, z, out=+1):
    """A pin in the local frame of the part that CARRIES it.

    It starts at that part's own plane and runs toward the rod plane, through
    the eye and 2 mm past so there is something to retain it against. `out` is
    which way that is: +1 for a part sitting inboard of the rods, -1 for one
    sitting outboard. Every pin in the linkage is this pin.
    """
    return (bd.Pos(x, out * PIN_REACH / 2, z) * bd.Rot(90, 0, 0)
            * bd.Cylinder(PIN_D / 2, PIN_REACH))


def idler():
    """The bar on the knee axis. Origin ON the axis, pins at U1 and U2.

    One part, two pins, and the only part in the linkage that turns on a
    BEARING rather than on a pin: every other pivot works against another link
    through a few degrees, and this one sits on the knee, which swings 50.

    The bar is straight and its two arms are opposite, which is not a
    simplification - it is what makes stage 1's offset and stage 2's offset
    independent. Each stage is a parallelogram in its own right and the idler
    is the only thing they share.
    """
    bar = bd.Rot(90, 0, 0) * bd.Cylinder(IDLER_HUB_R, IDLER_W)
    # Pin 1 reaches INBOARD to rod 1 and pin 2 OUTBOARD to rod 2, because the
    # idler sits between them in the stack. One 12 mm pin either way.
    for u, out in ((U1, -1), (U2, +1)):
        mid, L = np.asarray(u) / 2, float(np.linalg.norm(u))
        ang = float(np.degrees(np.arctan2(u[1], u[0])))
        bar += (bd.Pos(float(mid[0]), 0, float(mid[1])) * bd.Rot(0, -ang, 0)
                * bd.Box(L, IDLER_W, IDLER_T))
        bar += (bd.Pos(float(u[0]), 0, float(u[1])) * bd.Rot(90, 0, 0)
                * bd.Cylinder(EYE_R, IDLER_W))
        bar += _pin_at(float(u[0]), float(u[1]), out)
    # The 623ZZ's outer race sits in this bore; the inner race is on the stub
    # axle the shin carries. See knee_stub().
    bar -= bd.Rot(90, 0, 0) * bd.Cylinder(BEARING_OD / 2, IDLER_W * 4)
    return bar.clean()


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


# Where stage 2's arm starts. cad/ankle.py's YOKE_FWD is x = -56 .. 0,
# y = 56 .. 66, z = -8 .. 8, so the arm is that member carried forward from its
# front face and it needs no new attachment anywhere. Y_ARM sits inside that
# band, which is the reason Y_ARM is 59 and not some other number.
YOKE_ROOT = np.array([0.0, Y_ARM, 0.0])

ARM_T = 8.0
ARM_W = 6.0


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
    arm += bd.Pos(0, tip[1], 0) * _pin_at(float(tip[0]), float(tip[2]), -1)
    return arm.clean()


FIN_T = 8.0                   # thickness in x
FIN_H = 25.0                  # depth in z, and this is the dimension that
                              # matters: see the stiffness note below


def torso_boss(leg_y):
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
    from cad.chassis import SIDE_T, SIDE_Y
    y0 = (SIDE_Y + SIDE_T / 2) - leg_y          # the plate's OUTBOARD face
    y1 = Y_FIN
    fin = (bd.Pos(float(U1[0]), (y0 + y1) / 2, float(U1[1]))
           * bd.Box(FIN_T, abs(y1 - y0), FIN_H))
    fin += (bd.Pos(float(U1[0]), (y0 + y1) / 2, float(U1[1]))
            * bd.Rot(90, 0, 0) * bd.Cylinder(FIN_T / 2, abs(y1 - y0)))
    pin = bd.Pos(0, Y_FIN, 0) * _pin_at(float(U1[0]), float(U1[1]), +1)
    return (fin + pin).clean()


# Print orientation, per part: the layer normal each one is laid down along.
# The rods and the idler go flat, y up, because every hole in them is a pin bore
# on the y axis and a bore printed on its side needs support through it. The fin
# stands on the face that bolts to the plate.
LAYER = {"lk_rod1": (0, 1, 0), "lk_rod2": (0, 1, 0), "lk_idler": (0, 1, 0),
         "lk_arm": (0, 1, 0), "lk_fin": (1, 0, 0)}

OUT = __import__("pathlib").Path(__file__).parent / "out"


def export_stls(out_dir=None):
    """The linkage's own printed parts, each in its own frame.

    Only the three that are parts in their own right plus the two mount
    features, so cad/printability.py can be pointed at them. The features are
    exported separately on purpose: they will end up fused into the chassis and
    the yoke, and until they are, a check that says the ROD prints fine has
    said nothing about the fin.
    """
    d = out_dir or OUT
    d.mkdir(parents=True, exist_ok=True)
    for name, solid in (("lk_rod1", rod1()), ("lk_rod2", rod2()),
                        ("lk_idler", idler()), ("lk_arm", yoke_arm()),
                        ("lk_fin", torso_boss(60.0))):
        bd.export_stl(bd.Part() + solid, str(d / f"{name}.stl"))
    return d


def mass_g(leg_y=60.0, density=1.19, infill=1.0):
    """PA6-CF at 1.19 g/cm3, one leg's worth. The rods and the idler are small
    enough to print solid, and at this size infill would cost more in wall than
    it saves."""
    v = sum(s.volume for s in (rod1(), rod2(), idler(), yoke_arm(),
                               torso_boss(leg_y)))
    return v / 1000.0 * density * infill


# --- does it fit? --------------------------------------------------------------

def placed(m, d, side="l"):
    """[(name, solid)] the linkage in WORLD coordinates, read off the sim.

    Every part hangs off a body the simulator has already placed, the same way
    cad/assemble_check.py hangs the printed parts, so this cannot drift from
    the kinematics it is supposed to enforce.

    The frames are the whole content of this function and they are not
    interchangeable:

        the boss and the idler   are placed at a joint's POSITION with the
                                 TORSO's ROTATION, because a stage's offset is
                                 constant in the torso frame and in no other.
                                 That is what a parallelogram IS.
        the rods                 are placed at their upper pin with their own
                                 LINK's rotation, because a rod is parallel to
                                 the link it shadows.
        the yoke arm             is placed with the ankle body, which is level
                                 with the torso whenever the linkage is doing
                                 its job. If the sim ever stops holding
                                 hip + knee + ankle = 0 this part visibly
                                 stops pointing at its pin.
    """
    from cad.assemble_check import _loc
    sgn = 1 if side == "l" else -1

    def body(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        return _loc(d.xpos[i], d.xmat[i])

    torso, thigh = body("torso"), body(f"thigh_{side}")
    shin, ankle = body(f"shin_{side}"), body(f"ankle_{side}")
    leg_y = abs((torso.inverse() * thigh).position.Y)

    def mirror(s):
        return s if sgn > 0 else bd.mirror(s, bd.Plane.XZ)

    def level(at):
        """`at`'s position, carrying the torso's attitude."""
        return bd.Location(at.position, torso.orientation)

    def pin(at, u, plane):
        return level(at) * bd.Pos(float(u[0]), plane * sgn, float(u[1]))

    return [
        (f"lk_boss_{side}", level(thigh) * mirror(torso_boss(leg_y))),
        (f"lk_stub_{side}", shin * mirror(knee_stub())),
        (f"lk_rod1_{side}",
         bd.Location(pin(thigh, U1, Y_ROD1).position, thigh.orientation)
         * mirror(rod1())),
        (f"lk_idler_{side}",
         level(shin) * bd.Pos(0, Y_IDLER * sgn, 0) * mirror(idler())),
        (f"lk_rod2_{side}",
         bd.Location(pin(shin, U2, Y_ROD2).position, shin.orientation)
         * mirror(rod2())),
        (f"lk_arm_{side}", ankle * mirror(yoke_arm())),
    ]


def pivots(m, d, side="l"):
    """{name: (point from one side of the joint, point from the other)}.

    THERE ARE ONLY TWO PIVOTS WORTH MEASURING and the other three are listed
    here so that nobody adds them back. A pivot is only a check if its two
    sides are reached down DIFFERENT chains:

        rod1 top / boss        both are hip + u1 in the torso frame. Same
                               arithmetic twice. Not a check.
        rod2 top / idler       both are knee + u2 in the torso frame. Not a
                               check either.
        yoke pin / ankle       the arm is part of the yoke. Not a check.

        rod1 base / idler      one side is the torso's attitude carried to the
                               knee, the other is a rod of a fixed LENGTH hung
                               off the hip along the thigh. Tests that stage 1
                               closes.
        rod2 base / yoke       one side is a rod of fixed length hung off the
                               idler along the shin, the other is the ANKLE
                               BODY's own attitude. Tests that the simulator is
                               still holding hip + knee + ankle = 0, which is
                               the thing the linkage exists to do mechanically.

    Reporting the first three as 0.00 mm would read as five checks passing when
    there are two. See docs/how-checks-fail.md #1.
    """
    from cad.assemble_check import _loc
    sgn = 1 if side == "l" else -1

    def body(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        return _loc(d.xpos[i], d.xmat[i])

    torso, thigh = body("torso"), body(f"thigh_{side}")
    shin, ankle = body(f"shin_{side}"), body(f"ankle_{side}")

    def at(loc, x, yy, z):
        v = (loc * bd.Pos(float(x), float(yy) * sgn, float(z))).position
        return np.array([v.X, v.Y, v.Z])

    def level(x):
        return bd.Location(x.position, torso.orientation)

    a = at(level(thigh), U1[0], Y_ROD1, U1[1])
    b2 = at(level(shin), U2[0], Y_ROD2, U2[1])
    return {
        "stage 1: rod1 base / idler": (
            at(bd.Location(a, thigh.orientation), 0, 0, -THIGH_L),
            at(level(shin), U1[0], Y_ROD1, U1[1])),
        "stage 2: rod2 base / yoke": (
            at(bd.Location(b2, shin.orientation), 0, 0, -SHIN_L),
            at(ankle, U2[0], Y_ROD2, U2[1])),
    }


def seats(verbose=True):
    """Do the pivots actually land on each other, over the poses?

    cad/hardware.py's horn_joints() is the model for this: it found 10 of 10
    horn joints not seating, two of them clashing by 1.10 mm, on an assembly
    that every interference check called clean. A joint that does not MEET is
    not interference, and nothing else in this repo looks for it.
    """
    from fitcheck import pose as set_pose
    from src.rsbot.model import load

    m, d = load()
    rows = []
    for mode in ("wheel", "foot"):
        for h in (0.185, 0.207, 0.215):
            set_pose(m, d, mode, None, h)
            for side in ("l", "r"):
                for name, (p, q) in pivots(m, d, side).items():
                    rows.append((float(np.linalg.norm(p - q)), name,
                                 f"{mode} h={h:.3f} {side}"))
    rows.sort(reverse=True)
    if verbose:
        print("--- do the pivots meet?  (the two that are not tautologies)")
        seen = set()
        for gap, name, where in rows:
            if name in seen:
                continue
            seen.add(name)
            flag = "ok" if gap < 0.01 else "DOES NOT MEET"
            print(f"   {name:<28} {gap:6.3f} mm worst   {flag}   ({where})")
    return max(r[0] for r in rows)


# The ankle-pitch servo and its belt come OUT with this change, so they are not
# obstacles to the thing that replaces them. Named rather than inferred, so the
# exclusion can be argued with: an exclusion deserves the same suspicion as an
# assertion (docs/how-checks-fail.md #8).
REPLACED = {"vanksv_l", "vanksv_r"}

# Three of the six linkage parts are FEATURES of a part that already exists:
# the fin is chassis, the stub is shin, the arm is yoke. They are drawn
# separately here so this file can be read and checked on its own, and they
# overlap their host on purpose, because that is what being fused to it means.
# Nothing else is excused: the three of them are still checked against every
# other part in the robot, and against each other.
HOST = {"lk_boss": "torso", "lk_stub": "shin", "lk_arm": "ankle"}


def _fused(na, nb):
    """True if these two are the same part once the features are cut in."""
    for a, b in ((na, nb), (nb, na)):
        stem = a.rsplit("_", 1)[0] if a.startswith("lk_") else None
        if stem and HOST.get(stem) and b.startswith(HOST[stem]):
            return a.endswith(b.rsplit("_", 1)[-1]) or b == "torso"
    return False


def _poses():
    """The poses to sweep. Squat depth AND the flip, because the linkage hangs
    off the hip and knee and its clearances move with both. fitcheck.py sweeps
    only the flip, at one height per fraction, which is the trajectory the
    robot flies and not the space it can be in."""
    lo, hi = HEIGHT_BAND
    out = [("wheel", None, h) for h in np.linspace(lo, hi, 5)]
    out += [(None, f, None) for f in np.linspace(0, 1, 5)]
    out += [("foot", None, h) for h in np.linspace(lo, hi, 3)]
    return out


def ground(verbose=True):
    """How close the linkage comes to the floor in foot mode.

    Separate from clearance() because the floor is not a part and pairwise
    interference cannot see it. cad/shin.py records what this catches: in foot
    mode the axle is 12 mm off the ground and the shin stands at 27.5 degrees,
    so structure hung aft at axle height swings DOWN, and an earlier version of
    that part ended up 13 mm through the floor.

    It is also what ruled out the inboard route for stage 2: the only straight
    arm from the yoke that cleared the roll bracket cleared it by dropping to
    1 mm off the ground.
    """
    from fitcheck import pose as set_pose
    from src.rsbot.model import load

    m, d = load()
    worst = None
    for h in (0.185, 0.195, 0.215):
        set_pose(m, d, "foot", None, h)
        for side in ("l", "r"):
            for n, solid in placed(m, d, side):
                z = float(solid.bounding_box().min.Z)
                if worst is None or z < worst[0]:
                    worst = (z, n, h)
    if verbose:
        z, n, h = worst
        print(f"--- floor clearance in foot mode")
        print(f"   lowest linkage part      {n} at {z:.1f} mm, "
              f"h = {h:.3f}   {'ok' if z > 5.0 else 'TOO LOW'}")
        print(f"   for scale                the sole is 0 and the axle is "
              f"{WHEEL_HALF_W:.0f}")
    return worst[0]


def corridor(offsets=(26.0, 30.0, 34.0), planes=(48.0, 52.0, 56.0),
             verbose=True):
    """The sweep that chose R, run again.

    THIS EXISTS BECAUSE THE LAST ONE DID NOT. status/2026-08-03 published a
    corridor table for this linkage - one clear cell, everything else 774 to
    2243 mm3 - from a script that was never committed, and it does not
    reproduce: the clear region is a band tens of millimetres wide and it is on
    the other side of the leg. A number in a report is a copy, and a copy with
    no code behind it cannot be re-derived when it stops being true.

    A whole linkage in ONE plane, which is not how it is finally built - the
    parts end up in a stack across y - but it is the question "is there a
    window at all", and it is the question R and the stack were chosen from.
    """
    from cad.assemble_check import SEAM_MM, _thickness, parts
    from src.rsbot.model import load

    md = load()
    m, d = md
    rows = {}
    for mode, frac, height in _poses():
        others = [(n, s) for n, s in parts(mode, frac, height, md=md)
                  if n not in REPLACED]
        for u in offsets:
            for y in planes:
                for na, a in _one_plane(m, d, u, y):
                    for nb, b in others:
                        if _fused(na, nb):
                            continue
                        try:
                            inter = a & b
                            v = inter.volume if inter else 0.0
                        except Exception:
                            continue
                        if v > 1.0 and _thickness(inter) > SEAM_MM:
                            k = (u, y)
                            rows[k] = max(rows.get(k, 0.0), v)
    if verbose:
        print("--- corridor: a whole linkage in one plane, swept")
        print("   offset " + "".join(f"{y:>13.0f}" for y in planes))
        for u in offsets:
            line = f"   +{u:<5.0f}"
            for y in planes:
                v = rows.get((u, y))
                line += f"{'clear' if not v else f'{v:.0f} mm3':>13}"
            print(line)
    return rows


def _one_plane(m, d, offset, plane):
    """The linkage rebuilt at a trial offset and plane, left leg, in world.

    Deliberately crude - beams and no eyes - because this answers "is there
    room here at all", not "does this detail fit". placed() is the one that
    carries the real solids.
    """
    from cad.assemble_check import _loc

    def body(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        return _loc(d.xpos[i], d.xmat[i])

    torso, thigh = body("torso"), body("thigh_l")
    shin, ankle = body("shin_l"), body("ankle_l")

    def lev(x):
        return bd.Location(x.position, torso.orientation)

    def beam(p, q, t, w):
        p, q = np.asarray(p, float), np.asarray(q, float)
        v = q - p
        length = float(np.linalg.norm(v))
        z_dir = bd.Vector(*(v / length))
        x_dir = bd.Vector(0, 1, 0).cross(z_dir)
        if x_dir.length < 1e-9:          # the fin runs along y
            x_dir = bd.Vector(1, 0, 0)
        return bd.Plane(origin=bd.Vector(*((p + q) / 2)), x_dir=x_dir,
                        z_dir=z_dir) * bd.Box(t, w, length)

    from cad.chassis import SIDE_T, SIDE_Y
    leg_y = abs((torso.inverse() * thigh).position.Y)
    plate = (SIDE_Y + SIDE_T / 2) - leg_y
    a = lev(thigh) * bd.Pos(offset, plane, 0.0)
    b = lev(shin) * bd.Pos(offset, plane, 0.0)
    return [
        ("lk_rod1_l", bd.Location(a.position, thigh.orientation)
         * beam((0, 0, 0), (0, 0, -THIGH_L), ROD_T, ROD_W)),
        ("lk_rod2_l", bd.Location(b.position, shin.orientation)
         * beam((0, 0, 0), (0, 0, -SHIN_L), ROD_T, ROD_W)),
        ("lk_idler_l", lev(shin) * bd.Pos(0, plane + 7.0, 0)
         * beam((0, 0, 0), (offset, 0, 0), IDLER_T, IDLER_W)),
        ("lk_arm_l", ankle * beam((0.0, plane + 7.0, 0.0),
                             (offset, plane + 7.0, 0.0), ARM_T + 2, ARM_W + 2)),
        ("lk_boss_l", lev(thigh) * beam((offset, plate, 0.0),
                                  (offset, plane + 7.0, 0.0), FIN_T, FIN_H)),
    ]


def clearance(verbose=True):
    """Every linkage part against every other part, over every pose.

    Returns [(volume, a, b, where)] for anything that overlaps. Both legs, so
    the two linkages are checked against each other as well: they end up 24 mm
    apart in the corridor between the legs and nothing else was ever going to
    notice if they met.
    """
    from cad.assemble_check import SEAM_MM, _thickness, parts
    from src.rsbot.model import load

    md = load()
    m, d = md
    hits = []
    poses = _poses()
    for mode, frac, height in poses:
        others = [(n, s) for n, s in parts(mode, frac, height, md=md)
                  if n not in REPLACED]
        mine = [p for side in ("l", "r") for p in placed(m, d, side)]
        # EVERY linkage part against every other part, plus the linkage against
        # itself once. The first version wrote this as one product with an
        # `if na >= nb: continue` to dedupe the self-pairs, and that name
        # comparison silently threw away every pair whose partner sorted before
        # "lk_" - which was exactly one part, the ankle yoke, and the ankle yoke
        # is where stage 2 lives. It read as 13 poses of green.
        pairs = list(itertools.product(mine, others))
        pairs += list(itertools.combinations(mine, 2))
        for (na, a), (nb, b) in pairs:
            try:
                inter = a & b
                v = inter.volume if inter else 0.0
            except Exception:
                continue
            if v > 1.0 and _thickness(inter) > SEAM_MM and not _fused(na, nb):
                tag = mode or f"flip {frac * 100:.0f}%"
                hits.append((v, na, nb,
                             f"{tag}" + (f" h={height:.3f}" if height else "")))
    hits.sort(reverse=True)
    if verbose:
        print(f"--- clearance over {len(poses)} poses, squat and flip, "
              f"both legs")
        seen = set()
        for v, na, nb, at in hits:
            if (na, nb) in seen:
                continue
            seen.add((na, nb))
            print(f"   {na:<14} x {nb:<16} {v:8.1f} mm3   {at}")
        print(f"   {len(seen)} pair(s) collide" if seen
              else "   nothing collides, at any pose")
    return hits


def main():
    print(__doc__.strip().splitlines()[0])
    print()
    bad = 0
    worst, ctrl = closes()
    bad += worst > 1e-6 or ctrl < 0.1
    print()
    tilt = error_budget()
    print()
    transmission()
    print()
    loads()
    print()
    # 55 and 34 are docs/mass-budget.md's figures for one STS3215 and one
    # leg's belt drive, which makes them a COPY: see how-checks-fail #9. They
    # are quoted rather than computed because nothing in cad/ owns them yet.
    out = 55.0 + 34.0
    print(f"--- mass  {mass_g():.1f} g of linkage per leg, against {out:.0f} g "
          f"of ankle servo and belt drive: it SAVES {out - mass_g():.1f} g a "
          f"leg, {2 * (out - mass_g()):.0f} g on the robot")
    print()
    corridor()
    print()
    bad += ground() <= 5.0
    print()
    bad += seats() > 0.01
    print()
    bad += len(clearance())
    print()
    print("clean" if not bad else f"{bad} thing(s) to fix")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
