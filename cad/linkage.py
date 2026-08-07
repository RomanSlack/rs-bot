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

from cad.hardware import BEARING_ID, BEARING_OD
# Every dimension of the mechanism lives in cad/linkage_dims.py, which carries
# no build123d so the physics can import it too. One copy.
from cad.linkage_dims import (ARM_T, ARM_W, DZ, EYE_R, IDLER_HUB_R,  # noqa: F401
                              IDLER_T, IDLER_W, PINS, PIN_D, R, ROD_T, ROD_W,
                              SIM_BODIES, SIM_MESH, U1, U2, Y_ARM, Y_FIN,
                              Y_IDLER, Y_ROD1, Y_ROD2, sim_bought, sim_stance)
from cad.wheel_dims import HALF_W as WHEEL_HALF_W, R as WHEEL_R

# linkage_dims keeps them as plain tuples so it can stay free of numpy as well
# as of build123d. Everything here does vector arithmetic on them.
U1, U2 = np.array(U1), np.array(U2)

# --- the mechanism -------------------------------------------------------------
#
# All of it in the left leg's local frame, millimetres, x forward and z up, the
# same convention as cad/shin.py and cad/ankle.py. The right leg mirrors in y.

THIGH_L = 110.0               # hip to knee
SHIN_L = 110.0                # knee to ankle


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

# --- sections ------------------------------------------------------------------
#
# The rods are in pure tension and compression (a two-force member cannot be
# anything else), at 34 N on the live factored ankle torque (0.95 N.m; it was
# 38.7 N at the 1.08 the survey read before the robot dropped to PA6-CF). Sized
# by buckling, not by stress, and by a long way: 20.7x from Euler in loads(),
# against 2% of allowable at the eye once rod_stress() actually solves it. So
# 8 x 6 is the smallest section that still prints with four perimeters and does
# not look like a wire.
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

# --- the adjuster ---------------------------------------------------------------
#
# ROD 2 IS ADJUSTABLE IN LENGTH, and that is a requirement rather than a
# convenience. error_budget() puts the worst-case BUILD OFFSET at 4.21 degrees
# of fixed foot tilt, and with no ankle actuator there is nothing left to trim
# it with afterwards.
#
# What decides it is not that 4.21 sounds large, it is what happens at 4.21.
# Injecting the offset into the tendon equality - which is exactly what a
# mis-built linkage does, still rigid, just holding the wrong number - and
# flipping at 3 degrees of gear lash, 8 triggers each:
#
#     offset      both legs the same way      opposed
#     4.21 deg    8/8                         8/8   <- the worst case
#     5.50 deg    3/8                         4/8
#     8.00 deg    0/8                         0/8
#
# The worst case clears, by about 1.3 degrees, and that is thinner than it
# looks: the 4.21 is a bound with its own assumptions, the 3 degrees of gear
# lash has never been measured on hardware, and the two stack.
#
# The mechanism is a clamped lap: rod 2 is two printed halves overlapping in a
# joint that is split along y, so the assembled envelope is unchanged and the
# clearance sweep still sees one 8 x 6 rod. Two M3 screws through slots clamp
# it. Slip is not the risk it looks like - two M3 at a modest preload hold
# several hundred newtons by friction against a 39 N rod force - and a screw
# through a slot is the one adjustment that needs no thread in plastic, which
# matters while docs/road-to-order.md item 2 is still undecided.
ADJUST_MM = 3.0               # +/- along the rod, so 6 mm of slot
CLAMP_D = 3.4                 # M3 clearance
CLAMP_PITCH = 13.0            # between the two clamp screws
LAP_MM = 26.0                 # how far the two halves overlap


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
        rng = adjust_range_deg()
        print(f"   build offset print +/-{tol:.2f} mm, fixed per leg   "
              f"{tol_off:5.2f} deg")
        print(f"      rod 2 adjusts +/-{ADJUST_MM:.1f} mm, worth        "
              f"{rng:5.2f} deg   "
              f"{'covers it' if rng >= tol_off else 'NOT ENOUGH TRAVEL'}")
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
    cad/loads.py could produce the number; the live survey reads 0.95 N.m now,
    down from 1.08 when the robot was heavier. Pass a torque only to reproduce an
    old number; the default is the one that cannot go stale.
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


def rod_stress(force=40.0, size=1.6, verbose=True):
    """Peak von Mises at a rod's pin EYE under axial tension, from cad.fea.

    A two-force member fails two ways. Compression is buckling, and loads()
    above has it (Euler, 20x). Tension pulls on the eye, whose stress
    concentration around the pin bore is the one number the net-section hand
    calc cannot give and the reason cad.stress "never solved a linkage rod" sat
    on the list. This is that solve: it is the only place cad.fea is pointed at
    a rod, and it turns "sized by buckling, not stress" from a comment into a
    result.

    `force` defaults to a round 40 N, comfortably above the live factored 34 N,
    so the test that calls it stays fast and conservative rather than paying for
    the two-minute torque survey; the eye stress is linear in the force, so a
    fixed load over-states it cleanly. Pass loads()[0] for the live number.

    The bore load is spread over the whole ring rather than a cosine bearing
    patch, so it under-states the true bearing peak by a small factor - but at
    2% of allowable there is three orders of margin, and modelling pin contact
    to sharpen a 1 MPa number would be answering a question nobody is asking.
    """
    from cad import fea
    from cad.stress import near_axis, MATERIALS, knockdown
    OUT.mkdir(exist_ok=True)
    step = OUT / "lk_rod1_fea.step"
    bd.export_step(rod1(), str(step))
    nodes, elems = fea.mesh_step(str(step), size=size)
    # Eyes at z = 0 and z = -THIGH_L, bores along y, radius PIN_D / 2.
    fixed = near_axis(nodes, (0, 0, 0), (0, 1, 0), PIN_D / 2 + 0.5, half_len=ROD_W)
    loaded = near_axis(nodes, (0, 0, -THIGH_L), (0, 1, 0), PIN_D / 2 + 0.5,
                       half_len=ROD_W)
    f = fea.distribute(nodes, loaded, np.array([0.0, 0.0, -force]), np.zeros(3))
    _, s = fea.solve(nodes, elems, E_PA6CF, 0.38, fixed, [(loaded, f)])
    _, peak, _ = fea.report(fea.von_mises(s), nodes, fixed)
    # PA6-CF in-plane allowable, knocked for infill - from cad.stress, one source.
    row = next(r for r in MATERIALS if r[0] == "PA6-CF")
    allow = row[3] * knockdown("PA6-CF")
    if verbose:
        print("--- rod eye, tension")
        print(f"   axial force              {force:.1f} N")
        print(f"   mesh                     {len(nodes)} nodes at {size:.1f} mm")
        print(f"   peak von Mises           {peak:.2f} MPa")
        print(f"   PA6-CF allowable (x{knockdown('PA6-CF'):.1f})   {allow:.0f} MPa"
              f"   {peak / allow * 100:.0f}%")
    return peak, allow


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
    """Idler to yoke, parallel to the shin. Origin at the idler pin.

    TWO PIECES, clamped, so its length can be trimmed at assembly. See
    ADJUST_MM for why that is not optional. Drawn assembled, because that is
    what every clearance check needs to see; the split is along y, so the
    envelope is the same 8 x 6 as rod 1 and the lap adds nothing to sweep.
    """
    rod = _beam(SHIN_L, ROD_T, ROD_W)
    # The clamp slots, through the lap, on the rod's own axis. Slotted in one
    # half and round in the other; drawn as the slot, which is the envelope
    # that has to stay inside the rod.
    mid = -SHIN_L / 2
    for z in (mid - CLAMP_PITCH / 2, mid + CLAMP_PITCH / 2):
        slot = (bd.Pos(0, 0, z) * bd.Rot(90, 0, 0)
                * bd.Cylinder(CLAMP_D / 2, ROD_W * 4))
        slot += bd.Pos(0, 0, z) * bd.Box(CLAMP_D, ROD_W * 4, 2 * ADJUST_MM)
        rod -= slot
    return rod.clean()


def adjust_range_deg():
    """How much foot tilt the adjuster can take out.

    From the SOLVER, not from arithmetic: closes() measures 1 mm of rod-2 error
    as 2.03 degrees at the foot, so the range is that slope times the travel.
    Computing it here rather than writing 6 degrees down means it moves when the
    geometry does.
    """
    per_mm = abs(np.degrees(foot_angle(*_band(1)[0],
                                       {"l2": SHIN_L + 1.0})[0]))
    return per_mm * ADJUST_MM


def _pin(x, z, carrier_y, rod_y, seat, at_carrier=True):
    """A pin, spanning from inside its CARRIER to just past its ROD.

    IT IS A BOUGHT PART. The first version of this file ADDED it to whichever
    printed part carried it, so the fin, the idler and the yoke arm each came
    out of the printer with a 3 mm plastic spigot on them - while the order
    sheet listed eight 3 mm steel pins a robot. Two descriptions of the same
    joint, one of which you cannot print at that diameter and would shear
    anyway.

    AND IT WAS THE WRONG LENGTH, which only showed up once it stopped being
    fused to its carrier. It used to start AT the carrier's plane and run
    outward, which is what a boss growing off a part does; as a separate pin
    that leaves it engaging 3 mm of a 6 mm idler and ZERO of the fin, whose
    material stops exactly where the pin began. assembled() measures it.

    So both ends are derived: `seat` deep into the carrier at one end, and
    2 mm past the rod's eye at the other. `at_carrier` picks whose frame the
    result is in - the carrier's own plane, or leg-local y.
    """
    out = 1.0 if rod_y > carrier_y else -1.0
    near = carrier_y - out * seat
    far = rod_y + out * (ROD_W / 2 + 2.0)
    lo, hi = min(near, far), max(near, far)
    if at_carrier:
        lo, hi = lo - carrier_y, hi - carrier_y
    return (bd.Pos(x, (lo + hi) / 2, z) * bd.Rot(90, 0, 0)
            * bd.Cylinder(PIN_D / 2, hi - lo))


def _pin_bore(x, z, carrier_y, rod_y, seat):
    """The hole a pin is pressed into, in its carrier. Same axis, same span."""
    return _pin(x, z, carrier_y, rod_y, seat)


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
        bar -= _pin_bore(float(u[0]), float(u[1]), Y_IDLER,
                         Y_ROD1 if out < 0 else Y_ROD2, IDLER_W / 2 + 1.0)
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
    fin -= (bd.Pos(0, Y_FIN, 0)
            * _pin_bore(float(U1[0]), float(U1[1]), Y_FIN, Y_ROD1, 8.0))
    return fin.clean()


# Print orientation, per part: the layer normal each one is laid down along.
# The rods and the idler go flat, y up, because every hole in them is a pin bore
# on the y axis and a bore printed on its side needs support through it. The fin
# stands on the face that bolts to the plate.
LAYER = {"lk_rod1": (0, 1, 0), "lk_rod2": (0, 1, 0), "lk_idler": (0, 1, 0)}

OUT = __import__("pathlib").Path(__file__).parent / "out"


def export_stls(out_dir=None):
    """The linkage's own printed parts, each in its own frame.

    Three, so cad/printability.py can be pointed at them. The mount features
    are not here: they are printed as part of the chassis, the shin and the
    yoke, and are checked with those.
    """
    d = out_dir or OUT
    d.mkdir(parents=True, exist_ok=True)
    for name, make in SOLIDS.items():
        bd.export_stl(bd.Part() + make(), str(d / f"{name}.stl"))
    return d


def mass_g(density=1.19, infill=1.0):
    """PA6-CF at 1.19 g/cm3, one leg's worth of the linkage's OWN parts.

    The three that are parts. The fin, the stub and the arm are not counted
    here because they are not separate any more: they are chassis, shin and
    yoke, and cad/masses.py weighs them with those. Counting them twice was
    available and would have been silent.
    """
    v = sum(make().volume for make in SOLIDS.values())
    return v / 1000.0 * density * infill


# --- does it fit? --------------------------------------------------------------

def placements(m, d, side="l"):
    """[(name, stl stem, mirror sign, position mm, rotation 3x3)] per part.

    The frames are the whole content of this function and they are not
    interchangeable:

        the fin and the idler     sit at a joint's POSITION carrying the
                                  TORSO's ROTATION, because a stage's offset is
                                  constant in the torso frame and in no other.
                                  That is what a parallelogram IS.
        the rods                  sit at their upper pin carrying their own
                                  LINK's rotation, because a rod is parallel to
                                  the link it shadows.
        the stub                  belongs to the shin and turns with it.
        the yoke arm              belongs to the ankle body, which is level
                                  with the torso whenever the linkage is doing
                                  its job. If the sim ever stops holding
                                  hip + knee + ankle = 0 this part visibly
                                  stops pointing at its pin.

    Split out of placed() so cad/robot.py can hang the same parts off the same
    poses without either of them owning a second copy of that table.
    """
    sgn = 1 if side == "l" else -1

    def body(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        return (np.array(d.xpos[i]) * 1000.0,
                np.array(d.xmat[i]).reshape(3, 3))

    (torso_p, torso_R) = body("torso")
    (thigh_p, thigh_R) = body(f"thigh_{side}")
    (shin_p, shin_R) = body(f"shin_{side}")
    (ankle_p, ankle_R) = body(f"ankle_{side}")

    def pin(origin, u, plane):
        return origin + torso_R @ np.array([u[0], plane * sgn, u[1]])

    return [
        (f"lk_rod1_{side}", "lk_rod1", sgn,
         pin(thigh_p, U1, Y_ROD1), thigh_R),
        (f"lk_idler_{side}", "lk_idler", sgn,
         shin_p + torso_R @ np.array([0.0, Y_IDLER * sgn, 0.0]), torso_R),
        (f"lk_rod2_{side}", "lk_rod2", sgn,
         pin(shin_p, U2, Y_ROD2), shin_R),
    ]


# THE LINKAGE'S OWN PRINTED PARTS, and there are three, not six. The fin, the
# stub and the arm were here for a day as separate solids and are features of
# the chassis, the shin and the yoke: cad/linkage_mounts.py draws them and
# those three modules build them in. See cad/linkage_mounts.py for why a
# separate fin is not a smaller version of a fitted one, it is a loose piece.
SOLIDS = {"lk_rod1": rod1, "lk_idler": idler, "lk_rod2": rod2}


def placed(m, d, side="l"):
    """[(name, solid)] the linkage in WORLD coordinates, read off the sim.

    Every part hangs off a body the simulator has already placed, the same way
    cad/assemble_check.py hangs the printed parts, so this cannot drift from
    the kinematics it is supposed to enforce.
    """
    from cad.assemble_check import _loc

    out = []
    for name, stem, sgn, pos, R in placements(m, d, side):
        solid = SOLIDS[stem]()
        if sgn < 0:
            solid = bd.mirror(solid, bd.Plane.XZ)
        out.append((name, _loc(pos / 1000.0, R.flatten()) * solid))
    return out


def mounts_placed(m, d, side="l"):
    """[(name, solid)] the three MOUNT FEATURES in world, for measurement only.

    They are not parts any more - the chassis, the shin and the yoke print them
    - so they are correctly absent from SOLIDS and from placed(). But ground()
    still has to see them, and the yoke arm is exactly why: it is the lowest
    thing the linkage puts anywhere near the floor, and the moment it became a
    yoke feature ground() stopped measuring it and got 5 mm greener without
    anything moving. A check that improves because it stopped looking is the
    fault this repo keeps finding, so this puts the coverage back.
    """
    from cad.assemble_check import _loc
    from cad.linkage_mounts import knee_stub, torso_boss, yoke_arm

    sgn = 1 if side == "l" else -1

    def body(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        return np.array(d.xpos[i]), np.array(d.xmat[i]).flatten()

    torso_R = body("torso")[1]
    out = []
    for name, solid, (pos, R) in (
            (f"lk_fin_{side}", torso_boss(_leg_y(m, d, side)),
             (body(f"thigh_{side}")[0], torso_R)),
            (f"lk_stub_{side}", knee_stub(), body(f"shin_{side}")),
            (f"lk_arm_{side}", yoke_arm(), body(f"ankle_{side}"))):
        if sgn < 0:
            solid = bd.mirror(solid, bd.Plane.XZ)
        out.append((name, _loc(pos, R) * solid))
    return out


def _leg_y(m, d, side):
    """How far the hip axis is from the centreline, off the sim rather than
    written down again."""
    t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    h = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"thigh_{side}")
    return abs(float(d.xpos[h][1] - d.xpos[t][1])) * 1000.0


# Where each pin lives: (x, z) offset, the plane its CARRIER sits in, which way
# it reaches, and the body it is fixed to. Four a leg, plus the idler's bearing.
def bought(m, d, side="l"):
    """[(name, solid)] the linkage's BOUGHT hardware, in world.

    Four 3 mm pins and one 623ZZ a leg. They exist here because of
    docs/how-checks-fail.md #13 - the parts that do the assembling are the ones
    nobody checks - and because this file had just repeated that fault twice at
    once: the pins were drawn as printed spigots on their carriers, and the
    idler's bearing was on the order sheet and drawn nowhere at all. The idler
    sat 0.50 mm off its stub with nothing in between, which is what a joint
    made of two holes looks like.
    """
    from cad.assemble_check import _loc
    from cad.hardware import bearing

    sgn = 1 if side == "l" else -1

    def body(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        return _loc(d.xpos[i], d.xmat[i])

    torso, thigh = body("torso"), body(f"thigh_{side}")
    shin, ankle = body(f"shin_{side}"), body(f"ankle_{side}")
    idler = body(f"lkidler_{side}")

    def level(at):
        return bd.Location(at.position, torso.orientation)

    host = {"thigh": level(thigh), "idler": idler, "ankle": ankle}
    out = []
    for name, u, plane, rod, seat, where in PINS:
        pin = _pin(float(u[0]), float(u[1]), plane * sgn, rod * sgn, seat)
        # The idler carries its pins in its OWN frame, where the plane offset is
        # already baked into the body's position; the others are placed from a
        # joint, so the plane is part of the offset.
        at = (bd.Pos(0, 0, 0) if where == "idler"
              else bd.Pos(0, plane * sgn, 0))
        out.append((f"lkpin_{name}_{side}", host[where] * at * pin))
    # Inner race on the stub, outer in the idler's bore, on the knee axis.
    out.append((f"lkbrg_idler_{side}",
                level(shin) * bd.Pos(0, Y_IDLER * sgn, 0) * bd.Rot(90, 0, 0)
                * bearing()))
    return out


# What each bought part is supposed to join. Named rather than inferred,
# because "it touches something" is the question that passes when a pin grazes
# the wrong part, and this is the question that does not.
JOINS = {"lkpin_fin": ("torso", "lk_rod1"),
         "lkpin_idler1": ("lk_idler", "lk_rod1"),
         "lkpin_idler2": ("lk_idler", "lk_rod2"),
         "lkpin_arm": ("ankle", "lk_rod2"),
         "lkbrg_idler": ("shin", "lk_idler")}

# A pivot needs meat around it. 4 mm on a 3 mm pin is a length-to-diameter of
# 1.3, which is the least you would accept before the pin starts working the
# bore oval.
MIN_ENGAGE = 4.0


def assembled(verbose=True):
    """Is every pivot actually made, or does something just pass nearby?

    For each pin, how much of it is INSIDE each of the two parts it joins,
    measured by intersecting an oversized pin with the part and dividing by the
    annulus area. A pin that misses gives zero; a pin that grazes gives a
    millimetre; a pin properly through a 6 mm eye gives 6.

    This is the check that "1 connected group" cannot be. Union-find says every
    part is REACHABLE from every other, which a single 0.04 mm graze anywhere
    satisfies - and this robot has had exactly that: a wheel-drive servo
    floating 0.5 mm off the bracket it bolts to, found only because a boolean
    union came back as two solids instead of one.

    Control-tested by shortening the fin's pin so it stops 1 mm short of rod 1,
    which drops that pivot to 2.00 mm engaged and fails.
    """
    from fitcheck import pose as set_pose
    from src.rsbot.model import load

    m, d = load()
    over = PIN_D / 2 + 0.1
    area = np.pi * (over ** 2 - (PIN_D / 2) ** 2)
    rows = []
    for mode in ("wheel", "foot"):
        set_pose(m, d, mode)
        for side in ("l", "r"):
            parts = dict(placed(m, d, side))
            # The carriers are the HOST parts now - the chassis, the shin and
            # the yoke - so the assembly they come from is the whole robot's,
            # not the linkage's own.
            from cad.assemble_check import parts as robot_parts
            parts.update({("torso" if n == "torso" else n): sol
                          for n, sol in robot_parts(mode, md=(m, d))})
            for name, sol in bought(m, d, side):
                stem = name.rsplit("_", 1)[0]
                if stem == "lkbrg_idler":
                    for want in JOINS[stem]:
                        g = _gap(sol, parts[_host(want, side)])
                        rows.append((0.0 if g < 0.01 else -1.0, name, want, g,
                                     mode, side))
                    continue
                fat = _fatten(sol, over)
                for want in JOINS[stem]:
                    v = (fat & parts[_host(want, side)]).volume
                    rows.append((v / area, name, want, 0.0, mode, side))
    if verbose:
        print("--- is every pivot actually made?")
        seen = set()
        for eng, name, want, g, mode, side in sorted(rows):
            key = (name.rsplit("_", 1)[0], want)
            if key in seen:
                continue
            seen.add(key)
            if name.startswith("lkbrg"):
                print(f"   {key[0]:<14} in {want:<10} "
                      f"{'seated' if g < 0.01 else f'{g:.2f} mm OFF IT'}")
            else:
                flag = "ok" if eng >= MIN_ENGAGE else "NOT ENGAGED"
                print(f"   {key[0]:<14} in {want:<10} {eng:5.2f} mm "
                      f"engaged   {flag}")
    bad = [r for r in rows if r[0] < MIN_ENGAGE and not r[1].startswith("lkbrg")]
    bad += [r for r in rows if r[1].startswith("lkbrg") and r[3] > 0.01]
    return bad


# How much of each mount feature's root face is actually ON its host, as a
# fraction. A face butt has NO overlap volume - two solids meeting on a plane
# intersect in nothing - so neither an interference check nor a connectivity
# check can tell it from a graze. This measures the contact AREA instead, by
# taking a thin slab of the host immediately behind the root and comparing it
# to the root's own cross-section.
#
# It exists because of what it found. The stub axle was touching the shin
# through THIRTEEN cubic millimetres of the ankle servo's cradle wall, four
# millimetres off the knee axis, because SHIN_FACE_Y had been measured against
# that wall instead of against the hub. Every check in the repo said one solid.
# It would have stayed true until the cradle came out - and the cradle is
# already on the list for removal, since it holds a servo that no longer
# exists.
SLAB = 0.5
# 0.60, and the reason is the shin's hub rather than a comfortable bound. A root
# cannot seat on a hole that has to be open, and the hub carries the knee horn's
# Ø8 bore plus four M3 screw holes right under the stub's footprint: 123 mm2 of
# a gross 201 is material, and 123 is all there is. The other two come out at
# 100%, and tests/test_linkage.py asserts all three to a percent, which is the
# guard that actually catches drift. This bound only has to catch a root that is
# half on air, which is what the arm was.
MIN_SEATED = 0.60


def mounts(verbose=True):
    """Is each mount feature seated on its host, or just touching it?

    Returns every measurement, not only the failures: a check that hands back
    an empty list on success gives a test nothing to assert tightly, and a
    tight assertion is what catches drift.
    """
    import cad.ankle as ankle
    import cad.chassis as chassis
    import cad.linkage_mounts as lm
    import cad.shin as shin

    rows = []

    # The stub, on the shin's hub: a disc of the root's own radius.
    host = _without(shin.build, lm, "knee_stub")
    probe = (bd.Pos(0, lm.SHIN_FACE_Y - SLAB / 2, 0) * bd.Rot(90, 0, 0)
             * bd.Cylinder(lm.STUB_R, SLAB))
    rows.append(("stub on the shin hub", (probe & host).volume / SLAB,
                 np.pi * lm.STUB_R ** 2))

    # The arm, on the yoke's forward member: a rectangle normal to x.
    host = _without(ankle.yoke, lm, "yoke_arm")
    probe = (bd.Pos(-SLAB / 2, float(lm.YOKE_ROOT[1]), float(lm.YOKE_ROOT[2]))
             * bd.Box(SLAB, lm.ARM_W, lm.ARM_T))
    rows.append(("arm on YOKE_FWD", (probe & host).volume / SLAB,
                 lm.ARM_W * lm.ARM_T))

    # The fin, on the chassis side plate.
    host = _without(chassis.build, lm, "torso_boss")
    plate = chassis.SIDE_Y + chassis.SIDE_T / 2
    probe = (bd.Pos(float(U1[0]), plate - SLAB / 2,
                    lm.fin_z(chassis.SIDE_Z0))
             * bd.Box(lm.FIN_T, SLAB, lm.FIN_H))
    rows.append(("fin on the side plate", (probe & host).volume / SLAB,
                 lm.FIN_T * lm.FIN_H))

    if verbose:
        print("--- is each mount feature seated on its host?")
        for what, area, full in rows:
            f = area / full
            print(f"   {what:<24}{area:7.1f} of {full:6.1f} mm2  "
                  f"{f*100:5.1f}%   {'ok' if f >= MIN_SEATED else 'NOT SEATED'}")
    return rows          # [(what, seated mm2, root mm2)], callers judge


def _without(build, module, name):
    """`build()` with one of its features neutered, so the host can be measured
    on its own. Put somewhere harmless rather than made zero-size: a degenerate
    solid at the origin is a second solid in the middle of the part."""
    orig = getattr(module, name)
    setattr(module, name, lambda *a, **k: bd.Pos(0, 900, 0) * bd.Box(1, 1, 1))
    try:
        return max(build().solids(), key=lambda s: s.volume)
    finally:
        setattr(module, name, orig)


def _host(name, side):
    """A part key: the torso has no side, everything else does."""
    return name if name == "torso" else f"{name}_{side}"


def _fatten(pin, r):
    """The pin at radius `r`, same axis and length. Its intersection with a
    bored part is the annulus between pin and bore, whose length is the
    engagement."""
    b = pin.bounding_box()
    axis = int(np.argmax([b.size.X, b.size.Y, b.size.Z]))
    c = ((b.min.X + b.max.X) / 2, (b.min.Y + b.max.Y) / 2,
         (b.min.Z + b.max.Z) / 2)
    length = [b.size.X, b.size.Y, b.size.Z][axis]
    rot = {0: bd.Rot(0, 90, 0), 1: bd.Rot(90, 0, 0), 2: bd.Rot(0, 0, 0)}[axis]
    return bd.Pos(*c) * rot * bd.Cylinder(r, length)


def _gap(a, b):
    from cad.assemble_check import gap
    return gap(a, b)


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


def sim_matches_cad(verbose=True):
    """Does the SIM's linkage sit where this file says it does?

    Two routes to the same six poses, and they must agree.

        cad     placements() reads the thigh, shin and ankle out of the sim and
                hangs each part off them with the offsets at the top of this
                file, carrying the torso's attitude where a stage's offset is
                constant.
        sim     src/rsbot/model.py builds the rods and the idler as real
                bodies on real hinges, and four tendon equalities a leg pin
                those hinges to the angles the mechanism puts them at.

    Nothing forces those to agree. The sim's version could have a coupling sign
    backwards, or an offset in the wrong parent's frame, and the robot would
    balance perfectly well with its rods hanging off at an angle - which is
    exactly how the wheel-drive servo came to sit 12.5 mm off the joint it
    turns for eleven weeks. This is cad/twin.py's question asked of the one
    mechanism cad/twin.py cannot see.

    It earned itself the first time it ran: 32.8 degrees of divergence at full
    squat with the POSITIONS exact to the micron, which is what a wrong angle
    looks like. fitcheck.pose() was not setting the linkage hinges at all, and
    mj_forward does not solve equality constraints, so every geometric check in
    the repo had been posing the rods hanging straight down off their pins.

    Control-tested by flipping the idler's coupling sign, which takes it to
    65.5 degrees.
    """
    import mujoco as mj

    from fitcheck import pose as set_pose
    from src.rsbot.model import load

    m, d = load()
    rows = []
    for mode, height in (("wheel", 0.207), ("wheel", 0.185), ("foot", 0.195),
                         ("foot", 0.215)):
        set_pose(m, d, mode, None, height)
        for side in ("l", "r"):
            want = {n: (p, R) for n, _, _, p, R in placements(m, d, side)}
            for name, stem in SIM_MESH.items():
                bid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, f"{name}_{side}")
                got_p = np.array(d.xpos[bid]) * 1000.0
                got_R = np.array(d.xmat[bid]).reshape(3, 3)
                key = f"lk_{stem.split('_', 1)[1]}_{side}"
                exp_p, exp_R = want[key]
                dp = float(np.linalg.norm(got_p - exp_p))
                dr = float(np.degrees(np.arccos(np.clip(
                    (np.trace(got_R.T @ exp_R) - 1) / 2, -1, 1))))
                rows.append((max(dp, dr), name, side, dp, dr,
                             f"{mode} h={height:.3f}"))
    rows.sort(reverse=True)
    if verbose:
        print("--- does the sim's linkage sit where the CAD says?")
        seen = set()
        for _, name, side, dp, dr, where in rows:
            if name in seen:
                continue
            seen.add(name)
            flag = "ok" if max(dp, dr) < 0.01 else "DIVERGED"
            print(f"   {name:<9} {dp:7.4f} mm  {dr:7.4f} deg   {flag}   "
                  f"({where})")
    return max(r[0] for r in rows)


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

# NOTHING IS EXCUSED HERE ANY MORE. There used to be a HOST table saying which
# linkage parts were allowed to overlap which host, because the fin, the stub
# and the arm were drawn separately and fused to their hosts only in principle.
# They are built into the chassis, the shin and the yoke now, so there is no
# overlap to excuse and no exclusion to argue with. An exclusion deserves the
# same suspicion as an assertion (docs/how-checks-fail.md #8); the best thing to
# do with one is remove the reason for it.


def _fused(na, nb):
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
            for n, solid in placed(m, d, side) + mounts_placed(m, d, side):
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
        # The BOUGHT parts are in the sweep too. They were not, and fitcheck's
        # box model found a pin overlapping a rod that the solids never checked.
        mine = [q for side in ("l", "r")
                for q in placed(m, d, side) + bought(m, d, side)]
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
    bad += sim_matches_cad() > 0.01
    print()
    bad += len(assembled())
    print()
    bad += sum(1 for _, a, f in mounts() if a / f < MIN_SEATED)
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
