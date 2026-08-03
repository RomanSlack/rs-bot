"""The wheel, which is also the foot.  uv run python -m cad.wheel

No commercial wheel does this job. A wheel is designed to roll on its rim; this
one also has to STAND on its side face, and the two wants are in conflict:

  - the side face has to be flat and grippy, and it has to be the lowest thing
    on the part. The bought 80 x 24 wheel in the BOM has its hub standing 0.5 mm
    proud of the tyre, so in foot mode the robot stands on a hard plastic boss
    instead of rubber. Measured, not guessed: 0.50 mm static, 0.07 mm while
    actually standing.
  - the rim still has to be a tyre.

Pololu's 25T-spline range tops out at 90 x 10 mm and none of them have a usable
side face, so this is printed. Two parts:

    body    rigid, rim + spokes + hub, with a bought 25T metal horn bolted in.
            A printed 25T spline would strip; the horn is a 2 dollar part.
    tyre    TPU, a shallow cup - a band round the rim AND a flange over the
            outer 14 mm of the sole. One part doing both jobs.

Frame: spin axis is z, matching MuJoCo's cylinder convention. The SOLE is at
+z, which is inboard on the robot - the roll takes the local +z face and points
it at the floor. The hub is therefore at -z, outboard, where the wheel-drive
servo already is. That falls out nicely: nothing on the hub side can ever touch
the ground.
"""

import sys
from pathlib import Path

import build123d as bd
import numpy as np

OUT = Path(__file__).parent / "out"

# From cad/wheel_dims.py, which is the ONLY copy. These used to be written here
# with comments reading "the sim's WHEEL_R" - an author noticing the duplication
# and copying the number anyway. The one of the three that nobody kept in step
# was the tyre thickness, which the sim had at 8 mm against this file's 3 mm.
from cad.wheel_dims import R, HALF_W, TYRE_T  # noqa: E402,F401
SOLE_T = 2.5             # TPU flange on the sole face
SOLE_INNER = 26.0        # flange reaches in to here
RIM_T = 4.0              # rigid rim wall
HUB_R = 14.0
HUB_T = 10.0
SPOKES = 6
SPOKE_W = 7.0
SPOKE_T = 5.0

# 25T horn, the bought part that gives a real spline interface.
#
# WAS 10.0, A Ø20.00 POCKET, AND IT WOULD NOT HAVE FITTED. The horn measures
# Ø19.93 on the bench (cad/servo.py, 2026-08-01), so the pocket carried 0.035 mm
# of clearance per side from a service quoting +/-0.3 mm. Most wheels would have
# come back unable to accept the horn at all. It survived the clearance sweep
# that opened every other fit in the robot to 0.1-0.9 mm only because the horn's
# outside diameter was unknown at the time.
#
# THE POCKET HAS TO CLEAR TWO THINGS, NOT ONE, which is why the fix is bigger
# than the 0.07 mm the horn alone asks for. The wheel drops over the horn AND
# over the servo's own round seating boss, and that boss is also about Ø20. Both
# of them, the horn and the boss, and the pocket were all the same diameter,
# which is another way of saying nothing fitted.
#
#     Ø20.6 nominal    over the 19.93 horn   0.335/side, 0.185 worst case
#                      over the ~20.0 boss   0.300/side, 0.150 worst case
#
# Centring moves to the four M3 screws and the spline bore, which is where it
# belongs on this joint anyway: a printed pocket wall at +/-0.3 mm was never
# going to locate anything.
#
# THE BOSS DIAMETER IS STILL ASSUMED. cad/servo.py has it at 20.00 off a STEP
# that is probably the wrong servo variant, and it is one caliper reading away
# from being known. It is the only thing that could push this number higher.
HORN_R = 10.3            # Ø20.6 pocket; clears the 19.93 horn and the ~20 boss
# Imported rather than repeated. These were a local copy of the horn pattern
# and drifted from it: 4.95 x 5.00 with a 2.7 mm hole, against a measured
# 4.95 x 4.95 with a 3.2 mm one. A second copy of a number is a second place
# for it to be wrong.
from cad.servo import HORN_DX as HORN_BOLT_DX  # noqa: E402
from cad.servo import HORN_DY as HORN_BOLT_DY  # noqa: E402
from cad.servo import HORN_SCREW_R as HORN_BOLT_R  # noqa: E402
SHAFT_BORE = 3.2         # over the 5.9 mm spline boss

# Tread. Cut INTO the band, never added on top: every clearance rule in the
# robot is written as "radius > 40 mm from the spin axis" (docs/porting-a-link.md),
# so a lug standing proud of R would break all of them at once. Grooves keep the
# envelope and still give carpet something to bite.
TREAD_N = 24             # grooves; 10.5 mm pitch at R, so a 7 mm lug
TREAD_D = 1.2            # depth, leaving 1.8 mm of the 3 mm band
TREAD_W = 3.5            # groove width
TREAD_SKEW = 20.0        # degrees across the width, so lugs engage progressively
TREAD_MARGIN = 2.0       # stop short of the sole plane, leaving a continuous
                         # rubber ring at the foot's outer edge

# Retention. The tyre used to be a pure slip fit: bore and rim BOTH exactly
# r = 37.0, zero interference, nothing keying it in rotation and nothing
# stopping it walking off the open end. Wheel torque at the rim is about 78 N
# at stall, which a smooth zero-preload interface does not carry, so the tyre
# would have spun on the rim the first time the robot drove.
#
# Three separate jobs, two features:
#   bore interference   grips, and preloads the whole interface
#   castellated bead    stops it walking off +z, AND takes the drive torque
#
# The bead is on the hub-side edge ONLY. The sole edge cannot have a rigid
# flange, because rigid material at the sole plane is the fault this part was
# drawn to fix.
PRESS = 0.35             # radial interference; the TPU stretches over the rim
BEAD_R = 1.2             # bead height above the rim, so 1.8 mm of tyre over it
BEAD_W = 2.5             # bead width
# The bead has to be INSET from the edge, not on it. Sitting on the edge, the
# tyre's recess is open at the bottom and the bead slides straight out of it -
# which is what the escape direction is, because the sole flange only blocks
# the other one. The inset is the TPU lip that wraps under the bead and is what
# actually catches. It is also what you stretch over the bead to assemble.
BEAD_INSET = 1.5         # lip thickness under the bead
BEAD_N = 12              # teeth: these are what actually carry drive torque
BEAD_GAP = 5.0           # tangential gap between teeth, so a 14.7 mm tooth
BEAD_CLEAR = 0.15        # the bead snaps in; the bore does the gripping

PA6CF = 1.15             # g/cm3
TPU = 1.21
INFILL = 0.60


def _bead(r_out, gap_w, z_lo, z_hi, r_in):
    """The castellated retaining ring, as a solid.

    One feature doing two jobs. Its +z face is what the tyre's recess wall
    lands on, so the tyre cannot walk off the open end; its teeth are what the
    tyre's matching teeth push against, so drive torque goes through plastic in
    shear instead of through friction on a smooth bore.

    Built as a ring with the gaps cut out rather than as N boxes, because at
    r = 37 a box is 0.3 mm short of the arc at its ends and the bead is only
    1.2 mm tall.
    """
    zc, h = (z_lo + z_hi) / 2, z_hi - z_lo
    ring = (bd.Pos(0, 0, zc) * bd.Cylinder(r_out, h)
            - bd.Pos(0, 0, zc) * bd.Cylinder(r_in, h + 2))
    for i in range(BEAD_N):
        ring -= (bd.Rot(0, 0, 360.0 * i / BEAD_N)
                 * bd.Pos(R, 0, zc)
                 * bd.Box(4 * TYRE_T, gap_w, h + 2))
    return ring


def body():
    """Rigid part: rim, spokes, hub."""
    rim_o, rim_i = R - TYRE_T, R - TYRE_T - RIM_T
    # The rim stops SOLE_T short of the sole plane so the TPU flange has
    # somewhere to sit. Running it the full width makes the two parts
    # interpenetrate, and puts rigid plastic on the floor - the exact fault
    # this part exists to fix.
    rim_h = 2 * HALF_W - SOLE_T
    p = (bd.Pos(0, 0, -SOLE_T / 2) * bd.Cylinder(rim_o, rim_h)
         - bd.Pos(0, 0, -SOLE_T / 2) * bd.Cylinder(rim_i, rim_h + 2))

    # Sole backing: the TPU flange needs something to sit on.
    p += (bd.Pos(0, 0, HALF_W - SOLE_T - 1.5) * bd.Cylinder(rim_o, 3.0)
          - bd.Pos(0, 0, HALF_W - SOLE_T - 1.5) * bd.Cylinder(HUB_R - 2, 5))

    # Hub, on the -z side so it can never reach the floor.
    p += bd.Pos(0, 0, -HALF_W + HUB_T / 2) * bd.Cylinder(HUB_R, HUB_T)

    for i in range(SPOKES):
        a = 2 * np.pi * i / SPOKES
        p += (bd.Rot(0, 0, float(np.degrees(a)))
              * bd.Pos(rim_i / 2 + HUB_R / 2, 0, -HALF_W + HUB_T / 2)
              * bd.Box(rim_i - HUB_R + 4, SPOKE_W, SPOKE_T))

    # The retaining bead, at the hub-side edge where a rigid feature is allowed.
    p += _bead(rim_o + BEAD_R, BEAD_GAP, -HALF_W + BEAD_INSET,
               -HALF_W + BEAD_INSET + BEAD_W, rim_o)

    # Horn pocket and its four screws.
    #
    # 4.0 mm deep over a horn plate measured at 2.51, so the pocket floor lands
    # on the plate and the remaining 1.49 mm is counterbore reaching down past
    # it toward the servo. That leaves the pocket rim about 0.5 mm clear of the
    # servo's boss face, which is positive but thin, and it rests on a
    # HORN_FACE_Z that cad/servo.py flags as unverified. Worth re-checking once
    # the boss is measured; if that margin goes negative the wheel stands off
    # the horn and the whole joint loads the bolts in bending.
    p -= bd.Pos(0, 0, -HALF_W) * bd.Cylinder(HORN_R, 2 * 4.0)
    p -= bd.Pos(0, 0, 0) * bd.Cylinder(SHAFT_BORE, 4 * HALF_W)
    for dx in (-HORN_BOLT_DX, HORN_BOLT_DX):
        for dy in (-HORN_BOLT_DY, HORN_BOLT_DY):
            p -= bd.Pos(dx, dy, 0) * bd.Cylinder(HORN_BOLT_R, 4 * HALF_W)
    return p.clean()


def _tread():
    """The groove pattern, as one cutting solid.

    Skewed rather than axial so a lug enters contact at one edge and rolls
    across, instead of the whole lug landing at once and drumming.
    """
    # The tread has to stop clear of the bead: over the bead the tyre is
    # already down to 1.8 mm, and a groove there would take it to 0.6.
    hi = HALF_W - TREAD_MARGIN
    lo = -HALF_W + BEAD_INSET + BEAD_W + 0.5
    L = (hi - lo) / np.cos(np.radians(TREAD_SKEW))
    z0 = (hi + lo) / 2
    cut = None
    for i in range(TREAD_N):
        g = (bd.Rot(0, 0, 360.0 * i / TREAD_N)
             * bd.Pos(R, 0, z0)
             * bd.Rot(TREAD_SKEW, 0, 0)
             * bd.Box(2 * TREAD_D, TREAD_W, L))
        cut = g if cut is None else cut + g
    return cut


def tyre(tread=True):
    """TPU: a treaded band round the rim and a flange over the sole.

    `tread=False` is the smooth reference the tread check measures against.
    """
    # The bore is UNDERSIZE by PRESS. Modelling it at the rim's nominal radius
    # is what made this a slip fit, and the overlap check then read 0.00 mm3
    # and called it a pass - a fit of exactly nothing, reported as evidence.
    band = (bd.Cylinder(R, 2 * HALF_W)
            - bd.Cylinder(R - TYRE_T - PRESS, 2 * HALF_W + 2))
    flange = (bd.Pos(0, 0, HALF_W - SOLE_T / 2)
              * bd.Cylinder(R - TYRE_T, SOLE_T)
              - bd.Pos(0, 0, HALF_W - SOLE_T / 2)
              * bd.Cylinder(SOLE_INNER, SOLE_T + 2))
    # The sole face is left SMOOTH. Foot mode is static and carpet is the
    # compliant half of the pair, so contact area beats bite there; the tread
    # exists for rolling, which is the mode that has to shear against pile.
    t = band + flange
    # Recess for the bead, grown by the snap clearance in every direction: the
    # ring gets bigger, so the GAPS between teeth get smaller by the same.
    t -= _bead(R - TYRE_T + BEAD_R + BEAD_CLEAR, BEAD_GAP - 2 * BEAD_CLEAR,
               -HALF_W + BEAD_INSET - BEAD_CLEAR,
               -HALF_W + BEAD_INSET + BEAD_W + BEAD_CLEAR,
               R - TYRE_T - PRESS - 1.0)
    return ((t - _tread()) if tread else t).clean()


def main(export=True):
    b, t = body(), tyre()
    gb = b.volume / 1000 * PA6CF * INFILL
    gt = t.volume / 1000 * TPU          # tyres print solid
    if export:
        OUT.mkdir(exist_ok=True)
        for name, s in (("wheel_body", b), ("wheel_tyre", t)):
            bd.export_step(s, str(OUT / f"{name}.step"))
            bd.export_stl(s, str(OUT / f"{name}.stl"),
                          tolerance=0.01, angular_tolerance=0.1)

    bb = (b + t).bounding_box()
    print(f"wheel, {2*R:.0f} x {2*HALF_W:.0f} mm  (bbox "
          f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f})")
    print(f"  body   {b.volume/1000:5.1f} cm3   {gb:5.1f} g  PA6-CF @ {INFILL:.0%}")
    print(f"  tyre   {t.volume/1000:5.1f} cm3   {gt:5.1f} g  TPU solid")
    print(f"  horn                      2.0 g  bought 25T metal")
    print(f"  TOTAL                    {gb+gt+2.0:5.1f} g   (BOM assumed 60.0)")

    # The whole point: nothing rigid may reach the sole plane, and nothing
    # rigid may come close to the tread surface either.
    sole = bd.Pos(0, 0, HALF_W - 0.01) * bd.Box(2*R, 2*R, 0.02)
    print(f"\n  rigid at the sole plane   {(b & sole).volume:8.2f} mm3"
          f"   (must be 0 - the TPU is the contact)")
    print(f"  rigid outside r = {R - 1.5:.1f}     "
          f"{(b - bd.Cylinder(R - 1.5, 4*HALF_W)).volume:8.2f} mm3"
          f"   (must be 0 - 1.5 mm of rubber everywhere)")
    print(f"  hub stands {HALF_W - (-HALF_W + HUB_T):.1f} mm clear of the sole")

    # THE PRESS FIT. This check used to read "interference must be 0", and it
    # passed, because the bore and the rim were both exactly r = 37.0. That is
    # a fit of precisely nothing certified as a good fit - the exact failure
    # mode this project keeps finding. Now the overlap IS the fit, and what is
    # checked is that it equals the interference that was designed in.
    contact_h = 2 * HALF_W - SOLE_T - BEAD_W      # rim face, less the bead zone
    want = np.pi * ((R - TYRE_T) ** 2
                    - (R - TYRE_T - PRESS) ** 2) * contact_h
    got = (b & t).volume
    print(f"\n  press fit  {PRESS:.2f} mm radial on r = {R - TYRE_T:.1f}")
    print(f"  interference volume      {got:8.2f} mm3   "
          f"(predicted {want:.0f}, must agree within 10%)")
    print(f"  ratio                    {got/want:8.3f}")

    # RETENTION, tested by function rather than by inspection. A press fit
    # alone passes any static overlap check while still spinning on the rim and
    # walking off the open end, so move the tyre the two ways it can actually
    # fail and confirm the bead is in the way.
    #
    # Measured in the BEAD ZONE, not over the whole part. Over the whole part
    # the bore interference is 1500 mm3 and swamps everything: the first
    # version of this check reported 1.11x for a tooth clash and 0.95x for a
    # slide that was not blocked at all, which is a check too blunt to fail.
    zc = -HALF_W + BEAD_INSET + BEAD_W / 2
    zone = bd.Pos(0, 0, zc) * bd.Box(4 * R, 4 * R, BEAD_W)
    base = (b & t & zone).volume
    half_tooth = 180.0 / BEAD_N
    spun = (b & (bd.Rot(0, 0, half_tooth) * t) & zone).volume
    slid = (b & (bd.Pos(0, 0, 1.0) * t) & zone).volume
    print(f"\n  in the bead zone, seated {base:8.2f} mm3   (the bore press fit)")
    print(f"  rotated half a tooth     {spun:8.2f} mm3   "
          f"({spun/max(base,1e-9):.1f}x - teeth must clash, so >> 1)")
    # +z is the escape direction: the sole flange bears DOWN on the backing
    # ring, so -z is already blocked and +z is the one that needs the bead.
    print(f"  slid 1 mm toward the sole{slid:8.2f} mm3   "
          f"({slid/max(base,1e-9):.1f}x - the lip must catch, so >> 1)")
    print(f"  {BEAD_N} teeth, {BEAD_R:.1f} mm proud, "
          f"{2*np.pi*(R-TYRE_T)/BEAD_N - BEAD_GAP:.1f} mm long, "
          f"{BEAD_GAP:.1f} mm gaps")

    # Tread, with both halves able to fail. Volume removed catches a pattern
    # that silently did nothing (TREAD_N = 0, or a depth that misses the band);
    # the radius check catches the opposite mistake, a lug standing proud of R,
    # which would break every "radius > 40 mm" clearance rule in the robot at
    # once and would be invisible in the mass.
    cut = tyre(tread=False).volume - t.volume
    proud = (t - bd.Cylinder(R, 4 * HALF_W)).volume
    print(f"\n  tread removed            {cut:8.2f} mm3   (must be > 0)")
    print(f"  tyre outside r = {R:.0f}       {proud:8.2f} mm3   (must be 0 - "
          f"the clearance rules are written against it)")
    print(f"  {TREAD_N} grooves, {TREAD_D:.1f} mm deep, "
          f"{2*np.pi*R/TREAD_N:.1f} mm pitch, {TREAD_SKEW:.0f} deg skew")
    return (gb + gt + 2.0) / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
