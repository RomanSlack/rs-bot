"""The ankle-pitch belt drive.  SUPERSEDED 2026-08-03.

    uv run python -m cad.belt

> **This drive is deleted.** A passive parallelogram replaces the servo it
> served: see cad/linkage.py and docs/deleting-the-ankle-pitch-servo.md. The
> pulleys, the belt and their geometry are gone from cad/hardware.py and from
> the sim; renders/servos-before-after-*.png is what they looked like.
>
> The file is kept because the SIZING ARGUMENT below is still the reason the
> ratio could not be traded, and that argument is what made the drive
> undeletable-in-place and therefore worth replacing wholesale: below 2:1 the
> belt is overloaded, above it the servo runs out of travel, and at 2:1 the
> driven pulley is 2.23 mm through the floor in foot mode. Nothing about the
> ankle joint changed to fix that, so anyone who proposes putting a belt back
> has to start from here.
>
> BELT_Y0, BELT_W and C are still read by nothing. Do not add a reader without
> reading the line above.

The pitch servo cannot sit on its own joint, because the wheel already owns
that axle, so it drives down through a belt. That belt has been "specified but
not drawn" since before the joint existed. This sizes it from the real servo
and the real joint position, and gives the volume it sweeps so the rest of the
robot can be checked against it.

The ratio is pinned from both sides, which is why it is 2:1 and not a choice:

  - below 2:1 the belt is overloaded. Tension is joint torque over the driven
    pulley's pitch radius, so a smaller driven pulley means more tension.
  - above 2:1 the servo runs out of travel. The pitch joint needs its range,
    and the servo only has 360 degrees to give.

Everything here is a bought part - GT2 pulleys are a couple of dollars each -
so what matters is the envelope, the bore, and the belt length being a stock
size. It is: 94 teeth, 188 mm.
"""

import numpy as np

import cad.servo as sv
import cad.shin as shin

PITCH = 2.0                  # GT2
TEETH_DRIVE, TEETH_DRIVEN = 20, 40
BELT_TEETH = 94              # 188 mm, a stock closed loop
# 15 mm, not 9. The measured 1.68 N.m joint peak is 132 N of belt tension at
# the 12.73 mm driven pitch radius, and a 9 mm GT2 belt works to about 100 N.
# The ratio cannot be raised to relieve it - the servo runs out of travel - so
# the belt has to be wide.
BELT_W = 15.0

# Both pulleys sit outboard of the shin's bearing block, in the one corridor
# that is clear. The servo's output boss faces outboard at y = 73.6 and the
# shin's block ends at y = 76, so the belt plane starts there.
BELT_Y0 = 77.0
BELT_Y1 = BELT_Y0 + BELT_W

PR_DRIVE = TEETH_DRIVE * PITCH / np.pi / 2      # pitch RADIUS
PR_DRIVEN = TEETH_DRIVEN * PITCH / np.pi / 2
RATIO = TEETH_DRIVEN / TEETH_DRIVE

# Measured peak at the ankle-pitch joint over the whole envelope, from
# cad/loads.py. Not a guess.
JOINT_PEAK_NM = 1.68

# GT2 working tension, manufacturer figures, N per belt width.
GT2_WORKING_N = {6.0: 60.0, 9.0: 100.0, 15.0: 180.0}


def centre_distance(teeth=BELT_TEETH):
    """Solve the belt-length equation for the centre distance."""
    L = teeth * PITCH
    d1, d2 = 2 * PR_DRIVE, 2 * PR_DRIVEN
    lo, hi = 20.0, 200.0
    for _ in range(80):
        mid = (lo + hi) / 2
        Lm = 2 * mid + np.pi * (d1 + d2) / 2 + (d2 - d1) ** 2 / (4 * mid)
        if Lm < L:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


C = centre_distance()
SERVO_SHAFT_Z = shin.ANKLE_Z + C          # where the servo shaft must sit


def tension_n():
    """Tight-side tension at the driven pulley, from the measured joint peak."""
    return JOINT_PEAK_NM * 1000.0 / PR_DRIVEN


def swept():
    """The volume the belt and pulleys occupy, in the shin frame. Anything
    else in the robot has to miss this."""
    import build123d as bd
    span = abs(SERVO_SHAFT_Z - shin.ANKLE_Z)
    box = (bd.Pos(0, (BELT_Y0 + BELT_Y1) / 2,
                  (SERVO_SHAFT_Z + shin.ANKLE_Z) / 2)
           * bd.Box(2 * PR_DRIVEN + 2, BELT_W, span))
    for z, r in ((SERVO_SHAFT_Z, PR_DRIVE), (shin.ANKLE_Z, PR_DRIVEN)):
        box += (bd.Pos(0, (BELT_Y0 + BELT_Y1) / 2, z) * bd.Rot(90, 0, 0)
                * bd.Cylinder(r + 1.5, BELT_W))
    return box.clean()


def main():
    print(f"GT2 {TEETH_DRIVE}T -> {TEETH_DRIVEN}T, ratio {RATIO:.1f}:1")
    print(f"  pitch radii    {PR_DRIVE:.2f} and {PR_DRIVEN:.2f} mm")
    print(f"  belt           {BELT_TEETH}T = {BELT_TEETH*PITCH:.0f} mm, "
          f"{BELT_W:.0f} mm wide")
    print(f"  centre dist    {C:.2f} mm")
    print(f"  servo shaft    z = {SERVO_SHAFT_Z:.2f} in the shin frame")
    print(f"  belt plane     y = {BELT_Y0:.0f}..{BELT_Y1:.0f}")

    t = tension_n()
    print(f"\ntension from the measured {JOINT_PEAK_NM:.2f} N.m joint peak: "
          f"{t:.0f} N")
    for w, allow in sorted(GT2_WORKING_N.items()):
        mark = "  <- chosen" if abs(w - BELT_W) < 0.01 else ""
        ok = "ok" if allow > t else "OVER"
        print(f"  {w:4.0f} mm belt  working {allow:5.0f} N   {ok}"
              f"  ({t/allow*100:.0f}% used){mark}")

    print(f"\nservo travel needed: pitch range is exercised over "
          f"{29.4-19.5:.1f} deg at the joint, so {RATIO*(29.4-19.5):.0f} deg "
          f"at the servo.")
    print(f"the joint is mechanically clear over -95..+69 deg, which is "
          f"{RATIO*164:.0f} deg of servo travel - inside its 360.")
    sw = swept()
    print(f"\nbelt+pulley swept volume: {sw.volume/1000:.1f} cm3, bbox "
          f"{sw.bounding_box().size.X:.0f} x {sw.bounding_box().size.Y:.0f} x "
          f"{sw.bounding_box().size.Z:.0f} mm")


if __name__ == "__main__":
    main()
