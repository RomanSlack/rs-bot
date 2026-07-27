"""The STS3215, measured rather than assumed.

    uv run python -m cad.servo

Every servo dimension in this project was previously a number I made up from a
product listing: a 45.2 x 24.7 x 35.4 box with a bolt circle at radius 8. Four
of the five printed parts bolt to a servo, so those numbers set every mounting
face in the robot, and being wrong about them means every part is wrong at the
one place it has to be right.

These come from TheRobotStudio's SO-ARM100 STEP model of the same servo
(vendor/refs/STS3215_03a.step, Apache-2.0), read with OCC rather than typed in.

What changed:

    case height      35.4 -> 39.60 mm   the 4.2 mm is ALONG THE SHAFT, which
                                        is exactly where the horn clearances are
    horn bolts       r = 8.0 at 45 deg  -> a 9.9 x 10.0 mm rectangle, so the
                                        holes were 0.7 mm out of position
    horn screw       M2 (2.2 clear)     -> 2.5 mm clearance
    shaft inset      10.0 (guessed)     -> 10.20 mm from the end. Nearly right.

The frame the STEP arrives in: x along the 45.4 mm length, y across the 24.8 mm
width, z along the OUTPUT SHAFT. Shaft axis at x = +12.5, y = 0. Output end at
z = +20.2, rear idler boss at z = -18.0.
"""

from pathlib import Path

import build123d as bd

STEP = Path(__file__).parents[1] / "vendor" / "refs" / "STS3215_03a.step"

# --- measured off the STEP ---------------------------------------------------
LENGTH = 45.40           # x
WIDTH = 24.80            # y
HEIGHT = 39.60           # z, ALONG the output shaft
Z_MIN, Z_MAX = -19.40, 20.20

SHAFT_X = 12.50          # shaft axis, from the case centre
SHAFT_INSET = LENGTH / 2 - SHAFT_X       # 10.20 mm from the near end
SHAFT_R = 2.95           # 25T spline, 5.9 mm across
HORN_FACE_Z = 18.70      # where the horn seats
HORN_DX, HORN_DY = 4.95, 5.00            # four screws, about the shaft
HORN_SCREW_R = 1.25      # 2.5 mm clearance
IDLER_Z = -18.00         # rear pivot boss
IDLER_R = 3.00

MASS_G = 55.0            # from the BOM; the STEP has no material

# The old guesses, kept so the diff is legible from the code.
OLD = dict(length=45.2, width=24.7, height=35.4, horn_r=8.0, screw_r=1.1)


def solid():
    """The real servo, in its own frame. Use this for clearance checks: a box
    is 36.2 cm3 of bounding box round a 36.2 cm3 part, but it is the wrong
    36.2 cm3 near the horn."""
    return bd.import_step(str(STEP))


def envelope():
    """Bounding box, for the cheap checks that cannot afford the real solid."""
    return bd.Pos(0, 0, (Z_MIN + Z_MAX) / 2) * bd.Box(LENGTH, WIDTH, HEIGHT)


def horn_holes(depth=20.0, clearance=HORN_SCREW_R):
    """The four horn screws, as cutting cylinders in the servo's frame."""
    out = []
    for dx in (-HORN_DX, HORN_DX):
        for dy in (-HORN_DY, HORN_DY):
            out.append(bd.Pos(SHAFT_X + dx, dy, HORN_FACE_Z)
                       * bd.Cylinder(clearance, depth))
    return out


def main():
    p = solid()
    bb = p.bounding_box()
    print(f"STS3215, from {STEP.name}")
    print(f"  bounding box   {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm")
    print(f"  solid volume   {p.volume/1000:.1f} cm3")
    print(f"  shaft axis     x = {SHAFT_X:.2f}, {SHAFT_INSET:.2f} mm from the end")
    print(f"  output face    z = {HORN_FACE_Z:.2f}, tip at {Z_MAX:.2f}")
    print(f"  horn screws    4 x dia {2*HORN_SCREW_R:.1f} at "
          f"+/-{HORN_DX:.2f} x +/-{HORN_DY:.2f} about the shaft")
    print(f"  rear boss      z = {IDLER_Z:.2f}, dia {2*IDLER_R:.1f}")
    print()
    print("against what this project assumed before:")
    print(f"  height   {OLD['height']:.1f} -> {HEIGHT:.2f} mm  "
          f"({HEIGHT-OLD['height']:+.2f}, and it is along the shaft)")
    print(f"  horn     r={OLD['horn_r']:.1f} at 45 deg -> "
          f"+/-{HORN_DX:.2f} x +/-{HORN_DY:.2f}  "
          f"(holes {abs(OLD['horn_r']*0.7071-HORN_DX):.2f} mm out of position)")
    print(f"  screw    r={OLD['screw_r']:.2f} -> {HORN_SCREW_R:.2f} mm")


if __name__ == "__main__":
    main()
