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

R = 40.0                 # outer radius; the sim's WHEEL_R
HALF_W = 12.0            # half width; the sim's WHEEL_HALF_W

TYRE_T = 3.0             # tread thickness, so rigid stops at r = 37
SOLE_T = 2.5             # TPU flange on the sole face
SOLE_INNER = 26.0        # flange reaches in to here
RIM_T = 4.0              # rigid rim wall
HUB_R = 14.0
HUB_T = 10.0
SPOKES = 6
SPOKE_W = 7.0
SPOKE_T = 5.0

# 25T horn, the bought part that gives a real spline interface.
HORN_R = 10.0            # boss the horn sits in
HORN_BOLT_DX, HORN_BOLT_DY = 4.95, 5.00
HORN_BOLT_R = 1.35       # M2.5 clearance
SHAFT_BORE = 3.2         # over the 5.9 mm spline boss

PA6CF = 1.15             # g/cm3
TPU = 1.21
INFILL = 0.60


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

    # Horn pocket and its four screws.
    p -= bd.Pos(0, 0, -HALF_W) * bd.Cylinder(HORN_R, 2 * 4.0)
    p -= bd.Pos(0, 0, 0) * bd.Cylinder(SHAFT_BORE, 4 * HALF_W)
    for dx in (-HORN_BOLT_DX, HORN_BOLT_DX):
        for dy in (-HORN_BOLT_DY, HORN_BOLT_DY):
            p -= bd.Pos(dx, dy, 0) * bd.Cylinder(HORN_BOLT_R, 4 * HALF_W)
    return p.clean()


def tyre():
    """TPU: a band round the rim and a flange over the sole."""
    band = (bd.Cylinder(R, 2 * HALF_W)
            - bd.Cylinder(R - TYRE_T, 2 * HALF_W + 2))
    flange = (bd.Pos(0, 0, HALF_W - SOLE_T / 2)
              * bd.Cylinder(R - TYRE_T, SOLE_T)
              - bd.Pos(0, 0, HALF_W - SOLE_T / 2)
              * bd.Cylinder(SOLE_INNER, SOLE_T + 2))
    return (band + flange).clean()


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

    # The whole point: nothing rigid may reach the sole plane, and the two
    # parts must not occupy the same space.
    sole = bd.Pos(0, 0, HALF_W - 0.01) * bd.Box(2*R, 2*R, 0.02)
    print(f"\n  rigid at the sole plane   {(b & sole).volume:8.2f} mm3"
          f"   (must be 0 - the TPU is the contact)")
    print(f"  body/tyre interference   {(b & t).volume:8.2f} mm3   (must be 0)")
    print(f"  hub stands {HALF_W - (-HALF_W + HUB_T):.1f} mm clear of the sole")
    return (gb + gt + 2.0) / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
