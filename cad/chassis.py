"""The chassis.  uv run python cad/chassis.py

Origin at the hip axis, z up, millimetres. Two side plates carrying the hip
servos, tied by a top plate and two shelves that also mount the Pi and the
pack. Open front and back, as the sim has it.

Structurally the easy part: 11x margin in bending on 3 mm plates. What it does
have to get right is the hip mounting, because both hip servo cases bolt
straight to these plates and everything the robot weighs passes through them.
"""

import sys
from pathlib import Path

import build123d as bd

OUT = Path(__file__).parent / "out"
PETG_SOLID, INFILL = 1.270, 0.60
M25_CLEAR = 1.35

X0, X1 = -36.5, 53.5          # 90 mm deep, centred on the sim's 8.5 offset
SIDE_Y, SIDE_T = 38.1, 3.0    # plate centre-line and thickness
INNER_Y = SIDE_Y - SIDE_T / 2
TOP_Z, SHELF1_Z, SHELF2_Z = 178.5, 20.0, 61.0
PLATE_T = 3.0

# The hip servo lives INSIDE the chassis and its output boss passes out
# through the side plate to reach the thigh hub, so the plate needs a notch.
# Without it the servo interferes with its own mounting plate by 3467 mm3.
from cad.servo import LENGTH as SERVO_L, WIDTH as SERVO_W
HIP_NOTCH_X = SERVO_W / 2 + 0.8
HIP_NOTCH_Z = SERVO_L + 1.0

HIP_Z = 0.0                   # hip axis

# Capturing the hip servo instead of bolting into its case. The four M2 that
# used to go here were at +/-17.0, one of four different invented patterns this
# robot drilled into the same servo; nobody publishes where those holes are, so
# all of them are gone (cad/servo.py).
#
# THE CHASSIS WAS ALREADY MOST OF THE WAY THERE and nobody noticed. HIP_NOTCH_X
# is SERVO_W/2 + 0.8, so the lower shelf's notch already runs 0.8 mm off the
# servo's sides - it grips the case over the shelf's 3 mm thickness by accident
# of being sized to clear it. All this adds is depth: a rim on the inside of the
# side plate, running the length of the case.
CRADLE_CLEAR = 0.4            # per side, and it must EXCEED the print tolerance
CRADLE_WALL = 2.5
CRADLE_DEPTH = 8.0            # inboard from the side plate, along the shaft
CRADLE_Z = (-1.0, HIP_NOTCH_Z)   # the notch's own span, which is where the
                                 # servo demonstrably is

PI_HOLES = [(-35.0, -24.5), (-35.0, 24.5), (23.0, -24.5), (23.0, 24.5)]

# --- end panels ---------------------------------------------------------------
#
# The chassis was open front and back: a U-section, which is the worst shape
# there is in torsion, and docs/before-you-order.md has wanted a rear brace for
# a while. Closing the ends turns it into a box.
#
# They start at z = 45.4, not z = 0, and that is not a styling choice. The Pi
# runs x = -34.0..51.0 inside a 90 mm deep chassis, so it has 2.5 mm at each
# end and a 3 mm panel down to the floor would go straight through it. Above
# 45.4 the Pi is done (it tops out at 42.5) and the panel clears everything.
#
# The front panel is a FRAME, not a plate. The battery stands upright at
# y = +/-17, z = 62.5..167.5, and with both ends closed there is no other way
# to get it in or out. The window is sized to pass it with 2 mm to spare.
PANEL_T = 3.0
PANEL_Z0 = 45.4
BACK_BORDER = 9.0             # solid rim round the rear lightening window
FRONT_WIN_Y = 19.0            # battery is +/-17
FRONT_WIN_Z = (53.4, 172.0)   # battery is 62.5..167.5
CORNER_R = 6.0                # window corners, so it reads as designed


def _round_rect(y0, y1, z0, z1, x0, x1, r):
    """A rounded-corner window cutter, in the y-z plane.

    Built from two crossed boxes plus corner cylinders rather than by
    filleting afterwards: OCC refuses the fillet often enough on parts this
    busy that a silent fallback to sharp corners is a real risk, and a sharp
    re-entrant corner in a shear panel is exactly where it would crack.
    """
    cut = _plate((x0, x1), (y0 + r, y1 - r), (z0, z1))
    cut += _plate((x0, x1), (y0, y1), (z0 + r, z1 - r))
    for cy in (y0 + r, y1 - r):
        for cz in (z0 + r, z1 - r):
            cut += (bd.Pos((x0 + x1) / 2, cy, cz) * bd.Rot(0, 90, 0)
                    * bd.Cylinder(r, x1 - x0))
    return cut


def _plate(x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0)


def build():
    part = None
    for sgn in (1, -1):
        y0, y1 = sgn * (SIDE_Y - SIDE_T / 2), sgn * (SIDE_Y + SIDE_T / 2)
        side = _plate((X0, X1), (min(y0, y1), max(y0, y1)), (0, TOP_Z + 1.5))
        side -= _plate((-HIP_NOTCH_X, HIP_NOTCH_X),
                       (min(y0, y1) - 1, max(y0, y1) + 1),
                       (-1, HIP_NOTCH_Z))
        # Hip servo capture, replacing the four M2 that went through this plate.
        # A rim inboard of the side plate, gripping the case sides along the
        # length of the notch.
        inner = sgn * INNER_Y
        cy = tuple(sorted((inner, inner - sgn * CRADLE_DEPTH)))
        xi = SERVO_W / 2 + CRADLE_CLEAR
        xo = xi + CRADLE_WALL
        for sx in (-1, 1):
            side += _plate(tuple(sorted((sx * xi, sx * xo))), cy, CRADLE_Z)
        part = side if part is None else part + side

    for z, name in ((TOP_Z, "top"), (SHELF1_Z, "shelf1"), (SHELF2_Z, "shelf2")):
        shelf = _plate((X0, X1), (-INNER_Y, INNER_Y),
                       (z - PLATE_T / 2, z + PLATE_T / 2))
        if name == "shelf1":
            # At the hip servo's height, so it needs the same notch the side
            # plates do - the servo body passes right through this plane.
            shelf -= _plate((-HIP_NOTCH_X, HIP_NOTCH_X),
                            (-INNER_Y - 1, INNER_Y + 1),
                            (z - PLATE_T, z + PLATE_T))
        part += shelf

    # Pi standoffs and clearance holes on the lower shelf.
    for px, py in PI_HOLES:
        part += bd.Pos(px + 8.5, py, SHELF1_Z + 3.5) * bd.Cylinder(3.0, 4.0)
        part -= bd.Pos(px + 8.5, py, SHELF1_Z) * bd.Cylinder(M25_CLEAR, 20)

    # Close the two open ends. See the note by PANEL_T for why they start at
    # z = 45.4 and why the front one is a frame.
    ztop = TOP_Z + 1.5
    back = _plate((X0, X0 + PANEL_T), (-INNER_Y, INNER_Y), (PANEL_Z0, ztop))
    back -= _round_rect(-INNER_Y + BACK_BORDER, INNER_Y - BACK_BORDER,
                        PANEL_Z0 + BACK_BORDER, ztop - BACK_BORDER,
                        X0 - 1, X0 + PANEL_T + 1, CORNER_R)
    part += back

    front = _plate((X1 - PANEL_T, X1), (-INNER_Y, INNER_Y), (PANEL_Z0, ztop))
    front -= _round_rect(-FRONT_WIN_Y, FRONT_WIN_Y,
                         FRONT_WIN_Z[0], FRONT_WIN_Z[1],
                         X1 - PANEL_T - 1, X1 + 1, CORNER_R)
    part += front

    # Slots for a hook-and-loop strap over the pack.
    for sx in (-14.0, 31.0):
        part -= _plate((sx, sx + 4), (-20.0, -14.0),
                       (SHELF2_Z - 5, SHELF2_Z + 5))
        part -= _plate((sx, sx + 4), (14.0, 20.0),
                       (SHELF2_Z - 5, SHELF2_Z + 5))

    part = part.clean()

    # Soften the four long outside corners. Not only for looks: they are the
    # full-height edges of a shear box, and cad/shape.py explains why a fillet
    # that quietly did not happen is worse than none at all.
    from cad.shape import long_edges, soften
    sy = SIDE_Y + SIDE_T / 2
    part, r = soften(part,
                     long_edges(part, "z", 100.0,
                                at=[(X0, sy), (X0, -sy), (X1, sy), (X1, -sy)]),
                     what="chassis corners")
    build.corner_r = r
    return part


def main(export=True):
    p = build()
    g = p.volume / 1000.0 * PETG_SOLID * INFILL
    if export:
        OUT.mkdir(exist_ok=True)
        bd.export_step(p, str(OUT / "chassis.step"))
        bd.export_stl(p, str(OUT / "chassis.stl"),
                      tolerance=0.02, angular_tolerance=0.2)
    bb = p.bounding_box()
    print(f"bounding box   {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    print(f"solid volume   {p.volume/1000:.2f} cm3")
    print(f"printed @{INFILL:.0%}   {g:.1f} g")
    return g / 1000.0


if __name__ == "__main__":
    main(export="--no-export" not in sys.argv)
