"""Shin with its servos fitted.  uv run python cad/assembly.py

Two STS3215s touch this part: the KNEE servo, whose case bolts to the thigh
and whose horn drives the shin's hub, and the ANKLE-PITCH servo, whose case
bolts to the shin's standoff. Drawing them together is the only way to see
whether the mounts line up and whether anything fouls.

Servos are drawn as their bounding block plus an output horn. They are bought
parts; past their envelope and mass there is nothing to model.
"""

import os
import sys
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import build123d as bd  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

import cad.shin as shin  # noqa: E402

OUT = Path(__file__).parent / "out"
SERVO_L, SERVO_W, SERVO_H = 45.2, 24.7, 35.4
HORN_R, HORN_T = 10.0, 3.0
W, H = 640, 700


def _block(cx, cy, cz, sx, sy, sz):
    return bd.Pos(cx, cy, cz) * bd.Box(sx, sy, sz)


def build():
    """Returns [(solid, colour, label)] in the shin's own frame, millimetres.

    Both servos are placed by their MOUNTING, not by eye:

      knee  - horn on the knee axis (y) at z = 0, case reaching up into the
              thigh, its output face against the inboard side of the hub
      ankle - case flat on the +y face of the standoff, spanning the bolts
    """
    part = shin.build()

    # Knee servo. The shaft sits SHAFT_INSET from one end of the case, so the
    # case runs from just below the axis up into the thigh.
    ky = shin.SPINE_Y - shin.SPY - SERVO_H / 2
    kz = -shin.SHAFT_INSET + SERVO_L / 2
    knee = _block(0, ky, kz, SERVO_W, SERVO_H, SERVO_L)
    knee += bd.Pos(0, shin.SPINE_Y - shin.SPY - HORN_T / 2, 0) * \
        bd.Rot(90, 0, 0) * bd.Cylinder(HORN_R, HORN_T)

    # Ankle-pitch servo, flat on the standoff face.
    stand_face = shin.STANDOFF[1][1]
    az = (shin.STANDOFF[2][0] + shin.STANDOFF[2][1]) / 2
    ankle = _block(0, stand_face + SERVO_H / 2, az, SERVO_W, SERVO_H, SERVO_L)

    return [(part, "0.88 0.45 0.13 1", "shin"),
            (knee, "0.13 0.13 0.15 1", "knee servo"),
            (ankle, "0.22 0.22 0.26 1", "ankle-pitch servo")]


def main():
    OUT.mkdir(exist_ok=True)
    items = build()
    assets, bodies = [], []
    for i, (solid, rgba, _) in enumerate(items):
        f = OUT / f"_asm{i}.stl"
        bd.export_stl(solid, str(f), tolerance=0.02, angular_tolerance=0.2)
        assets.append(f'<mesh name="m{i}" file="{f}" scale="0.001 0.001 0.001"/>')
        bodies.append(f'<body><geom type="mesh" mesh="m{i}" rgba="{rgba}"/></body>')

    xml = f'''<mujoco>
  <visual><global offwidth="{W}" offheight="{H}"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.15 0.15 0.15"/>
    <quality shadowsize="4096"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.20 0.21 0.23" rgb2="0.26 0.27 0.29"/>
    <material name="grid" texture="grid" texrepeat="30 30"/>
    {chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 0.6" dir="-0.4 0.4 -1" diffuse="0.7 0.7 0.7"/>
    <geom type="plane" size="0 0 0.05" material="grid" pos="0 0 -0.13"/>
    {chr(10).join(bodies)}
  </worldbody>
</mujoco>'''
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (-0.015, 0.02, -0.04)

    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    views = [("side", 90, -8, 0.24), ("three-quarter", 140, -20, 0.24),
             ("front", 180, -8, 0.24)]
    tiles = []
    for name, az, el, dist in views:
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        r.update_scene(d, camera=cam)
        im = Image.fromarray(r.render())
        dr = ImageDraw.Draw(im, "RGBA")
        dr.rectangle([0, 0, W, 36], fill=(0, 0, 0, 155))
        dr.text((14, 7), name, font=font, fill=(255, 255, 255, 255))
        tiles.append(im)

    sheet = Image.new("RGB", (W * len(tiles), H))
    for i, t in enumerate(tiles):
        sheet.paste(t, (i * W, 0))
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = OUT / f"shin_assembly_{stamp}.png"
    sheet.save(out)
    print(f"wrote {out}")
    return out


if __name__ == "__main__":
    main()
