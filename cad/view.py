"""Render a CAD part to a PNG.  uv run python cad/view.py [part.stl]

Loads the exported STL into a bare MuJoCo scene and renders it from three
angles, so you can eyeball a part without a CAD GUI.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

OUT = Path(__file__).parent / "out"
W, H = 640, 720
VIEWS = [("side", 90, -10), ("three-quarter", 140, -20), ("front", 200, -10)]


def main(stl=None):
    stl = Path(stl or OUT / "shin.stl")
    xml = f'''<mujoco>
  <visual><global offwidth="{W}" offheight="{H}"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6" specular="0.2 0.2 0.2"/>
    <quality shadowsize="4096"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.20 0.21 0.23" rgb2="0.26 0.27 0.29"/>
    <material name="grid" texture="grid" texrepeat="30 30"/>
    <mesh name="part" file="{stl}" scale="0.001 0.001 0.001" inertia="exact"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.3 0.6" dir="-0.4 0.4 -1" diffuse="0.7 0.7 0.7"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" pos="0 0 -0.12"/>
    <body pos="0 0 0"><geom type="mesh" mesh="part" rgba="0.88 0.45 0.13 1"/></body>
  </worldbody>
</mujoco>'''
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.lookat[:] = 0.20, [-0.02, 0.02, -0.05]

    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
    tiles = []
    for name, az, el in VIEWS:
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, camera=cam)
        im = Image.fromarray(r.render())
        dr = ImageDraw.Draw(im, "RGBA")
        dr.rectangle([0, 0, W, 40], fill=(0, 0, 0, 150))
        dr.text((16, 9), name, font=font, fill=(255, 255, 255, 255))
        tiles.append(im)

    sheet = Image.new("RGB", (W * len(tiles), H))
    for i, t in enumerate(tiles):
        sheet.paste(t, (i * W, 0))
    out = OUT / (stl.stem + "_views.png")
    sheet.save(out)
    print(f"wrote {out}  ({sheet.size[0]}x{sheet.size[1]})")
    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
