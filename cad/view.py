"""Render a CAD part to a PNG.

    uv run python cad/view.py [part.stl] [out.png]

Loads the exported STL into a bare MuJoCo scene and renders it from three
angles, so you can eyeball a part without a CAD GUI.

Output is timestamped by default, so successive revisions accumulate side by
side instead of overwriting each other. Comparing a part against the version
from an hour ago is most of the value.
"""

import os
import sys
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

OUT = Path(__file__).parent / "out"
W, H = 560, 620
# Camera auto-frames from the mesh bounds, so this works on a 20 mm bracket and
# a 180 mm chassis without editing anything.
# Two rows: the four orthogonal-ish views on top, and views that actually show
# the features - the hub bolt circle and the bearing counterbore both face +y,
# so they are invisible from every angle in the first row.
VIEWS = [("side", 90, -8, 1.0), ("front", 180, -8, 1.0),
         ("three-quarter", 140, -22, 1.0), ("top", 90, -65, 1.0),
         ("close, +y face", 90, -4, 0.34), ("close, -y face", 270, -4, 0.34)]


def main(stl=None, out=None):
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
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" pos="0 0 -5"/>
    <body pos="0 0 0"><geom name="part" type="mesh" mesh="part" rgba="0.88 0.45 0.13 1"/></body>
  </worldbody>
</mujoco>'''
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)

    # Frame from the actual mesh bounds.
    # Use the geom's world AABB. Not the mesh vertices: MuJoCo re-centres a
    # mesh on its centroid at compile time and compensates with geom_pos, so
    # the vertex centroid is not where the part actually is.
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "part")
    centre = d.geom_xpos[gid] + m.geom_aabb[gid][:3]
    span = 2.0 * float(np.linalg.norm(m.geom_aabb[gid][3:]))
    cam = mujoco.MjvCamera()

    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    tiles = []
    for name, az, el, zoom in VIEWS:
        cam.azimuth, cam.elevation = az, el
        cam.distance = span * 1.25 * zoom
        cam.lookat[:] = centre
        r.update_scene(d, camera=cam)
        im = Image.fromarray(r.render())
        dr = ImageDraw.Draw(im, "RGBA")
        dr.rectangle([0, 0, W, 36], fill=(0, 0, 0, 155))
        dr.text((14, 7), name, font=font, fill=(255, 255, 255, 255))
        tiles.append(im)

    cols = 3
    rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new("RGB", (W * cols, H * rows), (14, 16, 19))
    for i, t in enumerate(tiles):
        sheet.paste(t, ((i % cols) * W, (i // cols) * H))
    if out is None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        out = OUT / f"{stl.stem}_views_{stamp}.png"
    out = Path(out)
    sheet.save(out)
    print(f"wrote {out}  ({sheet.size[0]}x{sheet.size[1]})")
    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None,
         sys.argv[2] if len(sys.argv) > 2 else None)
