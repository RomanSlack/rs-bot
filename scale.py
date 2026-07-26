"""Render a true-scale still beside a person.  uv run python scale.py [out.png]

The robot is 427 mm tall, which is hard to feel from a number. Next to a
1.75 m (5 ft 9 in) figure it reads as knee-high, which it is.
"""

import os
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from src.rsbot.model import load  # noqa: E402

W, H = 1600, 900
DIMS = ["height 16.8 in / 427 mm", "width 8.6 in / 219 mm",
        "depth 5.2 in / 131 mm", "wheel 3.15 in / 80 mm",
        "mass 4.5 lb / 2.05 kg"]


def main(out="renders/rsbot-scale.png"):
    m, d = load()
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 2.55, -6, 2
    cam.lookat[:] = [0.05, -0.36, 0.62]
    r.update_scene(d, camera=cam)

    im = Image.fromarray(r.render())
    dr = ImageDraw.Draw(im, "RGBA")
    bold = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
    reg = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 21)
    dr.rectangle([0, 0, W, 64], fill=(0, 0, 0, 155))
    dr.text((30, 17), "rs-bot at true scale, beside a 5 ft 9 in person",
            font=bold, fill=(255, 255, 255, 255))
    dr.rectangle([0, H - 48, W, H], fill=(0, 0, 0, 145))
    dr.text((30, H - 36), "      ".join(DIMS), font=reg,
            fill=(214, 220, 230, 255))
    im.save(out)
    print(f"wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "renders/rsbot-scale.png")
