"""HD stills of the full robot from several angles, real CAD meshes in.

    uv run python render_views.py

Writes renders/views/*.png. load(meshes=True) shows the real solids; fitcheck's
pose() stands it cleanly in wheel mode and in foot mode (and solves the
parallelogram, which mj_forward alone leaves hanging).
"""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from src.rsbot.model import load
from fitcheck import pose

OUT = Path("renders/views")
OUT.mkdir(parents=True, exist_ok=True)

SIZE = 1600
m, d = load(meshes=True)
m.vis.global_.offwidth = SIZE
m.vis.global_.offheight = SIZE
renderer = mujoco.Renderer(m, SIZE, SIZE)
print(f"rendering at {SIZE}x{SIZE}")

opt = mujoco.MjvOption()
mujoco.mjv_defaultOption(opt)

cam = mujoco.MjvCamera()


def shoot(label, az, el, dist, lookz=0.205):
    cam.lookat[:] = [0.0, 0.0, lookz]
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    renderer.update_scene(d, camera=cam, scene_option=opt)
    Image.fromarray(renderer.render()).save(OUT / f"rsbot-{label}.png")
    print(f"  wrote {label}")


# --- wheel mode, standing tall on the two wheels -----------------------------
pose(m, d, "wheel")
for label, az, el, dist in [
    ("wheel-front",        180, -6,  0.66),
    ("wheel-front-3q",     220, -10, 0.68),
    ("wheel-side",         270, -6,  0.66),
    ("wheel-hero-low",     215, -24, 0.62),
    ("wheel-back-3q",      320, -10, 0.68),
    ("wheel-top",          180, -50, 0.72),
]:
    shoot(label, az, el, dist)

# --- foot mode, wheels rolled flat, standing on their faces ------------------
pose(m, d, "foot")
for label, az, el, dist in [
    ("foot-front-3q",      220, -12, 0.64),
    ("foot-side",          270, -8,  0.64),
    ("foot-hero-low",      215, -22, 0.60),
]:
    shoot(label, az, el, dist, lookz=0.18)

print(f"done -> {OUT}/")
