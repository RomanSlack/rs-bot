"""A short orbit + flip clip of the robot: MP4 and GIF.

    uv run python render_anim.py

The camera orbits 360 degrees while the robot flips wheel -> foot -> wheel (its
signature move). Real CAD meshes. Writes renders/rsbot-orbit.mp4 and .gif.
"""
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from src.rsbot.model import load
from fitcheck import pose

OUT = Path("renders")
OUT.mkdir(exist_ok=True)
W = H = 960
N = 96
FPS = 24

m, d = load(meshes=True)
# hide the scale-reference human so the background is clean
_hb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "human")
for i in range(m.ngeom):
    if _hb >= 0 and m.geom_bodyid[i] == _hb:
        m.geom_rgba[i] = [0, 0, 0, 0]
m.vis.global_.offwidth = W
m.vis.global_.offheight = H
r = mujoco.Renderer(m, H, W)
opt = mujoco.MjvOption()
mujoco.mjv_defaultOption(opt)
cam = mujoco.MjvCamera()
torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")

mp4 = OUT / "rsbot-orbit.mp4"
ff = subprocess.Popen(
    ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
     "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
     "-vf", "format=yuv420p", "-crf", "18", str(mp4)],
    stdin=subprocess.PIPE)

gif = []
for i in range(N):
    t = i / N
    frac = (1 - np.cos(2 * np.pi * t)) / 2          # 0 -> 1 -> 0, smooth flip
    pose(m, d, "wheel", frac=frac)
    cam.lookat[:] = [d.xpos[torso][0], d.xpos[torso][1], 0.18]
    cam.azimuth = 110 + 360 * t
    cam.elevation = -10
    cam.distance = 0.74
    r.update_scene(d, camera=cam, scene_option=opt)
    rgb = r.render()
    ff.stdin.write(rgb.tobytes())
    if i % 2 == 0:
        gif.append(Image.fromarray(rgb).resize((440, 440)))
    if i % 16 == 0:
        print(f"  frame {i}/{N}")

ff.stdin.close()
ff.wait()
gif[0].save(OUT / "rsbot-orbit.gif", save_all=True, append_images=gif[1:],
            duration=int(1000 / (FPS / 2)), loop=0, optimize=True)
print(f"done -> {mp4} and {OUT}/rsbot-orbit.gif")
