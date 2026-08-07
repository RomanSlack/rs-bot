"""A driving demo: start in foot mode, unfold to wheels, drive a fast circle,
fold back to foot mode. Real controllers, real meshes. MP4 + GIF.

    uv run python render_drive.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from src.rsbot.model import load, ROLL_FOOT
from src.rsbot.sim import CTRL_HZ, obs
from src.rsbot.balance import pitch_from_quat
from src.rsbot.transition import DeployMachine, DeployCfg, NAMES, WHEEL, STAND
from fitcheck import pose as kpose

W = H = 900
FPS = 30
V, YAW = 0.42, 0.70                 # cruise speed, yaw rate -> circle ~0.6 m radius

m, d = load(meshes=True)
hb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "human")
for i in range(m.ngeom):
    if hb >= 0 and m.geom_bodyid[i] == hb:
        m.geom_rgba[i] = [0, 0, 0, 0]
m.vis.global_.offwidth = W
m.vis.global_.offheight = H

# --- start already in FOOT mode ---
kpose(m, d, "foot")
d.qvel[:] = 0.0
mach = DeployMachine(cfg=DeployCfg(flip_rate=2.0))
mach.state = STAND
mach.roll = ROLL_FOOT

torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
dt = decim * m.opt.timestep
frame_every = int(round(1.0 / (FPS * m.opt.timestep)))

renderer = mujoco.Renderer(m, H, W)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
cam = mujoco.MjvCamera()

STAND1, DRIVE_FOR, STAND2 = 1.6, 9.0, 2.0
retracted = back_wheel = drove_until = deployed = ended = None
frames = []
duration = 30.0
az0 = 120.0

for k in range(int(duration / m.opt.timestep)):
    t = k * m.opt.timestep

    if retracted is None and t >= STAND1:
        mach.start_retract(); retracted = t
    if retracted is not None and back_wheel is None and mach.state == WHEEL:
        back_wheel = t; drove_until = t + DRIVE_FOR
    if drove_until is not None and deployed is None and t >= drove_until:
        mach.start_deploy(); deployed = t
    if deployed is not None and mach.state == STAND and ended is None:
        ended = t
    if ended is not None and t >= ended + STAND2:
        duration = t; break

    driving = back_wheel is not None and deployed is None and mach.state == WHEEL
    v = V if driving else 0.0
    yaw = YAW if driving else 0.0

    if k % decim == 0:
        d.ctrl[:] = mach(obs(m, d), dt, v_des=v, yaw_des=yaw)
    mujoco.mj_step(m, d)

    if k % frame_every == 0:
        cam.lookat[:] = [d.xpos[torso][0], d.xpos[torso][1], 0.16]
        cam.azimuth = az0 + 20.0 * np.sin(t * 0.5)
        cam.elevation = -11
        cam.distance = 0.95
        renderer.update_scene(d, camera=cam, scene_option=opt)
        rgb = renderer.render()
        frames.append(rgb)

pitch = float(np.degrees(pitch_from_quat(obs(m, d)["quat"])))
print(f"{duration:.1f}s  final state={NAMES[mach.state]}  upright={d.xpos[torso][2]>0.18}  "
      f"pitch={pitch:+.1f}  x={d.xpos[torso][0]:+.2f} y={d.xpos[torso][1]:+.2f}")

OUT = Path("renders"); OUT.mkdir(exist_ok=True)
mp4 = OUT / "rsbot-drive.mp4"
ff = subprocess.Popen(
    ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
     "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
     "-vf", "format=yuv420p", "-crf", "18", str(mp4)], stdin=subprocess.PIPE)
for f in frames:
    ff.stdin.write(f.tobytes())
ff.stdin.close(); ff.wait()

# GIF from the MP4, palette-based (small and clean; PIL's whole-frame GIF is huge)
gifp = OUT / "rsbot-drive.gif"
pal = OUT / "_pal.png"
vf = "setpts=0.6*PTS,fps=10,scale=340:-1:flags=lanczos"
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4),
                "-vf", f"{vf},palettegen=stats_mode=diff", str(pal)], check=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4), "-i", str(pal),
                "-filter_complex",
                f"[0:v]{vf}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=3",
                str(gifp)], check=True)
pal.unlink(missing_ok=True)
print(f"wrote {mp4} and {gifp}  ({len(frames)} frames)")
