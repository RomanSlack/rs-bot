"""Render a demo run to MP4.  uv run python render.py [out.mp4]

Offscreen via EGL, so it works without a window. The camera tracks the torso.
"""

import os
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from src.rsbot.model import load  # noqa: E402
from src.rsbot.sim import CTRL_HZ, obs  # noqa: E402
from src.rsbot.balance import Balancer, pitch_from_quat  # noqa: E402

W, H, FPS = 960, 540, 60

# (t_start, label, v_des, shove_impulse)
SCRIPT = [
    (0.0, "standing", 0.0, None),
    (2.5, "drive forward 0.35 m/s", 0.35, None),
    (6.0, "stop", 0.0, None),
    (7.5, "drive back", -0.35, None),
    (10.0, "stop", 0.0, None),
    (11.5, "SHOVE  0.7 N.s", 0.0, 0.7),
    (15.0, "recovered", 0.0, None),
    (16.5, "SHOVE  1.4 N.s", 0.0, 1.4),
    (20.0, "recovered", 0.0, None),
]
DURATION = 22.0


def cue(t):
    for i in range(len(SCRIPT) - 1, -1, -1):
        if t >= SCRIPT[i][0]:
            return SCRIPT[i]
    return SCRIPT[0]


def main(out="rsbot-stage0.mp4"):
    m, d = load()
    bal = Balancer()
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    frame_every = int(round(1.0 / (FPS * m.opt.timestep)))

    renderer = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.95, -8, 132
    cam.lookat[:] = [0, 0, 0.20]

    # One drawtext per cue, switched on over that cue's time window.
    draws = []
    for i, (t0, label, _, _) in enumerate(SCRIPT):
        t1 = SCRIPT[i + 1][0] if i + 1 < len(SCRIPT) else DURATION
        txt = label.replace(":", r"\:").replace("'", "")
        draws.append(
            f"drawtext=text='{txt}':x=28:y=28:fontsize=30:fontcolor=white:"
            f"box=1:boxcolor=black@0.55:boxborderw=10:"
            f"enable='between(t,{t0},{t1})'")
    vf = ",".join(draws) + ",format=yuv420p"

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-vf", vf, "-crf", "20", out],
        stdin=subprocess.PIPE)

    fired = set()
    shove_until = -1.0
    for k in range(int(DURATION / m.opt.timestep)):
        t = k * m.opt.timestep
        t0, label, v_des, imp = cue(t)

        if k % decim == 0:
            d.ctrl[:] = bal(obs(m, d), dt, v_des)

        d.xfrc_applied[torso] = 0.0
        if imp is not None and t0 not in fired:
            fired.add(t0)
            shove_until = t + 0.010
            d.xfrc_applied[torso, 0] = imp / 0.010
        elif t < shove_until:
            d.xfrc_applied[torso, 0] = imp / 0.010

        mujoco.mj_step(m, d)

        if k % frame_every == 0:
            cam.lookat[0] = d.xpos[torso][0]
            renderer.update_scene(d, camera=cam)
            ff.stdin.write(renderer.render().tobytes())

    ff.stdin.close()
    ff.wait()

    pitch = pitch_from_quat(obs(m, d)["quat"])
    print(f"wrote {out}  ({DURATION:.0f} s)  final pitch {np.degrees(pitch):+.2f} deg, "
          f"x {d.xpos[torso][0]:+.3f} m, upright={d.xpos[torso][2] > 0.20}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "rsbot-stage0.mp4")
