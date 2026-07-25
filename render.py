"""Render the full round trip to MP4.  uv run python render.py [out.mp4]

Drive forward -> flip the wheels flat -> stand with the controller off ->
flip back -> keep driving. Offscreen via EGL, so it needs no window.

Captions are driven off the state machine itself rather than a fixed timeline,
so what the video says is what the controller is actually doing.
"""

import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from src.rsbot.model import load  # noqa: E402
from src.rsbot.sim import CTRL_HZ, obs  # noqa: E402
from src.rsbot.balance import pitch_from_quat  # noqa: E402
from src.rsbot.transition import (DeployMachine, DeployCfg, NAMES, STAND,
                                  WHEEL)  # noqa: E402

W, H, FPS = 1280, 720, 60
CRUISE = 0.35

CAPTION = {
    "WHEEL":  "wheel mode - actively balancing",
    "SETTLE": "stopping, waiting for a quiet moment",
    "FLIP":   "FLIP - ankles roll 90 degrees",
    "STAND":  "foot mode - balancer OFF, standing on the wheel faces",
    "UNFLIP": "UNFLIP - rolling back upright",
}

FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)


def annotate(rgb, caption, rows):
    im = Image.fromarray(rgb)
    dr = ImageDraw.Draw(im, "RGBA")
    dr.rectangle([0, 0, W, 66], fill=(0, 0, 0, 150))
    dr.text((26, 18), caption, font=FONT, fill=(255, 255, 255, 255))
    dr.rectangle([0, H - 46, W, H], fill=(0, 0, 0, 140))
    dr.text((26, H - 36), "     ".join(rows), font=SMALL, fill=(210, 216, 226, 255))
    return im


def default_out():
    Path("renders").mkdir(exist_ok=True)
    return f"renders/rsbot-cycle-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"


def main(out=None):
    out = out or default_out()
    m, d = load()
    mach = DeployMachine(cfg=DeployCfg(flip_rate=2.0))
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    wheel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")

    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    frame_every = int(round(1.0 / (FPS * m.opt.timestep)))

    renderer = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.05, -9, 128

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-vf", "format=yuv420p", "-crf", "19", out], stdin=subprocess.PIPE)

    FLIP_AT, STAND_FOR, DRIVE_AFTER = 4.0, 5.0, 6.0
    fired = stood_at = retracted = back_at = None
    duration = 40.0

    for k in range(int(duration / m.opt.timestep)):
        t = k * m.opt.timestep

        if fired is None and t >= FLIP_AT:
            mach.start_deploy()
            fired = t
        if mach.state == STAND and stood_at is None:
            stood_at = t
        if stood_at is not None and retracted is None and t >= stood_at + STAND_FOR:
            mach.start_retract()
            retracted = t
        if retracted is not None and mach.state == WHEEL and back_at is None:
            back_at = t
        if back_at is not None and t >= back_at + DRIVE_AFTER:
            duration = t
            break

        # Drive before the flip and again once it is back on its wheels. The
        # machine gates this to zero in every other state by itself.
        drive = CRUISE if (fired is None or back_at is not None) else 0.0

        if k % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt, v_des=drive)
        mujoco.mj_step(m, d)

        if k % frame_every == 0:
            cam.lookat[0] = d.xpos[torso][0]
            cam.lookat[2] = 0.20
            renderer.update_scene(d, camera=cam)
            pitch = np.degrees(pitch_from_quat(obs(m, d)["quat"]))
            rows = [f"state {NAMES[mach.state]}",
                    f"roll {np.degrees(mach.roll):5.1f} deg",
                    f"axle {d.xpos[wheel][2]*1000:5.1f} mm",
                    f"pitch {pitch:+5.1f} deg",
                    f"x {d.xpos[torso][0]:+5.2f} m"]
            frame = annotate(renderer.render(), CAPTION[NAMES[mach.state]], rows)
            ff.stdin.write(frame.tobytes())

    ff.stdin.close()
    ff.wait()
    print(f"wrote {out}  ({duration:.1f} s)  travelled {d.xpos[torso][0]:+.2f} m, "
          f"upright={d.xpos[torso][2] > 0.18}, state={NAMES[mach.state]}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
