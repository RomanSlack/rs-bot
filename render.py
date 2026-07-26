"""Render the full round trip to MP4.

    uv run python render.py [out.mp4] [backlash_degrees]

Drive forward -> flip the wheels flat -> stand with the controller off ->
flip back -> keep driving. Offscreen via EGL, so it needs no window.

Captions are driven off the state machine itself rather than a fixed timeline,
so what the video says is what the controller is actually doing.
"""

import math
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
from src.rsbot.balance import LASH_GAINS, pitch_from_quat  # noqa: E402
from src.rsbot.transition import (DeployMachine, DeployCfg, NAMES, STAND,
                                  WHEEL)  # noqa: E402

W, H, FPS = 1280, 720, 60
CRUISE = 0.35

CAPTION = {
    "WHEEL":  "wheel mode - actively balancing",
    "TURN":   "steering - differential wheel speed, no steering joint",
    "SETTLE": "stopping, waiting for a quiet moment",
    "FLIP":   "FLIP - ankles roll 90 degrees",
    "STAND":  "foot mode - standing on the wheel faces, weak ankle loop only",
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


def main(out=None, lash_deg=0.0):
    out = out or default_out()
    lash = math.radians(lash_deg)
    m, d = load(backlash=lash)
    # The rigid defaults survive no lash at all, so switch gain sets with it.
    mach = DeployMachine(gains=LASH_GAINS if lash else None,
                         cfg=DeployCfg(flip_rate=2.0))
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    wheel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")

    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    frame_every = int(round(1.0 / (FPS * m.opt.timestep)))

    renderer = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.82, -10, 128

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-vf", "format=yuv420p", "-crf", "19", out], stdin=subprocess.PIPE)

    # 0.6 rad/s: turning harder than that and then stopping can tip it,
    # because the yaw loop and the settle deceleration interact.
    TURN_AT, TURN_FOR, TURN_RATE = 2.0, 3.4, 0.6
    FLIP_AT, STAND_FOR, DRIVE_AFTER = 8.0, 5.0, 6.0
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
        turning = TURN_AT <= t < TURN_AT + TURN_FOR
        yaw = TURN_RATE if turning else 0.0

        if k % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt, v_des=drive, yaw_des=yaw)
        mujoco.mj_step(m, d)

        if k % frame_every == 0:
            cam.lookat[0] = d.xpos[torso][0]
            cam.lookat[2] = 0.20
            cam.azimuth = 128 + 14.0 * np.sin(t * 0.30)   # slow drift
            renderer.update_scene(d, camera=cam)
            pitch = np.degrees(pitch_from_quat(obs(m, d)["quat"]))
            rows = [f"state {NAMES[mach.state]}",
                    f"roll {np.degrees(mach.roll):5.1f} deg",
                    f"axle {d.xpos[wheel][2]*1000:5.1f} mm",
                    f"pitch {pitch:+5.1f} deg",
                    f"x {d.xpos[torso][0]:+5.2f} m"]
            if lash_deg:
                rows.append(f"backlash {lash_deg:g} deg")
            cap = CAPTION["TURN" if turning else NAMES[mach.state]]
            frame = annotate(renderer.render(), cap, rows)
            ff.stdin.write(frame.tobytes())

    ff.stdin.close()
    ff.wait()
    print(f"wrote {out}  ({duration:.1f} s)  travelled {d.xpos[torso][0]:+.2f} m, "
          f"upright={d.xpos[torso][2] > 0.18}, state={NAMES[mach.state]}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None,
         float(sys.argv[2]) if len(sys.argv) > 2 else 0.0)
