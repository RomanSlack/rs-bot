"""The parallelogram, moving.

    uv run python -m cad.linkage_anim [out.mp4]

A still cannot explain a mechanism. This is the same leg twice, side by side,
driven through the same squat and the same flip:

    left    with the parallelogram fitted
    right   with the ankle-pitch servo deleted and nothing put in its place,
            so the ankle joint just holds whatever angle it was left at

Both legs are given identical hip and knee angles. The ONLY difference is what
holds the ankle, and the whole argument for the mechanism is in the gap between
the two.

WHAT TO WATCH. The foot's angle is `hip + knee + ankle`, which is its tilt
relative to the torso, and it is drawn as a bar hung off the ankle against a
fixed reference bar on the torso.

  - squatting, on the left, the two bars stay parallel. The hip and knee both
    move and the ankle counter-rotates by exactly their sum, because the rods
    carry the torso's attitude down past the knee. Nothing commands it.
  - squatting, on the right, the foot follows the shin, because that is what an
    undriven joint does.
  - flipping, on the left, the wheel lies down flat and the robot stands on the
    whole 80 mm face.
  - flipping, on the right, it lands on an edge and leans, because the sole's
    normal IS the ankle body's own axis. This is why ankle pitch matters at all
    in wheel mode: it is aiming the sole for a landing that has not happened
    yet.

THE POSES ARE KINEMATIC, not simulated. This is a diagram of a mechanism, not
a physics result - docs/deleting-the-ankle-pitch-servo.md has the physics. Both
robots are dropped onto the floor by measuring their own lowest point, so the
lean on the right is the real geometric consequence and not a drawing trick.
"""

import os
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

import cad.robot as robot  # noqa: E402
from src.rsbot.model import (ROLL_FOOT, ankle_pitch_level,  # noqa: E402
                             axle_height, leg_ik, load)

PANEL_W, H, FPS = 660, 760, 30
W = PANEL_W * 2

TITLE = ImageFont.truetype(
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 25)
BODY = ImageFont.truetype(
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
BIG = ImageFont.truetype(
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 34)

# The two marker bars, which are the whole point of the picture. Outboard of
# everything on the leg so they read against the floor rather than through the
# wheel, and set at nearly the same height as each other so that "are these two
# parallel" is a question the eye can answer without measuring.
#
# One is bolted to the TORSO and is therefore always level. One is bolted to the
# ANKLE BODY, so its angle IS hip + knee + ankle. They are not structure and
# they appear in no check.
# BOTH bars sit at the ankle, one behind the other, which is the whole trick:
# co-located, the angle between them is a protractor and needs no measuring. The
# first version put the reference on the torso where it belongs physically, and
# the two ended up 200 mm apart with the leg in between, so the one comparison
# the video exists to make was the one thing you could not do.
MARK_Y = 0.115
MARK_L, MARK_T = 0.062, 0.003
C_REF = "0.80 0.82 0.86 1"
C_FOOT = "0.20 0.55 0.85 1"
C_BAD = "0.92 0.35 0.25 1"


def _pose(m, d, height, roll, ankle=None):
    """Pose the sim kinematically. `ankle` overrides the level ankle, which is
    the entire difference between the two panels."""
    hip, knee = leg_ik(height)
    want = {"hip": hip, "knee": knee, "ankle_roll": roll,
            "ankle_pitch": ankle_pitch_level(hip, knee) if ankle is None
            else ankle}
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = height + axle_height(roll), 1.0
    for j in range(m.njnt):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        if n and n != "root":
            d.qpos[m.jnt_qposadr[j]] = want.get(n.rsplit("_", 1)[0], 0.0)
    mujoco.mj_forward(m, d)
    return want


def _markers(m, d):
    """[(name, geom xml, pos, quat)] for the two reference bars.

    One on the torso and one on the ankle body, both pointing forward. They are
    parallel exactly when hip + knee + ankle = 0, which is the claim.
    """
    ankle = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ankle_l")
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    at = np.array(d.xpos[ankle])
    out = []
    # Same place, different ATTITUDE: one carries the torso's, one the ankle
    # body's own. 8 mm apart in y only so they do not fight for the same pixels.
    for name, src, colour, dy in (("mk_ref", torso, C_REF, 0.0),
                                  ("mk_foot", ankle, C_FOOT, 0.008)):
        R = np.array(d.xmat[src]).reshape(3, 3)
        # Offset FORWARD by half its own length, along its own body's x, so the
        # bar starts at the ankle and reaches into clear air. Centred on the
        # ankle, half of each bar was buried in the leg.
        p = at + R @ np.array([MARK_L, MARK_Y + dy, 0.0])
        out.append((name,
                    f'<geom name="g_{name}" type="box" '
                    f'size="{MARK_L} {MARK_T} {MARK_T}" '
                    f'rgba="{colour}"/>', p, robot._quat(R)))
    return out


def _items(m, d, linkage_on):
    assets, items = robot.scene_items(m, d, linkage_on)
    return assets, items + _markers(m, d)


def _scene(m, d, linkage_on):
    assets, items = _items(m, d, linkage_on)
    bodies = [f'<body name="{n}" pos="{p[0]} {p[1]} {p[2]}" '
              f'quat="{q[0]} {q[1]} {q[2]} {q[3]}">{g}</body>'
              for n, g, p, q in items]
    return f'''<mujoco>
  <compiler angle="radian"/>
  <visual><global offwidth="{PANEL_W}" offheight="{H}"/>
    <headlight ambient="0.50 0.50 0.50" diffuse="0.6 0.6 0.6"
               specular="0.12 0.12 0.12"/>
    <quality shadowsize="4096"/><map znear="0.01" zfar="40"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.20 0.21 0.23" rgb2="0.26 0.27 0.29"/>
    <material name="grid" texture="grid" texrepeat="40 40"/>
    {chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="0.4 -0.5 0.9" dir="-0.4 0.5 -1" diffuse="0.7 0.7 0.7"/>
    <geom type="plane" size="0 0 0.05" material="grid"/>
    {chr(10).join(bodies)}
  </worldbody>
</mujoco>'''


def _drop(sm, sd):
    """Shift the whole scene so its lowest point sits on the floor.

    The right-hand robot lands on the EDGE of a tilted disc, so it does not
    stand as tall as the left one and it leans. That is the geometric
    consequence of the foot angle and it is the thing worth seeing, so it is
    measured off the mesh vertices rather than assumed away by planting both
    robots at the same height.
    """
    lo = np.inf
    for g in range(sm.ngeom):
        mid = sm.geom_dataid[g]
        if sm.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or mid < 0:
            continue
        a, n = sm.mesh_vertadr[mid], sm.mesh_vertnum[mid]
        v = sm.mesh_vert[a:a + n].astype(float)
        w = v @ np.array(sd.geom_xmat[g]).reshape(3, 3).T + sd.geom_xpos[g]
        lo = min(lo, w[:, 2].min())
    sm.body_pos[1:, 2] -= lo
    mujoco.mj_forward(sm, sd)
    return lo


# t -> (leg length, ankle roll). Held flat at each end so there is time to read
# what happened before it moves again.
def _timeline(t):
    def ramp(a, b, t0, t1):
        u = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
        u = u * u * (3 - 2 * u)                      # ease, so it reads as motion
        return a + (b - a) * u

    if t < 1.2:
        return 0.207, 0.0, "the leg starts extended"
    if t < 3.2:
        return ramp(0.207, 0.185, 1.2, 3.2), 0.0, "SQUAT - hip and knee fold"
    if t < 5.2:
        return ramp(0.185, 0.207, 3.2, 5.2), 0.0, "and back out again"
    if t < 7.0:
        # 0.195 is transition.py's stand_height: the real machine squats to it
        # in SETTLE and flips from there. It matters here because it is the
        # angle the sole is aimed at while the wheel is still round.
        return ramp(0.207, 0.195, 5.2, 7.0), 0.0, "settle, ready to flip"
    if t < 9.5:
        return 0.195, ramp(0.0, ROLL_FOOT, 7.0, 9.5), "FLIP - the wheel lies down"
    return 0.195, ROLL_FOOT, "standing on the wheel faces"


DURATION = 12.5


def _annotate(rgb, title, subtitle, rows, warn=False):
    im = Image.fromarray(rgb)
    dr = ImageDraw.Draw(im, "RGBA")
    dr.rectangle([0, 0, PANEL_W, 84], fill=(0, 0, 0, 165))
    dr.text((20, 12), title, font=TITLE,
            fill=(255, 170, 150, 255) if warn else (150, 210, 255, 255))
    dr.text((20, 48), subtitle, font=BODY, fill=(200, 206, 216, 255))
    dr.rectangle([0, H - 96, PANEL_W, H], fill=(0, 0, 0, 150))
    for k, r in enumerate(rows):
        dr.text((20, H - 86 + 26 * k), r, font=BODY, fill=(210, 216, 226, 255))
    return im


def main(out=None):
    Path("renders").mkdir(exist_ok=True)
    out = out or "renders/linkage-explained.mp4"
    robot.export_all()

    m, d = load()
    # The locked panel keeps the ankle angle it starts with, forever. That is
    # what an ankle joint with nothing attached to it does.
    hip0, knee0 = leg_ik(0.207)
    locked = ankle_pitch_level(hip0, knee0)

    panels = []
    for linkage_on in (True, False):
        _pose(m, d, 0.207, 0.0, None if linkage_on else locked)
        sm = mujoco.MjModel.from_xml_string(_scene(m, d, linkage_on))
        sd = mujoco.MjData(sm)
        panels.append((linkage_on, sm, sd, mujoco.Renderer(sm, H, PANEL_W)))

    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 90, -7, 0.70

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
         "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-vf", "format=yuv420p", "-crf", "18", out], stdin=subprocess.PIPE)

    for k in range(int(DURATION * FPS)):
        t = k / FPS
        height, roll, caption = _timeline(t)
        tiles = []
        for linkage_on, sm, sd, r in panels:
            want = _pose(m, d, height, roll,
                         None if linkage_on else locked)
            _, items = _items(m, d, linkage_on)
            for name, _, p, q in items:
                b = mujoco.mj_name2id(sm, mujoco.mjtObj.mjOBJ_BODY, name)
                sm.body_pos[b], sm.body_quat[b] = p, q
            mujoco.mj_forward(sm, sd)
            _drop(sm, sd)

            foot = np.degrees(want["hip"] + want["knee"] + want["ankle_pitch"])
            # The foot bar turns red the moment it stops being level, which is
            # the one thing to watch and is easy to miss on a thin line.
            g = mujoco.mj_name2id(sm, mujoco.mjtObj.mjOBJ_GEOM, "g_mk_foot")
            sm.geom_rgba[g] = ([0.20, 0.55, 0.85, 1.0] if abs(foot) < 0.05
                               else [0.92, 0.35, 0.25, 1.0])
            cam.lookat[:] = (0.045, 0.06, 0.135)
            r.update_scene(sd, camera=cam)
            tiles.append(_annotate(
                r.render(),
                "with the parallelogram" if linkage_on
                else "ankle servo deleted, nothing fitted",
                caption,
                [f"hip {np.degrees(want['hip']):+6.1f}    "
                 f"knee {np.degrees(want['knee']):+6.1f}    "
                 f"ankle {np.degrees(want['ankle_pitch']):+6.1f} deg",
                 f"foot vs torso  {foot:+.1f} deg"
                 + ("   the two bars stay parallel" if abs(foot) < 0.05
                    else "   the foot has followed the shin")],
                warn=not linkage_on))

        sheet = Image.new("RGB", (W, H))
        sheet.paste(tiles[0], (0, 0))
        sheet.paste(tiles[1], (PANEL_W, 0))
        dr = ImageDraw.Draw(sheet)
        dr.line([(PANEL_W, 0), (PANEL_W, H)], fill=(90, 96, 104), width=2)
        ff.stdin.write(sheet.tobytes())

    ff.stdin.close()
    ff.wait()
    print(f"wrote {out}  ({DURATION:.0f} s, {int(DURATION * FPS)} frames)")
    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
