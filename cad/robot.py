"""The whole robot, built from the real CAD parts.

    uv run python -m cad.robot [wheel|foot]

Every part is placed by the SIMULATOR's kinematics rather than by hand: pose
the sim, read each body's world transform, and hang that body's CAD mesh off
it. So if the CAD and the sim ever disagree about where something goes, this
picture is wrong in an obvious way.

Right-hand parts are the left-hand meshes mirrored in y (negative mesh scale).
"""

import os
import sys
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

import cad.ankle as ankle  # noqa: E402
import cad.linkage as linkage  # noqa: E402
import cad.chassis as chassis  # noqa: E402
import cad.servo as servo  # noqa: E402
import cad.shin as shin  # noqa: E402
import cad.thigh as thigh  # noqa: E402
import cad.wheel as wheel  # noqa: E402
from fitcheck import pose  # noqa: E402
from src.rsbot.model import load  # noqa: E402

OUT = Path(__file__).parent / "out"
W, H = 620, 780

C_PRINT = "0.88 0.45 0.13 1"
C_SERVO = "0.13 0.13 0.15 1"
C_WHEEL = "0.09 0.09 0.10 1"
C_HUB = "0.55 0.56 0.60 1"
# The parallelogram, in its own colour. It is the newest thing on the robot
# and the one a picture is most useful for, so it does not get to hide among
# the orange.
C_LINK = "0.20 0.55 0.85 1"

# Was 45.2, 24.7, 35.4, the superseded listing guesses. One copy now.
from cad.servo_dims import (LENGTH as SERVO_L, WIDTH as SERVO_W,
                            HEIGHT as SERVO_H)

# body name -> (stl stem, colour). Left-hand parts; right mirrors in y.
PARTS = {
    "torso": ("chassis", C_PRINT),
    "thigh": ("thigh", C_PRINT),
    "shin": ("shin", C_PRINT),
    "ankle": ("ankle_yoke", C_PRINT),
    "rollbracket": ("roll_bracket", C_PRINT),
}

# The wheel is drawn from its real solids too, not as the pair of cylinders the
# sim carries for it. Two parts in two materials, and it needs a rotation the
# others do not: the CAD spins about z with the sole at +z, the sim body spins
# about y with the sole INBOARD. So +90 deg about x on the left and -90 on the
# right - which also says something useful, that the two wheels are the same
# part flipped over, not a mirrored pair. One part number, print two.
WHEEL_PARTS = [("wheel_body", C_HUB), ("wheel_tyre", C_WHEEL)]


def export_all():
    OUT.mkdir(exist_ok=True)
    linkage.export_stls(OUT)
    chassis.main(export=True)
    thigh.main(export=True)
    shin.main(export=True)
    ankle.main(export=True)
    wheel.main(export=True)
    # The servo is a BOUGHT part, so it has no main() and no mass to report -
    # but the sim shows it, so its mesh has to be exported here too or
    # load(meshes=True) fails on a missing file.
    servo.export_mesh()


def _quat(mat):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, mat.flatten())
    return q


def build_scene(mode="wheel"):
    m, d = load()
    pose(m, d, mode)

    assets, bodies, seen = [], [], {}
    for body_stem, (stem, rgba) in PARTS.items():
        for side, sgn in (("l", 1), ("r", -1)):
            name = body_stem if body_stem == "torso" else f"{body_stem}_{side}"
            if body_stem == "torso" and side == "r":
                continue
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            p = d.xpos[bid]
            q = _quat(d.xmat[bid].reshape(3, 3))
            key = f"{stem}_{side}"
            if key not in seen:
                seen[key] = True
                assets.append(
                    f'<mesh name="{key}" file="{OUT / (stem + ".stl")}" '
                    f'scale="0.001 {0.001*sgn} 0.001"/>')
            bodies.append(
                f'<body pos="{p[0]} {p[1]} {p[2]}" '
                f'quat="{q[0]} {q[1]} {q[2]} {q[3]}">'
                f'<geom type="mesh" mesh="{key}" rgba="{rgba}"/></body>')

    # The wheels, from their own solids.
    for side, euler in (("l", "1.5708 0 0"), ("r", "-1.5708 0 0")):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"wheel_{side}")
        p = d.xpos[bid]
        q = _quat(d.xmat[bid].reshape(3, 3))
        geoms = ""
        for stem, rgba in WHEEL_PARTS:
            key = f"{stem}_{side}"
            if key not in seen:
                seen[key] = True
                assets.append(
                    f'<mesh name="{key}" file="{OUT / (stem + ".stl")}" '
                    f'scale="0.001 0.001 0.001"/>')
            # Named so check_wheel_orientation can find one to measure.
            tag = f' name="wheelchk_{key}"' if stem == "wheel_tyre" else ""
            geoms += (f'<geom type="mesh" mesh="{key}"{tag} euler="{euler}" '
                      f'rgba="{rgba}"/>')
        bodies.append(f'<body pos="{p[0]} {p[1]} {p[2]}" '
                      f'quat="{q[0]} {q[1]} {q[2]} {q[3]}">{geoms}</body>')

    # The parallelogram. Poses come from cad/linkage.py, which reads them off
    # the same posed model, so this picture cannot show a linkage in a place
    # the checks did not test. That is not a hypothetical worry here: three of
    # the faults in docs/how-checks-fail.md are a viewer drawing a different
    # robot from the one being measured.
    for side in ("l", "r"):
        for name, stem, sgn, pos_mm, R in linkage.placements(m, d, side):
            key = f"{stem}_{side}"
            if key not in seen:
                seen[key] = True
                assets.append(
                    f'<mesh name="{key}" file="{OUT / (stem + ".stl")}" '
                    f'scale="0.001 {0.001 * sgn} 0.001"/>')
            p = pos_mm / 1000.0
            q = _quat(R)
            bodies.append(
                f'<body pos="{p[0]} {p[1]} {p[2]}" '
                f'quat="{q[0]} {q[1]} {q[2]} {q[3]}">'
                f'<geom type="mesh" mesh="{key}" rgba="{C_LINK}"/></body>')

    # Servo blocks and electronics, straight from the sim's own visual geoms.
    # vtire/vhub are NOT in this list any more: the real wheel is drawn above,
    # and leaving the cylinders in would hide it inside a solid 80 mm disc.
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if m.geom_group[i] != 0 or n == "floor" or n.startswith("h_"):
            continue
        if not n.startswith(("vhipsv", "vkneesv", "vanksv",
                             "vrollsv", "vwhlsv", "vpi", "vbatt", "vdriver")):
            continue
        p, q = d.geom_xpos[i], _quat(d.geom_xmat[i].reshape(3, 3))
        s = m.geom_size[i]
        col = C_WHEEL if n.startswith(("vtire", "vhub")) else C_SERVO
        if n.startswith(("vpi", "vbatt", "vdriver")):
            col = "0.05 0.33 0.17 1"
        if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            g = f'<geom type="cylinder" size="{s[0]} {s[1]}" rgba="{col}"/>'
        else:
            g = f'<geom type="box" size="{s[0]} {s[1]} {s[2]}" rgba="{col}"/>'
        bodies.append(f'<body pos="{p[0]} {p[1]} {p[2]}" '
                      f'quat="{q[0]} {q[1]} {q[2]} {q[3]}">{g}</body>')

    return f'''<mujoco>
  <!-- angle="radian" to match rsbot.xml. MuJoCo defaults to DEGREES, and this
       scene did not say either way: the wheel's euler="1.5708 0 0" was read as
       1.57 degrees, so it never rotated. The wheel then kept its CAD frame and
       the picture showed it flat in wheel mode and upright in foot mode - the
       two modes exactly swapped, in the one artefact whose whole job is to
       show the modes. Physics was never affected; every other consumer of this
       rotation uses build123d, whose Rot() really is degrees. -->
  <compiler angle="radian"/>
  <visual><global offwidth="{W}" offheight="{H}"/>
    <headlight ambient="0.48 0.48 0.48" diffuse="0.6 0.6 0.6" specular="0.15 0.15 0.15"/>
    <quality shadowsize="4096"/><map znear="0.01" zfar="40"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.20 0.21 0.23" rgb2="0.26 0.27 0.29"/>
    <material name="grid" texture="grid" texrepeat="40 40"/>
    {chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="0.4 -0.4 0.9" dir="-0.4 0.4 -1" diffuse="0.7 0.7 0.7"/>
    <geom type="plane" size="0 0 0.05" material="grid"/>
    {chr(10).join(bodies)}
  </worldbody>
</mujoco>'''


def check_wheel_orientation(xml, mode):
    """The wheel must be UPRIGHT in wheel mode and FLAT in foot mode.

    This exists because the picture got it backwards and nothing noticed. A
    render is the one check a human reads directly, so when it lies it is
    believed. Measured off the built scene's own mesh vertices, not off the
    numbers that went in, so a units bug in the XML cannot pass it.
    """
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    gid = next(i for i in range(m.ngeom)
               if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
               .startswith("wheelchk"))
    mid = m.geom_dataid[gid]
    lo, cnt = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    v = m.mesh_vert[lo:lo + cnt].astype(float)
    R = d.geom_xmat[gid].reshape(3, 3)
    w = v @ R.T                                   # into world axes
    span = w.max(axis=0) - w.min(axis=0)

    # 80 mm across the disc, 24 mm through it. Which axis is which is the
    # entire question.
    thin = int(np.argmin(span))
    want = 1 if mode == "wheel" else 2            # y when rolling, z when flat
    ok = thin == want and span[thin] < 0.030
    print(f"   wheel is {'UPRIGHT' if thin == 1 else 'FLAT' if thin == 2 else '?'}"
          f" in {mode} mode  (span {1000*span[0]:.0f} x {1000*span[1]:.0f} x "
          f"{1000*span[2]:.0f} mm)  {'ok' if ok else 'WRONG WAY ROUND'}")
    return ok


def main(mode="wheel"):
    export_all()
    xml = build_scene(mode)
    check_wheel_orientation(xml, mode)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()

    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    # 1.05, not 0.80: at 0.80 the top of the chassis is outside the frame, so
    # the one artefact whose job is to show the whole robot showed most of it.
    # The fourth view is close on the left leg, because the parallelogram is
    # five thin parts and at whole-robot scale they read as scratches.
    views = [("side", 90, -6, 1.05), ("three-quarter", 138, -16, 1.05),
             ("front", 180, -6, 1.05), ("linkage", 108, -8, 0.52)]
    tiles = []
    for name, az, el, dist in views:
        cam.lookat[:] = ((0.0, 0.06, 0.16) if name == "linkage"
                         else (0.0, 0.0, 0.215))
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        r.update_scene(d, camera=cam)
        im = Image.fromarray(r.render())
        dr = ImageDraw.Draw(im, "RGBA")
        dr.rectangle([0, 0, W, 36], fill=(0, 0, 0, 155))
        dr.text((14, 7), f"{name}  -  {mode} mode", font=font,
                fill=(255, 255, 255, 255))
        tiles.append(im)

    sheet = Image.new("RGB", (W * len(tiles), H))
    for i, t in enumerate(tiles):
        sheet.paste(t, (i * W, 0))
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = OUT / f"robot_{mode}_{stamp}.png"
    sheet.save(out)
    print(f"wrote {out}")
    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "wheel")
