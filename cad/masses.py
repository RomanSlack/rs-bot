"""Derive every body's mass from geometry and parts.  uv run python cad/masses.py

Replaces hand-assigned numbers. Each body is:

    printed structure (its own volume x PETG x infill)
  + the servos whose CASES bolt to it
  + bought parts it carries

Servo cases mount to the body PROXIMAL to the joint they drive, so the hip
servos belong to the torso, the knee servo to the thigh, the ankle-pitch servo
to the shin, the roll servo to the ankle, and the wheel servo to the roll
bracket. Getting that wrong is what left 100 g in the shin on the assumption
the wheel servo lived there.

Ported links (shin, thigh) use their real CAD mass; the rest fall back to
their visual volume until they are ported too.
"""

import mujoco
import numpy as np

from src.rsbot.model import load
import cad.shin as shin
import cad.thigh as thigh

PETG = 1.270          # g/cm3
INFILL = 0.60         # a print is not solid; see cad/shin.py
SERVO = 55.0          # STS3215
WHEEL = 60.0
PI5, BATT, DRIVER, IMU = 45.0, 180.0, 10.0, 5.0
BALLAST = 600.0       # arms (480) + head (120), stage 4

PRINTED = ("vthigh", "vshin", "vankstand", "vankpost", "vrollarm", "vrolltie",
           "vside", "vtop", "vshelf")
SERVOS = ("vhipsv", "vkneesv", "vanksv", "vrollsv", "vwhlsv")


def table():
    m, d = load()
    gname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
    bname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)

    rows = {}
    for i in range(m.ngeom):
        n = gname(i)
        if m.geom_group[i] != 0 or n == "floor" or n.startswith("h_"):
            continue
        b = bname(m.geom_bodyid[i])
        r = rows.setdefault(b, {"cm3": 0.0, "servos": 0, "bought": 0.0})
        sz = m.geom_size[i]
        if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX:
            vol = 8 * sz[0] * sz[1] * sz[2]
        elif m.geom_type[i] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            vol = np.pi * sz[0] ** 2 * 2 * sz[1]
        else:
            vol = 0.0
        if n.startswith(SERVOS):
            r["servos"] += 1
        elif n.startswith(("vtire", "vhub")):
            r["bought"] += WHEEL / 2
        elif n.startswith("vpi"):
            r["bought"] += PI5
        elif n.startswith("vbatt"):
            r["bought"] += BATT
        elif n.startswith("vdriver"):
            r["bought"] += DRIVER
        elif n.startswith(PRINTED):
            r["cm3"] += vol * 1e6

    cad_g = {"shin": shin.main(export=False) * 1000.0,
             "thigh": thigh.main(export=False) * 1000.0}
    out = {}
    for b, r in rows.items():
        link = b.rsplit("_", 1)[0]
        printed = cad_g.get(link, r["cm3"] * PETG * INFILL)
        total = printed + r["servos"] * SERVO + r["bought"]
        if b == "torso":
            total += IMU + BALLAST
        out[b] = (printed, r["servos"], r["bought"], total)
    return out


if __name__ == "__main__":
    t = table()
    print(f"{'body':<14}{'printed g':>10}{'servos':>8}{'bought g':>10}{'TOTAL g':>10}")
    for b, (p, s, bo, tot) in t.items():
        print(f"{b:<14}{p:>10.1f}{s:>8}{bo:>10.1f}{tot:>10.1f}")
    print(f"\ntotal {sum(v[3] for v in t.values()):.0f} g")
