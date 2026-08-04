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
import cad.ankle as ankle
import cad.chassis as chassis

PETG = 1.270          # g/cm3
INFILL = 0.60         # a print is not solid; see cad/shin.py
SERVO = 55.0          # STS3215
# 54.0, not 60: the wheel is now a designed part (cad/wheel.py) rather than a
# bought one, because nothing sold has a usable side face. Body + TPU + horn.
# It has moved twice: down to 52.1 when the tread was cut in, back up when the
# tyre got a real press fit and a retaining bead instead of being a slip fit
# that would have spun on the rim. `uv run python -m cad.wheel` prints the
# current figure and this has to be moved to match it by hand; cad/inertia.py's
# check_masses is what catches it if you forget.
WHEEL = 54.79
# PA6-CF, for the parallelogram's links. The printed structure is quoted at
# PETG x infill above because that is what those parts were costed in; the
# linkage is small, solid and highly loaded per gram, so it is quoted at the
# material cad/stress.py actually solves it in.
PA6CF = 1.19          # g/cm3, solid
PI5, BATT, DRIVER, IMU = 45.0, 180.0, 10.0, 5.0
BALLAST = 600.0       # arms (480) + head (120), stage 4

PRINTED = ("vthigh", "vshin", "vankstand", "vankpost", "vankface",
           "vrollarm", "vrolltie", "vside", "vtop", "vshelf", "vpistand")
# Four per leg, not five. "vanksv" is gone: see cad/linkage.py.
SERVOS = ("vhipsv", "vkneesv", "vrollsv", "vwhlsv")


# The parallelogram, per leg, from cad/linkage.py's own solids at PA6CF. Keyed
# by the body that carries each part's mass, the same split cad/inertia.py
# uses, because two tables of "which body owns which link" would drift.
def _linkage_g():
    import cad.inertia as inertia
    import cad.linkage as linkage
    out = {}
    for stem, host in inertia.LINKAGE_HOST.items():
        g = linkage.SOLIDS[stem.replace("lk_boss", "lk_fin")]().volume
        # The torso is ONE body carrying BOTH legs' fins; a leg body is one
        # leg's. Same asymmetry cad/inertia.py has to handle.
        n = 2 if host == "torso" else 1
        out[host] = out.get(host, 0.0) + n * g / 1000.0 * PA6CF
    return out


# The three linkage parts that are sim BODIES rather than features of one.
# They have no group-0 geoms in the plain build, so they never appear in the
# geom sweep below and have to be added by name - which is also the only reason
# cad/twin.py can weigh them.
def _linkage_bodies_g():
    import cad.linkage as linkage
    import cad.linkage_dims as ld
    return {name: linkage.SOLIDS[stem]().volume / 1000.0 * PA6CF
            for name, stem in ld.SIM_MESH.items()}


def table():
    LINKAGE_G = _linkage_g()
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
        # Order matters, and so does exactness: "vpistand" starts with "vpi",
        # so a prefix test counted the standoff as a second Raspberry Pi.
        if n.startswith(PRINTED):
            r["cm3"] += vol * 1e6
        elif n.startswith(SERVOS):
            r["servos"] += 1
        elif n.startswith(("vtire", "vhub")):
            r["bought"] += WHEEL / 2
        elif n == "vpi":
            r["bought"] += PI5
        elif n.startswith("vbatt"):
            r["bought"] += BATT
        elif n.startswith("vdriver"):
            r["bought"] += DRIVER

    yoke_kg, roll_kg = ankle.main(export=False)
    cad_g = {"shin": shin.main(export=False) * 1000.0,
             "thigh": thigh.main(export=False) * 1000.0,
             "ankle": yoke_kg * 1000.0,
             "rollbracket": roll_kg * 1000.0,
             "torso": chassis.main(export=False) * 1000.0}
    out = {}
    for b, r in rows.items():
        link = b.rsplit("_", 1)[0]
        printed = cad_g.get(link, r["cm3"] * PETG * INFILL)
        total = printed + r["servos"] * SERVO + r["bought"]
        if b == "torso":
            total += IMU + BALLAST
        total += LINKAGE_G.get(link, 0.0)
        out[b] = (printed, r["servos"], r["bought"], total)
    for name, g in _linkage_bodies_g().items():
        for side in ("l", "r"):
            out[f"{name}_{side}"] = (g, 0, 0.0, g)
    return out


if __name__ == "__main__":
    t = table()
    print(f"{'body':<14}{'printed g':>10}{'servos':>8}{'bought g':>10}{'TOTAL g':>10}")
    for b, (p, s, bo, tot) in t.items():
        print(f"{b:<14}{p:>10.1f}{s:>8}{bo:>10.1f}{tot:>10.1f}")
    print(f"\ntotal {sum(v[3] for v in t.values()):.0f} g")
