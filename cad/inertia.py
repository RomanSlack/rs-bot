"""Full inertial properties per body, from the CAD.

    uv run python -m cad.inertia            # the table, and the control tests
    uv run python -m cad.inertia --emit     # the block to paste into model.py

`cad/masses.py` already derives every body's MASS from geometry. This does the
rest of it: where that mass sits, and how it is spread out.

That matters more than it sounds. Until now each link's inertia came from the
simple capsule or box carrying its mass in the sim - a 28 mm capsule standing in
for an L-shaped part with a servo bolted to one side. The mass was right and the
distribution was a guess, and a balancer is sensitive to the distribution.

Each body is composed from three kinds of thing:

    printed structure   the real B-rep solid, exact inertia from OCC
    servo cases         uniform boxes at the positions the sim already uses
    bought parts        the same, for the wheels, Pi, pack and driver

and the three are combined properly - parallel axis on each, about the
composite centre of mass - rather than added as scalars.

Right-hand bodies are the left-hand ones mirrored in y, which flips the sign of
the centre of mass in y and of the Ixy and Iyz products. Getting that wrong is
invisible in a symmetric pose and shows up as a slow drift in a turn.
"""

import sys

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

import mujoco

import cad.ankle as ankle
import cad.chassis as chassis
import cad.masses as masses
import cad.shin as shin
import cad.thigh as thigh
from src.rsbot.model import load

# link -> the build123d solid that is its printed structure
SOLIDS = {"thigh": thigh.build, "shin": shin.build,
          "ankle": ankle.yoke, "rollbracket": ankle.roll_bracket,
          "torso": chassis.build}

PETG_G_MM3 = masses.PETG / 1000.0        # g/cm3 -> g/mm3
INFILL = masses.INFILL


def solid_inertia(part, mass_g):
    """(com mm, inertia about the com, g.mm2) for a B-rep solid of `mass_g`,
    assumed uniform. Exact on the geometry, not on a triangulation.

    OCC's MatrixOfInertia is already taken about the CENTRE OF MASS, despite
    the GProp being constructed at the origin. Subtracting the parallel-axis
    term here as well - which is the natural thing to write - drives the
    tensors indefinite, and nothing downstream notices: MuJoCo accepts them
    and the robot just behaves oddly. `check_positive_definite` is what caught
    it.
    """
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, g)
    vol = g.Mass()                                   # mm3, density 1
    c = g.CentreOfMass()
    com = np.array([c.X(), c.Y(), c.Z()])
    M = g.MatrixOfInertia()
    I = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return com, I * (mass_g / vol)                   # scale to the real mass


def _shift(m, r):
    """The parallel-axis term m(|r|^2 I - r r^T)."""
    r = np.asarray(r, float)
    return m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def _box_inertia(mass_g, half, R):
    """Uniform box, half-extents `half`, rotated by R. About its own centre."""
    a, b, c = np.asarray(half, float)
    I = np.diag([mass_g * (b * b + c * c) / 3.0,
                 mass_g * (a * a + c * c) / 3.0,
                 mass_g * (a * a + b * b) / 3.0])
    return R @ I @ R.T


def _cyl_inertia(mass_g, r, half_h, R):
    """Uniform cylinder about its own centre; MuJoCo's cylinder axis is z."""
    I = np.diag([mass_g * (3 * r * r + 4 * half_h * half_h) / 12.0,
                 mass_g * (3 * r * r + 4 * half_h * half_h) / 12.0,
                 mass_g * r * r / 2.0])
    return R @ I @ R.T


def _pieces(m, body):
    """Every bought/servo geom on `body`, as (mass_g, com_mm, I_about_com).

    Read off the sim's own visual geoms so the CAD and the sim cannot disagree
    about where a servo is. Positions are body-LOCAL (m.geom_pos), not world.
    """
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    out = []
    for i in range(m.ngeom):
        if m.geom_bodyid[i] != bid or m.geom_group[i] != 0:
            continue
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if n.startswith(masses.SERVOS):
            g = masses.SERVO
        elif n.startswith(("vtire", "vhub")):
            g = masses.WHEEL / 2
        elif n == "vpi":
            g = masses.PI5
        elif n.startswith("vbatt"):
            g = masses.BATT
        elif n.startswith("vdriver"):
            g = masses.DRIVER
        else:
            continue                                  # printed, or a shell
        pos = m.geom_pos[i] * 1000.0
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.geom_quat[i])
        R = R.reshape(3, 3)
        size = m.geom_size[i] * 1000.0
        if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            I = _cyl_inertia(g, size[0], size[1], R)
        else:
            I = _box_inertia(g, size, R)
        out.append((g, pos, I))
    return out


def compose(pieces):
    """Combine (mass, com, I_about_com) into one. Parallel axis on each."""
    M = sum(p[0] for p in pieces)
    com = sum(p[0] * np.asarray(p[1]) for p in pieces) / M
    I = sum(p[2] + _shift(p[0], np.asarray(p[1]) - com) for p in pieces)
    return M, com, I


def body_inertials():
    """{link: (mass_kg, com_m, inertia_kg_m2)} for the LEFT side and torso.

    The extras the torso carries that are not geometry - the IMU, and the 600 g
    of arms and head that stage 4 will add - stay exactly where the old lumped
    model put them. Moving them up to where arms would really go would raise the
    centre of mass and change how the robot balances, and that is a design
    change, not a CAD port.
    """
    m, _ = load()
    out = {}
    for link, build in SOLIDS.items():
        body = link if link == "torso" else f"{link}_l"
        part = build()
        vol = part.volume
        printed_g = vol * PETG_G_MM3 * INFILL
        com, I = solid_inertia(part, printed_g)
        pieces = [(printed_g, com, I)] + _pieces(m, body)
        if link == "torso":
            # The IMU, and 600 g standing in for the arms and head that arrive
            # at stage 4. Kept as the same uniform box, in the same place, that
            # the old lumped model used: it represents parts that do not exist,
            # so there is nothing to derive and any move would be a design
            # change smuggled in as a CAD port. As a POINT mass at the origin
            # instead it drops the standing centre of mass 57 mm, which is not
            # a port, it is a different robot.
            lump = masses.IMU + masses.BALLAST
            half = np.array([45.0, 35.0, 90.0])
            pieces.append((lump, np.array([8.5, 0.0, 90.0]),
                           _box_inertia(lump, half, np.eye(3))))
        M, com, I = compose(pieces)
        out[link] = (M / 1000.0, com / 1000.0, I / 1000.0 / 1e6)

    # The wheel is bought: tyre and hub, no printed structure.
    M, com, I = compose(_pieces(m, "wheel_l"))
    out["wheel"] = (M / 1000.0, com / 1000.0, I / 1000.0 / 1e6)
    return out


def mirror(com, I):
    """The right-hand part: mirrored in y. Ixy and Iyz change sign."""
    com = np.array([com[0], -com[1], com[2]])
    I = I.copy()
    I[0, 1] = I[1, 0] = -I[0, 1]
    I[1, 2] = I[2, 1] = -I[1, 2]
    return com, I


# --- control tests -----------------------------------------------------------

def check_masses():
    """Every body's composite mass must equal what cad/masses.py says, or the
    two derivations disagree and one of them is wrong."""
    table = masses.table()
    got = body_inertials()
    bad = []
    for link, (kg, _, _) in got.items():
        body = link if link == "torso" else f"{link}_l"
        want = table[body][3] / 1000.0
        if abs(kg - want) > 1e-4:
            bad.append((link, kg, want))
    return bad


def check_positive_definite():
    """An inertia tensor must be positive definite and satisfy the triangle
    inequality on its principal moments. A sign error in a product of inertia
    is otherwise completely silent."""
    bad = []
    for link, (_, _, I) in body_inertials().items():
        w = np.linalg.eigvalsh(I)
        if w.min() <= 0:
            bad.append((link, "not positive definite", w))
        elif w[0] + w[1] < w[2] * (1 - 1e-9):
            bad.append((link, "violates the triangle inequality", w))
    return bad


def compare_to_primitives():
    """How far the old capsule-and-box stand-ins were. This is the number that
    says whether the balancer needs re-tuning."""
    m, _ = load()
    got = body_inertials()
    rows = []
    for link, (kg, com, I) in got.items():
        body = link if link == "torso" else f"{link}_l"
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        old = m.body_inertia[i]
        new = np.linalg.eigvalsh(I)
        rows.append((link, old, new, np.linalg.norm(m.body_ipos[i] - com)))
    return rows


def _fmt(link, kg, com, I):
    return (f'    "{link}": ({kg:.6f}, ({com[0]:+.6f}, {com[1]:+.6f}, '
            f'{com[2]:+.6f}),\n              ('
            f'{I[0,0]:.6e}, {I[1,1]:.6e}, {I[2,2]:.6e}, '
            f'{I[0,1]:.6e}, {I[0,2]:.6e}, {I[1,2]:.6e})),')


if __name__ == "__main__":
    got = body_inertials()

    if "--emit" in sys.argv:
        print("# link -> (mass kg, com m, (ixx iyy izz ixy ixz iyz) kg.m2),")
        print("# LEFT side and torso; the right side mirrors in y.")
        print("# Regenerate with: uv run python -m cad.inertia --emit")
        print("SEG_INERTIA = {")
        for link, (kg, com, I) in got.items():
            print(_fmt(link, kg, com, I))
        print("}")
        raise SystemExit

    bad = check_masses()
    print("control: composite mass vs cad.masses ...",
          "closes" if not bad else f"DISAGREES {bad}")
    assert not bad
    bad = check_positive_definite()
    print("control: inertia tensors physical .......",
          "ok" if not bad else f"BAD {bad}")
    assert not bad

    print(f"\n{'link':<14}{'g':>8}{'com mm (x,y,z)':>28}"
          f"{'principal g.cm2':>30}")
    for link, (kg, com, I) in got.items():
        w = np.linalg.eigvalsh(I) * 1e7
        print(f"{link:<14}{kg*1000:>8.1f}"
              f"{com[0]*1000:>9.1f}{com[1]*1000:>9.1f}{com[2]*1000:>9.1f}"
              f"{w[0]:>10.1f}{w[1]:>10.1f}{w[2]:>10.1f}")

    print(f"\nwhat the primitive stand-ins were getting wrong:")
    print(f"{'link':<14}{'old diag g.cm2':>30}{'new principal g.cm2':>32}"
          f"{'com moved mm':>14}")
    for link, old, new, dcom in compare_to_primitives():
        print(f"{link:<14}"
              f"{old[0]*1e7:>10.1f}{old[1]*1e7:>10.1f}{old[2]*1e7:>10.1f}"
              f"{new[0]*1e7:>11.1f}{new[1]*1e7:>10.1f}{new[2]*1e7:>10.1f}"
              f"{dcom*1000:>14.1f}")
