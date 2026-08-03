"""Peak load carried across each joint, measured from the sim.

    uv run python -m cad.loads

Every part is a beam between two joints. What it has to survive is the wrench
its CHILD passes up through it, and that is not something to estimate: MuJoCo
computes it exactly. `d.cfrc_int[i]` is the 6D interaction force between body i
and its parent, including inertial and contact terms, so it is the whole load
path with nothing left out.

The catch is the reference frame. cfrc_int is "com-based": axes are world
aligned but the torque is taken about `subtree_com[rootid]`, not about the
joint. Reading it straight gives a moment tens of times too large, because it
folds in F x r for an r of a couple of hundred millimetres. It has to be
translated to the joint and rotated into the part's own frame first.

Three load cases are reported separately rather than blended, because they mean
different things and take different factors:

    quiet    standing still. What the robot does most of the time.
    flip     a full wheel -> foot -> wheel cycle. The worst NORMAL operation,
             and so the one that gets the full SF = 3.
    shove    the rated 1.0 N.s disturbance, FORE/AFT. A LIMIT event, already at
             the edge of what the controller is meant to survive, so it takes
             1.5 the way an ultimate-load check does. Factoring a limit case by
             3 as well counts the same conservatism twice.
    tipover  the same impulse applied SIDEWAYS. This is not a disturbance the
             robot rejects - it falls over. Factor 1.0, because it is the
             measured event and not a proxy for one.

Design load is the worst of flip x 3, shove x 1.5 and tipover x 1, and the
table says which governs.

The fore/aft and lateral cases are separated because they are not the same kind
of event at all, and averaging them hides the most important number here:

    fore/aft 1.0 N.s   ->  6.5 N at the hip. The wheels roll away and almost
                           nothing reaches the structure.
    lateral  1.0 N.s   ->  472 N at the hip, peaking 574 ms LATER. That is not
                           the impulse, it is the robot hitting the floor: it
                           rolls through -100 degrees and lands on its side.

There is no hip roll joint and no way to step sideways, so laterally the robot
has no compliance whatever. Same impulse, seventy times the load.

The shove is delivered the way `src/rsbot/sim.py` delivers it: 1.0 N.s as
100 N for 10 ms. That is an impulse approximation, not a measurement of any
real knock, and the peak interface force is sensitive to the duration - the
same impulse spread over 50 ms would load the structure far less. Treat the
shove row as an upper bound on a hand-swipe, not as a prediction.

What this does NOT include: the reaction torque of a servo whose case bolts to
the part. That is an internal couple between two bolt patterns on the same
body, so it never crosses a joint and cfrc_int cannot see it. Bolt spacing is
checked separately in each part's script.
"""

import numpy as np

import mujoco

from src.rsbot.balance import Balancer
from src.rsbot.model import load
from src.rsbot.sim import obs
from src.rsbot.transition import DeployMachine

# Sized for a desk robot that will get knocked off a desk.
SF = 3.0             # on normal operation
SF_LIMIT = 1.5       # on the rated shove, which is already a limit event
SF_FALL = 1.0        # on the tipover, which is a measured event, not a proxy
SHOVE = 1.0          # N.s, the rated disturbance in wheel mode
SHOVE_MS = 0.010     # applied over 10 ms, matching src/rsbot/sim.py

# part -> the body hanging below it. A part carries the wrench at the joint
# BELOW it, so each part is keyed by its child.
CHILD = {"torso": "thigh", "thigh": "shin", "shin": "ankle",
         "ankle": "rollbracket", "rollbracket": "wheel"}


def interface(m, d, body):
    """(force, torque) the parent applies to `body`, about that body's joint,
    in that body's own frame. Newtons and newton-metres."""
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    ref = d.subtree_com[m.body_rootid[i]]
    tau_c, f = d.cfrc_int[i][:3].copy(), d.cfrc_int[i][3:].copy()
    p = d.xanchor[m.body_jntadr[i]] if m.body_jntnum[i] else d.xpos[i]
    tau = tau_c + np.cross(ref - p, f)
    R = d.xmat[i].reshape(3, 3)
    return R.T @ f, R.T @ tau


class Peak:
    """Worst wrench seen at each joint, ranked by a combined force-and-moment
    severity so one metric picks a single self-consistent instant rather than
    mixing the worst force from one moment with the worst torque from another.
    The 40 is 1/25 mm, roughly the lever arm these parts have."""

    def __init__(self):
        self.best = {c: (np.zeros(3), np.zeros(3), -1.0) for c in CHILD.values()}
        # And the pitch torque on its own, which the severity ranking above
        # cannot give you. A part is sized by the worst wrench; an ACTUATOR, or
        # the linkage standing in for one, is sized by the worst torque about
        # its own axis, and the instant those two peak at is not the same
        # instant. cad/belt.py has carried a literal 1.68 N.m for the ankle
        # since before this class existed.
        self.ty = {c: 0.0 for c in CHILD.values()}

    def record(self, m, d):
        mujoco.mj_rnePostConstraint(m, d)
        for child in CHILD.values():
            for s in ("l", "r"):
                f, t = interface(m, d, f"{child}_{s}")
                sev = np.linalg.norm(f) + 40.0 * np.linalg.norm(t)
                if sev > self.best[child][2]:
                    self.best[child] = (f, t, sev)
                self.ty[child] = max(self.ty[child], abs(t[1]))

    def wrenches(self):
        return {k: (f, t) for k, (f, t, _) in self.best.items()}

    def pitch_torques(self):
        return dict(self.ty)


def _run(case, seconds):
    m, d = load()
    dt = m.opt.timestep
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    peak = Peak()

    if case == "flip":
        ctl = DeployMachine()
    else:
        ctl = Balancer()

    for k in range(int(seconds / dt)):
        t = k * dt
        d.xfrc_applied[torso] = 0.0
        if case == "flip":
            if abs(t - 2.0) < dt:
                ctl.start_deploy()
            if abs(t - 4.5) < dt:
                ctl.start_retract()
            d.ctrl[:] = ctl(obs(m, d), dt)
        else:
            axis = {"shove": 0, "tipover": 1}.get(case)
            if axis is not None and 2.0 <= t < 2.0 + SHOVE_MS:
                d.xfrc_applied[torso, axis] = SHOVE / SHOVE_MS
            d.ctrl[:] = ctl(obs(m, d), dt)
        mujoco.mj_step(m, d)
        # Skip the first moments: the keyframe is not exactly an equilibrium
        # and the settling transient is not a load case.
        if t > 0.5:
            peak.record(m, d)
    return peak


# The four cases and how long each has to run, in one place, because two
# surveys reading the same runs must not disagree about what a case IS.
CASES = (("quiet", 2.5), ("flip", 7.0), ("shove", 5.0), ("tipover", 5.0))

# The factor each case is carried at. Was written out longhand inside
# design_loads(); the pitch survey needs the same mapping and a second copy is
# a second place to be wrong.
FACTOR = {"quiet": SF, "flip": SF, "shove": SF_LIMIT, "tipover": SF_FALL}


def survey():
    """{case: {child: (force, torque)}}, unfactored."""
    return {c: _run(c, s).wrenches() for c, s in CASES}


def pitch_survey():
    """{case: {child: peak |torque about that joint's own axis|}}, unfactored.

    Separate from survey() because they answer different questions and peak at
    different instants. survey() ranks a whole wrench, which is what sizes a
    PART. This is the torque the joint itself has to hold, which is what sizes
    an actuator, a belt, or the linkage that replaces one.
    """
    return {c: _run(c, s).pitch_torques() for c, s in CASES}


def design_pitch_torque(child="ankle"):
    """(N.m, governing case) at one joint, factored. The number to size a
    drive from."""
    got = {c: v[child] * FACTOR[c] for c, v in pitch_survey().items()}
    case = max(got, key=got.get)
    return got[case], case


def design_loads(cases=None):
    """{part: (force, torque, governing_case)}, factored, in the child's frame
    (which is the part's own frame rotated by that joint's angle; at the poses
    these peaks occur in the two agree within a few degrees)."""
    cases = cases or survey()

    def worst(child):
        options = [(f"{c} x{FACTOR[c]:g}",
                    *[v * FACTOR[c] for v in cases[c][child]])
                   for c in ("flip", "shove", "tipover")]
        return max(options,
                   key=lambda o: np.linalg.norm(o[1]) + 40 * np.linalg.norm(o[2]))

    out = {}
    for part, child in CHILD.items():
        name, f, t = worst(child)
        out[part] = (f, t, name)

    # The wheel is the leaf, so it is never anyone's parent and never got a key
    # of its own - yet it is the part the whole robot stands on in foot mode.
    # Its load path is the axle reaction, which the recorder already has under
    # "wheel" because the wheel is the rollbracket's child. Same wrench, read
    # from the other end.
    name, f, t = worst("wheel")
    out["wheel"] = (f, t, name)
    return out


def static_check():
    """Control test. Balancing quietly, the wrench across the knee must equal
    the weight of everything BELOW the knee minus the ground reaction under
    that wheel. If this does not close, every load below is wrong the same way
    and the FEA inherits the error."""
    m, d = load()
    bal = Balancer()
    dt = m.opt.timestep
    for _ in range(int(2.0 / dt)):
        d.ctrl[:] = bal(obs(m, d), dt)
        mujoco.mj_step(m, d)
    mujoco.mj_rnePostConstraint(m, d)
    f, _ = interface(m, d, "shin_l")

    below = sum(m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)]
                for b in ("shin_l", "ankle_l", "rollbracket_l", "wheel_l"))
    wheel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")
    up, buf = 0.0, np.zeros(6)
    for c in range(d.ncon):
        con = d.contact[c]
        if wheel in (m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]):
            mujoco.mj_contactForce(m, d, c, buf)
            up += buf[0]
    return np.linalg.norm(f), abs(below * 9.81 - up), up


def _row(name, f, t):
    return (f"{name:<8}{f[0]:>8.1f}{f[1]:>8.1f}{f[2]:>8.1f}"
            f"{np.linalg.norm(f):>9.1f}{t[0]:>8.2f}{t[1]:>8.2f}{t[2]:>8.2f}"
            f"{np.linalg.norm(t):>9.2f}")


if __name__ == "__main__":
    got, expect, up = static_check()
    print(f"control test   knee wrench {got:.2f} N, statics says {expect:.2f} N "
          f"({up:.2f} N of ground under one wheel)")
    assert abs(got - expect) < 0.3, "interface force does not close"
    print("               closes to 0.3 N\n")

    cases = survey()
    hdr = (f"{'case':<8}{'Fx':>8}{'Fy':>8}{'Fz':>8}{'|F| N':>9}"
           f"{'Mx':>8}{'My':>8}{'Mz':>8}{'|M| N.m':>9}")
    for part, child in CHILD.items():
        print(f"{part}  (wrench at the {child} joint, part frame)")
        print(hdr)
        for c in ("quiet", "flip", "shove", "tipover"):
            print(_row(c, *cases[c][child]))
        f, t, gov = design_loads(cases)[part]
        print(_row(gov, f, t) + "   <- design")
        print()
