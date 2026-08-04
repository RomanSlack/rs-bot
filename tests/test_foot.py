"""Wheel-flip foot mode: the wheel face IS the sole."""

import math

import mujoco
import numpy as np
import pytest

from src.rsbot.model import (ROLL_FOOT, ROLL_WHEEL, WHEEL_HALF_W, WHEEL_R,
                            ankle_pitch_level, axle_height, leg_ik, load)
from src.rsbot.sim import rollout_cycle, rollout_deploy
from src.rsbot.transition import DeployCfg


def _axle_z(m, d):
    return float(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                          "wheel_l")][2])


def test_flip_dig_is_tiny():
    """The rim corner leads briefly. This is the whole cost of the mechanism."""
    peak = max(axle_height(r) for r in np.linspace(0, ROLL_FOOT, 2000))
    assert peak - WHEEL_R == pytest.approx(0.00176, abs=1e-4)
    assert axle_height(ROLL_FOOT) == pytest.approx(WHEEL_HALF_W, abs=1e-9)


def test_axle_descends_monotonically_after_the_bump():
    rolls = np.linspace(0.30, ROLL_FOOT, 500)
    h = [axle_height(r) for r in rolls]
    assert all(b <= a + 1e-12 for a, b in zip(h, h[1:]))


def test_nothing_hangs_below_the_wheel_face():
    """In foot mode the face is only 12 mm under the axle. Anything lower
    becomes the real contact and steals the 80 mm foot."""
    m, d = load()
    hip, knee = leg_ik(0.195)
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = 0.195 + WHEEL_HALF_W, 1.0
    for base in (7, 12):
        d.qpos[base], d.qpos[base + 1] = hip, knee
        d.qpos[base + 2] = ankle_pitch_level(hip, knee)
        d.qpos[base + 3] = ROLL_FOOT
    mujoco.mj_forward(m, d)
    mujoco.mj_collision(m, d)
    touching = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
                for c in d.contact[:d.ncon]}
    assert touching <= {"wheel_l", "wheel_r"}, f"unexpected ground contact: {touching}"


def test_shin_does_not_collide_with_its_own_wheel():
    """The ankle body makes these grandparent/grandchild, which MuJoCo does
    not auto-exclude."""
    m, d = load()
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(m.ngeom)]
    pairs = {tuple(sorted((names[c.geom1], names[c.geom2]))) for c in d.contact[:d.ncon]}
    assert ("shin_l", "wheel_l") not in pairs
    assert ("shin_r", "wheel_r") not in pairs


def test_ankle_pitch_levels_the_foot_at_any_leg_length():
    for h in (0.140, 0.170, 0.195, 0.210):
        hip, knee = leg_ik(h)
        assert hip + knee + ankle_pitch_level(hip, knee) == pytest.approx(0, abs=1e-12)


def test_transition_reaches_stand_and_holds():
    r = rollout_deploy(cfg=DeployCfg(flip_rate=2.0), duration=40.0)
    assert not r["fell"]
    assert r["state"] == "STAND"
    assert r["stand_duration"] > 30.0
    # 120 mm, re-MEASURED rather than relaxed. Over the same 40 s the split is:
    #
    #     43 mm   the balancer's own odometry drift, which it has whether it
    #             flips or not - the same 43 mm shows up in the 60 s standing
    #             test, where it passes its own 50 mm criterion
    #     59 mm   the manoeuvre itself
    #
    # It was ~80 mm at 1.756 kg. The robot is now 1.863 kg with a belt drive
    # 84 mm off the leg centreline, so the manoeuvre costs more. Re-tuning to
    # claw it back traded the standing and steering tests instead.
    assert abs(r["drift"]) < 0.12
    # Standing on the faces, not perched on the rims.
    assert abs(r["wheel_clear"]) < 0.002


@pytest.mark.parametrize("rate", [0.5, 2.0, 8.0])
def test_transition_is_robust_across_flip_rates(rate):
    """A 16x range of flip rates all work, which is the point: this mechanism
    is not living on a knife edge the way the earlier two were."""
    r = rollout_deploy(cfg=DeployCfg(flip_rate=rate), duration=25.0)
    assert not r["fell"]
    assert r["state"] == "STAND"


def test_round_trip_and_keeps_driving():
    """Stage 0b is only done if it can get back OUT of foot mode and stay
    useful, not just get into it."""
    r = rollout_cycle(cfg=DeployCfg(flip_rate=2.0), stand_for=4.0, duration=30.0)
    assert not r["fell"]
    assert r["reached_stand"] and r["back_on_wheels"]
    assert r["state"] == "WHEEL"
    # It stood still while standing, then carried on well past where it flipped.
    assert abs(r["slip_while_standing"]) < 0.02
    assert r["x_end"] > r["x_at_flip"] + 1.0


def test_twenty_consecutive_transitions():
    """The stage-2 hardware criterion, run in sim."""
    import mujoco
    from src.rsbot.sim import CTRL_HZ, obs
    from src.rsbot.transition import DeployMachine, STAND, WHEEL
    from src.rsbot.balance import pitch_from_quat

    m, d = load()
    mach = DeployMachine(cfg=DeployCfg(flip_rate=2.0))
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep

    cycles, phase, mark = 0, "drive", 2.0
    for k in range(int(400 / m.opt.timestep)):
        t = k * m.opt.timestep
        if phase == "drive" and t >= mark and mach.state == WHEEL:
            mach.start_deploy(); phase = "toFoot"
        elif phase == "toFoot" and mach.state == STAND:
            phase, mark = "standing", t + 1.5
        elif phase == "standing" and t >= mark:
            mach.start_retract(); phase = "toWheel"
        elif phase == "toWheel" and mach.state == WHEEL:
            cycles += 1; phase, mark = "drive", t + 1.5
            if cycles >= 20:
                break
        if k % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt,
                             v_des=0.30 if phase == "drive" else 0.0)
        mujoco.mj_step(m, d)
        assert abs(pitch_from_quat(obs(m, d)["quat"])) < 1.0, f"fell, cycle {cycles+1}"
        assert d.xpos[torso][2] > 0.12
    assert cycles == 20


# --- backlash ---------------------------------------------------------------

def test_backlash_zero_is_the_rigid_model():
    """backlash=0 must add no bodies, so every index-based test still holds.

    23, not 17: the parallelogram's rods and idler are real bodies in the sim
    now (cad/linkage_dims.py's SIM_BODIES), which is 3 hinges a leg. That is
    six more qpos than this test was written against, and NONE of them are lash
    joints - which is the thing it is actually guarding."""
    a, _ = load()
    b, _ = load(backlash=0.0)
    assert a.nq == b.nq == 23 and a.nbody == b.nbody


def test_backlash_adds_one_lash_joint_per_actuator():
    import mujoco
    m, _ = load(backlash=0.02)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
             for i in range(m.njnt)]
    assert sum(1 for n in names if n and n.endswith("_lash")) == m.nu


def test_lashed_leg_does_not_collide_with_itself():
    """Drive bodies break MuJoCo's parent-child exclusions; without the full
    exclude list the leg self-collides at rest."""
    m, d = load(backlash=0.05)
    assert d.ncon == 0


def test_stance_is_addressed_by_name_not_index():
    """Lash joints interleave into the ordering, so an index-written keyframe
    silently scrambles. Both models must reach the same physical stance."""
    import mujoco
    for bl in (0.0, 0.05):
        m, d = load(backlash=bl)
        for j, want in (("hip_l", 0.35), ("knee_l", -0.70),
                        ("ankle_pitch_l", 0.35), ("ankle_roll_l", 0.0)):
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            assert d.qpos[m.jnt_qposadr[i]] == pytest.approx(want, abs=1e-9)
        torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
        assert d.xpos[torso][2] == pytest.approx(0.247, abs=1e-3)


# --- foot-mode ankle loop ----------------------------------------------------

def _stand_in_foot_mode(backlash, cfg, impulse=0.0, duration=9.0):
    """Drop the robot straight into foot mode and see if it stays there."""
    import mujoco
    from src.rsbot.model import ROLL_FOOT, WHEEL_HALF_W, ankle_pitch_level
    from src.rsbot.sim import CTRL_HZ, obs
    from src.rsbot.transition import DeployMachine, STAND
    from src.rsbot.balance import pitch_from_quat

    m, d = load(backlash=backlash)
    mach = DeployMachine(cfg=cfg)
    mach.state, mach.roll, mach.unload = STAND, ROLL_FOOT, 0.0
    mach.bal.height = cfg.stand_height
    hip, knee = leg_ik(cfg.stand_height)
    pose = {"hip": hip, "knee": knee,
            "ankle_pitch": ankle_pitch_level(hip, knee), "ankle_roll": ROLL_FOOT}

    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = cfg.stand_height + WHEEL_HALF_W, 1.0
    for i in range(m.njnt):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        if not n or n == "root":
            continue
        key = n.rsplit("_", 1)[0] if n.endswith(("_l", "_r")) else None
        d.qpos[m.jnt_qposadr[i]] = pose.get(key, 0.0)
    mujoco.mj_forward(m, d)

    decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
    dt = decim * m.opt.timestep
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    for j in range(int(duration / m.opt.timestep)):
        t = j * m.opt.timestep
        if j % decim == 0:
            d.ctrl[:] = mach(obs(m, d), dt)
        d.xfrc_applied[torso] = 0.0
        if impulse and 3.0 <= t < 3.010:
            d.xfrc_applied[torso, 0] = impulse / 0.010
        mujoco.mj_step(m, d)
        if abs(pitch_from_quat(obs(m, d)["quat"])) > 0.8 or d.xpos[torso][2] < 0.10:
            return False
    return True


# THE ANKLE LOOP IS GONE, and with it the two tests that guarded it. They
# asserted that a weak PID on the ankle was what made foot mode survive gear
# lash, and that removing it broke the stance - which was true of a robot that
# had an ankle servo. It does not, and the loop it tested drove an actuator
# that is not in the model any more. See docs/deleting-the-ankle-pitch-servo.md
# and src/rsbot/transition.py's STAND branch.
#
# What replaced them is below: the same question asked of the mechanism.


@pytest.mark.parametrize("deg", [1.0, 2.0, 3.0, 4.0, 6.0])
def test_standing_in_foot_mode_is_not_what_lash_breaks(deg):
    """Dropped straight into foot mode, already flat, the robot stands at any
    lash tried - including 6 degrees, which the FLIP cannot survive.

    That is worth an assertion of its own, because it says where the difficulty
    actually is. It is not holding the pose, it is arriving in it: the flip
    carries momentum in, and the deadzone is what lets the body use it.

    It also caught a bad check. The first version of the cliff test below used
    this harness with ten different durations, which is not ten trials - the
    initial condition never changes, so it is one trial run ten times, and it
    came back 10/10 at a lash the robot demonstrably cannot flip at.
    """
    cfg = DeployCfg(flip_rate=2.0)
    assert _stand_in_foot_mode(math.radians(deg), cfg), f"fell at {deg} deg"


def test_the_flip_still_has_a_cliff_and_it_is_past_the_design_point():
    """Guards test_linkage.py's 10/10 from becoming a check that cannot fail.
    If the robot flipped at any lash you cared to name it would be measuring
    nothing.

    Ten real trials, varying the moment the flip is triggered, which is the
    thing that changes what the robot carries into the manoeuvre.

    TWELVE DEGREES. It was six, then ten, and the moves have been real each
    time: mass leaving the shin into bodies of its own, the ankle servo coming
    out, and now the linkage's fin, stub and arm becoming features of the
    chassis, shin and yoke, which is 12 g and a different distribution.

    Measured rather than nudged, over the same ten trigger times:

        4.3 deg  6/6      10 deg  4/10
        6   deg  5/6      12 deg  0/10
        8   deg  3/6      14 deg  0/10   16 deg  0/10

    Ten is NOT the cliff and had stopped being one: a six-trial sweep reads
    0/6 there and the full ten reads 4/10, because the last four trigger times
    survive. Between about 5 and 8 degrees the answer is also sensitive to
    LINKAGE_DAMPING, a modelling constant, so nothing in that band is a
    measurement. 12 comes back 0/10 at both zero damping and the value the
    model ships, which makes it the honest place to put a guard - and it is
    2.8x the 4.3 deg the robot is actually built to.
    """
    from src.rsbot.sim import rollout_deploy
    stood = 0
    for i in range(10):
        t = 2.0 + 0.1 * i
        r = rollout_deploy(backlash=math.radians(12.0), trigger=t,
                           duration=t + 15.0)
        stood += (not r["fell"]) and r["state"] == "STAND"
    assert stood == 0, f"12 deg of lash flipped {stood}/10, so where is the cliff?"


def test_round_trip_survives_backlash():
    from src.rsbot.balance import LASH_GAINS
    r = rollout_cycle(cfg=DeployCfg(flip_rate=2.0), gains=LASH_GAINS,
                      stand_for=3.0, duration=30.0, backlash=math.radians(1.0))
    assert not r["fell"] and r["back_on_wheels"]


# --- real-part packaging -----------------------------------------------------

def test_visual_parts_carry_no_mass_or_collision():
    """The real-part geometry must stay decorative: mass and contacts live on
    the simple shapes, or every dynamics result we have is invalid."""
    import mujoco
    m, _ = load()
    for i in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
        if m.geom_group[i] == 0 and name not in ("floor",) \
                and not name.startswith("h_"):   # visual build
            assert m.geom_contype[i] == 0 and m.geom_conaffinity[i] == 0, name
    t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    # AGAINST THE CAD, not against a literal. It was 1.799 here and the CAD now
    # builds 1.787, because the linkage's fin, stub and arm stopped being loose
    # solids and became features of the chassis, shin and yoke. That is the
    # third time this line has gone stale on a change that was correct, which is
    # the definition of a number that should not have had a second copy.
    #
    # The tolerance stays tight, because tightness is the point: it has moved
    # for real reasons every time: tread cut into the wheel, the press fit and
    # retaining bead that stop the tyre spinning on the rim, every horn hole
    # growing 0.7 mm when the pattern turned out to be M3, +17 g of chassis end
    # panels that close an open U-section into a box, and five servo cradles.
    # Then 1.907 to 1.799, which is the ankle-pitch servos and their belt drive
    # coming out and the parallelogram going in: see cad/linkage.py.
    #
    # It sat at 1.875 for a fortnight while the CAD said 1.907, because
    # SEG_INERTIA is pasted from `cad.inertia --emit` and nobody re-ran it after
    # the cradles landed. This assertion was RIGHT to be tight - it is the one
    # that would have caught it, if the value had been regenerated alongside.
    import cad.masses as masses
    built = sum(v[3] for v in masses.table().values()) / 1000.0
    assert m.body_subtreemass[t] == pytest.approx(built, abs=3e-3)


def test_no_real_part_hits_the_floor_in_either_mode():
    """The ankle roll maps y onto z, so anything mounted on the roll bracket
    has to be checked in BOTH orientations."""
    import mujoco
    from src.rsbot.model import (ROLL_FOOT, ROLL_WHEEL, WHEEL_HALF_W, WHEEL_R,
                                 ankle_pitch_level)
    m, d = load()

    def lowest_point(i):
        R = d.geom_xmat[i].reshape(3, 3)
        c, sz, t = d.geom_xpos[i], m.geom_size[i], m.geom_type[i]
        if t == mujoco.mjtGeom.mjGEOM_BOX:
            return min((c + R @ np.array([sx * sz[0], sy * sz[1], sz2 * sz[2]]))[2]
                       for sx in (-1, 1) for sy in (-1, 1) for sz2 in (-1, 1))
        if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
            az = R[2, 2]
            return c[2] - abs(sz[1] * az) - sz[0] * math.sqrt(max(0.0, 1 - az * az))
        if t == mujoco.mjtGeom.mjGEOM_CAPSULE:
            ends = [c + R @ np.array([0, 0, s * sz[1]]) for s in (-1, 1)]
            return min(e[2] for e in ends) - sz[0]
        return c[2]

    for height, roll, z in ((0.207, ROLL_WHEEL, 0.207 + WHEEL_R),
                            (0.195, ROLL_FOOT, 0.195 + WHEEL_HALF_W)):
        hip, knee = leg_ik(height)
        pose = {"hip": hip, "knee": knee,
                "ankle_pitch": ankle_pitch_level(hip, knee), "ankle_roll": roll}
        d.qpos[:] = 0
        d.qpos[2], d.qpos[3] = z, 1.0
        for j in range(m.njnt):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if not n or n == "root":
                continue
            d.qpos[m.jnt_qposadr[j]] = pose.get(n.rsplit("_", 1)[0], 0.0)
        mujoco.mj_forward(m, d)
        for i in range(m.ngeom):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
            if n in (None, "floor") or n.startswith(("vtire", "vhub", "wheel")):
                continue                     # the wheel IS the contact
            assert lowest_point(i) > -1e-3, f"{n} below the floor at roll={roll:.2f}"


@pytest.mark.parametrize("mode", ["wheel", "foot"])
def test_no_parts_interpenetrate(mode):
    """Real parts must not pass through each other. MuJoCo will not catch this
    for us: geoms in the same body never collide, and the visual build is
    non-colliding by design, so it needs its own oriented-box check."""
    from fitcheck import audit
    hits = audit(mode, verbose=False)
    assert not hits, "; ".join(f"{a} x {b} {p*1000:.1f}mm" for p, a, b in hits[:5])


@pytest.mark.parametrize("mode", ["wheel", "foot"])
def test_robot_is_one_assembly_not_a_cloud_of_parts(mode):
    """Non-overlapping is not the same as assembled. Two parts that miss each
    other by 30 mm pass a penetration check and still look like they hover."""
    from fitcheck import connectivity
    groups = connectivity(mode, verbose=False)
    assert len(groups) == 1, f"{len(groups)} disconnected groups"


def test_nothing_collides_during_the_flip():
    """The two end poses being clean says nothing about the 90 degrees in
    between. The wheel-drive servo turns with the roll bracket and sweeps an
    annulus through the space the shin and ankle occupy."""
    from fitcheck import sweep_audit
    hits = sweep_audit(steps=25, verbose=False)
    assert not hits, "; ".join(
        f"{a} x {b} {p*1000:.1f}mm at {f*100:.0f}%"
        for (a, b), (p, f) in sorted(hits.items(), key=lambda kv: -kv[1][0])[:5])


@pytest.mark.parametrize("mode", ["wheel", "foot"])
def test_every_body_is_one_rigid_piece(mode):
    """Parts on the same body must actually TOUCH each other. The robot-wide
    connectivity check uses a 4 mm tolerance, which is a running clearance
    across a joint but is just a gap inside a rigid part - that is what
    'floating servos' looks like."""
    import itertools
    from fitcheck import _obb, gap, penetration, pose

    m, d = load()
    pose(m, d, mode)
    name = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
    vis = [i for i in range(m.ngeom)
           if m.geom_group[i] == 0 and (name(i) or "") != "floor"
           and not (name(i) or "").startswith("h_")]
    bodies = {}
    for i in vis:
        bodies.setdefault(m.geom_bodyid[i], []).append(i)

    for b, gs in bodies.items():
        parent = {i: i for i in gs}

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for i, j in itertools.combinations(gs, 2):
            if (gap(_obb(m, d, i), _obb(m, d, j)) <= 1e-4
                    or penetration(_obb(m, d, i), _obb(m, d, j)) > 0):
                parent[find(i)] = find(j)
        loose = {find(i) for i in gs}
        bn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        assert len(loose) == 1, f"{bn} is {len(loose)} loose pieces, not one part"


@pytest.mark.parametrize("mode", ["wheel", "foot"])
def test_whole_robot_assembles_without_interference(mode):
    """Every CAD part, at the pose the simulator puts it in, intersected
    against every other. This is the one that answers 'does it actually go
    together', and it runs on the real solids rather than the sim's
    simplified boxes."""
    from cad.assemble_check import main as check
    assert not check(mode)
