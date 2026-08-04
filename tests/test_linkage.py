"""The passive ankle parallelogram, as a mechanism.

docs/deleting-the-ankle-pitch-servo.md settled the CONTROL question - a linkage
holding hip + knee + ankle = 0 stands foot mode where the servo does not - and
left the mechanical one open. cad/linkage.py answers it; this is what pins the
answers down so they cannot quietly stop being true.

Every number here is asserted tightly rather than bounded comfortably, for the
reason docs/how-checks-fail.md #4 gives: a bound wide enough to be comfortable
is a bound that will absorb a real fault.
"""

import numpy as np
import pytest

import cad.linkage as lk
from src.rsbot.sim import rollout_deploy


# --- the servo is really gone --------------------------------------------------
#
# There is no `parallel` flag any more. It was a proposal sitting beside the
# design it was competing with, and keeping both meant keeping two robots: two
# actuator counts, two ctrl layouts, two mass budgets. These assert that the one
# that is left is the one that was chosen.

@pytest.mark.parametrize("deg", [2.0, 3.0])
def test_the_linkage_stands_where_the_servo_did_not(deg):
    """10 flips at the lash these servos actually have. The belt-driven servo
    and its ankle loop managed 2 and 3 out of 10 at these numbers."""
    stood = 0
    for i in range(10):
        trigger = 2.0 + 0.1 * i
        r = rollout_deploy(backlash=np.deg2rad(deg),
                           trigger=trigger, duration=trigger + 15.0)
        stood += (not r["fell"]) and r["state"] == "STAND"
    assert stood == 10


def test_there_are_eight_actuators_and_no_ankle_pitch():
    """The mechanical claim, in the model: four servos a leg, not five. Checked
    by NAME rather than by counting, because a count of 8 would also be
    satisfied by losing the wrong two."""
    import mujoco
    from src.rsbot.model import load
    m, _ = load()
    names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
             for i in range(m.nu)}
    assert names == {"hip_l", "knee_l", "aroll_l", "wheel_l",
                     "hip_r", "knee_r", "aroll_r", "wheel_r"}


def test_the_ankle_joint_still_exists_and_is_held_by_the_tendon():
    """Deleting the SERVO is not deleting the JOINT. The ankle still moves; it
    is the parallelogram, modelled as a tendon equality, that decides where."""
    import mujoco
    from src.rsbot.model import load
    m, d = load()
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "ankle_pitch_l")
    assert j >= 0
    # EIGHT, not two. One `par` tendon a leg holds the ankle, and three more
    # pin the linkage's own bodies to the angles the mechanism puts them at.
    assert m.ntendon == 8 and m.neq == 8
    hip = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_JOINT, "hip_l")]]
    knee = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_JOINT, "knee_l")]]
    assert d.qpos[m.jnt_qposadr[j]] == pytest.approx(-(hip + knee), abs=1e-6)


def test_the_belt_drive_is_gone_from_the_sim():
    """It was a body with its own DOF that existed only when meshes=True, so
    the twin's degree-of-freedom count depended on a display flag."""
    import mujoco
    from src.rsbot.model import load
    plain, _ = load()
    mesh, _ = load(meshes=True)
    assert plain.nv == mesh.nv
    assert mujoco.mj_name2id(mesh, mujoco.mjtObj.mjOBJ_BODY, "beltdrive_l") < 0


# --- the mechanism closes ------------------------------------------------------

def test_the_foot_is_exactly_level_at_every_squat_depth():
    """Not approximately. Both stages are exact parallelograms, so the foot
    angle is zero by construction and not by fit, and if that ever becomes
    'small' instead of 'zero' the geometry has stopped being a parallelogram."""
    worst, _ = lk.closes(verbose=False)
    assert worst < 1e-9


def test_the_closure_solver_can_fail():
    """Control test. A rod 1 mm long has to show up, or nothing above means
    anything: the solver would be reporting the ankle angle it was handed
    rather than the one the links produce."""
    _, control = lk.closes(verbose=False)
    assert control == pytest.approx(2.03, abs=0.05)


def test_the_solver_stays_on_the_parallelogram_branch():
    """A four-bar has two assembly branches and the other one folds the foot
    over. Stepping from zero with Newton walks onto it; bracketing does not."""
    for hip, knee in lk._band(5):
        foot, idler = lk.foot_angle(hip, knee)
        assert abs(foot) < 1e-9 and abs(idler) < 1e-9


# --- what the pins cost --------------------------------------------------------

def test_pin_play_beats_the_gear_lash_it_replaces():
    """The whole argument. Free play at four pins has to come in under the
    2-3 degrees of ankle gear lash, or the mechanism has bought nothing."""
    on_bearings, _, _ = lk.error_budget(radial=lk.BEARING_PLAY, tol=0.0,
                                        verbose=False)
    in_bores, _, _ = lk.error_budget(radial=lk.PLAY, tol=0.0, verbose=False)
    assert on_bearings == pytest.approx(0.10, abs=0.02)
    assert in_bores == pytest.approx(1.00, abs=0.05)
    assert in_bores < 2.0


def test_the_build_tolerance_needs_an_adjustable_rod():
    """4 degrees of fixed foot tilt at the print service's own tolerance. It is
    not lash and it is not free play, it is a part that comes out of the machine
    aimed slightly wrong, and one of the two rods has to be adjustable to take
    it out. This asserts the SIZE of the problem, so that shrinking it counts
    as progress and forgetting it does not read as success."""
    _, offset, spread = lk.error_budget(tol=lk.TOL, verbose=False)
    assert offset == pytest.approx(4.21, abs=0.1)
    assert spread == pytest.approx(1.09, abs=0.1)


def test_the_adjuster_covers_the_build_offset():
    """The requirement, in one line: rod 2's travel has to be worth more foot
    tilt than the print tolerance can put in."""
    _, offset, _ = lk.error_budget(tol=lk.TOL, verbose=False)
    assert lk.adjust_range_deg() >= offset


def _flip_with_offset(deg, lash=3.0, trials=5, opposite=False):
    """Flip into foot mode with a deliberate foot-angle build error.

    Injected into the tendon equality's constant, which is exactly what a
    mis-built linkage does: it still holds hip + knee + ankle rigidly, it just
    holds it at the wrong number.
    """
    import mujoco
    from src.rsbot.model import load
    from src.rsbot.sim import CTRL_HZ, obs
    from src.rsbot.transition import DeployCfg, DeployMachine, STAND

    stood = 0
    for i in range(trials):
        trigger = 2.0 + 0.1 * i
        m, d = load(backlash=np.deg2rad(lash))
        r = np.deg2rad(deg)
        # BY NAME. This was `m.eq_data[0]` and `[1]`, which were the two ankle
        # tendons right up until the linkage's own bodies arrived and added
        # three equalities a leg. After that, index 1 was a ROD's coupling, and
        # the test went on reporting 8/8 at offsets that used to floor the
        # robot - because it was no longer injecting the fault it named.
        for side, sign in (("l", 1.0), ("r", -1.0 if opposite else 1.0)):
            t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, f"par_{side}")
            e = next(k for k in range(m.neq)
                     if m.eq_type[k] == mujoco.mjtEq.mjEQ_TENDON
                     and m.eq_obj1id[k] == t)
            m.eq_data[e, 0] = sign * r
        mujoco.mj_resetDataKeyframe(m, d, 0)
        mujoco.mj_forward(m, d)
        mach = DeployMachine(cfg=DeployCfg(flip_rate=2.0))
        torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
        decim = int(round(1.0 / (CTRL_HZ * m.opt.timestep)))
        dt = decim * m.opt.timestep
        fired = fell = False
        for k in range(int((trigger + 15.0) / m.opt.timestep)):
            t = k * m.opt.timestep
            if not fired and t >= trigger:
                mach.start_deploy()
                fired = True
            if k % decim == 0:
                d.ctrl[:] = mach(obs(m, d), dt)
            mujoco.mj_step(m, d)
            if d.xpos[torso][2] < 0.12:
                fell = True
                break
        stood += (not fell) and mach.state == STAND
    return stood


def test_an_adjusted_linkage_flips_every_time():
    """Nulled, which is what the adjuster is for, and it has to be perfect.
    Both legs the same way and opposed, because two independently printed legs
    are not wrong in the same direction."""
    assert _flip_with_offset(0.0, trials=8) == 8
    assert _flip_with_offset(0.0, trials=8, opposite=True) == 8


def test_there_is_a_cliff_and_the_worst_case_is_inside_it():
    """Why the adjuster is not optional.

    Measured, at 3 degrees of gear lash, 8 triggers each:

        offset      same way    opposed
        4.21 deg    8/8         8/8      <- the worst case, unadjusted
        5.50 deg    3/8         4/8
        8.00 deg    0/8         0/8

    So the worst case clears, by about 1.3 degrees. That is the whole argument
    for the adjuster and it is a thinner one than it looks: the 4.21 is a bound
    with its own assumptions, the 3 degrees of gear lash has never been
    measured on hardware, and the two stack.

    An earlier run of this put the worst case at 7/8 opposed and concluded it
    was already over the line. That was the TENDON model, before the linkage
    had bodies of its own; moving 10.6 g off the shin moved the answer. Both
    models agree about where the cliff is, which is the part worth trusting.
    """
    assert _flip_with_offset(8.0, trials=8) == 0


def test_the_error_budget_is_zero_when_nothing_is_wrong():
    """Control: with no play and no tolerance there is no error to find."""
    play, offset, spread = lk.error_budget(radial=0.0, tol=0.0, verbose=False)
    assert max(play, offset, spread) < 1e-9


# --- what it is pushed with ----------------------------------------------------

def test_the_rods_never_approach_a_singularity():
    """A parallelogram goes singular when a rod lines up with its link, and the
    rod force runs as 1/sin of that angle. Both offsets are horizontal-ish in
    the torso frame and the leg is never near straight, so this stays high."""
    assert lk.transmission(verbose=False) == pytest.approx(0.931, abs=0.01)


def test_the_rods_are_nowhere_near_buckling():
    """Sized on the measured ankle torque, not on cad/belt.py's literal."""
    force, critical = lk.loads(torque_nm=1.082, verbose=False)
    assert force == pytest.approx(38.7, abs=0.5)
    assert critical / force > 10.0


# --- and it fits ---------------------------------------------------------------

def test_the_pivots_meet():
    """The two that are not tautologies. Stage 2's is the interesting one: it
    measures a rod of fixed length against the ANKLE BODY's own attitude, so it
    opens up the moment the simulator stops holding the foot level."""
    assert lk.seats(verbose=False) < 0.01


def test_the_linkage_stays_off_the_floor_in_foot_mode():
    """The sole is at 0 and the axle at 12, so there is not much room under the
    linkage's lowest part. This is what ruled out routing stage 2 inboard: the
    only arm that cleared the roll bracket that way ended up 1 mm up.

    13.0 mm and it was 8.1, and BOTH halves of that move are worth knowing:

    - the yoke arm really did rise, from ARM_T 8 -> 6 and its root moving off
      the ankle-pitch shaft bore to z = +5, which is the seating fix.
    - and ground() had stopped measuring the arm at all. It walks placed(), the
      arm stopped being a linkage part when the yoke started printing it, and
      the check got 5 mm greener on its own. cad/linkage.py's mounts_placed()
      exists to put that back; the lowest part it reports is lk_arm again.

    Control-tested by dropping the arm root 19 mm, which reads -4.5 mm."""
    assert lk.ground(verbose=False) == pytest.approx(13.0, abs=0.3)


def test_nothing_collides_over_squat_and_flip():
    """Both legs, 13 poses, on the real solids.

    Minutes, not seconds, and it runs with everything else anyway. A check
    behind a marker is a check nobody runs, which is the one thing this repo
    has decided is worse than no check at all.
    """
    assert lk.clearance(verbose=False) == []


# --- what the servo mounts give away ------------------------------------------

def test_no_servo_can_turn_far_in_its_own_mount():
    """Capture slop is in series with the gearbox, so it adds to that joint's
    gear lash, and this robot has 1.5 degrees of margin to spend.

    Asserted per servo and tightly. A single "worst is under X" would pass while
    one mount quietly got worse and another got better.
    """
    from cad.fasteners import capture
    got = {name: free for free, name, _ in capture(verbose=False)}
    want = {"vhipsv1": 1.30, "vhipsv-1": 1.30, "vrollsv_l": 1.30,
            "vrollsv_r": 1.30, "vkneesv_l": 0.70, "vkneesv_r": 0.70,
            "vwhlsv_l": 0.10, "vwhlsv_r": 0.10}
    assert got == pytest.approx(want, abs=0.05)


def test_every_servo_is_stopped_by_something():
    """The check reports its ceiling when nothing stops a servo, and a ceiling
    read as a measurement is how a check comes to say a floating part is fine."""
    from cad.fasteners import CAPTURE_MAX, capture
    for free, name, by in capture(verbose=False):
        assert by is not None, f"{name} turns freely"
        assert free < CAPTURE_MAX


# --- the horn joints ----------------------------------------------------------

def test_every_horn_joint_seats():
    """Every joint in this robot that is not a bearing is a part bolted to a
    servo horn. Eight of them, and the wheel was the last one that did not
    reach: 4.40 mm of air where it was supposed to be resting.

    Tight, at 0.05 mm. A part that stands off its horn rocks on the horn's rim
    and puts the whole joint's load through the bolts in bending, and 4.40 mm
    of it read as a working robot in every interference check in the repo,
    because the thing in between was not modelled until 2026-08-02.
    """
    from cad.hardware import horn_joints
    rows = horn_joints(verbose=False)
    assert len(rows) == 8
    for servo, part, clear in rows:
        assert abs(clear) < 0.05, f"{servo} -> {part} is {clear:+.2f} mm out"


# --- is it actually held together? --------------------------------------------

def test_every_pivot_is_actually_made():
    """Not "does it touch something" - how much of each pin is INSIDE the two
    parts it joins.

    This caught two faults that a connectivity check cannot. The pins were
    drawn as printed plastic spigots fused to their carriers, while the order
    sheet listed eight 3 mm steel pins; and once they became separate parts
    they turned out to be the wrong length, engaging 3 mm of a 6 mm idler and
    ZERO of the chassis fin, whose material stopped exactly where the pin
    began. Both read as one connected group throughout.
    """
    assert lk.assembled(verbose=False) == []


def test_the_idler_has_a_bearing_to_turn_on():
    """It sat 0.50 mm off its stub with nothing in between: the 623ZZ was on
    the order sheet and drawn nowhere, which is docs/how-checks-fail.md #13
    exactly - the parts that do the assembling are the ones nobody checks."""
    import mujoco

    from src.rsbot.model import load
    m, _ = load()
    for side in ("l", "r"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,
                                 f"lkbrg_idler_{side}") >= 0


@pytest.mark.parametrize("mode", ["wheel", "foot"])
def test_the_whole_robot_is_one_object(mode):
    """63 solids, on the real geometry, at 0.05 mm. Not the sim's boxes at
    4 mm, which is a much easier question."""
    from cad.assemble_check import connected
    assert connected(mode, verbose=False) == []


def test_each_mount_feature_is_seated_on_its_host():
    """Not "does it touch" - how much of each root's face is ON material.

    Two solids meeting on a plane intersect in NOTHING, so a face butt has no
    overlap volume and neither an interference check nor a connectivity check
    can tell it from a graze. That is not hypothetical: the stub axle was
    attached to the shin through 13 mm3 of the ankle servo's CRADLE WALL, four
    millimetres off the knee axis, because its face had been measured against
    that wall instead of against the hub. One connected group throughout, and it
    would have stayed true until the cradle came out - which is already on the
    list, because it holds a servo that no longer exists.

    Asserted per feature and tightly. The stub's 61% is the hub's horn bore and
    screw holes, which have to be open; the other two are the whole face.
    """
    got = {what: round(area / full, 3) for what, area, full in
           lk.mounts(verbose=False)}
    assert got == pytest.approx({"stub on the shin hub": 0.613,
                                 "arm on YOKE_FWD": 1.0,
                                 "fin on the side plate": 1.0}, abs=0.02)
