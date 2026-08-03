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


# --- the flag means what it says ----------------------------------------------
#
# This is a regression test for a trap, not for a feature. `parallel=True` used
# to build the linkage AND leave the ankle actuator in, so the STAND loop fought
# the constraint and foot mode went to 0/10 - worse than the servo it replaces.
# Reproducing the published 10/10 needed the caller to know to switch the ankle
# loop off, and nothing anywhere said so.

@pytest.mark.parametrize("deg", [2.0, 3.0])
def test_the_linkage_stands_where_the_servo_does_not(deg):
    """10 flips at 2 and 3 degrees of lash, with the DEFAULT config."""
    stood = 0
    for i in range(10):
        trigger = 2.0 + 0.1 * i
        r = rollout_deploy(backlash=np.deg2rad(deg), parallel=True,
                           trigger=trigger, duration=trigger + 15.0)
        stood += (not r["fell"]) and r["state"] == "STAND"
    assert stood == 10


def test_a_parallel_model_has_no_ankle_actuator():
    """The mechanical claim, in the model: there is no ankle servo left."""
    from src.rsbot.model import load
    import mujoco
    m, _ = load(parallel=True)
    for name in ("apit_l", "apit_r"):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert m.actuator_gear[i, 0] == 0.0
    m, _ = load()
    for name in ("apit_l", "apit_r"):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert m.actuator_gear[i, 0] != 0.0


def test_the_lash_only_control_still_has_its_servo():
    """The control test asks whether the win is just the missing lash. It only
    means anything if the ankle can still be driven; without this the control
    is a robot with no ankle actuator and no linkage, which falls for a reason
    that has nothing to do with the question."""
    import mujoco
    from src.rsbot.model import load
    from src.rsbot.sim import _lash_only
    m, d = load(parallel=True)
    _lash_only(m, d)
    ref = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip_l")
    for name in ("apit_l", "apit_r"):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert m.actuator_gear[i, 0] == m.actuator_gear[ref, 0]
    assert not m.eq_active0.any()


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
    only arm that cleared the roll bracket that way ended up 1 mm up."""
    assert lk.ground(verbose=False) == pytest.approx(8.1, abs=0.3)


def test_nothing_collides_over_squat_and_flip():
    """Both legs, 13 poses, on the real solids.

    Minutes, not seconds, and it runs with everything else anyway. A check
    behind a marker is a check nobody runs, which is the one thing this repo
    has decided is worse than no check at all.
    """
    assert lk.clearance(verbose=False) == []
