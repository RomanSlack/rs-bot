"""The stress pipeline, and the control tests that make it worth believing.

The full study takes minutes; these are the fast parts that would catch it
going wrong.
"""

import numpy as np
import pytest

from cad import fea, loads, stress


def test_solver_matches_beam_theory():
    """The whole pipeline rests on this. Deflection should land slightly soft
    (shear, which beam theory omits) and mid-span stress slightly high."""
    rd, rs = fea.verify(verbose=False)
    assert 0.95 < rd < 1.05, f"deflection off by {rd - 1:+.1%}"
    assert 0.95 < rs < 1.10, f"stress off by {rs - 1:+.1%}"


def test_ordering_check_rejects_a_scrambled_mesh():
    """A check that cannot fail is worse than no check. Swapping two mid-edge
    slots must be caught, or the shape functions could be wrong and every
    result would be plausible nonsense."""
    # 3 mm, not coarser: the M2 bolt holes are 2.2 mm across and gmsh cannot
    # mesh them at 6 mm - it fails with overlapping facets rather than
    # quietly producing a bad mesh.
    nodes, elems = fea.mesh_step(stress.OUT / "shin.step", size=3.0)
    fea.check_ordering(nodes, elems)
    bad = elems.copy()
    bad[:, [4, 8]] = bad[:, [8, 4]]
    with pytest.raises(AssertionError):
        fea.check_ordering(nodes, bad)


def test_interface_force_closes_against_statics():
    """The knee wrench must equal what is below the knee minus the ground
    under that wheel. Getting the cfrc_int reference frame wrong is silent
    otherwise."""
    got, expect, _ = loads.static_check()
    assert abs(got - expect) < 0.3, f"{got:.2f} N vs {expect:.2f} N"


def test_load_survey_is_ordered_by_severity():
    """Quiet must be the mildest case, the flip worse, and the tipover worst of
    all - the robot falls on its side there, so nothing should beat it.

    The fore/aft shove sits BELOW the flip, which is the interesting part: the
    wheels roll away and almost nothing reaches the structure. Laterally the
    same impulse has nowhere to go.
    """
    cases = loads.survey()
    for child in loads.CHILD.values():
        n = {c: np.linalg.norm(cases[c][child][0]) for c in cases}
        assert n["quiet"] < n["flip"] < n["tipover"], f"{child}: {n}"
        assert n["shove"] < n["flip"], f"{child}: fore/aft shove beat the flip: {n}"


def test_roll_bracket_reaches_its_own_roll_axis():
    """The bracket used to be a crank: an outboard arm crabbing round the
    shin's post to a bearing it never reached, joined by a 2.00 x 0.50 x
    2.65 mm lap that FEA put at 1596% of PETG's allowable.

    With the shin's post gone from that corridor it is a plate straight from
    the arm to the roll axis. Pinning the two things that matter: the web has
    real thickness, and the bore is ON the axis.
    """
    import build123d as bd
    import cad.ankle as ankle

    part = ankle.roll_bracket()
    x0, x1 = ankle.ROLL_WEB_X
    assert x1 - x0 >= 3.0, "the web is thinner than 3 mm"

    # A 6 mm probe down the roll axis must find material to grip.
    probe = (bd.Pos((x0 + x1) / 2, 0, 0) * bd.Rot(0, 90, 0)
             * bd.Cylinder(ankle.ROLL_HUB_R, x1 - x0))
    assert (part & probe).volume > 100.0, "nothing on the roll axis to grip"


def test_roll_bracket_corner_is_filleted():
    """The hottest twelve nodes all sat inside a 3 mm cube on a sharp
    re-entrant corner, which is a singularity - refine the mesh and it grows.
    OCC refuses the fillet if the edge selection is widened, and swallowing
    that exception returns the SHARP part while looking like a small
    improvement."""
    import cad.ankle as ankle
    assert ankle.roll_bracket().volume > 6325.0, "the corner fillet is missing"


def test_ankle_pitch_joint_is_drawn_and_on_its_axis():
    """This joint did not exist. The shin's bore was 52.8 mm from the axis the
    sim rotates about, the yoke had no matching feature, and assembled the two
    parts came no closer than 10.3 mm with nothing between them.

    cad/envelope.py solves for where the bearing may go and finds exactly two
    bands, y = -80..-47 and y = +57..+80; everything between is inside the
    wheel in one of its two shapes or inside the wheel-drive servo.
    """
    import build123d as bd
    import cad.ankle as ankle
    import cad.shin as shin

    # Both features must sit ON the axis: x = 0, z = ANKLE_Z, along y. The
    # probe is wider than the bearing seat, because a bore centred on the axis
    # removes most of a narrow probe and the check would read as a miss.
    axis = (bd.Pos(0, 0, shin.ANKLE_Z) * bd.Rot(90, 0, 0)
            * bd.Cylinder(9.0, 200))
    assert (shin.build() & axis).volume > 200.0, "shin has no bearing on the axis"

    yoke_in_shin = bd.Pos(0, 0, shin.ANKLE_Z) * ankle.yoke()
    assert (yoke_in_shin & axis).volume > 200.0, "yoke has no shaft on the axis"

    # And in one of the legal bands.
    assert 57.0 <= shin.PITCH_Y <= 80.0, f"bearing at y={shin.PITCH_Y} is illegal"

    # They must meet, not merely both be near the axis.
    gap = shin.build().distance_to(yoke_in_shin)
    assert gap < 0.5, f"shin and yoke still {gap:.1f} mm apart at the joint"


def test_ankle_pitch_range_is_clear_of_the_shin():
    """The yoke swings past the shin as the ankle pitches, and nothing checked
    that until this joint existed. The robot uses +19.5 .. +29.4 deg."""
    import build123d as bd
    import cad.ankle as ankle
    import cad.shin as shin

    sh, yk = shin.build(), ankle.yoke()
    for deg in range(-90, 46, 15):
        moved = bd.Pos(0, 0, shin.ANKLE_Z) * bd.Rot(0, float(deg), 0) * yk
        v = (sh & moved).volume
        assert v < 1.0, f"yoke hits the shin at {deg:+d} deg pitch ({v:.0f} mm3)"


# --- the wheel, which is also the foot ---------------------------------------
#
# These run on the REAL solids placed by the REAL robot pose, because every
# claim the wheel makes is about where it sits on the machine, not about its
# own drawing.

def _wheel_on_the_robot(mode):
    """(rigid body, TPU tyre) posed as they sit on the robot in `mode`."""
    import build123d as bd
    import mujoco

    import cad.assemble_check as ac
    import cad.wheel as W
    from fitcheck import pose
    from src.rsbot.model import load

    m, d = load()
    pose(m, d, mode)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_l")
    loc = ac._loc(d.xpos[bid], d.xmat[bid])
    return (loc * (bd.Rot(90, 0, 0) * W.body()),
            loc * (bd.Rot(90, 0, 0) * W.tyre()))


@pytest.mark.parametrize("mode,least", [("wheel", 1.5), ("foot", 2.0)])
def test_the_robot_stands_on_rubber_not_on_plastic(mode, least):
    """The whole reason this part is printed instead of bought.

    The 80 x 24 wheel in the original BOM stood its plastic hub 0.5 mm PROUD of
    the tyre, so in foot mode the robot would have stood on a hard boss. The
    margin has to beat a print tolerance, not just be positive: the services
    quote +/-0.3 mm, so anything under about a millimetre is not a result.
    """
    body, tyre = _wheel_on_the_robot(mode)
    rigid = body.bounding_box().min.Z
    rubber = tyre.bounding_box().min.Z
    assert rigid > rubber + least, (
        f"{mode} mode: rigid is {rigid - rubber:.2f} mm above the rubber, "
        f"wanted more than {least}")


def test_the_tyre_is_actually_retained_on_the_rim():
    """It used to be a slip fit - bore and rim both exactly r = 37.0, zero
    interference - and the check said "must be 0" and passed. Drive torque at
    the rim is about 78 N at stall, so it would have spun the first time the
    robot moved.

    Tested by function: move the tyre the two ways it can come off and confirm
    the bead is in the way. Measured in the bead zone, because over the whole
    part the bore press fit is 1500 mm3 and drowns both signals.
    """
    import build123d as bd

    import cad.wheel as W

    b, t = W.body(), W.tyre()
    zc = -W.HALF_W + W.BEAD_INSET + W.BEAD_W / 2
    zone = bd.Pos(0, 0, zc) * bd.Box(4 * W.R, 4 * W.R, W.BEAD_W)
    seated = (b & t & zone).volume
    spun = (b & (bd.Rot(0, 0, 180.0 / W.BEAD_N) * t) & zone).volume
    slid = (b & (bd.Pos(0, 0, 1.0) * t) & zone).volume

    # ABSOLUTE engagement volumes, not ratios to `seated`. The first version of
    # this test asserted spun > 3 x seated, and it passed with the press fit
    # and the bead both removed: seated went to nearly zero, so the ratio stayed
    # large while the actual engagement was nothing. A ratio with a denominator
    # that can vanish is a check that cannot fail.
    assert seated > 20.0, (
        f"press fit is only {seated:.1f} mm3 in the bead zone - the bore is "
        f"not undersize enough to grip")
    assert spun - seated > 100.0, (
        f"teeth engage only {spun - seated:.1f} mm3: it can spin on the rim")
    assert slid - seated > 100.0, (
        f"lip catches only {slid - seated:.1f} mm3: it can walk off the rim")


def test_the_wheel_has_running_clearance_to_everything_static():
    """The wheel turns continuously, so for it any contact is a rub rather than
    a mating face. The roll bracket's arm ran parallel to the wheel's whole
    face at 0.1 mm, which is inside the tolerance band of both parts.
    """
    import cad.assemble_check as ac

    for mode in ("wheel", "foot"):
        items = dict(ac.parts(mode))
        wheel = items["wheel_l"]
        for name, other in items.items():
            if name == "wheel_l" or not ac.running_pair("wheel_l", name):
                continue
            g = ac.gap(wheel, other)
            assert g >= ac.TIGHT_MM, (
                f"{mode} mode: wheel_l to {name} is {g:.2f} mm, "
                f"needs {ac.TIGHT_MM} to survive a +/-0.3 mm print tolerance")


@pytest.mark.parametrize("mode", ["wheel", "foot"])
def test_the_assembled_picture_shows_the_right_mode(mode):
    """The render had the two modes swapped and nothing caught it.

    cad/robot.py builds its own MJCF, and that scene never said what units its
    angles were in. MuJoCo defaults to DEGREES, so the wheel's euler of 1.5708
    was applied as 1.57 degrees and the mesh never turned: flat in wheel mode,
    upright in foot mode, exactly backwards. Physics was untouched - everything
    else rotates the wheel through build123d, whose Rot() is really degrees -
    but the render is the artefact a human actually looks at, so when it lies
    it gets believed.
    """
    import cad.robot as robot

    assert robot.check_wheel_orientation(robot.build_scene(mode), mode)


def test_the_leg_is_one_connected_load_path():
    """The assembled model exists because every other entry in cad/stress.py is
    one part rigidly clamped at its own bolt holes - a wall where a compliant
    neighbour should be, and nothing ever adds the deflections up.

    Fusing it is also a check in its own right. An exact boolean union only
    closes if the parts really touch, and the first attempt came back as TWO
    solids: the wheel-drive servo floats 0.5 mm off the bracket it bolts to.
    That is excluded deliberately and is tracked in docs/assembled-strength.md;
    if anything ELSE stops touching, this is what catches it.
    """
    import cad.assemble_check as ac

    leg = ac.fused_leg("wheel")          # raises if it is not exactly 1 solid
    assert leg.volume > 200_000.0, "the fused leg lost most of its volume"


def test_every_servo_seats_flat_on_what_it_bolts_to():
    """A servo case on a standoff puts its reaction couple into the mounting
    bolts in BENDING, with no friction preload to help.

    The wheel-drive servo used to float 0.5 mm off the roll bracket, from
    clearing a 0.05 mm interference by moving 0.55. Nothing reported it,
    because the only assembly check was for interference and 0.5 mm of air
    reads exactly like 20 mm of air.

    Seating is exact contact, not interference: the case is a bought part and
    the bracket may not eat into it.
    """
    import cad.assemble_check as ac

    pairs = [("rollbracket_l", "vwhlsv_l"), ("ankle_l", "vrollsv_l"),
             ("shin_l", "vanksv_l"), ("thigh_l", "vkneesv_l")]
    items = dict(ac.parts("wheel"))
    for bracket, servo in pairs:
        assert ac.gap(items[bracket], items[servo]) < 1e-6, (
            f"{servo} does not seat on {bracket}: it is on a standoff")
        hit = items[bracket] & items[servo]
        v = hit.volume if hit else 0.0
        assert v < 1.0, f"{bracket} cuts {v:.1f} mm3 into the {servo} case"


# --- fasteners ---------------------------------------------------------------

def test_servo_shaft_axis_matches_the_joint_it_drives():
    """The control test, kept because it caught a complete false alarm.

    cad/servo.py writes the servo as (length, width, height-along-shaft), so
    the shaft is its local z. The SIM does not use that order: it writes each
    servo's three dimensions in whatever sequence puts the case where it goes,
    which puts the shaft on local y for four of the five and local x for the
    roll servo.

    Assume local z and cad/fasteners reports every screw in the robot as
    perpendicular to its servo - "nothing bolts to anything", entirely false,
    and entirely convincing. A check whose input is wrong does not fail; it
    answers a different question fluently. This is what makes the input
    checkable: a servo's shaft IS the axis of the joint it drives.
    """
    import mujoco

    import cad.fasteners as F
    from fitcheck import pose
    from src.rsbot.model import load

    m, d = load()
    pose(m, d, "wheel")
    frames = F._servo_frames("wheel")
    pairs = [("vhipsv1", "hip_l"), ("vkneesv_l", "knee_l"),
             ("vanksv_l", "ankle_pitch_l"), ("vrollsv_l", "ankle_roll_l"),
             ("vwhlsv_l", "wheel_l")]
    for sname, jname in pairs:
        _, _, shaft = frames[sname]
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        axis = d.xmat[m.jnt_bodyid[jid]].reshape(3, 3) @ m.jnt_axis[jid]
        assert abs(float(np.dot(shaft, axis))) > 0.99, (
            f"{sname}'s derived shaft is not {jname}'s axis - the servo box "
            f"axis order was misread and every fastener result is wrong")


def test_every_servo_screw_runs_along_its_servo_shaft():
    """A screw at 90 degrees to the hole it enters cannot be fitted, whatever
    the bolt pattern turns out to be. This is the fastener check that does not
    depend on the pattern, which is unverified."""
    import cad.fasteners as F

    rows = F.audit(verbose=False)
    assert rows, "no M2-scale holes found near any servo - the audit is blind"
    perp = [r for r in rows if r["align"] < 0.1]
    assert not perp, f"{len(perp)} screws are perpendicular to their servo"


def test_every_screw_can_be_reached_when_it_is_fitted():
    """Tool access, in BUILD order rather than on the finished robot.

    The distinction is the whole check. On the assembled machine 32 screws are
    buried; at the moment each is actually fitted, almost all are open, because
    the shin and the ankle servo that bury the thigh's knee bolts are not on
    the robot yet.

    A full-size driver still cannot reach four of the thigh's, and a 2 mm hex
    key reaches all of them - a handle-clearance problem, not a design fault,
    so it is recorded as a tool requirement rather than a geometry change.
    """
    import cad.toolaccess as T

    rows = T.check(stubby=True, verbose=False)
    assert rows, "no screws found at all - the check is blind"
    blocked = [r for r in rows
               if r["blocked"] > T.TOUCH and not r["horn"]]
    # Four per side on the roll bracket sit under the ankle yoke. They are
    # tight rather than impossible; if this grows, something moved.
    assert len(blocked) <= 4, (
        f"{len(blocked)} screws unreachable even with a hex key: "
        f"{sorted(set(r['part'] for r in blocked))}")


def test_tool_access_check_notices_an_obstruction():
    """Control. Drop a block over a screw that is reachable and the check has
    to stop calling it reachable."""
    import build123d as bd

    import cad.assemble_check as ac
    import cad.toolaccess as T

    hs = [h for h in T.holes("wheel") if h["part"] == "wheel_l"]
    assert hs, "no wheel screws to test with"
    h = hs[0]
    start = h["centre"] + h["axis"] * h["t1"]
    tool = T.driver(start, h["axis"])
    wall = bd.Pos(*(start + h["axis"] * 20.0)) * bd.Box(60, 60, 4)
    hit = tool & wall
    assert (hit.volume if hit else 0.0) > T.TOUCH, (
        "a slab straight across the driver path was not detected")


def test_no_running_clearance_closes_under_a_worst_case_tolerance_stack():
    """Nominal gaps are not real gaps. The services quote +/-0.3 mm and the
    error between two parts is one tolerance PER INTERFACE along the chain, so
    the wheel can be five tolerances out relative to the chassis.

    The tightest pair is the wheel against the roll bracket: 0.90 mm nominal,
    two interfaces, 0.30 mm left at worst case. That is why the running-clearance
    threshold is 0.8 and not something rounder.
    """
    import cad.assemble_check as ac

    for mode in ("wheel", "foot"):
        rows = ac.stack(mode, verbose=False)
        assert rows, "the stack check found no running pairs at all"
        closed = [r for r in rows if r[0] <= 0]
        assert not closed, (
            f"{mode} mode: {len(closed)} pair(s) close at worst case, "
            f"tightest {closed[0][3]} x {closed[0][4]} at {closed[0][0]:.2f} mm")


@pytest.mark.parametrize("part,sizes", [("roll_bracket", (3.0, 2.2)),
                                        ("shin", (3.0, 2.2)),
                                        ("thigh", (2.6, 2.2))])
def test_the_reported_peak_is_a_number_and_not_a_mesh_artifact(part, sizes):
    """A peak that moves under refinement is not the part's stress.

    Both directions are bad. Growing without limit is a singularity - a sharp
    re-entrant corner, where the exact answer is infinite. Collapsing is a
    coarse mesh reading spurious stress off badly-shaped elements. The thigh
    did the second: 135 MPa at 3.0 mm against 17 at 2.2, which arrived in the
    table as 143% of PA6-CF and a failure that did not exist.

    Each part is bracketed around ITS OWN default size, not a fixed pair. The
    thigh's default is 2.2 precisely because 3.0 is wrong for it, so testing it
    at 3.0 would assert that a mesh we already rejected disagrees with the one
    we chose - true, and useless.

    The threshold is tied to how much the answer matters. Refinement is not
    free here: cad/fea.MAX_NODES caps the direct solve, and the thigh cannot be
    meshed finer than 2.2 without going past it. At 30% of allowable, a 10%
    uncertainty on its peak changes no decision; at 88%, like the shin, it
    changes everything. So a part near its limit must converge tightly, and a
    part with a large margin need only be BOUNDED - which still catches the
    six-fold jump that started all this.
    """
    from cad import stress

    out = stress.converge(part, sizes=sizes, verbose=False)
    coarse, fine = out[0][2], out[-1][2]
    assert fine > 0, f"{part}: the fine solve returned nothing"
    change = abs(fine - coarse) / max(coarse, fine)
    util = fine / 57.0                       # PA6-CF, knocked down
    limit = 0.10 if util > 0.60 else 0.35
    assert change < limit, (
        f"{part}: peak moved {change:.1%} between {sizes[0]} and {sizes[-1]} mm "
        f"({coarse:.1f} -> {fine:.1f} MPa) at {util:.0%} of allowable, over the "
        f"{limit:.0%} allowed at that margin. Whichever way it moved, the "
        f"utilisation quoted for this part is not a number")


def test_nothing_fails_in_fatigue_in_the_material_we_would_order():
    """Everything else here is static ultimate, but stage 2 asks for 20
    consecutive transitions and the intent is thousands.

    The load case is the point: the static table reports whichever case is
    worst, and for the thigh and shin that is a TIPOVER, which is a one-off
    event and not a fatigue cycle. What repeats is the flip, so the flip at 1x
    is what is solved against the endurance limit.
    """
    from cad import stress

    rows = stress.fatigue(verbose=False)
    assert rows, "the fatigue pass produced nothing"
    bad = [r for r in rows if r[1] == "PA6-CF" and max(r[2], r[3]) >= 1.0]
    assert not bad, f"PA6-CF fails in fatigue on {[r[0] for r in bad]}"


def test_the_roll_joint_needs_a_service_loop():
    """The ankle rolls 90 degrees, so the ports either side of it move relative
    to each other and the cable between them changes length. That slack has to
    exist in both poses or it goes tight in one and snags in the other. Nobody
    had ever computed it."""
    import cad.wiring as W

    rows = W.slack(verbose=False)
    worst = max(abs(r[3]) for r in rows)
    assert worst > 1.0, (
        "no run changes length across the flip, which cannot be right for a "
        "joint that turns 90 degrees - the port model is probably broken")
    assert worst < 25.0, f"a run moves {worst:.0f} mm; that needs a real route"


def test_the_cable_channels_are_cut_and_the_cables_fit():
    """Cutting them found a real bug and a real limit, and both are pinned.

    THE BUG: the left thigh's channel cleared its cable and the right's did
    not. runs() had been choosing each side's connector pairing independently
    by "whichever two ports are closest", and on a mirrored assembly that
    picked a DIFFERENT pair - y = -10.0 in the left part's frame against -0.4
    in the right's. A channel cut into the left and mirrored onto the right
    lined up with nothing. The right chain is now the left one mirrored.

    THE LIMIT: the roll-to-wheel run crosses the ankle roll joint, so its far
    end is on a bracket that turns 90 degrees. Its straight line passes through
    the WHEEL in foot mode, and you cannot cut a channel in a part that spins.
    That run needs a routed path tucked inboard with a clip, taking up the
    6.3 mm of length change as a service loop - a design task, not a groove.
    It is expected to remain blocked here until that route exists.
    """
    import cad.wiring as W

    for mode in ("wheel", "foot"):
        rows = W.check(mode, verbose=False)
        assert len(rows) == 8, f"{mode}: expected 8 runs, got {len(rows)}"
        blocked = {r["label"]: r for r in rows if r["blocked"] > W.TOUCH}
        # Symmetry: whatever is true of one side must be true of the other.
        left = {k for k in blocked if k.endswith("_l")}
        right = {k for k in blocked if k.endswith("_r")}
        assert len(left) == len(right), (
            f"{mode}: {len(left)} left runs blocked but {len(right)} right - "
            f"the two sides have stopped being mirror images")
        for label, r in blocked.items():
            assert "vrollsv" in label, (
                f"{mode}: {label} is blocked by {r['by']}, and it is not the "
                f"roll-to-wheel run - that is a channel that needs cutting")
            # And the only thing left in its way should be the wheel it has to
            # route around, plus a little of the bracket.
            assert set(r["by"]) <= {"wheel_l", "wheel_r",
                                    "rollbracket_l", "rollbracket_r"}, (
                f"{mode}: {label} now hits {set(r['by'])}, which is more than "
                f"the wheel envelope it was known to")
