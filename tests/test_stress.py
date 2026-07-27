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
