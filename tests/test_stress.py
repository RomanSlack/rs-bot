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
