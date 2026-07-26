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
    """Quiet must be the mildest case and the shove the worst, in every part.
    If the flip ever beat the shove it would mean the flip had gone wrong."""
    cases = loads.survey()
    for child in loads.CHILD.values():
        n = {c: np.linalg.norm(cases[c][child][0]) for c in cases}
        assert n["quiet"] < n["flip"] < n["shove"], f"{child}: {n}"


def test_roll_bracket_members_barely_touch():
    """The fault that made the roll bracket fail at 16x. Pinning it so the fix
    is verifiable: this must get much bigger, not silently stay small."""
    import build123d as bd
    import cad.ankle as ankle

    def box(spec):
        (x0, x1), (y0, y1), (z0, z1) = spec
        return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(
            x1 - x0, y1 - y0, z1 - z0)

    overlap = box(ankle.ROLL_ARM) & box(ankle.ROLL_TIE)
    bb = overlap.bounding_box()
    thinnest = min(bb.size.X, bb.size.Y, bb.size.Z)
    assert thinnest == pytest.approx(0.5, abs=0.01), (
        f"the arm/tie lap is now {thinnest:.2f} mm - if this was fixed, update "
        f"the number and re-run cad.stress")


def test_ankle_pitch_joint_is_not_drawn():
    """Pinning the second fault. The sim's ankle-pitch axis and the shin's
    bearing bore are 52.8 mm apart, and no part bridges the gap."""
    import build123d as bd
    import cad.ankle as ankle
    import cad.shin as shin

    gap = (shin.build()).distance_to(bd.Pos(0, 0, -110) * ankle.yoke())
    assert gap > 1.0, (
        f"shin and yoke are now {gap:.1f} mm apart - if the ankle-pitch joint "
        f"has been drawn, this test and docs/stress.md need updating")
