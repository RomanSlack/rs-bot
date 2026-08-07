"""Is anything held by nothing?  See cad/floating.py.

The robot-wide connectivity checks answer "is this one object" and have said
yes throughout, including while four servos sat 0.4 mm off the parts they are
supposed to seat against. These tests are the local version of the question.
"""

import build123d as bd
import pytest

import cad.floating as fl


@pytest.fixture(scope="module")
def scene():
    return fl._scene("wheel")


# Nothing floats. The four servos that did - both hips 0.4 mm off the chassis,
# both roll cradles 0.4 mm off the yoke - got a seating face on 2026-08-05, and
# they are asserted seated in test_the_servos_that_do_seat_still_seat below.
#
# PINNED AS A LIST, not as a count. A count would let one fault be swapped for
# another silently, and the point of this file is that a servo held by nothing
# is invisible to every other check here.
KNOWN = set()


def test_nothing_new_is_floating(scene):
    got = {r["name"] for r in fl.floaters("wheel", scene)}
    assert got == KNOWN, (
        f"floating changed: new {sorted(got - KNOWN)}, "
        f"fixed {sorted(KNOWN - got)} (update KNOWN if fixed)")


def test_the_servos_that_do_seat_still_seat(scene):
    """Every servo bears on its host at 0.000, so this file cannot pass by
    everything being equally broken. The first two seated from the start; the
    four hips and rolls got their face on 2026-08-05 and are pinned here so a
    regression fails loudly instead of quietly re-floating."""
    for servo, host in (("vkneesv_l", "thigh_l"), ("vwhlsv_l", "rollbracket_l"),
                        ("vhipsv1", "torso"), ("vhipsv-1", "torso"),
                        ("vrollsv_l", "ankle_l"), ("vrollsv_r", "ankle_r")):
        assert fl._gap(scene[servo], scene[host]) == pytest.approx(0.0, abs=1e-6)


def test_every_connector_reaches_two_bodies(scene):
    """A shaft that touches one body drives nothing. Reach is transitive
    through other connectors, because a shaft touches its BEARING and not the
    yoke that bearing is pressed into."""
    assert fl.unreached("wheel", scene) == []


def test_it_can_actually_fail(scene):
    """Control. Push a seated servo off its host and see it reported.

    +y, and the direction is the whole test. Nudging the knee servo -y, +x or
    +z leaves the gap at 0.000 because the thigh WRAPS it: a cradle plus a
    cross-member plus a fork, so shoving it about mostly slides it along a face
    it is still touching. The first version of this control moved it -y 2 mm,
    got nothing, and looked exactly like a broken check.
    """
    broken = dict(scene)
    broken["vkneesv_l"] = bd.Pos(0, 2.0, 0) * scene["vkneesv_l"]
    fl._BB.clear()
    try:
        got = {r["name"] for r in fl.floaters("wheel", broken)}
        assert "vkneesv_l" in got
    finally:
        fl._BB.clear()
