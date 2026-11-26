import pytest

from simulation.groundtruth.road.config import Config

# (helper, regulation interval) pairs
HELPERS = [
    (Config.get_prio_sign_dist, Config.PRIORITY_SIGN_DISTANCE_INTERVALL),
    (Config.get_turn_sign_dist, Config.TURN_SIGN_DISTANCE_INTERVALL),
    (Config.get_surface_mark_dist, Config.SURFACE_MARKING_DISTANCE_INTERVALL),
    (Config.get_sign_road_padding, Config.SIGN_ROAD_PADDING_INTERVALL),
]
HELPER_IDS = [helper.__name__ for helper, _ in HELPERS]


@pytest.mark.parametrize(("helper", "interval"), HELPERS, ids=HELPER_IDS)
@pytest.mark.parametrize("rand", [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0])
def test_helper_stays_inside_regulation_interval(helper, interval, rand):
    lower, upper = interval
    assert lower <= helper(rand) <= upper


@pytest.mark.parametrize(("helper", "interval"), HELPERS, ids=HELPER_IDS)
def test_helper_hits_interval_bounds_and_midpoint(helper, interval):
    lower, upper = interval
    assert helper(0.0) == pytest.approx(lower)
    assert helper(1.0) == pytest.approx(upper)
    assert helper() == pytest.approx((lower + upper) / 2)


def test_priority_sign_distance_at_rand_one_is_regulation_maximum():
    # Intersection places priority/stop/yield signs with get_prio_sign_dist(1).
    # The old formula (lo + hi) * rand returned 0.8 m here instead of 0.5 m.
    assert Config.get_prio_sign_dist(1) == pytest.approx(0.5)
