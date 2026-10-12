from track_planner.circumcircle import Circumcircle, ccw
from track_planner.geometry import Point


def test_ccw_reorders_clockwise_vertices():
    clockwise = (Point(0, 0), Point(0, 2), Point(2, 0))

    assert ccw(*clockwise) == (clockwise[0], clockwise[2], clockwise[1])


def test_circumcircle_contains_inside_and_boundary_points_but_not_outside():
    circle = Circumcircle(Point(0, 0), Point(2, 0), Point(0, 2))

    assert Point(0.5, 0.5) in circle
    assert Point(2, 0) in circle
    assert Point(3, 3) not in circle


def test_circumcircle_result_is_independent_of_input_orientation():
    counter_clockwise = Circumcircle(Point(0, 0), Point(2, 0), Point(0, 2))
    clockwise = Circumcircle(Point(0, 0), Point(0, 2), Point(2, 0))
    inside = Point(0.5, 0.5)
    outside = Point(3, 3)

    assert (inside in counter_clockwise) == (inside in clockwise)
    assert (outside in counter_clockwise) == (outside in clockwise)
