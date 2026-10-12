import pytest

from track_planner.geometry import Edge, Point


def test_point_supports_vector_arithmetic_and_string_conversion():
    point = Point(2, 4)

    assert point + Point(3, -1) == Point(5, 3)
    assert point - Point(1, 2) == Point(1, 2)
    assert point / 2 == Point(1, 2)
    assert str(point) == "(2,4)"


def test_points_order_by_coordinates():
    assert Point(0, 2) < Point(1, 0)
    assert Point(1, 0) < Point(1, 1)


def test_edge_equality_and_hash_are_independent_of_endpoint_order():
    first = Edge(Point(0, 0), Point(1, 1))
    reversed_edge = Edge(Point(1, 1), Point(0, 0))

    assert first == reversed_edge
    assert hash(first) == hash(reversed_edge)
    assert len({first, reversed_edge}) == 1


def test_edge_is_not_equal_to_an_unrelated_type():
    assert Edge(Point(0, 0), Point(1, 1)) != "edge"


def test_point_is_immutable():
    point = Point(0, 0)

    with pytest.raises(AttributeError):
        point.x = 1
