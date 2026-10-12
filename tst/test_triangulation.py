import pytest

from track_planner.geometry import Point
from track_planner.triangulation import Triangle, to_edge_set, triangulate


def test_triangle_has_three_edges_and_contains_only_its_vertices():
    points = (Point(0, 0), Point(2, 0), Point(0, 2))
    triangle = Triangle(*points)

    assert len(triangle.edges) == 3
    assert all(point in triangle for point in points)
    assert Point(1, 1) not in triangle


def test_triangulate_three_non_collinear_points():
    points = {Point(0, 0), Point(2, 0), Point(0, 2)}

    result = triangulate(points)

    assert len(result) == 1
    triangle = next(iter(result))
    assert {triangle.v0, triangle.v1, triangle.v2} == points


def test_triangulate_square_into_two_triangles_with_five_unique_edges():
    points = {
        Point(0, 0),
        Point(2, 0),
        Point(2, 2),
        Point(0, 2),
    }

    result = triangulate(points)
    vertices = {
        vertex
        for triangle in result
        for vertex in (triangle.v0, triangle.v1, triangle.v2)
    }

    assert len(result) == 2
    assert vertices == points
    assert len(to_edge_set(result)) == 5


def test_triangulate_general_position_points_satisfies_delaunay_invariants():
    points = {
        Point(0, 0),
        Point(4, 0),
        Point(4, 3),
        Point(0, 4),
        Point(1, 1),
    }

    result = triangulate(points)
    vertices = {
        vertex
        for triangle in result
        for vertex in (triangle.v0, triangle.v1, triangle.v2)
    }

    assert vertices == points
    assert len(result) == 4
    assert len(to_edge_set(result)) == 8
    assert all(
        point not in triangle.circumcircle
        for triangle in result
        for point in points - {triangle.v0, triangle.v1, triangle.v2}
    )


def test_triangulate_known_near_collinear_points_without_dropping_vertices():
    points = {
        Point(0, 0),
        Point(1, 1e-12),
        Point(2, 0),
        Point(1, 2),
    }

    result = triangulate(points)
    vertices = {
        vertex
        for triangle in result
        for vertex in (triangle.v0, triangle.v1, triangle.v2)
    }

    assert vertices == points


@pytest.mark.parametrize(
    "points",
    [
        pytest.param(set(), id="empty"),
        pytest.param({Point(0, 0)}, id="one-point"),
        pytest.param({Point(0, 0), Point(1, 1)}, id="two-points"),
    ],
)
def test_triangulate_rejects_fewer_than_three_points(points):
    with pytest.raises(ValueError, match="At least three distinct points"):
        triangulate(points)


def test_triangulate_rejects_collinear_points():
    points = {Point(0, 0), Point(1, 1), Point(2, 2)}

    with pytest.raises(ValueError, match="must not all be collinear"):
        triangulate(points)


@pytest.mark.parametrize("coordinate", [float("inf"), float("nan")])
def test_triangulate_rejects_non_finite_coordinates(coordinate):
    points = {
        Point(0, 0),
        Point(1, 0),
        Point(0, 1),
        Point(coordinate, 2),
    }

    with pytest.raises(ValueError, match="coordinates must be finite"):
        triangulate(points)
