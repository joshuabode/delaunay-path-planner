from importlib import import_module

import pytest

from track_planner.beam_search import (
    beam_search,
    angle_between,
    closest_edge,
    compute_path,
    cost,
    midpoint,
    to_adjacency_map,
)
from track_planner.geometry import Edge, Point
from track_planner.triangulation import Triangle


def test_closest_edge_selects_nearest_midpoint():
    near = Edge(Point(0, 0), Point(2, 0))
    far = Edge(Point(8, 0), Point(10, 0))

    assert closest_edge(Point(0.5, 0), [far, near]) == near


def test_adjacency_map_connects_edges_in_the_same_triangle():
    triangle = Triangle(Point(0, 0), Point(2, 0), Point(0, 2))

    adjacency = to_adjacency_map({triangle})

    assert set(adjacency) == set(triangle.edges)
    assert all(
        children == set(triangle.edges) - {edge}
        for edge, children in adjacency.items()
    )


def test_compute_path_connects_vehicle_position_to_edge_midpoints():
    position = Point(-1, 0)
    edges = [
        Edge(Point(0, 0), Point(2, 0)),
        Edge(Point(2, 0), Point(4, 0)),
    ]

    path = compute_path(position, edges)

    assert path == [
        Edge(position, midpoint(edges[0])),
        Edge(midpoint(edges[0]), midpoint(edges[1])),
    ]


def test_compute_path_rejects_no_edges():
    with pytest.raises(ValueError, match="At least one edge"):
        compute_path(Point(0, 0), [])


def test_cost_prefers_path_aligned_with_initial_velocity():
    forward = [
        Edge(Point(0, 0), Point(2, 0)),
        Edge(Point(2, 0), Point(4, 0)),
    ]
    backward = [
        Edge(Point(0, 0), Point(-2, 0)),
        Edge(Point(-2, 0), Point(-4, 0)),
    ]

    assert cost(Point(1, 0), forward, Point(0, 0)) < cost(
        Point(1, 0), backward, Point(0, 0)
    )


def test_cost_is_finite_for_a_single_edge():
    path = [Edge(Point(0, 0), Point(2, 0))]

    assert cost(Point(1, 0), path, Point(-1, 0)) == pytest.approx(-1.1)


@pytest.mark.parametrize(
    ("first", "second"),
    [
        (Point(0, 0), Point(1, 0)),
        (Point(1, 0), Point(0, 0)),
    ],
)
def test_angle_between_rejects_zero_length_vectors(first, second):
    with pytest.raises(ValueError, match="zero-length vector"):
        angle_between(first, second)


def test_beam_search_starts_at_position_and_does_not_revisit_edges():
    points = [
        Point(0, 0),
        Point(1, 0),
        Point(0, 1),
        Point(1, 1),
        Point(2, 0),
        Point(2, 1),
    ]
    triangulation = {
        Triangle(points[0], points[1], points[2]),
        Triangle(points[1], points[3], points[2]),
        Triangle(points[1], points[3], points[4]),
        Triangle(points[1], points[4], points[5]),
        Triangle(points[1], points[2], points[3]),
    }
    position = Point(0.2, 0.2)

    path = beam_search(position, Point(1, 0), triangulation)

    assert path
    assert path[0].v0 == position
    assert len(path) == len(set(path))


def test_beam_search_rejects_empty_triangulation():
    with pytest.raises(ValueError, match="No edges"):
        beam_search(Point(0, 0), Point(1, 0), set())


def test_beam_search_prunes_frontier_globally(monkeypatch):
    search = import_module("track_planner.beam_search")
    edges = [Edge(Point(index, 0), Point(index, 1)) for index in range(11)]
    (
        start,
        left,
        right,
        left_first,
        left_second,
        right_first,
        right_second,
        left_end_first,
        left_end_second,
        right_end_first,
        right_end_second,
    ) = edges
    adjacency = {
        start: {left, right},
        left: {left_first, left_second},
        right: {right_first, right_second},
        left_first: {left_end_first},
        left_second: {left_end_second},
        right_first: {right_end_first},
        right_second: {right_end_second},
    }
    scores = {
        left: 10,
        right: 20,
        left_first: 1,
        left_second: 2,
        right_first: 3,
        right_second: 4,
        left_end_first: -1,
        left_end_second: 0,
        right_end_first: 5,
        right_end_second: 6,
    }
    evaluated_paths = []

    def score_path(_velocity, path, _position):
        evaluated_paths.append(tuple(path))
        return scores[path[-1]]

    monkeypatch.setattr(
        search,
        "to_adjacency_map",
        lambda _triangulation: adjacency,
    )
    monkeypatch.setattr(
        search,
        "closest_edge",
        lambda _position, _edges: start,
    )
    monkeypatch.setattr(search, "cost", score_path)
    monkeypatch.setattr(search, "compute_path", lambda _position, path: path)
    monkeypatch.setattr(search, "BEAM_WIDTH", 2)
    monkeypatch.setattr(search, "BEAM_SEARCH_DEPTH", 3)

    result = search.beam_search(Point(-1, 0), Point(1, 0), set())

    expanded_parents = {
        path[-2] for path in evaluated_paths if len(path) == 4
    }
    assert expanded_parents == {left_first, left_second}
    assert result[-1] == left_end_first
