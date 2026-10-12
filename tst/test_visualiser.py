from collections.abc import Iterable
from io import BytesIO
from typing import cast

import matplotlib.pyplot as plt
import pytest

from track_planner.geometry import Edge, Point
from track_planner.triangulation import Triangle
from track_planner.visualiser import Visualiser


@pytest.fixture(autouse=True)
def close_figures():
    yield
    plt.close("all")


def test_visualiser_draws_points_edges_and_triangulation():
    triangle = Triangle(Point(0, 0), Point(2, 0), Point(0, 2))
    points = {Point(0, 0), Point(1, 1)}

    Visualiser.add_point_cloud(points)
    Visualiser.add_edges([Edge(Point(0, 0), Point(1, 1))])
    Visualiser.add_triangulation({triangle})

    axes = plt.gca()
    assert len(axes.collections) == 1
    assert len(axes.lines) == 4
    offsets = cast(
        Iterable[Iterable[float]],
        axes.collections[0].get_offsets(),
    )
    assert {
        tuple(offset) for offset in offsets
    } == {(point.x, point.y) for point in points}
    edge_x = cast(Iterable[float], axes.lines[0].get_xdata())
    edge_y = cast(Iterable[float], axes.lines[0].get_ydata())
    assert list(edge_x) == [0, 1]
    assert list(edge_y) == [0, 1]

    expected_triangle_edges = {
        frozenset({(0, 0), (2, 0)}),
        frozenset({(0, 0), (0, 2)}),
        frozenset({(2, 0), (0, 2)}),
    }
    rendered_triangle_edges = set()
    for line in axes.lines[1:]:
        x_data = cast(Iterable[float], line.get_xdata())
        y_data = cast(Iterable[float], line.get_ydata())
        rendered_triangle_edges.add(frozenset(zip(x_data, y_data)))
    assert rendered_triangle_edges == expected_triangle_edges

    image = BytesIO()
    plt.savefig(image, format="png")
    assert image.getvalue().startswith(b"\x89PNG\r\n\x1a\n")
