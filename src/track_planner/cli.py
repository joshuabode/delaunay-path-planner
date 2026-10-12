from __future__ import annotations

import argparse
from collections.abc import Iterable
from sys import stdin

from .beam_search import beam_search
from .config import CLI_TRIANGULATION_COLOR, GRAPH_COLOR, HIGHLIGHT_COLOR
from .geometry import Point
from .triangulation import triangulate
from .visualiser import Visualiser


def parse_input(lines: Iterable[str]) -> tuple[Point, Point, set[Point]]:
    """Parse the car start, velocity, and sampled points from input lines."""
    rows = [line.strip() for line in lines if line.strip()]
    if len(rows) < 2:
        raise ValueError("Expected at least a start point and velocity.")

    start_x, start_y = map(float, rows[0].split())
    vel_x, vel_y = map(float, rows[1].split())
    points: set[Point] = set()

    for row in rows[2:]:
        x, y = map(float, row.split())
        points.add(Point(x, y))

    return Point(start_x, start_y), Point(vel_x, vel_y), points


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments for the track planner."""
    parser = argparse.ArgumentParser(description="Plan a track through cone points.")
    parser.add_argument(
        "--show-graph",
        action="store_true",
        help="Also draw the triangulation adjacency graph over the rendered path.",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)
    start, velocity, points = parse_input(stdin)
    triangulation = triangulate(points)
    path = beam_search(start, velocity, triangulation)

    Visualiser.add_triangulation(triangulation, CLI_TRIANGULATION_COLOR)
    Visualiser.add_point_cloud(points)
    if args.show_graph:
        Visualiser.add_graph(triangulation, GRAPH_COLOR)
    Visualiser.add_edges(path, HIGHLIGHT_COLOR)
    Visualiser.add_point_cloud({start}, HIGHLIGHT_COLOR)
    Visualiser.show()


if __name__ == "__main__":
    main()
