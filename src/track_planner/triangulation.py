from math import isfinite

from .circumcircle import Circumcircle
from .config import SUPER_TRIANGLE_SCALE
from .geometry import Edge, Point


class Triangle:
    """Represents a triangle."""

    def __init__(self, v0: Point, v1: Point, v2: Point):
        self.v0: Point = v0
        self.v1: Point = v1
        self.v2: Point = v2
        self.edges: list[Edge] = [Edge(v0, v1), Edge(v0, v2), Edge(v1, v2)]
        self.circumcircle = Circumcircle(v0, v1, v2)

    def __contains__(self, p: Point):
        return p in (self.v0, self.v1, self.v2)

    def __repr__(self):
        return f"Triangle({self.v0}, {self.v1}, {self.v2})"


def super_triangle(points: set[Point]) -> Triangle:
    """Calculates a triangle large enough to enclose all points."""
    min_x = float("inf")
    min_y = float("inf")
    max_x = float("-inf")
    max_y = float("-inf")
    for point in points:
        min_x = min(min_x, point.x)
        min_y = min(min_y, point.y)
        max_x = max(max_x, point.x)
        max_y = max(max_y, point.y)

    dx = max_x - min_x
    dy = max_y - min_y

    mid_x = (min_x + max_x) / 2
    mid_y = (min_y + max_y) / 2

    d = max(dx, dy)
    return Triangle(
        Point(mid_x - SUPER_TRIANGLE_SCALE * d, mid_y - d),
        Point(mid_x + SUPER_TRIANGLE_SCALE * d, mid_y - d),
        Point(mid_x, mid_y + SUPER_TRIANGLE_SCALE * d),
    )


Triangulation = set[Triangle]


def triangulate(points: set[Point]) -> Triangulation:
    """Calculates the Delaunay triangulation of a set of points using Bowyer-Watson."""
    if len(points) < 3:
        raise ValueError("At least three distinct points are required.")
    if any(not isfinite(point.x) or not isfinite(point.y) for point in points):
        raise ValueError("Point coordinates must be finite.")

    ordered_points = sorted(points)
    origin, baseline = ordered_points[:2]
    dx = baseline.x - origin.x
    dy = baseline.y - origin.y
    if all(
        dx * (point.y - origin.y) == dy * (point.x - origin.x)
        for point in ordered_points[2:]
    ):
        raise ValueError("Input points must not all be collinear.")

    st = super_triangle(points)
    triangulation: set[Triangle] = {st}
    for point in points:
        bad_triangles = set()
        for triangle in triangulation:
            if point in triangle.circumcircle:
                bad_triangles.add(triangle)

        polygon: list[Edge] = []

        for triangle in bad_triangles:
            for edge in triangle.edges:
                shared = any(
                    edge == other_edge
                    for other_triangle in bad_triangles
                    if other_triangle is not triangle
                    for other_edge in other_triangle.edges
                )
                if not shared:
                    polygon.append(edge)

        for triangle in bad_triangles:
            triangulation.remove(triangle)

        for edge in polygon:
            new_tri = Triangle(edge.v0, edge.v1, point)
            triangulation.add(new_tri)

    for triangle in triangulation.copy():
        if triangle.v0 in st or triangle.v1 in st or triangle.v2 in st:
            triangulation.remove(triangle)

    return triangulation


def to_edge_set(triangulation: Triangulation) -> set[Edge]:
    edges = set()
    for tri in triangulation:
        for edge in tri.edges:
            edges.add(edge)

    return edges
