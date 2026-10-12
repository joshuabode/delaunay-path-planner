from collections.abc import Iterable

import matplotlib.pyplot as plt

from .beam_search import midpoint, to_adjacency_map
from .config import (
    EDGE_COLOR,
    GRAPH_COLOR,
    POINT_CLOUD_COLOR,
    TRIANGULATION_COLOR,
)
from .geometry import Edge, Point
from .triangulation import Triangulation, to_edge_set


class Visualiser:
    """Uses matplotlib to show point clouds and triangulations."""

    @staticmethod
    def add_point_cloud(points: set[Point], color: str = POINT_CLOUD_COLOR):
        xs = [point.x for point in points]
        ys = [point.y for point in points]
        plt.scatter(xs, ys, c=color)

    @staticmethod
    def add_edges(edges: Iterable[Edge], color: str = EDGE_COLOR):
        for edge in edges:
            xs = [edge.v0.x, edge.v1.x]
            ys = [edge.v0.y, edge.v1.y]
            plt.plot(xs, ys, c=color)

    @classmethod
    def add_triangulation(
        cls,
        triangulation: Triangulation,
        color: str = TRIANGULATION_COLOR,
    ):
        cls.add_edges(to_edge_set(triangulation), color)

    @classmethod
    def add_graph(cls, triangulation: Triangulation, color: str = GRAPH_COLOR):
        edges = []
        adjacency = to_adjacency_map(triangulation)
        for edge, children in adjacency.items():
            for child in children:
                edges.append(Edge(midpoint(edge), midpoint(child)))

        cls.add_edges(edges, color)

    @staticmethod
    def show():
        plt.axis("equal")
        plt.show()

