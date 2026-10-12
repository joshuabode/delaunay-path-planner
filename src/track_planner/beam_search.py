from collections import defaultdict
from collections.abc import Iterable
from heapq import heappop, heappush
from math import acos, sqrt

from .config import (
    BEAM_SEARCH_DEPTH,
    BEAM_WIDTH,
    DELTA_THETA_WEIGHT,
    DIRECTION_REVERSAL_PENALTY,
    PATH_LENGTH_WEIGHT,
    STDEV_WIDTH_WEIGHT,
)
from .geometry import Edge, Point
from .triangulation import Triangle

Vector = Point


def midpoint(edge: Edge) -> Point:
    """Calculates the midpoint of an edge."""
    return edge.v0 + (edge.v1 - edge.v0) / 2


def d2(v0: Point, v1: Point) -> float:
    """Calculates the squared Euclidean distance between two points."""
    dx = v0.x - v1.x
    dy = v0.y - v1.y
    return dx * dx + dy * dy


def clip(val: float, _min: float, _max: float):
    """Bounds a value in the interval [_min, _max]."""
    if val < _min:
        return _min
    if val > _max:
        return _max
    return val


def stdev(vals: list[float]) -> float:
    """Computes the sample standard deviation of a list of values."""
    if len(vals) < 2:
        return 0.0
    mean = sum(vals) / len(vals)
    devs = 0.0
    for val in vals:
        devs += (val - mean) * (val - mean)

    return sqrt(devs / (len(vals) - 1))


def angle_between(v0: Vector, v1: Vector) -> float:
    """Computes the angle between two vectors."""
    numerator = v0.x * v1.x + v0.y * v1.y
    denominator = (
        sqrt(v0.x * v0.x + v0.y * v0.y)
        * sqrt(v1.x * v1.x + v1.y * v1.y)
    )
    if denominator == 0:
        raise ValueError("Cannot calculate an angle for a zero-length vector.")
    return acos(clip(numerator / denominator, -1, 1))


def to_adjacency_map(triangulation: set[Triangle]) -> dict[Edge, set[Edge]]:
    """Represents a graph using an adjacency map for path search from a triangulation."""
    edge_map = defaultdict(set)
    for triangle in triangulation:
        for edge in triangle.edges:
            for other_edge in triangle.edges:
                if other_edge != edge:
                    edge_map[edge].add(other_edge)
    return edge_map


def closest_edge(position: Point, edges: Iterable[Edge]) -> Edge:
    """Finds the closest edge to a point by Euclidean distance."""
    closest_distance_sq = float("inf")
    closest = None
    for edge in edges:
        current_distance_sq = d2(midpoint(edge), position)
        if current_distance_sq < closest_distance_sq:
            closest_distance_sq = current_distance_sq
            closest = edge
    if closest is None:
        raise ValueError("No edges were provided to search from.")
    return closest


def cost(
    initial_velocity: Vector,
    path: list[Edge],
    position: Point,
) -> float:
    """Score a candidate path.

    The planner prefers low-turning, consistent-width, and long-distance paths
    by combining three terms:

    1. average turn angle: penalises sharp steering changes and frequent
       direction changes along the trajectory;
    2. width variance: penalises large fluctuations in the local track width;
    3. path length: weighted negatively so that longer, forward-progressing
       routes are preferred.

    The direction reversal penalty multiplies a turn cost when the path would
    reverse travel direction, which strongly discourages U-turn-like behaviour.
    See src/track_planner/config.py for the tuned constants.
    """
    if not path:
        return 0.0

    total_delta_theta = 0.0
    trajectory = initial_velocity
    path_directions: list[Vector] = [midpoint(path[0]) - position]

    for i in range(1, len(path)):
        path_directions.append(midpoint(path[i]) - midpoint(path[i - 1]))

    for next_trajectory in path_directions:
        turn_angle = angle_between(trajectory, next_trajectory)
        direction_bias = 1.0

        if trajectory.x * next_trajectory.x + trajectory.y * next_trajectory.y < 0:
            direction_bias = DIRECTION_REVERSAL_PENALTY

        total_delta_theta += direction_bias * turn_angle
        trajectory = next_trajectory

    track_widths = [sqrt(d2(edge.v0, edge.v1)) for edge in path]

    return (
        DELTA_THETA_WEIGHT * total_delta_theta / max(len(path_directions), 1)
        + STDEV_WIDTH_WEIGHT * stdev(track_widths)
        + PATH_LENGTH_WEIGHT * len(path)
    )


def compute_path(position: Point, edges: list[Edge]) -> list[Edge]:
    """Calculates the path from the ordered list of triangulation edges that it goes through."""
    if not edges:
        raise ValueError("At least one edge is required to compute a path.")

    path = [Edge(position, midpoint(edges[0]))]
    for i in range(1, len(edges)):
        path.append(Edge(midpoint(edges[i - 1]), midpoint(edges[i])))
    return path


def beam_search(
    position: Point,
    velocity: Vector,
    triangulation: set[Triangle],
) -> list[Edge]:
    """Finds the optimal path based on a cost function."""
    edge_map = to_adjacency_map(triangulation)
    start_edge = closest_edge(position, edge_map.keys())
    beam: list[tuple[float, list[Edge]]] = [(0.0, [start_edge])]

    best_cost = float("inf")
    best_candidate: list[Edge] = []

    for _ in range(BEAM_SEARCH_DEPTH):
        if not beam:
            break

        candidates: list[tuple[float, list[Edge]]] = []
        for candidate_cost, path in beam:
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_candidate = path

            node = path[-1]
            visited = set(path)
            for child in edge_map[node]:
                if child in visited:
                    continue
                next_path = path + [child]
                next_cost = cost(velocity, next_path, position)
                if next_cost < best_cost:
                    best_cost = next_cost
                    best_candidate = next_path
                heappush(candidates, (next_cost, next_path))

        beam = []
        for _ in range(min(BEAM_WIDTH, len(candidates))):
            beam.append(heappop(candidates))

    if not best_candidate:
        raise ValueError("No valid path was found from the starting position.")

    return compute_path(position, best_candidate)
