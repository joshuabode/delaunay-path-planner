from dataclasses import dataclass


@dataclass(frozen=True)
class Point:
    """Defines a point on a 2D plane. Supports addition and subtraction."""

    x: float
    y: float

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, other):
        return Point(self.x / other, self.y / other)

    def __str__(self):
        return f"({self.x},{self.y})"


@dataclass(frozen=True)
class Edge:
    """Defines an edge on a 2D plane."""

    v0: Point
    v1: Point

    def __lt__(self, other):
        left = sorted((self.v0, self.v1))
        right = sorted((other.v0, other.v1))
        return left < right

    def __eq__(self, other):
        if not isinstance(other, Edge):
            return NotImplemented
        return {self.v0, self.v1} == {other.v0, other.v1}

    def __hash__(self):
        return hash(frozenset({self.v0, self.v1}))
