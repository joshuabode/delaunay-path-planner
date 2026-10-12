from .geometry import Point


class Circumcircle:
    """A circle defined by three points lying on its circumference."""

    def __init__(self, v0: Point, v1: Point, v2: Point):
        self.v0, self.v1, self.v2 = ccw(v0, v1, v2)

    def __contains__(self, p: Point):
        a = self.v0 - p
        b = self.v1 - p
        c = self.v2 - p

        a2 = a.x * a.x + a.y * a.y
        b2 = b.x * b.x + b.y * b.y
        c2 = c.x * c.x + c.y * c.y

        det = (
            (a.x * (b.y * c2 - c.y * b2))
            - (a.y * (b.x * c2 - c.x * b2))
            + (a2 * (b.x * c.y - b.y * c.x))
        )
        return det >= 0


def ccw(v0: Point, v1: Point, v2: Point) -> tuple[Point, Point, Point]:
    """Returns the inputs in counter-clockwise order."""
    signed_area = (
        (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x)
    )
    if signed_area < 0:
        return (v0, v2, v1)
    return (v0, v1, v2)
