from pathlib import Path

import pytest

from track_planner.beam_search import beam_search
from track_planner.cli import parse_input
from track_planner.triangulation import triangulate


INPUTS_DIR = Path(__file__).resolve().parents[1] / "inputs"
SAMPLE_FILES = (
    "straight.txt",
    "track.txt",
    "track2.txt",
    "track3.txt",
    "comb.txt",
)


@pytest.mark.parametrize("filename", SAMPLE_FILES)
def test_sample_pipeline_returns_a_forward_non_repeating_path(filename):
    lines = (INPUTS_DIR / filename).read_text().splitlines()
    start, velocity, points = parse_input(lines)

    path = beam_search(start, velocity, triangulate(points))

    assert path
    assert path[0].v0 == start
    assert len(path) == len(set(path))
    first_direction = path[0].v1 - path[0].v0
    heading_dot_product = (
        first_direction.x * velocity.x + first_direction.y * velocity.y
    )
    assert heading_dot_product > 0
