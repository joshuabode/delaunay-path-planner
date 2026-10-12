import pytest

from track_planner.cli import parse_args, parse_input
from track_planner.geometry import Point


def test_parse_args_tracks_show_graph_flag():
    args = parse_args(["--show-graph"])

    assert args.show_graph is True


def test_parse_input_reads_start_velocity_and_sample_points():
    lines = ["1 2", "3 4", "5 6", "7 8"]

    start, velocity, points = parse_input(lines)

    assert start == Point(1, 2)
    assert velocity == Point(3, 4)
    assert points == {Point(5, 6), Point(7, 8)}


def test_parse_input_ignores_blank_lines_and_deduplicates_points():
    lines = ["", " 1 2 ", "\n", "3 4", "5 6", "5 6"]

    start, velocity, points = parse_input(lines)

    assert start == Point(1, 2)
    assert velocity == Point(3, 4)
    assert points == {Point(5, 6)}


def test_parse_input_requires_start_and_velocity():
    with pytest.raises(ValueError, match="start point and velocity"):
        parse_input(["1 2"])


@pytest.mark.parametrize("row", ["1", "1 2 3", "not-a-number 2"])
def test_parse_input_rejects_malformed_rows(row):
    with pytest.raises(ValueError):
        parse_input(["0 0", "1 0", row])
