# Path Planner

A small Python project for generating and visualising a path for a robot to drive through a track outlined by cones.

The core of the project is a Delaunay triangulation of the cone positions, followed by a beam search over the resulting graph. The key implementation files are `src/track_planner/triangulation.py` and `src/track_planner/beam_search.py`.

### Core algorithm files

- `src/track_planner/triangulation.py`: Bowyer-Watson Delaunay triangulation
- `src/track_planner/beam_search.py`: cost-based beam search over the track graph
- `src/track_planner/geometry.py`: geometric primitives and vector helpers

### Motivation

This project was inspired by my work with Manchester Stinger Motorsports, my university’s Formula Student team. We use a similar approach to plan paths for an autonomous vehicle through a set of physical cones. Beam search is especially useful here because heuristic guidance fits well with the strict track-layout constraints of the competition.

### Cost model and heuristic weights

The search objective prefers smooth, continuous, and consistent routes:

- `DELTA_THETA_WEIGHT`: penalises rapid direction changes. Large steering corrections are unlikely to be the correct path, and even sharp corners are typically composed of many shorter cone-to-cone segments.
- `DIRECTION_REVERSAL_PENALTY`: strongly penalises reversing direction, discouraging U-turn-like behaviour.
- `STDEV_WIDTH_WEIGHT`: penalises variance in track width, encouraging a stable lane width across the route.
- `PATH_LENGTH_WEIGHT`: is negative, so longer paths are rewarded rather than penalised, favouring forward progress.

The cost is approximately:

```text
cost = DELTA_THETA_WEIGHT * average_turn_angle
    + STDEV_WIDTH_WEIGHT * width_stddev
    + PATH_LENGTH_WEIGHT * path_length
```

with an extra multiplier applied when the path reverses direction. This encourages long, straight, stable trajectories while still allowing the route to bend around cone geometry when necessary.

### Configuration

Search limits, cost weights, the super-triangle scale, and visualisation colours are centralised in `src/track_planner/config.py`.

## Run

```bash
PYTHONPATH=src python -m track_planner < inputs/straight.txt
```

To also overlay the triangulation graph over the route, use:

```bash
PYTHONPATH=src python -m track_planner --show-graph < inputs/straight.txt
```

This keeps the default view focused on the planned path, while `--show-graph` reveals the underlying adjacency graph used by the beam search.

The package can also be imported from Python:

```python
from track_planner import Point, beam_search, triangulate
```

## Tests

```bash
python -m pip install -r requirements-dev.txt
pytest -q
```
