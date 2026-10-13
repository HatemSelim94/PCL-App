"""Shared test data: synthetic clouds written as text files.

The geometry matches the C++ tests: cubes of 4 x 4 x 4 points on a 0.25 grid,
so every expected count is exact.
"""

import itertools
from pathlib import Path

import pytest


def cube_points(origin: float = 0.0, n: int = 4, spacing: float = 0.25):
    """n*n*n grid points with the cube's corner at (origin, origin, origin)."""
    return [
        (origin + x * spacing, origin + y * spacing, origin + z * spacing)
        for x, y, z in itertools.product(range(n), repeat=3)
    ]


def write_xyz(path: Path, points) -> Path:
    path.write_text("".join(f"{x} {y} {z}\n" for x, y, z in points))
    return path


@pytest.fixture
def cube_file(tmp_path: Path) -> Path:
    """One cube: 64 points spanning [0, 0.75] on each axis."""
    return write_xyz(tmp_path / "cube.txt", cube_points())


@pytest.fixture
def two_blobs_file(tmp_path: Path) -> Path:
    """Two cubes 10 units apart: 128 points, two obvious clusters."""
    return write_xyz(tmp_path / "two_blobs.txt", cube_points() + cube_points(origin=10.0))
