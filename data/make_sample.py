"""Regenerates data/sample_objects.pcd, the synthetic scene used in the examples.

Three separate objects plus a few stray points, so that clustering has an
obvious right answer: a hollow box (2000 points), a sphere (1200), a small
solid cube (400) and 30 stray points. Run from the repository root:

    python data/make_sample.py
"""

import math
import random
import tempfile
from pathlib import Path

import pclapp

rng = random.Random(42)  # fixed seed: the file is the same on every run


def jitter(value: float, sigma: float = 0.004) -> float:
    """Adds a little sensor-like noise."""
    return value + rng.gauss(0.0, sigma)


points = []

# A box of 1.2 x 0.8 x 0.6 with points on its six faces.
for _ in range(2000):
    x, y, z = rng.uniform(0, 1.2), rng.uniform(0, 0.8), rng.uniform(0, 0.6)
    face = rng.randrange(6)
    if face == 0:
        x = 0.0
    elif face == 1:
        x = 1.2
    elif face == 2:
        y = 0.0
    elif face == 3:
        y = 0.8
    elif face == 4:
        z = 0.0
    else:
        z = 0.6
    points.append((jitter(x), jitter(y), jitter(z)))

# A sphere of radius 0.4 around (3.0, 0.5, 0.4).
for _ in range(1200):
    u, angle = rng.uniform(-1, 1), rng.uniform(0, 2 * math.pi)
    r = math.sqrt(1 - u * u)
    points.append(
        (
            jitter(3.0 + 0.4 * r * math.cos(angle)),
            jitter(0.5 + 0.4 * r * math.sin(angle)),
            jitter(0.4 + 0.4 * u),
        )
    )

# A small solid cube with edge 0.3 at (1.5, 2.5, 0).
for _ in range(400):
    points.append(
        (
            jitter(1.5 + rng.uniform(0, 0.3)),
            jitter(2.5 + rng.uniform(0, 0.3)),
            jitter(rng.uniform(0, 0.3)),
        )
    )

# Stray points above the objects, too far apart to form a cluster.
for _ in range(30):
    points.append((rng.uniform(-1, 5), rng.uniform(-1, 4), rng.uniform(1.5, 3.0)))

rng.shuffle(points)

# Write the points as text, then let the library convert them to binary PCD.
with tempfile.TemporaryDirectory() as tmp:
    text_file = Path(tmp) / "sample.txt"
    text_file.write_text("".join(f"{x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in points))
    cloud = pclapp.load(text_file)

target = Path(__file__).with_name("sample_objects.pcd")
pclapp.save(cloud, target)
print(f"wrote {target} with {len(cloud)} points")
