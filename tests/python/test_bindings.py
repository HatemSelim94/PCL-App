"""Tests for the pybind11 bindings (pclapp._core)."""

import pytest

import pclapp


def test_load_reports_size_and_point_type(cube_file):
    cloud = pclapp.load(cube_file)
    assert len(cloud) == 64
    assert not cloud.has_intensity
    assert repr(cloud) == "<pclapp.Cloud 64 points, XYZ>"

    assert pclapp.load(str(cube_file), intensity=True).has_intensity  # str paths work too


def test_intensity_column_is_optional_in_text_files(tmp_path):
    file = tmp_path / "points.txt"
    file.write_text("0 0 0 7\n\n1 2 3\n")

    cloud = pclapp.load(file, intensity=True)
    assert len(cloud) == 2
    assert cloud.has_intensity


def test_summary_has_count_bounding_box_and_centroid(cube_file):
    summary = pclapp.load(cube_file).summary()
    assert summary == {
        "point_count": 64,
        "has_intensity": False,
        "bounding_box": {"min": [0.0, 0.0, 0.0], "max": [0.75, 0.75, 0.75]},
        "centroid": [0.375, 0.375, 0.375],
    }


def test_summary_of_an_empty_cloud_has_no_extent(tmp_path):
    empty = tmp_path / "empty.txt"
    empty.write_text("")

    summary = pclapp.load(empty).summary()
    assert summary["point_count"] == 0
    assert summary["bounding_box"] is None
    assert summary["centroid"] is None


def test_voxel_downsample_returns_a_new_smaller_cloud(cube_file):
    cloud = pclapp.load(cube_file)

    small = pclapp.voxel_downsample(cloud, leaf_size=0.5)  # two voxels per axis
    assert len(small) == 8
    assert len(cloud) == 64  # the input is untouched


def test_euclidean_cluster_separates_two_blobs(two_blobs_file):
    cloud = pclapp.load(two_blobs_file)

    clusters = pclapp.euclidean_cluster(cloud, tolerance=0.3, min_size=1, max_size=1000)
    assert [len(c) for c in clusters] == [64, 64]

    centroids = sorted(c.summary()["centroid"] for c in clusters)
    assert centroids == [[0.375, 0.375, 0.375], [10.375, 10.375, 10.375]]


@pytest.mark.parametrize("name", ["out.pcd", "out.txt"])
def test_save_and_load_round_trip(cube_file, tmp_path, name):
    cloud = pclapp.load(cube_file)

    pclapp.save(cloud, tmp_path / name)
    assert pclapp.load(tmp_path / name).summary() == cloud.summary()


def test_bad_parameters_raise_value_error(cube_file):
    cloud = pclapp.load(cube_file)
    with pytest.raises(ValueError, match="leaf size"):
        pclapp.voxel_downsample(cloud, leaf_size=0.0)
    with pytest.raises(ValueError, match="tolerance"):
        pclapp.euclidean_cluster(cloud, tolerance=-1.0)


def test_file_problems_raise_io_error(tmp_path, cube_file):
    with pytest.raises(pclapp.IoError, match="cannot open"):
        pclapp.load(tmp_path / "missing.txt")
    with pytest.raises(pclapp.IoError, match="unsupported file type"):
        pclapp.load(tmp_path / "scan.ply")
    with pytest.raises(pclapp.IoError, match="unsupported file type"):
        pclapp.save(pclapp.load(cube_file), tmp_path / "scan.ply")

    broken = tmp_path / "broken.txt"
    broken.write_text("1 2 3\nnot a point\n")
    with pytest.raises(pclapp.IoError, match=":2:"):  # the message names the line
        pclapp.load(broken)
