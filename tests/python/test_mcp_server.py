"""Tests for the MCP server, driven through a real MCP client.

`Client(server)` connects in memory: no subprocess and no pipe, but tools are
listed, validated and called through the actual protocol layer. One extra
test at the end starts the server as a subprocess to cover the stdio transport.
"""

import sys

import pytest
from mcp import Client, StdioServerParameters

from pclapp.mcp_server import create_server

from conftest import cube_points, write_xyz


@pytest.fixture
def anyio_backend():
    return "asyncio"


@pytest.fixture
def data_dir(tmp_path, two_blobs_file):
    """The server's data directory; it already holds two_blobs.txt."""
    return tmp_path


@pytest.fixture
async def client(data_dir):
    async with Client(create_server(data_dir)) as connected:
        yield connected


async def call(client, tool, **arguments):
    """Calls a tool and returns its structured result, failing on tool errors."""
    result = await client.call_tool(tool, arguments)
    assert not result.is_error, result.content
    return result.structured_content


async def error_text(client, tool, **arguments):
    """Calls a tool that is expected to fail and returns the message the model sees."""
    result = await client.call_tool(tool, arguments)
    assert result.is_error
    return result.content[0].text


@pytest.mark.anyio
async def test_the_six_tools_are_offered(client):
    tools = await client.list_tools()
    assert sorted(tool.name for tool in tools.tools) == [
        "cluster",
        "describe_cloud",
        "downsample",
        "list_clouds",
        "load_cloud",
        "save_cloud",
    ]


@pytest.mark.anyio
async def test_list_clouds_shows_loadable_files_then_loaded_clouds(client):
    listing = await call(client, "list_clouds")
    assert listing == {"clouds": [], "files": ["two_blobs.txt"], "files_truncated": False}

    await call(client, "load_cloud", path="two_blobs.txt")
    listing = await call(client, "list_clouds")
    assert [cloud["cloud_id"] for cloud in listing["clouds"]] == ["cloud-1"]


@pytest.mark.anyio
async def test_load_then_describe(client):
    loaded = await call(client, "load_cloud", path="two_blobs.txt")
    assert loaded == {
        "cloud_id": "cloud-1",
        "origin": "loaded from two_blobs.txt",
        "point_count": 128,
        "has_intensity": False,
        "bounding_box": {"min": [0.0, 0.0, 0.0], "max": [10.75, 10.75, 10.75]},
        "centroid": [5.375, 5.375, 5.375],
    }
    assert await call(client, "describe_cloud", cloud_id="cloud-1") == loaded


@pytest.mark.anyio
async def test_downsample_makes_a_new_cloud_and_keeps_the_original(client):
    await call(client, "load_cloud", path="two_blobs.txt")

    small = await call(client, "downsample", cloud_id="cloud-1", leaf_size=0.5)
    assert small["cloud_id"] == "cloud-2"
    assert small["point_count"] == 16  # 8 voxels per cube
    assert small["origin"] == "voxel downsample of cloud-1, leaf 0.5"

    original = await call(client, "describe_cloud", cloud_id="cloud-1")
    assert original["point_count"] == 128


@pytest.mark.anyio
async def test_cluster_reports_size_and_centroid_per_cluster(client):
    await call(client, "load_cloud", path="two_blobs.txt")

    result = await call(client, "cluster", cloud_id="cloud-1", tolerance=0.3, min_size=1)
    assert result["cluster_count"] == 2
    assert not result["clusters_truncated"]
    assert [c["point_count"] for c in result["clusters"]] == [64, 64]
    assert sorted(c["centroid"] for c in result["clusters"]) == [
        [0.375, 0.375, 0.375],
        [10.375, 10.375, 10.375],
    ]
    # Clusters are clouds of their own, so they can be described or saved by id.
    assert [c["cloud_id"] for c in result["clusters"]] == ["cloud-2", "cloud-3"]
    assert result["clusters"][0]["origin"] == "cluster 0 of cloud-1"


@pytest.mark.anyio
async def test_save_writes_into_the_data_directory_and_refuses_to_overwrite(client, data_dir):
    await call(client, "load_cloud", path="two_blobs.txt")
    await call(client, "downsample", cloud_id="cloud-1", leaf_size=0.5)

    saved = await call(client, "save_cloud", cloud_id="cloud-2", path="small.pcd")
    assert saved == {"path": "small.pcd", "point_count": 16}
    assert (data_dir / "small.pcd").is_file()

    message = await error_text(client, "save_cloud", cloud_id="cloud-2", path="small.pcd")
    assert "already exists" in message
    await call(client, "save_cloud", cloud_id="cloud-2", path="small.pcd", overwrite=True)

    reloaded = await call(client, "load_cloud", path="small.pcd")
    assert reloaded["point_count"] == 16


@pytest.mark.anyio
@pytest.mark.parametrize("tool", ["load_cloud", "save_cloud"])
async def test_paths_outside_the_data_directory_are_refused(client, data_dir, tool):
    outside = write_xyz(data_dir.parent / "outside.txt", cube_points())
    await call(client, "load_cloud", path="two_blobs.txt")

    for path in ("../outside.txt", str(outside), "sub/../../outside.txt"):
        arguments = {"path": path}
        if tool == "save_cloud":
            arguments.update(cloud_id="cloud-1", overwrite=True)
        message = await error_text(client, tool, **arguments)
        assert "outside the data directory" in message

    assert len(outside.read_text().splitlines()) == 64  # untouched


@pytest.mark.anyio
async def test_a_symlink_cannot_be_used_to_leave_the_data_directory(client, data_dir):
    outside = write_xyz(data_dir.parent / "secret.txt", cube_points())
    try:
        (data_dir / "link.txt").symlink_to(outside)
    except OSError:
        pytest.skip("creating symlinks needs extra rights on this system")

    message = await error_text(client, "load_cloud", path="link.txt")
    assert "outside the data directory" in message


@pytest.mark.anyio
async def test_errors_are_readable_for_the_model(client):
    message = await error_text(client, "describe_cloud", cloud_id="cloud-9")
    assert "Unknown cloud id 'cloud-9'" in message

    message = await error_text(client, "load_cloud", path="missing.pcd")
    assert "missing.pcd" in message

    await call(client, "load_cloud", path="two_blobs.txt")
    message = await error_text(
        client, "cluster", cloud_id="cloud-1", tolerance=0.3, min_size=50, max_size=10
    )
    assert "1 <= min <= max" in message

    # Schema violations are caught by the SDK before the tool runs.
    result = await client.call_tool("downsample", {"cloud_id": "cloud-1", "leaf_size": 0})
    assert result.is_error


@pytest.mark.anyio
async def test_server_works_over_stdio_as_a_subprocess(data_dir):
    server = StdioServerParameters(
        command=sys.executable,
        args=["-m", "pclapp.mcp_server", "--data-dir", str(data_dir)],
    )
    async with Client(server) as connected:
        loaded = await call(connected, "load_cloud", path="two_blobs.txt")
        assert loaded["point_count"] == 128
