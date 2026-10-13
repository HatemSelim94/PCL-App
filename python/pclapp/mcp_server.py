"""MCP server: lets an LLM load, describe, downsample, cluster and save point clouds.

Runs locally over stdio and makes no network calls. Clouds live in this
process's memory and are addressed by short ids ("cloud-1", "cloud-2", ...).
The only files it reads or writes are inside one data directory chosen at
start-up.

    python -m pclapp.mcp_server --data-dir /path/to/clouds

Stdout is the protocol channel, so nothing here may print to it.
"""

import argparse
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated

from mcp.server import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

import pclapp

SUPPORTED_EXTENSIONS = (".pcd", ".txt", ".xyz")
MAX_LISTED = 50  # cap on files / clusters in one reply, to keep replies small


# --- reply shapes (the SDK turns these models into each tool's output schema) --


class BoundingBox(BaseModel):
    min: list[float]
    max: list[float]


class CloudInfo(BaseModel):
    cloud_id: str
    origin: str  # how this cloud came to be, e.g. "loaded from scan.pcd"
    point_count: int
    has_intensity: bool
    bounding_box: BoundingBox | None  # None for a cloud without points
    centroid: list[float] | None


class CloudList(BaseModel):
    clouds: list[CloudInfo]  # clouds currently in memory
    files: list[str]  # loadable files in the data directory
    files_truncated: bool


class ClusterResult(BaseModel):
    source_cloud_id: str
    cluster_count: int
    clusters: list[CloudInfo]  # largest first; each one is a cloud of its own
    clusters_truncated: bool


class SavedFile(BaseModel):
    path: str  # relative to the data directory
    point_count: int


# --- state ------------------------------------------------------------------


@dataclass(frozen=True)
class StoredCloud:
    cloud: pclapp.Cloud
    origin: str


class CloudStore:
    """Clouds held in memory, by id. Tools run on worker threads, hence the lock."""

    def __init__(self) -> None:
        self._clouds: dict[str, StoredCloud] = {}
        self._lock = threading.Lock()

    def add(self, cloud: pclapp.Cloud, origin: str) -> str:
        with self._lock:
            cloud_id = f"cloud-{len(self._clouds) + 1}"  # clouds are never removed
            self._clouds[cloud_id] = StoredCloud(cloud, origin)
            return cloud_id

    def get(self, cloud_id: str) -> StoredCloud:
        with self._lock:
            stored = self._clouds.get(cloud_id)
            known = ", ".join(self._clouds) or "none"
        if stored is None:
            raise ToolError(f"Unknown cloud id {cloud_id!r}. Loaded clouds: {known}.")
        return stored

    def ids(self) -> list[str]:
        with self._lock:
            return list(self._clouds)


def resolve_in_data_dir(data_dir: Path, user_path: str) -> Path:
    """Turns a client-supplied path into a real path inside data_dir, or refuses.

    resolve() removes ".." and follows symlinks first, so neither can be used
    to step outside. An absolute path is accepted only if it already points
    into the data directory.
    """
    candidate = (data_dir / user_path).resolve()
    if not candidate.is_relative_to(data_dir):
        raise ToolError(f"{user_path!r} is outside the data directory; access refused.")
    return candidate


# --- server -----------------------------------------------------------------


def create_server(data_dir: Path) -> MCPServer:
    """Builds the server. All file access is confined to `data_dir`."""
    data_dir = data_dir.resolve(strict=True)
    store = CloudStore()
    mcp = MCPServer(
        "pclapp",
        instructions=(
            "Point cloud tools backed by PCL. Call list_clouds to see which files can be "
            "loaded. Every operation returns a new cloud id; clouds are never changed in place."
        ),
    )
    read_only = ToolAnnotations(read_only_hint=True)

    def describe(cloud_id: str) -> CloudInfo:
        stored = store.get(cloud_id)
        return CloudInfo(cloud_id=cloud_id, origin=stored.origin, **stored.cloud.summary())

    def relative(path: Path) -> str:
        return path.relative_to(data_dir).as_posix()

    @mcp.tool(annotations=read_only)
    def list_clouds() -> CloudList:
        """List the clouds in memory and the point cloud files that can be loaded."""
        files = sorted(
            relative(path)
            for path in data_dir.rglob("*")
            if path.suffix.lower() in SUPPORTED_EXTENSIONS and path.is_file()
        )
        return CloudList(
            clouds=[describe(cloud_id) for cloud_id in store.ids()],
            files=files[:MAX_LISTED],
            files_truncated=len(files) > MAX_LISTED,
        )

    @mcp.tool(annotations=read_only)
    def load_cloud(
        path: Annotated[str, Field(description="File in the data directory (.pcd, .txt or .xyz).")],
        intensity: Annotated[
            bool, Field(description="Keep a fourth intensity column (point type XYZI).")
        ] = False,
    ) -> CloudInfo:
        """Load a point cloud file into memory and return its id and summary."""
        file = resolve_in_data_dir(data_dir, path)
        try:
            cloud = pclapp.load(file, intensity=intensity)
        except pclapp.IoError as error:
            raise ToolError(str(error)) from error
        return describe(store.add(cloud, f"loaded from {relative(file)}"))

    @mcp.tool(annotations=read_only)
    def describe_cloud(cloud_id: str) -> CloudInfo:
        """Point count, bounding box, centroid and origin of a loaded cloud."""
        return describe(cloud_id)

    @mcp.tool()
    def downsample(
        cloud_id: str,
        leaf_size: Annotated[
            float, Field(gt=0, description="Voxel edge length, in the cloud's own units.")
        ],
    ) -> CloudInfo:
        """Voxel-grid downsample: one point per occupied voxel. Returns a new cloud."""
        source = store.get(cloud_id).cloud
        result = pclapp.voxel_downsample(source, leaf_size=leaf_size)
        return describe(store.add(result, f"voxel downsample of {cloud_id}, leaf {leaf_size:g}"))

    @mcp.tool()
    def cluster(
        cloud_id: str,
        tolerance: Annotated[
            float, Field(gt=0, description="Largest gap between neighbours in one cluster.")
        ],
        min_size: Annotated[int, Field(ge=1, description="Smallest cluster to keep.")] = 20,
        max_size: Annotated[
            int | None, Field(ge=1, description="Largest cluster to keep; omit for no limit.")
        ] = None,
    ) -> ClusterResult:
        """Euclidean clustering. Each cluster becomes a new cloud with its own id."""
        source = store.get(cloud_id).cloud
        if max_size is None:
            max_size = max(len(source), min_size)
        try:
            clusters = pclapp.euclidean_cluster(
                source, tolerance=tolerance, min_size=min_size, max_size=max_size
            )
        except ValueError as error:  # e.g. min_size > max_size
            raise ToolError(str(error)) from error

        listed = [
            describe(store.add(found, f"cluster {index} of {cloud_id}"))
            for index, found in enumerate(clusters[:MAX_LISTED])
        ]
        return ClusterResult(
            source_cloud_id=cloud_id,
            cluster_count=len(clusters),
            clusters=listed,
            clusters_truncated=len(clusters) > MAX_LISTED,
        )

    @mcp.tool()
    def save_cloud(
        cloud_id: str,
        path: Annotated[
            str, Field(description="Target file in the data directory (.pcd, .txt or .xyz).")
        ],
        overwrite: Annotated[bool, Field(description="Replace the file if it exists.")] = False,
    ) -> SavedFile:
        """Write a cloud to a file in the data directory."""
        cloud = store.get(cloud_id).cloud
        file = resolve_in_data_dir(data_dir, path)
        if file.exists() and not overwrite:
            raise ToolError(f"{relative(file)} already exists; pass overwrite=true to replace it.")
        if not file.parent.is_dir():
            raise ToolError(f"Folder {relative(file.parent)} does not exist.")
        try:
            pclapp.save(cloud, file)
        except pclapp.IoError as error:
            raise ToolError(str(error)) from error
        return SavedFile(path=relative(file), point_count=len(cloud))

    return mcp


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=os.environ.get("PCLAPP_DATA_DIR"),
        help="the only directory the server may read from or write to "
        "(default: $PCLAPP_DATA_DIR)",
    )
    args = parser.parse_args()
    if args.data_dir is None:
        parser.error("no data directory: pass --data-dir or set PCLAPP_DATA_DIR")
    if not Path(args.data_dir).is_dir():
        parser.error(f"data directory does not exist: {args.data_dir}")

    create_server(Path(args.data_dir)).run()  # no arguments = stdio transport


if __name__ == "__main__":
    main()
