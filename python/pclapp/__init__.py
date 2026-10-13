"""Point cloud loading, voxel downsampling and Euclidean clustering.

Thin Python layer over the C++ core (see core/ in the repository). The
geometry algorithms are PCL's; this package only exposes them.

    import pclapp

    cloud = pclapp.load("scan.pcd")
    small = pclapp.voxel_downsample(cloud, leaf_size=0.1)
    for cluster in pclapp.euclidean_cluster(small, tolerance=0.5):
        print(cluster.summary())
"""

from ._core import Cloud, IoError, euclidean_cluster, load, save, voxel_downsample

__all__ = ["Cloud", "IoError", "euclidean_cluster", "load", "save", "voxel_downsample"]
