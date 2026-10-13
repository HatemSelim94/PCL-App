#pragma once
// Geometry processing steps. Each step is a pure function: an input cloud and
// a parameter struct go in, a new cloud (or set of clouds) comes out. Nothing
// here holds state between calls, which keeps the steps trivial to test and
// safe to run from any thread.

#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <stdexcept>
#include <utility>
#include <vector>

#include "cloud_io.h"

namespace pclapp {

struct VoxelGridParams {
    float leafSize = 0.1f;  ///< Edge length of one voxel, in cloud units.
};

struct EuclideanClusterParams {
    float distanceTolerance = 0.5f;  ///< Max gap between neighbours in one cluster.
    int   minClusterSize    = 20;
    int   maxClusterSize    = 3000;
};

/// Downsamples by replacing all points inside each voxel with their centroid.
template <typename PointT>
[[nodiscard]] CloudPtr<PointT> voxelDownsample(const CloudPtr<PointT>& input,
                                               const VoxelGridParams& params) {
    if (params.leafSize <= 0.0f) {
        throw std::invalid_argument("voxel leaf size must be positive");
    }

    pcl::VoxelGrid<PointT> grid;
    grid.setInputCloud(input);
    grid.setLeafSize(params.leafSize, params.leafSize, params.leafSize);

    auto output = pcl::make_shared<Cloud<PointT>>();
    grid.filter(*output);
    return output;
}

/// Splits the cloud into connected components: two points belong to the same
/// cluster when a chain of neighbours within `distanceTolerance` links them.
template <typename PointT>
[[nodiscard]] std::vector<CloudPtr<PointT>> euclideanClusters(
    const CloudPtr<PointT>& input, const EuclideanClusterParams& params) {
    if (params.distanceTolerance <= 0.0f) {
        throw std::invalid_argument("cluster distance tolerance must be positive");
    }
    if (params.minClusterSize < 1 || params.minClusterSize > params.maxClusterSize) {
        throw std::invalid_argument("cluster size range must satisfy 1 <= min <= max");
    }
    if (input->empty()) return {};

    auto tree = pcl::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(input);

    pcl::EuclideanClusterExtraction<PointT> extraction;
    extraction.setClusterTolerance(params.distanceTolerance);
    extraction.setMinClusterSize(params.minClusterSize);
    extraction.setMaxClusterSize(params.maxClusterSize);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(input);

    std::vector<pcl::PointIndices> indices;
    extraction.extract(indices);

    std::vector<CloudPtr<PointT>> clusters;
    clusters.reserve(indices.size());
    for (const pcl::PointIndices& cluster : indices) {
        auto cloud = pcl::make_shared<Cloud<PointT>>();
        pcl::copyPointCloud(*input, cluster.indices, *cloud);
        clusters.push_back(std::move(cloud));
    }
    return clusters;
}

}  // namespace pclapp
