#pragma once
// Read-only facts about a cloud: how many points it has and where they lie.
// Used wherever a cloud has to be described without being drawn, for example
// by the Python bindings.
//
// The bounding box and centroid come from PCL (getMinMax3D, compute3DCentroid).
// Both skip NaN points, which PCD files from depth sensors can contain.

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <array>
#include <cstddef>
#include <optional>

#include "cloud_io.h"

namespace pclapp {

using Vec3 = std::array<float, 3>;

struct Extent {
    Vec3 min;       ///< Smallest x, y and z over all points.
    Vec3 max;       ///< Largest x, y and z over all points.
    Vec3 centroid;  ///< Mean position of all points.
};

struct CloudSummary {
    std::size_t pointCount = 0;
    std::optional<Extent> extent;  ///< Empty when the cloud has no valid point.
};

template <typename PointT>
[[nodiscard]] CloudSummary summarize(const Cloud<PointT>& cloud) {
    CloudSummary summary;
    summary.pointCount = cloud.size();

    // Accumulated in double: summing many large float coordinates loses digits.
    Eigen::Vector4d centroid;
    if (pcl::compute3DCentroid(cloud, centroid) == 0) return summary;  // no valid point

    PointT min, max;
    pcl::getMinMax3D(cloud, min, max);

    summary.extent = Extent{{min.x, min.y, min.z},
                            {max.x, max.y, max.z},
                            {static_cast<float>(centroid.x()), static_cast<float>(centroid.y()),
                             static_cast<float>(centroid.z())}};
    return summary;
}

}  // namespace pclapp
