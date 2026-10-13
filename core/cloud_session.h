#pragma once
// A CloudSession owns one loaded point cloud and everything derived from it:
// the processed (filtered) copy and the extracted clusters. It also knows
// where the file came from, so derived files are written beside it.
//
// AnyCloudSession is a std::variant over the supported point types. UI code
// handles it with a single std::visit, instead of switching on the point type
// in every handler.

#include <pcl/point_types.h>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "cloud_io.h"
#include "processing.h"

namespace pclapp {

template <typename PointT>
class CloudSession {
public:
    /// Loads `path`. The processed cloud starts as a copy of the source.
    /// Throws IoError if the file cannot be read.
    explicit CloudSession(std::filesystem::path path)
        : path_(std::move(path)),
          format_(requireFormat(path_)),
          source_(load<PointT>(path_)),
          processed_(pcl::make_shared<Cloud<PointT>>(*source_)) {}

    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
    [[nodiscard]] const CloudPtr<PointT>& source() const noexcept { return source_; }
    [[nodiscard]] const CloudPtr<PointT>& processed() const noexcept { return processed_; }
    [[nodiscard]] const std::vector<CloudPtr<PointT>>& clusters() const noexcept {
        return clusters_;
    }

    /// Replaces the processed cloud with a downsampled source. Clusters computed
    /// from the previous processed cloud are stale, so they are dropped.
    void applyVoxelFilter(const VoxelGridParams& params) {
        processed_ = voxelDownsample(source_, params);
        clusters_.clear();
    }

    void extractClusters(const EuclideanClusterParams& params) {
        clusters_ = euclideanClusters(processed_, params);
    }

    /// Writes the processed cloud as "<stem>_filtered<ext>" beside the source.
    /// Returns the path written.
    std::filesystem::path saveProcessed() const {
        const auto target = siblingPath("_filtered", format_);
        save(*processed_, target, format_);
        return target;
    }

    /// Writes the source cloud in the other supported format (.pcd <-> .txt).
    std::filesystem::path convertSource() const {
        const auto target = siblingPath("", otherFormat(format_));
        save(*source_, target, otherFormat(format_));
        return target;
    }

private:
    std::filesystem::path siblingPath(std::string_view suffix, Format format) const {
        return path_.parent_path() /
               (path_.stem().string() + std::string(suffix) + std::string(extensionOf(format)));
    }

    std::filesystem::path path_;
    Format format_;
    CloudPtr<PointT> source_;
    CloudPtr<PointT> processed_;
    std::vector<CloudPtr<PointT>> clusters_;
};

enum class PointKind { Xyz, Xyzi };

/// Nothing loaded yet, or a session for one of the supported point types.
using AnyCloudSession =
    std::variant<std::monostate, CloudSession<pcl::PointXYZ>, CloudSession<pcl::PointXYZI>>;

/// The one place where a runtime point kind is turned into a static type.
[[nodiscard]] inline AnyCloudSession openCloud(const std::filesystem::path& path,
                                               PointKind kind) {
    switch (kind) {
        case PointKind::Xyz:  return CloudSession<pcl::PointXYZ>(path);
        case PointKind::Xyzi: return CloudSession<pcl::PointXYZI>(path);
    }
    throw std::invalid_argument("unknown point kind");  // unreachable with a valid enum
}

}  // namespace pclapp
