// Unit tests for the Qt-free processing core. Geometry is synthetic and
// chosen so every expected count is exact (grid spacing 0.25 is a power of two).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>

#include "core/cloud_session.h"
#include "core/summary.h"

namespace fs = std::filesystem;
using pcl::PointXYZ;
using pcl::PointXYZI;

namespace {

/// A cube of n*n*n points on a regular grid, with its corner at `origin`.
pclapp::CloudPtr<PointXYZ> makeCube(int n, float spacing, float origin = 0.0f) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointXYZ>>();
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < n; ++y) {
            for (int z = 0; z < n; ++z) {
                cloud->push_back(
                    PointXYZ(origin + x * spacing, origin + y * spacing, origin + z * spacing));
            }
        }
    }
    return cloud;
}

/// A scratch directory that disappears when the test ends.
class TempDir {
public:
    TempDir() {
        std::random_device seed;
        path_ = fs::temp_directory_path() / ("pclapp-test-" + std::to_string(seed()));
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ignored;
        fs::remove_all(path_, ignored);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;

    fs::path file(const char* name) const { return path_ / name; }

private:
    fs::path path_;
};

void writeText(const fs::path& file, const std::string& content) {
    std::ofstream(file) << content;
}

}  // namespace

TEST_CASE("format detection uses the extension, case-insensitively") {
    CHECK(pclapp::formatOf("scan.PCD") == pclapp::Format::Pcd);
    CHECK(pclapp::formatOf("scan.txt") == pclapp::Format::Text);
    CHECK(pclapp::formatOf("scan.xyz") == pclapp::Format::Text);
    CHECK_FALSE(pclapp::formatOf("scan.ply").has_value());
    CHECK_THROWS_AS(pclapp::requireFormat("scan.ply"), pclapp::IoError);
}

TEST_CASE("text loader reads an optional intensity column and skips blank lines") {
    TempDir tmp;
    const auto file = tmp.file("points.txt");
    writeText(file, "0 0 0 7\n\n1 2 3\n");

    const auto cloud = pclapp::load<PointXYZI>(file);
    REQUIRE(cloud->size() == 2);
    CHECK(cloud->points[0].intensity == 7.0f);
    CHECK(cloud->points[1].z == 3.0f);
    CHECK(cloud->points[1].intensity == 0.0f);
}

TEST_CASE("text loader reports malformed lines with their line number") {
    TempDir tmp;
    const auto file = tmp.file("broken.txt");
    writeText(file, "1 2 3\nnot a point\n");

    CHECK_THROWS_WITH(pclapp::load<PointXYZ>(file), Catch::Matchers::ContainsSubstring(":2:"));
}

TEST_CASE("save and load round-trip in both formats") {
    TempDir tmp;
    const auto cube = makeCube(4, 0.25f);

    for (const auto format : {pclapp::Format::Pcd, pclapp::Format::Text}) {
        const auto file = tmp.file(format == pclapp::Format::Pcd ? "cube.pcd" : "cube.txt");
        pclapp::save(*cube, file, format);

        const auto loaded = pclapp::load<PointXYZ>(file);
        REQUIRE(loaded->size() == cube->size());
        CHECK(loaded->points.back().x == cube->points.back().x);
        CHECK(loaded->points.back().z == cube->points.back().z);
    }
}

TEST_CASE("voxel filter keeps one point per occupied voxel") {
    const auto cube = makeCube(4, 0.25f);  // spans [0, 0.75] on each axis

    const auto filtered = pclapp::voxelDownsample(cube, {0.5f});  // two voxels per axis
    CHECK(filtered->size() == 8);

    CHECK_THROWS_AS(pclapp::voxelDownsample(cube, {0.0f}), std::invalid_argument);
}

TEST_CASE("euclidean clustering separates two distant blobs") {
    auto cloud = makeCube(4, 0.25f);
    *cloud += *makeCube(4, 0.25f, /*origin=*/10.0f);

    pclapp::EuclideanClusterParams params;
    params.distanceTolerance = 0.3f;  // neighbours are 0.25 apart, the blobs are 10 apart
    params.minClusterSize = 1;
    params.maxClusterSize = 1000;

    const auto clusters = pclapp::euclideanClusters(cloud, params);
    REQUIRE(clusters.size() == 2);
    CHECK(clusters[0]->size() == 64);
    CHECK(clusters[1]->size() == 64);

    params.minClusterSize = 2000;  // larger than the maximum
    CHECK_THROWS_AS(pclapp::euclideanClusters(cloud, params), std::invalid_argument);

    CHECK(pclapp::euclideanClusters(pcl::make_shared<pcl::PointCloud<PointXYZ>>(), {}).empty());
}

TEST_CASE("summary reports count, bounding box and centroid") {
    const auto cube = makeCube(4, 0.25f, /*origin=*/1.0f);  // spans [1, 1.75] on each axis

    const auto summary = pclapp::summarize(*cube);
    CHECK(summary.pointCount == 64);
    REQUIRE(summary.extent.has_value());
    CHECK(summary.extent->min == pclapp::Vec3{1.0f, 1.0f, 1.0f});
    CHECK(summary.extent->max == pclapp::Vec3{1.75f, 1.75f, 1.75f});
    CHECK(summary.extent->centroid == pclapp::Vec3{1.375f, 1.375f, 1.375f});

    const auto empty = pclapp::summarize(pcl::PointCloud<PointXYZ>());
    CHECK(empty.pointCount == 0);
    CHECK_FALSE(empty.extent.has_value());
}

TEST_CASE("a session writes derived files beside its source and never appends") {
    TempDir tmp;
    const auto source = tmp.file("scan.txt");
    pclapp::save(*makeCube(4, 0.25f), source, pclapp::Format::Text);

    pclapp::CloudSession<PointXYZ> session(source);
    REQUIRE(session.processed()->size() == 64);

    session.applyVoxelFilter({0.5f});
    const auto filtered = session.saveProcessed();
    session.saveProcessed();  // a second save must overwrite, not append
    CHECK(filtered.filename() == "scan_filtered.txt");
    CHECK(pclapp::load<PointXYZ>(filtered)->size() == 8);

    const auto converted = session.convertSource();
    CHECK(converted.filename() == "scan.pcd");
    CHECK(pclapp::load<PointXYZ>(converted)->size() == 64);

    CHECK_THROWS_AS(pclapp::CloudSession<PointXYZ>(tmp.file("missing.txt")), pclapp::IoError);
}
