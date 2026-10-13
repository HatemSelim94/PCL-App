// Python bindings for the processing core (pybind11).
//
// Python has no templates, so the point type becomes a runtime property here:
// a PyCloud holds either an XYZ or an XYZI cloud in a std::variant, and every
// function dispatches with one std::visit to the same templated core functions
// the C++ application uses. No geometry code lives in this file.
//
// Error mapping (pybind11 translates C++ exceptions into Python ones):
//   pclapp::IoError        -> pclapp.IoError (a RuntimeError)
//   std::invalid_argument  -> ValueError

#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <filesystem>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "core/cloud_io.h"
#include "core/processing.h"
#include "core/summary.h"

namespace py = pybind11;

namespace {

using AnyCloudPtr = std::variant<pclapp::CloudPtr<pcl::PointXYZ>, pclapp::CloudPtr<pcl::PointXYZI>>;

/// What Python sees as `pclapp.Cloud`. Clouds are immutable from Python: every
/// operation returns a new Cloud, so sharing the pointer is safe.
struct PyCloud {
    AnyCloudPtr cloud;
};

PyCloud load(const std::filesystem::path& path, bool intensity) {
    if (intensity) return {pclapp::load<pcl::PointXYZI>(path)};
    return {pclapp::load<pcl::PointXYZ>(path)};
}

void save(const PyCloud& self, const std::filesystem::path& path) {
    const pclapp::Format format = pclapp::requireFormat(path);
    std::visit([&](const auto& cloud) { pclapp::save(*cloud, path, format); }, self.cloud);
}

PyCloud voxelDownsample(const PyCloud& self, float leafSize) {
    return std::visit(
        [&](const auto& cloud) { return PyCloud{pclapp::voxelDownsample(cloud, {leafSize})}; },
        self.cloud);
}

std::vector<PyCloud> euclideanCluster(const PyCloud& self, float tolerance, int minSize,
                                      int maxSize) {
    return std::visit(
        [&](const auto& cloud) {
            std::vector<PyCloud> result;
            for (auto& cluster : pclapp::euclideanClusters(cloud, {tolerance, minSize, maxSize})) {
                result.push_back(PyCloud{std::move(cluster)});
            }
            return result;
        },
        self.cloud);
}

std::size_t size(const PyCloud& self) {
    return std::visit([](const auto& cloud) { return cloud->size(); }, self.cloud);
}

bool hasIntensity(const PyCloud& self) {
    return std::holds_alternative<pclapp::CloudPtr<pcl::PointXYZI>>(self.cloud);
}

/// Plain dict so the result can go straight to JSON.
py::dict summary(const PyCloud& self) {
    const pclapp::CloudSummary info =
        std::visit([](const auto& cloud) { return pclapp::summarize(*cloud); }, self.cloud);

    py::dict result;
    result["point_count"] = info.pointCount;
    result["has_intensity"] = hasIntensity(self);
    if (info.extent) {
        py::dict box;
        box["min"] = info.extent->min;  // std::array<float, 3> becomes a Python list
        box["max"] = info.extent->max;
        result["bounding_box"] = box;
        result["centroid"] = info.extent->centroid;
    } else {
        result["bounding_box"] = py::none();
        result["centroid"] = py::none();
    }
    return result;
}

std::string repr(const PyCloud& self) {
    return "<pclapp.Cloud " + std::to_string(size(self)) + " points, " +
           (hasIntensity(self) ? "XYZI" : "XYZ") + ">";
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "Point cloud loading, voxel downsampling and Euclidean clustering (PCL).";

    // PCL prints info messages to stdout. A host that uses stdout as a data
    // channel (the MCP server does) must not see them, so only warnings and
    // errors stay on, and PCL sends those to stderr.
    pcl::console::setVerbosityLevel(pcl::console::L_WARN);

    py::register_exception<pclapp::IoError>(m, "IoError", PyExc_RuntimeError);

    // The heavy functions release the GIL while they run, so other Python
    // threads (for example a server's event loop) keep going. That is safe
    // because they only touch C++ data until they return.
    const auto releaseGil = py::call_guard<py::gil_scoped_release>();

    const pclapp::EuclideanClusterParams clusterDefaults;

    py::class_<PyCloud>(m, "Cloud", "An immutable point cloud (XYZ, or XYZ plus intensity).")
        .def("__len__", &size)
        .def("__repr__", &repr)
        .def_property_readonly("has_intensity", &hasIntensity)
        .def("summary", &summary,
             "Point count, has_intensity, bounding_box {min, max} and centroid.\n"
             "bounding_box and centroid are None for a cloud without valid points.");

    m.def("load", &load, py::arg("path"), py::arg("intensity") = false, releaseGil,
          "Load a .pcd, .txt or .xyz file. Set intensity=True to keep a fourth\n"
          "intensity column (point type XYZI).");

    m.def("save", &save, py::arg("cloud"), py::arg("path"), releaseGil,
          "Write the cloud; the format follows the file extension (.pcd, .txt, .xyz).");

    m.def("voxel_downsample", &voxelDownsample, py::arg("cloud"), py::arg("leaf_size"), releaseGil,
          "Return a new cloud with one point (the centroid) per occupied voxel.");

    m.def("euclidean_cluster", &euclideanCluster, py::arg("cloud"),
          py::arg("tolerance") = clusterDefaults.distanceTolerance,
          py::arg("min_size") = clusterDefaults.minClusterSize,
          py::arg("max_size") = clusterDefaults.maxClusterSize, releaseGil,
          "Split the cloud into clusters of points linked by gaps <= tolerance.\n"
          "Returns a list of clouds, largest first.");
}
