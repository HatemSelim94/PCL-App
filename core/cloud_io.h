#pragma once
// Point-cloud file I/O for two formats: PCD and whitespace-separated text
// ("x y z" or "x y z intensity" per line).
//
// Header-only and Qt-free so the whole core can be unit tested on its own.
// The intensity channel is handled with `if constexpr` on PCL's field traits,
// so one implementation serves pcl::PointXYZ, pcl::PointXYZI and any other
// point type with the same fields.

#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pclapp {

template <typename PointT> using Cloud    = pcl::PointCloud<PointT>;
// Spelled as pcl::shared_ptr<...> and not as Cloud<PointT>::Ptr: a nested
// `::Ptr` is a non-deduced context, so function templates taking a
// CloudPtr<PointT> could not deduce PointT from their argument.
template <typename PointT> using CloudPtr = pcl::shared_ptr<Cloud<PointT>>;

/// True when PointT carries an intensity channel (e.g. pcl::PointXYZI).
template <typename PointT>
inline constexpr bool hasIntensity =
    pcl::traits::has_field<PointT, pcl::fields::intensity>::value;

/// Raised for unreadable, unwritable or malformed files. The message always
/// names the file, and the line number where one applies.
class IoError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

enum class Format { Pcd, Text };

[[nodiscard]] constexpr std::string_view extensionOf(Format format) noexcept {
    return format == Format::Pcd ? ".pcd" : ".txt";
}

/// The other supported format, used when converting a file.
[[nodiscard]] constexpr Format otherFormat(Format format) noexcept {
    return format == Format::Pcd ? Format::Text : Format::Pcd;
}

/// Detects the format from the file extension, case-insensitively.
[[nodiscard]] inline std::optional<Format> formatOf(const std::filesystem::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".pcd") return Format::Pcd;
    if (ext == ".txt" || ext == ".xyz") return Format::Text;
    return std::nullopt;
}

[[nodiscard]] inline Format requireFormat(const std::filesystem::path& path) {
    if (const auto format = formatOf(path)) return *format;
    throw IoError("unsupported file type: " + path.string());
}

namespace detail {

template <typename PointT>
CloudPtr<PointT> loadText(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw IoError("cannot open " + path.string());

    auto cloud = pcl::make_shared<Cloud<PointT>>();
    std::string line;
    for (std::size_t lineNo = 1; std::getline(in, line); ++lineNo) {
        std::istringstream fields(line);
        PointT point{};
        if (!(fields >> point.x >> point.y >> point.z)) {
            if (line.find_first_not_of(" \t\r") == std::string::npos) continue;  // blank line
            throw IoError(path.string() + ':' + std::to_string(lineNo) +
                          ": expected 'x y z' or 'x y z intensity'");
        }
        if constexpr (hasIntensity<PointT>) {
            if (!(fields >> point.intensity)) point.intensity = 0.0f;  // column is optional
        }
        cloud->push_back(point);  // keeps width/height consistent for an unorganised cloud
    }
    return cloud;
}

template <typename PointT>
void saveText(const Cloud<PointT>& cloud, const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) throw IoError("cannot write " + path.string());

    out << std::setprecision(std::numeric_limits<float>::max_digits10);  // exact round trip
    for (const PointT& point : cloud) {
        out << point.x << ' ' << point.y << ' ' << point.z;
        if constexpr (hasIntensity<PointT>) out << ' ' << point.intensity;
        out << '\n';
    }
}

}  // namespace detail

/// Loads a cloud, choosing the reader from the file extension.
template <typename PointT>
[[nodiscard]] CloudPtr<PointT> load(const std::filesystem::path& path) {
    if (requireFormat(path) == Format::Text) return detail::loadText<PointT>(path);

    auto cloud = pcl::make_shared<Cloud<PointT>>();
    if (pcl::io::loadPCDFile(path.string(), *cloud) < 0) {
        throw IoError("failed to read PCD file " + path.string());
    }
    return cloud;
}

/// Writes a cloud in the given format, replacing any existing file.
template <typename PointT>
void save(const Cloud<PointT>& cloud, const std::filesystem::path& path, Format format) {
    if (format == Format::Text) {
        detail::saveText(cloud, path);
    } else if (pcl::io::savePCDFileBinary(path.string(), cloud) < 0) {
        throw IoError("failed to write PCD file " + path.string());
    }
}

}  // namespace pclapp
