#include "PointCloud.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

PointCloudPtr makePointCloud(const cv::Mat &xyz, const cv::Mat &shading,
                            const PointCloudSettings &settings) {
  if (xyz.dims != 2 || shading.dims != 2 ||
      xyz.type() != CV_32FC3 || shading.type() != CV_8UC1 ||
      xyz.size() != shading.size())
    throw std::invalid_argument("Expected matching XYZ float and grayscale images");
  return makePointCloud(cv::Mat3f(xyz), cv::Mat1b(shading), settings);
}

PointCloudPtr makePointCloud(const cv::Mat3f &xyz, const cv::Mat1b &shading,
                            const PointCloudSettings &settings) {
  if (xyz.dims != 2 || shading.dims != 2 ||
      xyz.type() != CV_32FC3 || shading.type() != CV_8UC1 || xyz.size() != shading.size())
    throw std::invalid_argument("Expected matching XYZ float and grayscale images");
  if (!std::isfinite(settings.minDepth) ||
      !std::isfinite(settings.maxDepth) || settings.minDepth > settings.maxDepth)
    throw std::invalid_argument("Invalid point cloud depth range");

  auto cloud = std::make_shared<PointCloud>();
  cloud->pixelIndices = cv::Mat1i(xyz.size(), -1);
  cloud->points.reserve(xyz.total());
  for (int row = 0; row < xyz.rows; ++row) {
    const auto *positions = xyz.ptr<cv::Vec3f>(row);
    const auto *shades = shading.ptr<std::uint8_t>(row);
    for (int col = 0; col < xyz.cols; ++col) {
      const auto position = positions[col];
      if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
          !std::isfinite(position[2]) || position[2] < settings.minDepth ||
          position[2] > settings.maxDepth)
        continue;
      const auto shade = shades[col];
      if (cloud->points.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("Point cloud exceeds organized index capacity");
      cloud->pixelIndices(row, col) = static_cast<int>(cloud->points.size());
      cloud->points.push_back({position[0], position[1], position[2], shade, shade, shade});
    }
  }
  return cloud;
}

bool savePointCloudPLY(const std::string &filename, const PointCloud &cloud) {
  std::ofstream stream(filename);
  stream << "ply\nformat ascii 1.0\nelement vertex " << cloud.points.size()
         << "\nproperty float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
  stream << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (const auto &point : cloud.points)
    stream << point.x << ' ' << point.y << ' ' << point.z << ' '
           << static_cast<int>(point.r) << ' ' << static_cast<int>(point.g)
           << ' ' << static_cast<int>(point.b) << '\n';
  stream.flush();
  return stream.good();
}

bool savePointCloudPCD(const std::string &filename, const PointCloud &cloud) {
  std::ofstream stream(filename);
  stream << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\n"
            "TYPE F F F U\nCOUNT 1 1 1 1\nWIDTH " << cloud.points.size()
         << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << cloud.points.size()
         << "\nDATA ascii\n";
  stream << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (const auto &point : cloud.points) {
    const auto rgb = (std::uint32_t(point.r) << 16) |
                     (std::uint32_t(point.g) << 8) | point.b;
    stream << point.x << ' ' << point.y << ' ' << point.z << ' ' << rgb << '\n';
  }
  stream.flush();
  return stream.good();
}