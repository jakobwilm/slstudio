#include "PointCloud.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function> void rejects(Function action) {
  bool rejected = false;
  try { action(); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Invalid point-cloud input was accepted");
}

void testOrganizedCloud() {
  cv::Mat3f xyz(2, 3, cv::Vec3f(1, 2, 100));
  cv::Mat1b shading(2, 3, std::uint8_t(120));
  rejects([&] { makePointCloud(cv::Mat(2, 3, CV_64FC3, cv::Scalar(1, 2, 3)), cv::Mat(shading)); });
  xyz(0, 0)[0] = std::numeric_limits<float>::quiet_NaN();
  xyz(0, 1)[2] = 200;
  const auto cloud = makePointCloud(xyz, shading, {50, 150});
  require(cloud->points.size() == 4, "Depth and non-finite filtering failed");
  require(cloud->pixelIndices.size() == xyz.size() && cloud->pixelIndices(0, 0) == -1 &&
      cloud->pixelIndices(0, 1) == -1 && cloud->pixelIndices(0, 2) == 0 &&
      cloud->pixelIndices(1, 0) == 1 && cloud->pixelIndices(1, 2) == 3,
      "Organized point indices lost image neighbors or filtered holes");
  require(cloud->points.front().r == 120, "Point color was lost");
  rejects([&] { makePointCloud(xyz, shading, {150, 50}); });

  cv::Mat3f paddedXYZ(5, 7, cv::Vec3f(1, 2, 100));
  cv::Mat1b paddedShading(5, 7, std::uint8_t(90));
  const cv::Rect region(1, 1, 5, 3);
  paddedXYZ(3, 5) = cv::Vec3f(7, 8, 101);
  paddedShading(3, 5) = 170;
  const auto full = makePointCloud(paddedXYZ(region), paddedShading(region), {50, 150});
  require(full->points.size() == 15 && full->pixelIndices.size() == region.size(),
      "Cloud creation lost pixels or dimensions from an odd-sized image");
  for (int row = 0; row < region.height; ++row)
    for (int col = 0; col < region.width; ++col)
      require(full->pixelIndices(row, col) == row * region.width + col,
          "Organized point indices did not preserve every image pixel");
  require(full->points.back().x == 7 && full->points.back().z == 101 &&
      full->points.back().r == 170, "Non-contiguous image rows were read incorrectly");
}

void testExports() {
  const auto cloud = makePointCloud(cv::Mat3f(2, 3, cv::Vec3f(1, 2, 100)),
      cv::Mat1b(2, 3, std::uint8_t(120)));
  const auto base = std::filesystem::temp_directory_path() /
      ("slstudio-point-cloud-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  for (const auto *extension : {".ply", ".pcd"}) {
    const auto path = std::filesystem::path(base.string() + extension);
    struct Cleanup {
      std::filesystem::path path;
      ~Cleanup() {
        std::error_code error;
        std::filesystem::remove(path, error);
        if (error) std::cerr << "Cannot remove point-cloud test file: " << error.message() << '\n';
      }
    } cleanup{path};
    require(std::string(extension) == ".ply" ? savePointCloudPLY(path.string(), *cloud) : savePointCloudPCD(path.string(), *cloud),
        "Point-cloud export failed");
    std::ifstream file(path);
    std::string header;
    std::getline(file, header);
    require(file.good() && (std::string(extension) == ".ply" ? header == "ply" : header.find("PCD") != std::string::npos),
        "Point-cloud export did not write the expected format");
  }
}
}

int main() {
  try {
    testOrganizedCloud();
    testExports();
    std::cout << "Point-cloud filtering, organized indices, image ownership and exports passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
