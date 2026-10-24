#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

struct PointXYZRGB {
  float x, y, z;
  std::uint8_t r, g, b;
};

struct PointCloud {
  std::vector<PointXYZRGB> points;
  // Empty for unorganized samples; otherwise maps image pixels to point indices.
  cv::Mat1i pixelIndices;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;
using PointCloudConstPtr = std::shared_ptr<const PointCloud>;

struct PointCloudSettings {
  float minDepth = 0.0f;
  float maxDepth = std::numeric_limits<float>::max();
};

PointCloudPtr makePointCloud(const cv::Mat &xyz, const cv::Mat &shading,
                            const PointCloudSettings &settings = {});
PointCloudPtr makePointCloud(const cv::Mat3f &xyz, const cv::Mat1b &shading,
                            const PointCloudSettings &settings = {});
bool savePointCloudPLY(const std::string &filename, const PointCloud &cloud);
bool savePointCloudPCD(const std::string &filename, const PointCloud &cloud);