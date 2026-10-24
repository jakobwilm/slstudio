#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace slstudio {

enum class PixelLayout { Rectangular, Diamond };

struct ProjectorGeometry {
  cv::Size logicalSize;
  cv::Size nativeSize;
  PixelLayout layout = PixelLayout::Rectangular;
};

cv::Mat prepareProjectorPattern(const cv::Mat &pattern, const ProjectorGeometry &geometry);
std::vector<cv::Mat1b> prepareMonochromeProjectorPatterns(const std::vector<cv::Mat> &patterns,
  const ProjectorGeometry &geometry, std::size_t maximumCount);
std::vector<cv::Mat3b> packLightCrafter4500Patterns(const std::vector<cv::Mat1b> &nativePatterns);

}