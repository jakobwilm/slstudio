#include "patternTools.h"

#include <limits>
#include <stdexcept>

namespace slstudio {

cv::Mat prepareProjectorPattern(const cv::Mat &pattern, const ProjectorGeometry &geometry) {
  if (geometry.nativeSize.width <= 0 || geometry.nativeSize.height <= 0 ||
      geometry.logicalSize.width <= 0 || geometry.logicalSize.height <= 0 ||
      pattern.dims != 2 || pattern.size() != geometry.logicalSize ||
      (pattern.type() != CV_8UC1 && pattern.type() != CV_8UC3))
    throw std::invalid_argument("Projector patterns must be byte images matching the logical resolution");
  if (geometry.layout == PixelLayout::Rectangular) {
    if (geometry.logicalSize != geometry.nativeSize)
      throw std::invalid_argument("Rectangular projector geometry must be pixel-for-pixel");
    return pattern.clone();
  }
  if (geometry.layout != PixelLayout::Diamond ||
      geometry.nativeSize.width > std::numeric_limits<int>::max() / 2 ||
      geometry.logicalSize.width != geometry.nativeSize.width * 2 ||
      geometry.logicalSize.height != geometry.nativeSize.height)
    throw std::invalid_argument("Diamond geometry needs twice the native width and matching height");
  cv::Mat native(geometry.nativeSize, pattern.type());
  const int channels = pattern.channels();
  for (int row = 0; row < native.rows; ++row) {
    const auto *source = pattern.ptr<unsigned char>(row);
    auto *destination = native.ptr<unsigned char>(row);
    for (int column = 0; column < native.cols; ++column)
      for (int channel = 0; channel < channels; ++channel)
        destination[column * channels + channel] = source[(column * 2 + row % 2) * channels + channel];
  }
  return native;
}

std::vector<cv::Mat1b> prepareMonochromeProjectorPatterns(const std::vector<cv::Mat> &patterns,
    const ProjectorGeometry &geometry, std::size_t maximumCount) {
  if (patterns.empty() || patterns.size() > maximumCount)
    throw std::invalid_argument("Unsupported LightCrafter pattern count");
  std::vector<cv::Mat1b> prepared;
  prepared.reserve(patterns.size());
  for (const auto &pattern : patterns) {
    const auto native = prepareProjectorPattern(pattern, geometry);
    if (native.channels() == 1) {
      prepared.emplace_back(native);
    } else {
      std::vector<cv::Mat> channels;
      cv::split(native, channels);
      if (cv::norm(channels[0], channels[1], cv::NORM_INF) != 0 ||
          cv::norm(channels[0], channels[2], cv::NORM_INF) != 0)
        throw std::invalid_argument("LightCrafter sequences require monochrome patterns");
      prepared.emplace_back(channels[0]);
    }
  }
  return prepared;
}

std::vector<cv::Mat3b> packLightCrafter4500Patterns(const std::vector<cv::Mat1b> &nativePatterns) {
  if (nativePatterns.empty() || nativePatterns.size() > 48)
    throw std::invalid_argument("LC4500 needs between 1 and 48 eight-bit patterns");
  const auto size = nativePatterns.front().size();
  for (const auto &pattern : nativePatterns)
    if (pattern.empty() || pattern.dims != 2 || pattern.type() != CV_8UC1 || pattern.size() != size)
      throw std::invalid_argument("LC4500 native patterns must be matching byte images");
  std::vector<cv::Mat3b> splash;
  for (std::size_t index = 0; index < (nativePatterns.size() + 2) / 3; ++index)
    splash.emplace_back(size, cv::Vec3b(0, 0, 0));
  for (std::size_t index = 0; index < nativePatterns.size(); ++index)
    cv::insertChannel(nativePatterns[index], splash[index / 3], static_cast<int>((index + 1) % 3));
  return splash;
}

}