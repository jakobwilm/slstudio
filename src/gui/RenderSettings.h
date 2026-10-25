#pragma once

#include <opencv2/core.hpp>

namespace slstudio::gui {
struct RenderSettings {
  enum class ColorMode { Modulation = 0, Depth = 1, Solid = 2 };
  float pointSize = 2.0f;
  ColorMode colorMode = ColorMode::Modulation;
  cv::Vec3f solidColor{0.8f, 0.8f, 0.8f};
  bool surfaceShading = false;
};

inline bool operator==(const RenderSettings &a, const RenderSettings &b) {
  return a.pointSize == b.pointSize && a.colorMode == b.colorMode &&
      a.solidColor == b.solidColor && a.surfaceShading == b.surfaceShading;
}
}
