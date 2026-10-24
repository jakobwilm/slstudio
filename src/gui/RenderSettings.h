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
}
