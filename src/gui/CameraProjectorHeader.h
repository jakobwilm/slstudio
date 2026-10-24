#pragma once

#include <opencv2/core.hpp>
#include <functional>

namespace slstudio::gui {
class CameraProjectorHeader {
public:
  static void draw(cv::Size cameraSize, cv::Size projectorSize,
      const std::function<void()> &openSettings);
};
}
