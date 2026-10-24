#pragma once

#include "ApplicationSettings.h"
#include "CloudRenderer.h"
#include "FrameSequence.h"
#include "GeometricCalibration.h"
#include "ProjectorLightCrafter3000.h"
#include "CaptureWorker.h"
#include "ScanSession.h"
#include <GL/glew.h>
#include <array>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace slstudio::gui {
class LightCrafter3000Window {
public:
  bool open = false;
  cv::Size viewportSize;

  void draw(ApplicationSettings &settings, bool available, bool busy, bool projectorOpen,
      const std::function<void()> &apply);
};

}
