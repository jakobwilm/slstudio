#pragma once

#include "ApplicationSettings.h"
#include "CloudRenderer.h"
#include "FrameSequence.h"
#include "GeometricCalibration.h"
#include "ProjectorLightCrafter4500.h"
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
class LightCrafter4500Window {
public:
  bool open = false;
  cv::Size viewportSize;

  void draw(ApplicationSettings &settings, bool available, const std::optional<bool> &connected,
      const std::optional<slstudio::LightCrafter4500Firmware> &firmware, bool firmwareQueryActive,
      const std::string &firmwareError, bool busy, bool projectorOpen,
      const std::function<void()> &apply, const std::function<void()> &retry,
      const std::optional<slstudio::LightCrafter4500Status> &playback, const std::string &playbackError,
      const std::function<void(slstudio::LightCrafter4500Action)> &control,
      const std::function<void()> &refreshPlayback);
};

}
