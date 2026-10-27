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
  std::string ipChangeStatus;
  std::optional<slstudio::LightCrafter3000DeviceState> deviceState;
  std::string diagnosticsStatus;

  void populateControls(ApplicationSettings &settings, const slstudio::LightCrafter3000DeviceState &state);
  void draw(ApplicationSettings &settings, bool available, const std::optional<bool> &connected,
      bool connectionCheckPaused, const std::array<std::string, 3> &firmware,
      const std::string &connectionError, bool busy, bool projectorOpen,
      const std::function<void()> &apply, const std::function<void(const std::string &)> &changeIp,
      const std::function<void(const slstudio::LightCrafter3000Control &)> &control);

private:
  enum class Mode { Hdmi, TestPattern, SolidColor, PatternSequence };
  Mode mode = Mode::Hdmi;
  std::array<char, 64> newIpAddress{};
  slstudio::LightCrafter3000Control manualControl;
};

}
