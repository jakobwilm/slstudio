#pragma once

#include "ApplicationSettings.h"
#include "CloudRenderer.h"
#include "FrameSequence.h"
#include "GeometricCalibration.h"
#include "ProjectorFactory.h"
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
#include "IntensityCurveWindow.h"
#include "MessageDialog.h"

namespace slstudio::gui {
struct SettingsWindowContext {
  using Attempt = std::function<void(const std::function<void()> &, bool)>;

  ApplicationSettings &settings;
  std::vector<slstudio::ProjectorOutputInfo> &outputs;
  std::optional<CaptureWorker::ProjectorInfo> &projector;
  std::future<void> &projectorJob;
  bool busy = false, cameraRequested = false;
  MessageDialog &uploadConfirmation;
  IntensityCurveWindow &intensityCurveWindow;
  Attempt attempt;
  std::function<void(CodecType, const CodecAlgorithmSettings &)> configureCodec;
  std::function<bool()> cameraSettingsControls;
  std::function<bool()> cameraTriggerControls;
  std::function<void()> invalidateGeometricCalibration;
  std::function<void()> startCamera;
  std::function<void()> refreshOutputs;
  std::function<void()> openOutput;
  std::function<void()> closeOutput;
  std::function<void()> validateProjectorCrop;
  std::function<std::future<void>(bool)> blankProjector;
  std::function<void(bool)> startProjectorProgramming;
};

class SettingsWindow {
public:
  void draw(bool &open, SettingsWindowContext &context);

private:
  bool calibrationPatterns_ = false;
  void drawControls(SettingsWindowContext &context);
};

}
