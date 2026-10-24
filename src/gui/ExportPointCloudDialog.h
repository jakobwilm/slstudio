#pragma once

#include "ApplicationSettings.h"
#include "CloudRenderer.h"
#include "FrameSequence.h"
#include "GeometricCalibration.h"
#include "Projector.h"
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
class ExportPointCloudDialog {
public:
  bool requested = false;
  std::string error;

  void draw(ApplicationSettings &settings, const PointCloudConstPtr &cloud,
      std::string &status, const std::function<void()> &save);
};

}
