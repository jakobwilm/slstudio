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
class ProjectorPatternUploadDialog {
public:
  struct Progress {
    std::mutex mutex;
    float fraction = 0;
    std::string stage = "Preparing patterns";
  };

  std::shared_ptr<Progress> progress;
  void draw(bool uploadActive);
};

}
