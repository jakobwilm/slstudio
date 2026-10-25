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

struct ImVec2;

namespace slstudio::gui {
inline constexpr const char *desiredIntensityAxis = "Desired intensity (0-255)";
inline constexpr const char *projectorInputAxis = "Projector input (0-255)";
inline constexpr const char *cameraIntensityAxis = "Camera intensity (0-255)";

void drawIntensityPlot(const char *label, const float *values, int count, ImVec2 size,
    const char *xLabel = desiredIntensityAxis, const char *yLabel = projectorInputAxis,
    const int *levels = nullptr);

class IntensityCurveWindow {
public:
  bool open = false;
  cv::Size viewportSize;

  void draw(const ApplicationSettings &settings);
};

}
