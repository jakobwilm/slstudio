#pragma once

#include "CaptureWorker.h"
#include "IntensityCalibrationLut.h"
#include "ApplicationSettings.h"
#include "CameraPreviewTexture.h"
#include "MessageDialog.h"
#include <GL/glew.h>
#include <array>
#include <chrono>
#include <functional>
#include <string>

namespace slstudio::gui {
struct IntensityCalibrationDialogContext {
  ApplicationSettings &settings;
  CaptureWorker *cameraWorker;
  bool cameraRequested, cameraReady;
  cv::Mat &cameraPreview;
  cv::Size &cameraPreviewFrameSize;
  int &cameraPreviewFrameCount, &cameraNextPreviewFrame;
  bool &cameraPreviewDirty;
  CameraPreviewTexture &cameraPreviewTexture;
  bool processing = false, scanActive = false, projectorOpen = false;
  std::string &status;
  std::function<void()> drawCameraProjectorHeader;
  std::function<void()> start;
  std::function<void()> cancel;
  std::function<void()> accept;
  std::function<void()> replace;
};

class IntensityCalibrationDialog {
public:
  using Clock = std::chrono::steady_clock;
  enum class Progress { Waiting, NextLevel, Complete };

  CaptureWorker *worker = nullptr;
  cv::Mat preview;
  std::array<int, 4> roi{};
  bool open = false, measuring = false, frameRequested = false, measureRequested = false, compensated = false;
  std::vector<IntensitySample> samples;
  std::optional<IntensityCalibrationLut> result;
  MessageDialog replaceCurveDialog;
  cv::Size viewportSize;
  cv::Point dragOrigin;

  void prepare(CaptureWorker &capture);
  void start(bool compensationEnabled, int inputLevels, int framesPerLevel);
  Progress consume(cv::Mat frame);
  void clearResult() { samples.clear(); result.reset(); }
  int level() const;
  float progress() const {
    return measureRequested ? 0 : static_cast<float>(levelIndex * averages + frames) / (levels * averages);
  }
  void requestIfDue();
  void cancel();
  void draw(const IntensityCalibrationDialogContext &context);

private:
  int levels = 17, averages = 4;
  int frames = 0, levelIndex = 0;
  double brightnessSum = 0, saturationSum = 0;
  Clock::time_point after;
};
}
