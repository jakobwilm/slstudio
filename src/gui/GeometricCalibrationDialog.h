#pragma once

#include "ApplicationSettings.h"
#include "CloudRenderer.h"
#include "FrameSequence.h"
#include "GeometricCalibration.h"
#include "Projector.h"
#include "CaptureWorker.h"
#include "ScanSession.h"
#include "CameraPreviewTexture.h"
#include "MessageDialog.h"
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
struct GeometricCalibrationDialogContext {
  ApplicationSettings &settings;
  CaptureWorker *cameraWorker;
  bool cameraRequested, cameraReady;
  cv::Mat &cameraPreview;
  cv::Size &cameraPreviewFrameSize;
  int &cameraPreviewFrameCount, &cameraNextPreviewFrame;
  bool &cameraPreviewDirty;
  CameraPreviewTexture &cameraPreviewTexture;
  ScanSession &scan;
  std::vector<GeometricCalibrationPoseDiagnostics> &diagnostics;
  std::vector<std::string> &poseNames;
  std::vector<std::vector<cv::Mat>> &poses;
  std::optional<GeometricCalibrationData> &pendingCalibration;
  bool busy = false, projectorOpen = false, calibrating = false;
  std::function<void()> drawCameraProjectorHeader;
  std::function<void()> invalidate;
  std::function<void()> capturePose;
  std::function<void()> calibrate;
  std::function<void()> accept;
  std::function<void()> stopCapture;
};

class GeometricCalibrationDialog {
public:
  bool open = false;
  int selectedPose = -1;
  cv::Size viewportSize;

  void draw(GeometricCalibrationDialogContext &context);
  void selectPose(int pose, int frame = -1) {
    selectedPose = pose;
    selectedFrame_ = frame;
    invalidatePreview();
  }
  void invalidatePreview() { previewDirty_ = true; }
  void refreshPreview(const ApplicationSettings &settings,
      const std::vector<GeometricCalibrationPoseDiagnostics> &diagnostics,
      const std::vector<std::vector<cv::Mat>> &poses);
  GLuint previewTexture() const { return previewTexture_; }
  cv::Size previewSize() const { return previewSize_; }
  void release();

private:
  void updatePreview(const GeometricCalibrationDialogContext &context);
  MessageDialog removePoseConfirmation_, clearPosesConfirmation_;
  int selectedFrame_ = -1;
  GLuint previewTexture_ = 0;
  cv::Size previewSize_;
  bool previewDirty_ = true;
};

}
