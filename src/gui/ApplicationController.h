#pragma once

#include "CloudRenderer.h"
#include "ExportPointCloudDialog.h"
#include "FilePickerDialog.h"
#include "FrameSequenceWindow.h"
#include "GeometricCalibrationDialog.h"
#include "IntensityCurveWindow.h"
#include "LightCrafter3000Window.h"
#include "LightCrafter4500Window.h"
#include "ProjectorPatternUploadDialog.h"
#include "MessageDialog.h"
#include "RenderSettingsWindow.h"
#include "SettingsWindow.h"
#include "CameraPreviewTexture.h"
#include "ScanSession.h"
#include "IntensityCalibrationDialog.h"
#include "ApplicationSettings.h"
#include "DecoderWorker.h"
#include "TriangulatorWorker.h"
#include "CaptureWorker.h"
#include "ProjectorPresentationController.h"
#include "ProjectorFactory.h"
#include "GeometricCalibration.h"
#include "CameraFactory.h"
#include "IntensityCalibrationLut.h"
#include "FrameSequence.h"
#include "Ros2Output.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace slstudio::gui {
using Clock = std::chrono::steady_clock;

struct GeometricCalibrationTaskResult {
  std::optional<GeometricCalibrationData> calibration;
  std::vector<GeometricCalibrationPoseDiagnostics> poses;
  std::string error;
};

struct CameraDiscoveryResult {
  std::vector<slstudio::CameraDescriptor> devices;
  int index = 0;
  std::vector<std::string> triggerSources;
  std::string triggerError;
  bool triggersInspected = false;
};

struct ApplicationController {
  ApplicationSettings settings;
  PointCloudRenderer renderer;
  PointCloudConstPtr cloud;
  std::unique_ptr<slstudio::Ros2Output> ros2Output;
  std::string ros2OutputError;
  std::array<char, 256> ros2Topic{"/slstudio/points"};
  std::array<char, 256> ros2FrameId{"slstudio_camera_optical_frame"};
  double ros2MetresPerUnit = 1.0;
  std::deque<Clock::time_point> cloudTimes;
  std::vector<cv::Mat> frames;
  std::optional<FrameSequence> lastSequence;
  FrameSequenceWindow frameSequenceWindow;
  std::unique_ptr<TriangulatorWorker> triangulator;
  std::unique_ptr<DecoderWorker> decoder;
  CaptureWorker acquisition;
  const int maximumPreviewSize;
  CaptureWorker::CameraFactory previewCamera;
  std::future<void> cameraJob, acquisitionJob;
  std::vector<std::future<void>> pendingAcquisitionJobs;
  bool recoveringAcquisition = false;
  CaptureWorker *cameraWorker = nullptr;
  cv::Size cameraResolution;
  bool cameraRequested = false;
  cv::Mat cameraPreview;
  cv::Size cameraPreviewFrameSize;
  int cameraPreviewFrameCount = 1, cameraNextPreviewFrame = 0;
  bool cameraPreviewDirty = false;
  std::deque<Clock::time_point> cameraPreviewTimes;
  ScanSession scan;
  CameraPreviewTexture cameraPreviewTexture;
  std::optional<ReconstructionSettings> pipelineSettings;
  std::string pipelineGeometricCalibrationPath;
  std::filesystem::file_time_type pipelineGeometricCalibrationTime{};
  bool sequencePending = false;
  std::future<GeometricCalibrationTaskResult> geometricCalibrationJob;
  std::optional<GeometricCalibrationData> pendingGeometricCalibration;
  GeometricCalibrationDialog geometricCalibrationDialog;
  IntensityCalibrationDialog intensityCalibration;
  IntensityCurveWindow intensityCurveWindow;
  std::vector<GeometricCalibrationPoseDiagnostics> geometricCalibrationDiagnostics;
  std::vector<std::string> geometricCalibrationPoseNames;
  std::string geometricCalibrationState = "Selected";
  std::vector<std::vector<cv::Mat>> geometricCalibrationPoses;
  std::optional<CaptureWorker::ProjectorInfo> projector;
  std::unique_ptr<ProjectorPresentationController> projectorPresentationController;
  bool scanAfterProjectorOpen = false;
  std::future<void> projectorJob;
  ProjectorPatternUploadDialog projectorPatternUploadDialog;
  std::string projectorJobMessage = "Projector patterns programmed";
  LightCrafter3000Window lc3000Window;
  LightCrafter4500Window lc4500Window;
  std::future<bool> lc4500ConnectionJob;
  std::optional<bool> lc4500Connected;
  std::future<slstudio::LightCrafter4500Firmware> lc4500FirmwareJob;
  std::optional<slstudio::LightCrafter4500Firmware> lc4500Firmware;
  std::string lc4500FirmwareError;
  Clock::time_point lc4500PollAfter{};
  std::future<slstudio::LightCrafter4500Status> lc4500PlaybackJob;
  std::optional<slstudio::LightCrafter4500Status> lc4500Playback;
  std::string lc4500PlaybackError, lc4500PlaybackMessage;
  std::function<void()> lc4500DeferredAction;
  Clock::time_point lc4500PlaybackPollAfter{};
  MessageDialog projectorUploadConfirmation;
  std::vector<slstudio::ProjectorOutputInfo> outputs;
  std::vector<slstudio::CameraBackend> cameraBackends;
  std::vector<slstudio::CameraDescriptor> cameraList;
  std::future<CameraDiscoveryResult> cameraDiscoveryJob;
  int cameraDiscoveryInterface = 0, cameraDiscoveryIndex = 0;
  std::optional<slstudio::CameraDescriptor> triggerSourcesCamera;
  std::vector<std::string> frameTriggerSources;
  std::string triggerSourcesError;
  ExportPointCloudDialog exportPointCloudDialog;
  FilePickerDialog filePickerDialog;
  RenderSettingsWindow renderSettingsWindow;
  SettingsWindow settingsWindow;
  std::string status = "Ready";
  bool settingsEnabled = true;
  std::string savedSettings;
  std::optional<std::filesystem::path> settingsDirectory;

  ApplicationController(bool smoke);
  bool processing() const;
  ~ApplicationController();
  static std::filesystem::path settingsPath();
  std::filesystem::path dataPath(const char *filename) const;
  void loadSettings();
  void saveSettings();
  void serializeSettings(cv::FileStorage &storage, bool reading);
  void applySettings(const ApplicationSettings &value);
  enum class FailurePolicy { Report, StopAcquisition };
  bool lc4500QueryActive() const { return lc4500FirmwareJob.valid() || lc4500PlaybackJob.valid(); }
  void afterLightCrafter4500Queries(std::function<void()> action, FailurePolicy policy = FailurePolicy::Report);
  template <typename Function> void attempt(Function action, FailurePolicy policy = FailurePolicy::Report) {
    try { action(); }
    catch (const std::exception &error) {
      status = error.what();
      if (policy == FailurePolicy::StopAcquisition) recoverAcquisition();
    }
  }
  void recoverAcquisition();
  void submitAcquisition(std::future<void> operation);
  void ensurePipeline();
  bool reconstructionBusy() const;
  void stopReconstruction();
  void reconstruct();
  void refreshOutputs();
  void openOutput();
  void closeOutput();
  cv::Size projectorSize() const;
  void validateProjectorCrop() const;
  std::vector<cv::Mat> projectorPatterns(bool calibration) const;
  void configureCodec(CodecType type, const CodecAlgorithmSettings &algorithmSettings);
  void startProjectorProgramming(bool calibration);
  void applyLightCrafterSettings(slstudio::OutputBackend backend);
  void controlLightCrafter4500(slstudio::LightCrafter4500Action action);
  void updateLightCrafter4500Settings();
  void tickLightCrafter4500();
  void beginCapture(bool geometricCalibration = false, bool continuous = false);
  void tick();
  void invalidateGeometricCalibration();
  void loadGeometricCalibrationPoses(const std::filesystem::path &folder);
  void startGeometricCalibration();
  void saveAndActivateGeometricCalibration();
  bool cameraSettingsControls();
  bool cameraTriggerControls();
  void setLastSequence(FrameSequence sequence);
  std::string frameSequenceWindowTitle() const;
  void startCamera(CaptureWorker::Reader reader = {});
  void openCalibrationDialog(bool intensityCalibrationDialog, CaptureWorker::Reader reader = {});
  bool cameraReady() const;
  void setCameraRequested(bool requested) { cameraRequested = requested; }
  std::future<void> releaseCameraWorker();
  bool requestCameraFrame();
  bool cameraFramePending() const;
  std::optional<cv::Mat> takeCameraFrame();
  void stopCamera();
  void setCameraPreviewFrameCount(int count);
  void setCameraPreviewFrameCount(int count, int configuredWidth);
  void appendCameraPreview(const cv::Mat &frame, Clock::time_point now = Clock::now());
  std::size_t cameraPreviewSampleCount() const { return cameraPreviewTimes.size(); }
  double cameraPreviewFps(Clock::time_point now = Clock::now()) const;
  void refreshCameras(std::function<CameraDiscoveryResult()> probe = {});
  void discoverCameras(bool enumerate, std::function<CameraDiscoveryResult()> probe = {});
  void refreshCameraTriggerSources(bool inspect = false);
  bool nativeCameraSelected() const { return settings.cameraInterface >= 0 && static_cast<std::size_t>(settings.cameraInterface) < cameraBackends.size(); }
  void loadSequenceFolder(const std::filesystem::path &directory);
  void stopCapture();
  void startScan();
  bool scanActive() const;
  void menuBar();
  void saveCloud();
  void openFilePicker(FilePickerMode mode);
  void browseDirectory(const std::filesystem::path &directory);
  void loadGeometricCalibrationFile(const std::filesystem::path &path);
  void prepareIntensityCalibration(CaptureWorker::Reader reader = {});
  void activateMeasuredLut(bool replaceExisting = false);
  void showProjectorLevel(std::uint8_t level);
  void startIntensityCalibration(CaptureWorker::Reader reader = {});
  void stopIntensity();
  void tickIntensity();
  void draw();
};
}
