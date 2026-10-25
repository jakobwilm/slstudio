#include "ApplicationController.h"
#include "CameraProjectorHeader.h"
#include <imgui.h>

namespace slstudio::gui {
namespace {
int maximumTextureSize() {
  GLint size = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &size);
  return size;
}


}

ApplicationController::ApplicationController(bool smoke) : maximumPreviewSize(maximumTextureSize()) {
  if (maximumPreviewSize <= 0) throw std::invalid_argument("Camera preview size limit must be positive");
  settingsEnabled = !smoke;
  const std::filesystem::path source(SLSTUDIO_SOURCE_DIR);
  const bool examplesAvailable = std::filesystem::is_directory(source / "example_data");
  const auto sequence = examplesAvailable ? source / "example_data/skull_2x3" : dataPath("sequences");
  const auto calibration = examplesAvailable ? source / "example_data/calibration/calibration.xml" : dataPath("calibration.xml");
  std::snprintf(settings.sequencePath.data(), settings.sequencePath.size(), "%s", sequence.string().c_str());
  std::snprintf(settings.geometricCalibrationPath.data(), settings.geometricCalibrationPath.size(), "%s", calibration.string().c_str());
  std::snprintf(settings.exportPath.data(), settings.exportPath.size(), "pointCloud.ply");
  std::snprintf(settings.projectorAddress.data(), settings.projectorAddress.size(), "192.168.1.100");
  settings.reconstruction.points.maxDepth = 10000.0f;
  std::snprintf(settings.frameTriggerSource.data(), settings.frameTriggerSource.size(), "Line1");
  for (const auto backend : {slstudio::CameraBackend::Aravis, slstudio::CameraBackend::Ximea, slstudio::CameraBackend::Spinnaker})
    if (slstudio::cameraBackendAvailable(backend)) cameraBackends.push_back(backend);
  if (nativeCameraSelected()) settings.exposure = 16.667f;
  if (!smoke) attempt([&] {
    const auto calibration = dataPath("calibration.xml");
    if (std::filesystem::exists(calibration))
      std::snprintf(settings.geometricCalibrationPath.data(), settings.geometricCalibrationPath.size(), "%s", calibration.string().c_str());
    loadSettings();
  });
  if (!smoke) attempt([&] { refreshCameras(); });
  attempt([&] { refreshOutputs(); });
}

bool ApplicationController::reconstructionBusy() const {
  return (decoder && decoder->busy()) || (triangulator && triangulator->busy());
}

void ApplicationController::stopReconstruction() {
  if (decoder) decoder->cancel();
  if (triangulator) triangulator->cancel();
  decoder.reset();
  triangulator.reset();
}

bool ApplicationController::processing() const {
  return intensityCalibration.worker || sequencePending || scan.capture() || (scan.continuous() && cameraWorker) || reconstructionBusy() ||
      cameraJob.valid() || acquisitionJob.valid() || !pendingAcquisitionJobs.empty() || recoveringAcquisition ||
      geometricCalibrationJob.valid() || projectorJob.valid() ||
      (lc4500PlaybackJob.valid() && !lc4500PlaybackMessage.empty()) || lc4500DeferredAction || cameraDiscoveryJob.valid();
}

void ApplicationController::recoverAcquisition() {
  lc4500DeferredAction = {};
  scan.stop();
  sequencePending = false;
  intensityCalibration.cancel();
  stopCamera();
  stopReconstruction();
  scanAfterProjectorOpen = false;
  recoveringAcquisition = true;
  for (auto *job : {&cameraJob, &acquisitionJob, &projectorJob})
    if (job->valid()) pendingAcquisitionJobs.push_back(std::move(*job));
  pendingAcquisitionJobs.push_back(acquisition.closeCamera());
  pendingAcquisitionJobs.push_back(acquisition.stopSequence());
}

void ApplicationController::submitAcquisition(std::future<void> operation) {
  if (acquisitionJob.valid()) pendingAcquisitionJobs.push_back(std::move(acquisitionJob));
  acquisitionJob = std::move(operation);
}

ApplicationController::~ApplicationController() {
  if (projectorPresentationController) projectorPresentationController->cancel();
  stopCamera();
  if (cameraDiscoveryJob.valid()) cameraDiscoveryJob.wait();
  intensityCalibration.cancel();
  scan.stop();
  stopReconstruction();
  if (projectorJob.valid()) projectorJob.wait();
  if (lc4500FirmwareJob.valid()) lc4500FirmwareJob.wait();
  if (lc4500PlaybackJob.valid()) lc4500PlaybackJob.wait();
  acquisition.cancel();
  geometricCalibrationDialog.release();
  cameraPreviewTexture.release();
  frameSequenceWindow.release();
}

std::filesystem::path ApplicationController::settingsPath() {
  if (const auto *config = std::getenv("XDG_CONFIG_HOME"))
    return std::filesystem::path(config) / "slstudio/settings.yml";
  const auto *home = std::getenv("HOME");
  if (!home) home = std::getenv("USERPROFILE");
  if (!home) throw std::runtime_error("No user settings directory is available");
  return std::filesystem::path(home) / ".config/slstudio/settings.yml";
}

std::filesystem::path ApplicationController::dataPath(const char *filename) const {
  return (settingsDirectory ? *settingsDirectory : settingsPath().parent_path()) / filename;
}

void ApplicationController::applySettings(const ApplicationSettings &value) {
  setCameraPreviewFrameCount(value.previewFrameCount, value.cameraWidth);
  settings = value;
}

void ApplicationController::serializeSettings(cv::FileStorage &storage, bool reading) {
  if (reading) applySettings(settings.read(storage));
  else settings.write(storage);
}

void ApplicationController::loadSettings() {
  const auto path = dataPath("settings.yml");
  if (!std::filesystem::exists(path)) return;
  cv::FileStorage storage(path.string(), cv::FileStorage::READ);
  if (!storage.isOpened()) throw std::runtime_error("Cannot read settings");
  serializeSettings(storage, true);
  settings.directionIndex = std::clamp(settings.directionIndex, 0, 2);
  settings.reconstruction.coding.direction = static_cast<CodecDir>(settings.directionIndex + 1);
  settings.cameraInterface = std::clamp(settings.cameraInterface, 0, static_cast<int>(cameraBackends.size()));
  settings.reconstruction.minimumModulation = std::clamp(settings.reconstruction.minimumModulation, 0, 255);
  validateReconstructionSettings(settings.reconstruction);
  settings.settleMilliseconds = std::clamp(settings.settleMilliseconds, 0, 1000);
  const auto intensityPath = dataPath("intensity-lut.yml");
  if (std::filesystem::exists(intensityPath)) {
    intensityCalibration.samples = loadIntensitySamples(intensityPath);
    intensityCalibration.result = analyzeIntensity(intensityCalibration.samples);
  }
}

void ApplicationController::saveSettings() {
  if (!settingsEnabled) return;
  cv::FileStorage snapshot("settings.yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  serializeSettings(snapshot, false);
  auto serialized = snapshot.releaseAndGetString();
  if (serialized == savedSettings) return;
  const auto path = dataPath("settings.yml");
  std::filesystem::create_directories(path.parent_path());
  cv::FileStorage storage(path.string(), cv::FileStorage::WRITE);
  if (!storage.isOpened()) throw std::runtime_error("Cannot save settings");
    serializeSettings(storage, false);
    storage.release();
    savedSettings = std::move(serialized);
}


void ApplicationController::draw() {
  const auto *viewport = ImGui::GetMainViewport();
  const float statusHeight = ImGui::GetFrameHeight() + 2 * ImGui::GetStyle().WindowPadding.y;
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, viewport->WorkSize.y - statusHeight));
  ImGui::Begin("SLStudio workspace", nullptr, ImGuiWindowFlags_NoDecoration |
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
               ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_MenuBar);
  menuBar();
  ImGui::BeginChild("Point cloud", ImVec2(0, 0));
  auto available = ImGui::GetContentRegionAvail();
  const auto scale = ImGui::GetIO().DisplayFramebufferScale;
  const auto texture = renderer.render(static_cast<int>(available.x * scale.x),
                                       static_cast<int>(available.y * scale.y), settings.rendering);
  ImGui::Image(static_cast<ImTextureID>(texture), available, ImVec2(0, 1), ImVec2(1, 0));
  double pcps = 0;
  if (cloudTimes.size() > 1 && Clock::now() - cloudTimes.back() < std::chrono::seconds(2)) {
    const double elapsed = std::chrono::duration<double>(cloudTimes.back() - cloudTimes.front()).count();
    if (elapsed > 0) pcps = (cloudTimes.size() - 1) / elapsed;
  }
  char rate[32];
  std::snprintf(rate, sizeof(rate), "PCPS: %.2f", pcps);
  const auto corner = ImGui::GetItemRectMax();
  const auto textSize = ImGui::CalcTextSize(rate);
  const ImVec2 ratePosition(corner.x - textSize.x - 8, corner.y - textSize.y - 8);
  ImGui::GetWindowDrawList()->AddText(ImVec2(ratePosition.x + 1, ratePosition.y + 1), IM_COL32(0, 0, 0, 255), rate);
  ImGui::GetWindowDrawList()->AddText(ratePosition, IM_COL32(240, 240, 240, 255), rate);
  if (ImGui::IsItemHovered()) {
    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      const auto origin = ImGui::GetItemRectMin();
      const auto mouse = ImGui::GetMousePos();
      renderer.setRotationCenter((mouse.x - origin.x) / available.x, (mouse.y - origin.y) / available.y);
    }
    if (ImGui::GetIO().MouseWheel != 0)
      renderer.zoom(ImGui::GetIO().MouseWheel);
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      renderer.yaw += ImGui::GetIO().MouseDelta.x * 0.01f;
      renderer.pitch += ImGui::GetIO().MouseDelta.y * 0.01f;
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
      renderer.pan(ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y, available.y);
  }
  ImGui::EndChild();
  filePickerDialog.draw(lastSequence ? &*lastSequence : nullptr, processing(), scanActive(), status,
      [this](const auto &directory) { browseDirectory(directory); },
      [this](const auto &path) { loadGeometricCalibrationFile(path); },
      [this](const auto &directory) { loadSequenceFolder(directory); },
      [](const auto &directory, const auto &sequence) { saveSequence(directory, sequence); });
  exportPointCloudDialog.draw(settings, cloud, status, [this] { saveCloud(); });
  ImGui::End();
  SettingsWindowContext settingsContext{
      settings, outputs, projector, projectorJob, processing() || scan.index() >= 0,
      cameraRequested, projectorUploadConfirmation, intensityCurveWindow,
      [this](const std::function<void()> &action, bool stopAcquisition) {
        attempt(action, stopAcquisition ? FailurePolicy::StopAcquisition : FailurePolicy::Report);
      },
      [this](CodecType type, const CodecAlgorithmSettings &algorithmSettings) {
        configureCodec(type, algorithmSettings);
      },
      [this] { return cameraSettingsControls(); },
      [this] { return cameraTriggerControls(); },
      [this] { invalidateGeometricCalibration(); },
      [this] { startCamera(); },
      [this] { refreshOutputs(); },
      [this] { afterLightCrafter4500Queries([this] { openOutput(); }); },
      [this] { closeOutput(); },
      [this] { validateProjectorCrop(); },
      [this](bool white) { return acquisition.blankProjector(white); },
      [this](bool calibration) { startProjectorProgramming(calibration); }};
  settingsWindow.draw(settings.settingsWindowOpen, settingsContext);
  renderSettingsWindow.draw(settings, cloud, [this] { renderer.resetView(); });
  GeometricCalibrationDialogContext geometricContext{
      settings, cameraWorker, cameraRequested, cameraReady(), cameraPreview, cameraPreviewFrameSize,
      cameraPreviewFrameCount, cameraNextPreviewFrame, cameraPreviewDirty, cameraPreviewTexture,
      scan, geometricCalibrationDiagnostics, geometricCalibrationPoseNames,
      geometricCalibrationPoses, pendingGeometricCalibration, processing() || scan.index() >= 0, projector.has_value(),
      geometricCalibrationJob.valid(),
      [this] {
        const auto cameraSize = cameraPreviewFrameSize.empty()
            ? cv::Size(settings.cameraWidth, settings.cameraHeight) : cameraPreviewFrameSize;
        CameraProjectorHeader::draw(cameraSize, projectorSize(), [this] { settings.settingsWindowOpen = true; });
      },
      [this] { invalidateGeometricCalibration(); },
      [this] { attempt([&] { beginCapture(true); }, FailurePolicy::StopAcquisition); },
      [this] { attempt([&] { startGeometricCalibration(); }); },
      [this] { attempt([&] { saveAndActivateGeometricCalibration(); }); },
      [this] { attempt([&] { stopCapture(); }, FailurePolicy::StopAcquisition); }};
  geometricCalibrationDialog.draw(geometricContext);
  IntensityCalibrationDialogContext intensityContext{
      settings, cameraWorker, cameraRequested, cameraReady(), cameraPreview, cameraPreviewFrameSize,
      cameraPreviewFrameCount, cameraNextPreviewFrame, cameraPreviewDirty, cameraPreviewTexture,
      processing(), scanActive(), projector.has_value(), status,
      [this] {
        const auto cameraSize = cameraPreviewFrameSize.empty()
            ? cv::Size(settings.cameraWidth, settings.cameraHeight) : cameraPreviewFrameSize;
        CameraProjectorHeader::draw(cameraSize, projectorSize(), [this] { settings.settingsWindowOpen = true; });
      },
      [this] { attempt([&] { startIntensityCalibration(); }, FailurePolicy::StopAcquisition); },
      [this] { attempt([&] { stopIntensity(); status = "Measurement cancelled"; }, FailurePolicy::StopAcquisition); },
      [this] { attempt([&] { activateMeasuredLut(); }); },
      [this] { attempt([&] { activateMeasuredLut(true); }); }};
  intensityCalibration.draw(intensityContext);
  intensityCurveWindow.draw(settings);
  lc3000Window.draw(settings,
      slstudio::lightCrafterAvailable(slstudio::OutputBackend::LightCrafter3000),
      processing() || scan.index() >= 0 || projector.has_value(), projector.has_value(),
      [this] { attempt([&] { applyLightCrafterSettings(slstudio::OutputBackend::LightCrafter3000); }); });
  updateLightCrafter4500Settings();
  frameSequenceWindow.draw(lastSequence ? &*lastSequence : nullptr,
      [this](const std::function<void()> &action) { attempt(action); }, frameSequenceWindowTitle());
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - statusHeight));
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, statusHeight));
  ImGui::Begin("Status bar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing);
  if (scan.continuous()) {
    const auto capture = scan.statistics();
    const auto processingDrops = decoder && triangulator ?
        decoder->statistics().dropped + triangulator->statistics().dropped : 0;
    const struct {
      const char *label;
      unsigned long long value;
    } fields[] = {
        {"Sequence no.:", static_cast<unsigned long long>(capture.completed)},
        {"Capture drops:", static_cast<unsigned long long>(capture.dropped)},
        {"Processing drops:", static_cast<unsigned long long>(processingDrops)},
        {"Points:", static_cast<unsigned long long>(cloud ? cloud->points.size() : 0)}};
    if (ImGui::BeginTable("Continuous scan status", IM_ARRAYSIZE(fields) + 1,
            ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoPadOuterX | ImGuiTableFlags_BordersInnerV)) {
      const float numberWidth = ImGui::CalcTextSize("00000000").x;
      for (const auto &field : fields)
        ImGui::TableSetupColumn(field.label, ImGuiTableColumnFlags_WidthFixed,
            ImGui::CalcTextSize(field.label).x + ImGui::GetStyle().ItemSpacing.x + numberWidth);
      ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch);
      for (const auto &field : fields) {
        ImGui::TableNextColumn();
        ImGui::Text("%s %8llu", field.label, field.value);
      }
      ImGui::TableNextColumn();
      if (status != "Capturing") ImGui::TextUnformatted(status.c_str());
      ImGui::EndTable();
    }
  } else {
    ImGui::TextUnformatted(status.c_str());
    if (scan.capture()) {
      ImGui::SameLine();
      ImGui::Text("| Sequences: %llu, capture drops: %llu",
          static_cast<unsigned long long>(scan.statistics().completed),
          static_cast<unsigned long long>(scan.statistics().dropped));
    }
    if (decoder && triangulator) {
      ImGui::SameLine();
      ImGui::Text("| Processing drops: %llu", static_cast<unsigned long long>(
          decoder->statistics().dropped + triangulator->statistics().dropped));
    }
  }
  ImGui::End();
  projectorPatternUploadDialog.draw(projectorJob.valid());
  try { saveSettings(); }
  catch (const std::exception &error) { status = error.what(); }
}

}
