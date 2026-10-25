#include "ApplicationController.h"
#include <imgui.h>

namespace slstudio::gui {
void ApplicationController::afterLightCrafter4500Queries(std::function<void()> action, FailurePolicy policy) {
  if (!lc4500QueryActive()) {
    attempt(action, policy);
    return;
  }
  attempt([&] {
    if (lc4500DeferredAction) throw std::logic_error("Wait for the pending projector action");
    lc4500DeferredAction = [this, action = std::move(action), policy] { attempt(action, policy); };
    status = "Waiting for LC4500 device query";
  });
}

void ApplicationController::invalidateGeometricCalibration() {
  pendingGeometricCalibration.reset();
  geometricCalibrationDiagnostics.clear();
  geometricCalibrationDialog.invalidatePreview();
}

void ApplicationController::loadGeometricCalibrationPoses(const std::filesystem::path &folder) {
  std::vector<std::filesystem::path> directories;
  for (const auto &entry : std::filesystem::directory_iterator(folder))
    if (entry.is_directory() && entry.path().filename().string().rfind("calseq", 0) == 0)
      directories.push_back(entry.path());
  std::sort(directories.begin(), directories.end());
  if (directories.empty()) throw std::runtime_error("No calseq pose folders found");
  std::vector<std::vector<cv::Mat>> loaded;
  std::vector<std::string> names;
  for (const auto &directory : directories) {
    loaded.push_back(loadFrameSequence(directory));
    names.push_back(directory.filename().string());
  }
  geometricCalibrationPoses = std::move(loaded);
  geometricCalibrationPoseNames = std::move(names);
  geometricCalibrationDialog.selectPose(0);
  GeometricCalibrationData stored;
  if (!projector && std::filesystem::exists(folder / "calibration.xml") &&
      stored.load(folder / "calibration.xml")) {
    settings.projectorWidth = stored.screenResX;
    settings.projectorHeight = stored.screenResY;
  }
  invalidateGeometricCalibration();
  status = std::to_string(geometricCalibrationPoses.size()) + " calibration poses loaded";
}

void ApplicationController::startGeometricCalibration() {
  if (processing() || scan.index() >= 0) throw std::runtime_error("Capture or processing is already running");
  invalidateGeometricCalibration();
  const auto poses = geometricCalibrationPoses;
  const auto requestedSettings = settings.geometricCalibrationSettings;
  validateProjectorCrop();
  const cv::Size size = projectorSize();
  geometricCalibrationJob = std::async(std::launch::async, [poses, size, requestedSettings] {
    GeometricCalibrationTaskResult result;
    try { result.calibration = calibrateGeometry(poses, size, requestedSettings, &result.poses); }
    catch (const std::exception &error) { result.error = error.what(); }
    return result;
  });
  status = "Calibrating";
}

void ApplicationController::saveAndActivateGeometricCalibration() {
  if (!pendingGeometricCalibration || processing() || scan.index() >= 0)
    throw std::runtime_error("No completed calibration is available");
  const auto path = dataPath("calibration.xml");
  std::filesystem::create_directories(path.parent_path());
  if (!pendingGeometricCalibration->save(path)) throw std::runtime_error("Cannot save calibration XML");
  std::snprintf(settings.geometricCalibrationPath.data(), settings.geometricCalibrationPath.size(), "%s", path.string().c_str());
  geometricCalibrationState = "Active";
  saveSettings();
  status = "Geometric Calibration saved and activated";
  geometricCalibrationDialog.open = false;
}

void ApplicationController::setLastSequence(FrameSequence sequence) {
  if (sequence.frames.empty()) throw std::invalid_argument("Frame sequence must contain at least one frame");
  const auto size = sequence.frames.front().size();
  for (const auto &frame : sequence.frames)
    if (frame.empty() || frame.dims != 2 || frame.type() != CV_8UC1 || frame.size() != size)
      throw std::invalid_argument("Frame sequence requires matching Mono8 frames");
  lastSequence = std::move(sequence);
  frameSequenceWindow.markDirty();
}

std::string ApplicationController::frameSequenceWindowTitle() const {
  if (!lastSequence) return "Frame sequence###FrameSequence";
  const auto &frames = lastSequence->frames;
  return "Frame sequence | " + std::to_string(frames.size()) + " frames | " +
      std::to_string(frames.front().cols) + " x " + std::to_string(frames.front().rows) + "###FrameSequence";
}


void ApplicationController::tickLightCrafter4500() {
  const auto now = Clock::now();
  auto ready = [](const auto &job) {
    return job.valid() && job.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  };
  if (ready(lc4500FirmwareJob)) {
    try {
      lc4500Firmware = lc4500FirmwareJob.get();
      lc4500FirmwareError.clear();
    } catch (const std::exception &error) {
      lc4500FirmwareError = status = error.what();
    }
  }
  if (ready(lc4500PlaybackJob)) {
    try {
      lc4500Playback = lc4500PlaybackJob.get();
      lc4500PlaybackError.clear();
      if (!lc4500PlaybackMessage.empty()) status = lc4500PlaybackMessage;
    } catch (const std::exception &error) {
      lc4500Playback.reset();
      lc4500PlaybackError = status = error.what();
    }
    lc4500PlaybackMessage.clear();
    lc4500PlaybackPollAfter = now + std::chrono::seconds(1);
  }
  if (lc4500DeferredAction && !lc4500QueryActive()) {
    auto action = std::exchange(lc4500DeferredAction, {});
    action();
  }
  if (projector || scan.index() >= 0) {
    lc4500Playback.reset();
    lc4500PlaybackError.clear();
    lc4500PlaybackPollAfter = {};
  }
  if (ready(lc4500ConnectionJob)) {
    try {
      lc4500Connected = lc4500ConnectionJob.get();
      if (!*lc4500Connected) {
        lc4500Firmware.reset();
        lc4500FirmwareError.clear();
        lc4500Playback.reset();
        lc4500PlaybackError.clear();
      }
    }
    catch (const std::exception &error) {
      lc4500Connected = false;
      lc4500Playback.reset();
      lc4500FirmwareError = status = error.what();
    }
    lc4500PollAfter = now + std::chrono::seconds(1);
  }
  if (!lc4500Window.open || !slstudio::lightCrafterAvailable(slstudio::OutputBackend::LightCrafter4500)) return;
  if (!lc4500ConnectionJob.valid() && now >= lc4500PollAfter)
    lc4500ConnectionJob = std::async(std::launch::async, slstudio::lightCrafter4500Connected);
  if (!lc4500Connected.value_or(false) || lc4500QueryActive() || processing() || scan.index() >= 0 || projector) return;
  if (!lc4500Firmware && lc4500FirmwareError.empty())
    lc4500FirmwareJob = std::async(std::launch::async, [] { return slstudio::lightCrafter4500Firmware(); });
  else if (lc4500PlaybackError.empty() && now >= lc4500PlaybackPollAfter)
    lc4500PlaybackJob = std::async(std::launch::async, [] { return slstudio::lightCrafter4500Status(); });
}

void ApplicationController::updateLightCrafter4500Settings() {
  const bool available = slstudio::lightCrafterAvailable(slstudio::OutputBackend::LightCrafter4500);
  lc4500Window.draw(settings, available, lc4500Connected, lc4500Firmware,
      lc4500FirmwareJob.valid(), lc4500FirmwareError,
      processing() || scan.index() >= 0 || projector.has_value(), projector.has_value(),
      [this] { afterLightCrafter4500Queries([this] { applyLightCrafterSettings(slstudio::OutputBackend::LightCrafter4500); }); },
      [this] { lc4500FirmwareError.clear(); },
      lc4500Playback, lc4500PlaybackError,
      [this](auto action) { afterLightCrafter4500Queries([this, action] { controlLightCrafter4500(action); }); },
      [this] { lc4500PlaybackError.clear(); lc4500PlaybackPollAfter = {}; });
}


}
