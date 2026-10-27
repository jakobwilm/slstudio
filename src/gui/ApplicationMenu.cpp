#include "ApplicationController.h"
#include <imgui.h>

namespace slstudio::gui {

bool ApplicationController::scanActive() const {
  return scan.index() >= 0 || sequencePending || scan.capture() ||
      reconstructionBusy();
}

void ApplicationController::startScan() {
  if (processing() || scan.index() >= 0)
    throw std::logic_error("Stop capture and processing before starting a scan");
  withProjector([this] { beginCapture(false, settings.continuousCapture); });
}

void ApplicationController::saveCloud() {
  if (!cloud || cloud->points.empty()) throw std::logic_error("No point cloud to save");
  const std::filesystem::path path(settings.exportPath.data());
  bool saved = false;
  if (path.extension() == ".ply") saved = savePointCloudPLY(path.string(), *cloud);
  else if (path.extension() == ".pcd") saved = savePointCloudPCD(path.string(), *cloud);
  else throw std::runtime_error("Export extension must be .ply or .pcd");
  if (!saved) throw std::runtime_error("Cannot write point cloud");
  status = "Saved " + path.string();
}

void ApplicationController::menuBar() {
  const bool canStart = !processing() && scan.index() < 0;
  const bool canStop = scanActive();
  auto stop = [&] { stopCapture(); status = "Scan stopped"; };
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("Scan")) {
      if (ImGui::MenuItem("Start Scan", nullptr, false, canStart))
        afterLightCrafter4500Queries([this] { startScan(); }, FailurePolicy::StopAcquisition);
      if (ImGui::MenuItem("Stop Scan", nullptr, false, canStop)) attempt(stop, FailurePolicy::StopAcquisition);
      ImGui::Separator();
      if (ImGui::MenuItem("Load Sequence", nullptr, false, canStart))
        openFilePicker(FilePickerMode::LoadSequence);
      if (ImGui::MenuItem("Save Sequence", nullptr, false, lastSequence && !lastSequence->frames.empty()))
        openFilePicker(FilePickerMode::SaveSequence);
      ImGui::Separator();
      if (ImGui::MenuItem("Export Point Cloud", nullptr, false, cloud && !cloud->points.empty())) {
        exportPointCloudDialog.requested = true;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Calibration")) {
      if (ImGui::MenuItem("Load Geometric Calibration", nullptr, false, canStart)) openFilePicker(FilePickerMode::GeometricCalibration);
      if (ImGui::MenuItem("Geometric Calibration", nullptr, false, canStart)) {
        attempt([&] { openCalibrationDialog(false); }, FailurePolicy::StopAcquisition);
      }
      if (ImGui::MenuItem("Intensity Calibration", nullptr, false, canStart)) {
        attempt([&] { openCalibrationDialog(true); }, FailurePolicy::StopAcquisition);
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Settings", nullptr, &settings.settingsWindowOpen) && settings.settingsWindowOpen) {
        ImGui::SetWindowCollapsed("Settings", false);
        ImGui::SetWindowFocus("Settings");
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Frame sequence", nullptr, &frameSequenceWindow.open) && frameSequenceWindow.open) {
        ImGui::SetWindowCollapsed("###FrameSequence", false);
        ImGui::SetWindowFocus("###FrameSequence");
      }
      if (ImGui::MenuItem("Render Settings", nullptr, &settings.renderSettingsOpen) && settings.renderSettingsOpen) {
        ImGui::SetWindowCollapsed("Render Settings", false);
        ImGui::SetWindowFocus("Render Settings");
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Reset View")) renderer.resetView();
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Tools")) {
      if (ImGui::BeginMenu("ROS 2 Output")) {
        if (!slstudio::Ros2Output::available())
          ImGui::TextDisabled("ROS 2 support was not built.");
        ImGui::BeginDisabled(!slstudio::Ros2Output::available());
        ImGui::BeginDisabled(ros2Output != nullptr);
        ImGui::InputText("Topic", ros2Topic.data(), ros2Topic.size());
        ImGui::InputText("Frame ID", ros2FrameId.data(), ros2FrameId.size());
        ImGui::InputDouble("Metres per calibration unit", &ros2MetresPerUnit, 0, 0, "%.6g");
        ImGui::EndDisabled();
        ImGui::TextDisabled("Timestamps are reconstruction completion times.");
        if (!ros2OutputError.empty()) ImGui::TextWrapped("%s", ros2OutputError.c_str());
        if (ImGui::MenuItem("Publish point clouds", nullptr, ros2Output != nullptr)) {
          attempt([&] {
            if (ros2Output) {
              ros2Output.reset();
              status = "ROS 2 output stopped";
            } else {
              try {
                ros2Output = std::make_unique<slstudio::Ros2Output>(
                    slstudio::Ros2OutputSettings{ros2Topic.data(), ros2FrameId.data(), ros2MetresPerUnit});
              } catch (const std::exception &error) {
                ros2OutputError = error.what();
                throw;
              }
              ros2OutputError.clear();
              status = "ROS 2 output enabled";
            }
          });
        }
        ImGui::EndDisabled();
        ImGui::EndMenu();
      }
      if (ImGui::MenuItem("LightCrafter3000 Configuration", nullptr, &lc3000Window.open) && lc3000Window.open) {
        ImGui::SetWindowCollapsed("LightCrafter3000 Configuration", false);
        ImGui::SetWindowFocus("LightCrafter3000 Configuration");
      }
      if (ImGui::MenuItem("LightCrafter4500 Configuration", nullptr, &lc4500Window.open) && lc4500Window.open) {
        ImGui::SetWindowCollapsed("LightCrafter4500 Configuration", false);
        ImGui::SetWindowFocus("LightCrafter4500 Configuration");
      }
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }
}


void ApplicationController::browseDirectory(const std::filesystem::path &directory) {
  const auto absolute = std::filesystem::absolute(directory).lexically_normal();
  const auto text = absolute.string();
  if (text.size() >= filePickerDialog.state.path.size()) throw std::runtime_error("Directory path is too long");
  std::vector<std::filesystem::path> directories, files;
  for (const auto &entry : std::filesystem::directory_iterator(absolute)) {
    if (entry.is_directory()) directories.push_back(entry.path());
    else if (entry.is_regular_file() &&
        (filePickerDialog.state.mode == FilePickerMode::LoadSequence ||
         (filePickerDialog.state.mode == FilePickerMode::GeometricCalibration && entry.path().extension() == ".xml")))
      files.push_back(entry.path());
  }
  std::sort(directories.begin(), directories.end());
  std::sort(files.begin(), files.end());
  filePickerDialog.state.directory = absolute;
  filePickerDialog.state.selectedDirectory.reset();
  filePickerDialog.state.directories = std::move(directories);
  filePickerDialog.state.files = std::move(files);
  std::snprintf(filePickerDialog.state.path.data(), filePickerDialog.state.path.size(), "%s", text.c_str());
  filePickerDialog.state.filename[0] = '\0';
  filePickerDialog.state.error.clear();
}

void ApplicationController::openFilePicker(FilePickerMode mode) {
  filePickerDialog.state.mode = mode;
  filePickerDialog.state.requested = true;
  filePickerDialog.state.error.clear();
  try {
    const auto selected = std::filesystem::absolute(mode == FilePickerMode::GeometricCalibration ?
        settings.geometricCalibrationPath.data() : settings.sequencePath.data());
    browseDirectory(mode == FilePickerMode::GeometricCalibration ? selected.parent_path() : selected);
    if (mode == FilePickerMode::GeometricCalibration) {
      const auto filename = selected.filename().string();
      if (filename.size() >= filePickerDialog.state.filename.size()) throw std::runtime_error("Geometric Calibration filename is too long");
      std::snprintf(filePickerDialog.state.filename.data(), filePickerDialog.state.filename.size(), "%s", filename.c_str());
    }
  } catch (const std::exception &error) {
    filePickerDialog.state.error = status = error.what();
  }
}

void ApplicationController::loadGeometricCalibrationFile(const std::filesystem::path &path) {
  if (processing() || scan.index() >= 0)
    throw std::logic_error("Stop capture and processing before loading a calibration");
  const auto absolute = std::filesystem::absolute(path).lexically_normal();
  const auto text = absolute.string();
  if (text.size() >= settings.geometricCalibrationPath.size()) throw std::runtime_error("Geometric Calibration path is too long");
  GeometricCalibrationData calibration;
  if (!calibration.load(absolute)) throw std::runtime_error("Cannot load calibration XML: " + text);
  stopReconstruction();
  pipelineSettings.reset();
  pipelineGeometricCalibrationPath.clear();
  invalidateGeometricCalibration();
  std::snprintf(settings.geometricCalibrationPath.data(), settings.geometricCalibrationPath.size(), "%s", text.c_str());
  geometricCalibrationState = "Loaded";
  status = "Loaded calibration: " + text;
}

void ApplicationController::loadSequenceFolder(const std::filesystem::path &directory) {
  if (processing() || scan.index() >= 0)
    throw std::logic_error("Stop capture and processing before loading a sequence");
  const auto absolute = std::filesystem::absolute(directory).lexically_normal();
  const auto path = absolute.string();
  if (path.size() >= settings.sequencePath.size()) throw std::runtime_error("Sequence folder path is too long");
  auto loaded = loadSequence(absolute, settings.reconstruction.coding);
  const auto reconstructionSettings = withCodecSettings(settings.reconstruction, loaded.coding);
  GeometricCalibrationData calibration;
  if (!calibration.load(settings.geometricCalibrationPath.data())) throw std::runtime_error("Cannot load calibration XML");
  DecodeStage decoder(calibration, reconstructionSettings);
  if (loaded.frames.size() != decoder.patternCount())
    throw std::runtime_error("Frame count does not match the sequence coding method");
  if (loaded.frames.front().size() != cv::Size(calibration.frameWidth, calibration.frameHeight))
    throw std::runtime_error("Sequence resolution does not match calibration");
  settings.reconstruction = reconstructionSettings;
  settings.directionIndex = static_cast<int>(settings.reconstruction.coding.direction) - 1;
  std::snprintf(settings.sequencePath.data(), settings.sequencePath.size(), "%s", path.c_str());
  frames = std::move(loaded.frames);
  reconstruct();
}


}
