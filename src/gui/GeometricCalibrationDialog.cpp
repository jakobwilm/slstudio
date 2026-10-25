#include "GeometricCalibrationDialog.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace slstudio::gui {
void GeometricCalibrationDialog::draw(GeometricCalibrationDialogContext &context) {
  auto &settings = context.settings;
  auto &cameraPreview = context.cameraPreview;
  auto &cameraPreviewFrameSize = context.cameraPreviewFrameSize;
  auto &cameraPreviewFrameCount = context.cameraPreviewFrameCount;
  auto &cameraNextPreviewFrame = context.cameraNextPreviewFrame;
  auto &scan = context.scan;
  auto &geometricCalibrationDiagnostics = context.diagnostics;
  auto &geometricCalibrationPoseNames = context.poseNames;
  auto &geometricCalibrationPoses = context.poses;
  auto &pendingGeometricCalibration = context.pendingCalibration;
  const bool busy = context.busy;
  if (!open) return;
  constexpr const char *title = "Geometric Calibration";
  const auto *viewport = ImGui::GetMainViewport();
  const cv::Size viewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
  const auto placement = !this->viewportSize.empty() && this->viewportSize != viewportSize
    ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
  this->viewportSize = viewportSize;
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
                 viewport->WorkPos.y + viewport->WorkSize.y * 0.5f),
         placement, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(std::min(uiPixels(1000), viewport->WorkSize.x),
                   std::min(uiPixels(700), viewport->WorkSize.y)), placement);
  ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(uiPixels(560), viewport->WorkSize.x),
                       std::min(uiPixels(520), viewport->WorkSize.y)), viewport->WorkSize);
  if (!ImGui::Begin(title, &open, ImGuiWindowFlags_NoCollapse)) { ImGui::End(); return; }
  context.drawCameraProjectorHeader();
  const auto available = ImGui::GetContentRegionAvail();
  const float footerHeight = ImGui::GetTextLineHeightWithSpacing();
  const float contentHeight = std::max(uiPixels(100), available.y - footerHeight -
      2 * (ImGui::GetStyle().CellPadding.y + ImGui::GetStyle().ItemSpacing.y));
  const bool capturingPose = scan.geometricCalibration() && scan.active();
  if (capturingPose && selectedPose >= 0) selectPose(-1);
  const bool selected = selectedPose >= 0 &&
    static_cast<std::size_t>(selectedPose) < geometricCalibrationPoses.size();
  if (ImGui::BeginTable("Geometric Calibration layout", 2, ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Image", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Controls", ImGuiTableColumnFlags_WidthFixed, uiPixels(290));
    ImGui::TableNextColumn();
    const float captureButtonHeight = ImGui::GetFrameHeight() * 1.5f;
    ImGui::BeginChild("Geometric Calibration image",
        ImVec2(0, std::max(1.0f, contentHeight - captureButtonHeight - ImGui::GetStyle().ItemSpacing.y)),
        ImGuiChildFlags_Borders);
    if (selected && selectedFrame_ >= 0)
      ImGui::Text("%s - Frame %d/%zu", geometricCalibrationPoseNames[selectedPose].c_str(),
          selectedFrame_ + 1, geometricCalibrationPoses[selectedPose].size());
    else
      ImGui::TextUnformatted(selected ? geometricCalibrationPoseNames[selectedPose].c_str() : "Live camera");
    GLuint texture = 0;
    cv::Size imageSize;
    ImVec2 uv0(0, 0), uv1(1, 1);
    if (selected) {
      updatePreview(context);
      texture = previewTexture_;
      imageSize = previewSize_;
    } else if (!cameraPreview.empty()) {
      context.cameraPreviewTexture.update(cameraPreview, context.cameraPreviewDirty);
      texture = context.cameraPreviewTexture.texture();
      imageSize = cameraPreviewFrameSize;
      const int slot = (cameraNextPreviewFrame + cameraPreviewFrameCount - 1) % cameraPreviewFrameCount;
      uv0.x = static_cast<float>(slot) / cameraPreviewFrameCount;
      uv1.x = static_cast<float>(slot + 1) / cameraPreviewFrameCount;
    }
    const GeometricCalibrationPoseDiagnostics *diagnostics = selected && selectedFrame_ < 0 &&
        static_cast<std::size_t>(selectedPose) < geometricCalibrationDiagnostics.size() ?
        &geometricCalibrationDiagnostics[selectedPose] : nullptr;
    auto cornerSummary = diagnostics ? std::to_string(diagnostics->corners.size()) + " corners / " +
        std::to_string(diagnostics->correspondenceCount) + " correspondences" : std::string{};
    if (diagnostics && std::isfinite(diagnostics->cameraRms)) {
      char errors[160];
      std::snprintf(errors, sizeof(errors), "\nRMS: camera %.3f px, projector %.3f px\n"
          "Residuals x10: green=camera, red=projector pixels", diagnostics->cameraRms, diagnostics->projectorRms);
      cornerSummary += errors;
    }
    auto imageArea = ImGui::GetContentRegionAvail();
    if (diagnostics)
      imageArea.y -= ImGui::CalcTextSize(cornerSummary.c_str(), nullptr, false, imageArea.x).y +
          ImGui::CalcTextSize(diagnostics->reason.c_str(), nullptr, false, imageArea.x).y +
          2 * ImGui::GetStyle().ItemSpacing.y;
    imageArea.y = std::max(1.0f, imageArea.y);
    const auto origin = ImGui::GetCursorPos();
    if (!imageSize.empty()) {
      const float scale = std::min(imageArea.x / imageSize.width, imageArea.y / imageSize.height);
      const ImVec2 size(imageSize.width * scale, imageSize.height * scale);
      ImGui::SetCursorPos(ImVec2(origin.x + (imageArea.x - size.x) / 2, origin.y + (imageArea.y - size.y) / 2));
      ImGui::Image(static_cast<ImTextureID>(texture), size, uv0, uv1);
    } else {
      ImGui::TextUnformatted(selected ? "No pose image available" :
          context.cameraRequested ? "Waiting for camera frames" : "Camera inactive");
    }
    if (diagnostics) {
      ImGui::SetCursorPos(ImVec2(origin.x, origin.y + imageArea.y));
      ImGui::TextWrapped("%s", cornerSummary.c_str());
      ImGui::TextWrapped("%s", diagnostics->reason.c_str());
    }
    ImGui::EndChild();
    char captureLabel[64]{};
    if (capturingPose)
      std::snprintf(captureLabel, sizeof(captureLabel), "Cancel capture (%d/%u)###Capture sequence", scan.index() + 1, scan.count());
    else
      std::snprintf(captureLabel, sizeof(captureLabel), "%s###Capture sequence", selected ? "Live camera" : "Capture sequence");
    ImGui::BeginDisabled(!capturingPose && !selected && (busy || !context.projectorOpen));
    if (ImGui::Button(captureLabel, ImVec2(-1, captureButtonHeight))) {
      if (capturingPose) context.stopCapture();
      else if (selected) selectPose(-1);
      else context.capturePose();
    }
    ImGui::EndDisabled();
    ImGui::TableNextColumn();
    ImGui::BeginChild("Geometric Calibration controls", ImVec2(0, contentHeight));
    ImGui::PushItemWidth(-uiPixels(90));
    const auto heading = "Poses (" + std::to_string(geometricCalibrationPoses.size()) + ")";
    ImGui::SeparatorText(heading.c_str());
    ImGui::BeginChild("Pose list", ImVec2(0, std::clamp(contentHeight * 0.3f, uiPixels(100), uiPixels(180))), ImGuiChildFlags_Borders);
    ImGui::BeginDisabled(capturingPose);
    for (std::size_t index = 0; index < geometricCalibrationPoses.size(); ++index) {
      ImGui::PushID(static_cast<int>(index));
      auto flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
          ImGuiTreeNodeFlags_SpanAvailWidth;
      if (selectedPose == static_cast<int>(index) && selectedFrame_ < 0)
        flags |= ImGuiTreeNodeFlags_Selected;
      const bool expanded = ImGui::TreeNodeEx(geometricCalibrationPoseNames[index].c_str(), flags);
      if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
        selectPose(static_cast<int>(index));
      if (ImGui::IsItemHovered() && index < geometricCalibrationDiagnostics.size())
        ImGui::SetTooltip("%s", geometricCalibrationDiagnostics[index].reason.c_str());
      if (expanded) {
        for (std::size_t frame = 0; frame < geometricCalibrationPoses[index].size(); ++frame) {
          const auto label = "Frame " + std::to_string(frame + 1);
          if (ImGui::Selectable(label.c_str(),
                  selectedPose == static_cast<int>(index) && selectedFrame_ == static_cast<int>(frame)))
            selectPose(static_cast<int>(index), static_cast<int>(frame));
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    ImGui::EndDisabled();
    ImGui::EndChild();
    ImGui::BeginDisabled(busy || selectedPose < 0 ||
        static_cast<std::size_t>(selectedPose) >= geometricCalibrationPoses.size());
    if (ImGui::Button("Remove pose"))
      removePoseConfirmation_.request("Remove pose?",
          "Remove \"" + geometricCalibrationPoseNames[selectedPose] + "\" and all its frames?",
          "Remove pose", "This cannot be undone.");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || geometricCalibrationPoses.empty());
    if (ImGui::Button("Clear poses"))
      clearPosesConfirmation_.request("Clear poses?",
          "Remove all " + std::to_string(geometricCalibrationPoses.size()) + " poses and their frames?",
          "Clear poses", "This cannot be undone.");
    ImGui::EndDisabled();
    removePoseConfirmation_.draw([&] {
      geometricCalibrationPoses.erase(geometricCalibrationPoses.begin() + selectedPose);
      geometricCalibrationPoseNames.erase(geometricCalibrationPoseNames.begin() + selectedPose);
      selectPose(std::min(selectedPose, static_cast<int>(geometricCalibrationPoses.size()) - 1));
      context.invalidate();
    });
    clearPosesConfirmation_.draw([&] {
      geometricCalibrationPoses.clear();
      geometricCalibrationPoseNames.clear();
      selectPose(-1);
      context.invalidate();
    });

    ImGui::SeparatorText("Target");
    bool changed = false;
    ImGui::BeginDisabled(busy);
    int target = settings.geometricCalibrationSettings.circles ? 1 : 0;
    const char *targets[] = {"Checkerboard", "Symmetric circles (white)"};
    if (ImGui::Combo("Pattern", &target, targets, IM_ARRAYSIZE(targets))) {
      settings.geometricCalibrationSettings.circles = target == 1;
      settings.geometricCalibrationSettings.partialCircles = settings.geometricCalibrationSettings.circles;
      changed = true;
    }
    if (ImGui::BeginTable("Target dimensions", 2, ImGuiTableFlags_SizingStretchSame)) {
      ImGui::TableNextColumn();
      ImGui::SetNextItemWidth(-uiPixels(55));
      changed |= ImGui::InputInt("Columns", &settings.geometricCalibrationSettings.boardCorners.width);
      ImGui::TableNextColumn();
      ImGui::SetNextItemWidth(-uiPixels(40));
      changed |= ImGui::InputInt("Rows", &settings.geometricCalibrationSettings.boardCorners.height);
      ImGui::EndTable();
    }
    changed |= ImGui::InputFloat("Spacing (mm)", &settings.geometricCalibrationSettings.spacing);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Center-to-center spacing for circles, or corner spacing for checkerboards. Circle diameter is not required.");
    ImGui::SeparatorText("Geometric Calibration parameters");
    int interpolation = static_cast<int>(settings.geometricCalibrationSettings.interpolation);
    const char *interpolationMethods[] = {"Local homography", "Gaussian RBF"};
    if (ImGui::Combo("Mapping", &interpolation, interpolationMethods, IM_ARRAYSIZE(interpolationMethods))) {
      settings.geometricCalibrationSettings.interpolation = static_cast<GeometricCalibrationInterpolation>(interpolation);
      changed = true;
    }
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Map subpixel camera features into projector coordinates using nearby decoded pixels.");
    if (ImGui::BeginTable("Geometric Calibration parameters", 3, ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("Parameter", ImGuiTableColumnFlags_WidthStretch, 1.6f);
      ImGui::TableSetupColumn("Camera", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("Projector", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableHeadersRow();
      struct Parameter {
        const char *name;
        int mask, fixedFlag;
      };
      const Parameter parameters[] = {
          {"Principal point", cv::CALIB_FIX_PRINCIPAL_POINT, cv::CALIB_FIX_PRINCIPAL_POINT},
          {"k1", cv::CALIB_FIX_K1, cv::CALIB_FIX_K1},
          {"k2", cv::CALIB_FIX_K2, cv::CALIB_FIX_K2},
          {"k3", cv::CALIB_FIX_K3, cv::CALIB_FIX_K3},
          {"p1 / p2", cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_TANGENT_DIST, cv::CALIB_ZERO_TANGENT_DIST}};
      for (const auto &parameter : parameters) {
        ImGui::PushID(parameter.name);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(parameter.name);
        for (int *flags : {&settings.geometricCalibrationSettings.cameraFlags, &settings.geometricCalibrationSettings.projectorFlags}) {
          ImGui::TableNextColumn();
          bool enabled = (*flags & parameter.mask) == 0;
          if (ImGui::Checkbox(flags == &settings.geometricCalibrationSettings.cameraFlags ? "##Camera" : "##Projector", &enabled)) {
            *flags = enabled ? *flags & ~parameter.mask : (*flags & ~parameter.mask) | parameter.fixedFlag;
            changed = true;
          }
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Checked: estimate this parameter. Unchecked: fix the principal point at the image center or distortion at zero.");
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    if (changed) context.invalidate();
    ImGui::EndDisabled();

    ImGui::SeparatorText("Results");
    if (pendingGeometricCalibration) {
      const auto accepted = std::count_if(geometricCalibrationDiagnostics.begin(), geometricCalibrationDiagnostics.end(),
          [](const auto &pose) { return pose.accepted; });
      ImGui::Text("%zu / %zu poses accepted", static_cast<std::size_t>(accepted), geometricCalibrationPoses.size());
      ImGui::Text("Camera RMS: %.3f px", pendingGeometricCalibration->cam_error);
      ImGui::Text("Projector RMS: %.3f px", pendingGeometricCalibration->proj_error);
      ImGui::Text("Stereo RMS: %.3f px", pendingGeometricCalibration->stereo_error);
    } else ImGui::TextDisabled("No calibration result");
    ImGui::BeginDisabled(busy || geometricCalibrationPoses.size() < 3);
    if (ImGui::Button("Calibrate")) context.calibrate();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || !pendingGeometricCalibration);
    if (ImGui::Button("Accept")) context.accept();
    ImGui::EndDisabled();
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::EndTable();
  }
  ImGui::TextUnformatted(context.calibrating ? "Calibrating poses" : pendingGeometricCalibration ? "Geometric Calibration ready for review" :
      context.cameraWorker && context.cameraReady ? "Camera active" :
          context.cameraRequested ? "Waiting for camera" : "Camera inactive");
  ImGui::End();
}

void GeometricCalibrationDialog::updatePreview(const GeometricCalibrationDialogContext &context) {
  refreshPreview(context.settings, context.diagnostics, context.poses);
}

void GeometricCalibrationDialog::refreshPreview(const ApplicationSettings &settings,
    const std::vector<GeometricCalibrationPoseDiagnostics> &diagnostics,
    const std::vector<std::vector<cv::Mat>> &poses) {
  if (!previewDirty_) return;
  previewDirty_ = false;
  previewSize_ = {};
  if (selectedPose < 0 || static_cast<std::size_t>(selectedPose) >= poses.size()) return;
  const auto &pose = poses.at(selectedPose);
  if (pose.empty())
    throw std::runtime_error("Geometric Calibration preview requires at least one frame");
  for (const auto &frame : pose) {
    if (frame.empty() || frame.size() != pose.front().size() || frame.type() != CV_8UC1)
      throw std::runtime_error("Geometric Calibration preview requires matching grayscale frames");
  }
  cv::Mat preview;
  if (selectedFrame_ >= 0) {
    cv::cvtColor(pose.at(selectedFrame_), preview, cv::COLOR_GRAY2BGR);
  } else {
    cv::Mat mean = cv::Mat::zeros(pose.front().size(), CV_32F);
    for (const auto &frame : pose) {
      cv::Mat floating;
      frame.convertTo(floating, CV_32F);
      mean += floating;
    }
    mean.convertTo(preview, CV_8U, 1.0 / pose.size());
    cv::cvtColor(preview, preview, cv::COLOR_GRAY2BGR);
  }
  if (selectedFrame_ < 0 && static_cast<std::size_t>(selectedPose) < diagnostics.size()) {
    const auto &poseDiagnostics = diagnostics[selectedPose];
    if (!poseDiagnostics.corners.empty()) {
      if (settings.geometricCalibrationSettings.partialCircles)
        for (const auto &point : poseDiagnostics.corners)
          cv::drawMarker(preview, point, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 9);
      else
        cv::drawChessboardCorners(preview, settings.geometricCalibrationSettings.boardCorners,
            poseDiagnostics.corners, true);
      for (std::size_t index = 0; index < poseDiagnostics.cameraResiduals.size(); ++index) {
        const auto point = poseDiagnostics.acceptedCorners.at(index);
        cv::arrowedLine(preview, point, point + 10 * poseDiagnostics.cameraResiduals[index], cv::Scalar(0, 255, 0));
        cv::arrowedLine(preview, point, point + 10 * poseDiagnostics.projectorResiduals.at(index), cv::Scalar(0, 0, 255));
      }
    }
  }
  if (!previewTexture_) glGenTextures(1, &previewTexture_);
  glBindTexture(GL_TEXTURE_2D, previewTexture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, preview.cols, preview.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, preview.data);
  glBindTexture(GL_TEXTURE_2D, 0);
  previewSize_ = preview.size();
}

void GeometricCalibrationDialog::release() {
  if (previewTexture_) glDeleteTextures(1, &previewTexture_);
  previewTexture_ = 0;
}
}
