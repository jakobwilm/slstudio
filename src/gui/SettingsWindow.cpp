#include "SettingsWindow.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
namespace {
bool cropDimension(const char *label, int &value) {
  ImGui::PushID(label);
  const float resetWidth = ImGui::CalcTextSize("Reset").x + 2 * ImGui::GetStyle().FramePadding.x;
  ImGui::SetNextItemWidth(std::max(1.0f, ImGui::CalcItemWidth() - resetWidth - ImGui::GetStyle().ItemSpacing.x));
  ImGui::InputInt("##value", &value, 0, 0);
  bool changed = ImGui::IsItemDeactivatedAfterEdit();
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("0 selects the full dimension.");
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    value = 0;
    changed = true;
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(label);
  ImGui::PopID();
  return changed;
}
}

void SettingsWindow::draw(bool &open, SettingsWindowContext &context) {
  if (!open) return;
  const auto *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 20, viewport->WorkPos.y + 40), ImGuiCond_FirstUseEver);
  const float width = std::min(420.0f, viewport->WorkSize.x);
  ImGui::SetNextWindowSizeConstraints(ImVec2(width, std::min(300.0f, viewport->WorkSize.y)),
      ImVec2(width, viewport->WorkSize.y));
  ImGui::SetNextWindowSize(ImVec2(width,
      std::min(720.0f, viewport->WorkSize.y)), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Settings", &open)) {
    ImGui::PushItemWidth(-150);
    drawControls(context);
    ImGui::PopItemWidth();
  }
  ImGui::End();
}

void SettingsWindow::drawControls(SettingsWindowContext &context) {
  auto &settings = context.settings;
  auto &outputs = context.outputs;
  auto &projector = context.projector;
  auto &projectorJob = context.projectorJob;
  const bool busy = context.busy;
  auto &uploadConfirmation = context.uploadConfirmation;
  auto &intensityCurveWindow = context.intensityCurveWindow;
  auto attempt = [&](auto action, bool stopAcquisition = false) { context.attempt(action, stopAcquisition); };
  ImGui::SeparatorText("Coding Method");
  ImGui::BeginDisabled(busy);
  int selected = 0;
  for (std::size_t index = 0; index < Codecs.size(); ++index)
    if (Codecs[index].type == settings.reconstruction.coding.codec) selected = static_cast<int>(index);
  if (ImGui::Combo("Codec", &selected, [](void *, int index) { return Codecs[index].name; },
                   nullptr, static_cast<int>(Codecs.size())))
    attempt([&] { context.configureCodec(Codecs[selected].type, settings.reconstruction.coding.algorithmSettings); });
  const bool sparseGray = settings.reconstruction.coding.codec == CodecType::GrayCode;
  if (sparseGray) ImGui::TextDisabled("Sparse subpixel transitions (unorganized cloud)");
  const char *directions[] = {"Horizontal", "Vertical", "Both"};
  ImGui::Combo("Direction", &settings.directionIndex, directions, 3);
  settings.reconstruction.coding.direction = static_cast<CodecDir>(settings.directionIndex + 1);
  if (settings.reconstruction.coding.codec == CodecType::PhaseShift) {
    auto requestedSettings = settings.reconstruction.coding.algorithmSettings;
    bool changed = ImGui::InputInt("Phase steps", &requestedSettings.phaseShift.steps);
    changed |= ImGui::InputInt("Fringe periods", &requestedSettings.phaseShift.periods);
    if (changed) attempt([&] { context.configureCodec(CodecType::PhaseShift, requestedSettings); });
    if (requestedSettings.phaseShift.periods > 1)
      ImGui::TextDisabled("Includes three single-period cue patterns per axis");
  } else if (sparseGray) {
    auto requestedSettings = settings.reconstruction.coding.algorithmSettings;
    bool changed = false;
    if (settings.reconstruction.coding.direction & CodecDirHorizontal)
      changed |= ImGui::InputInt("Horizontal bits", &requestedSettings.grayCode.horizontalBits);
    if (settings.reconstruction.coding.direction & CodecDirVertical)
      changed |= ImGui::InputInt("Vertical bits", &requestedSettings.grayCode.verticalBits);
    if (changed) attempt([&] { context.configureCodec(CodecType::GrayCode, requestedSettings); });
    ImGui::TextDisabled("Includes white and black reference patterns");
  } else {
    auto requestedSettings = settings.reconstruction.coding.algorithmSettings;
    bool changed = false;
    switch (settings.reconstruction.coding.codec) {
    case CodecType::PhaseShiftHeterodyne:
      changed |= ImGui::InputInt("Phase steps", &requestedSettings.heterodyne.steps);
      changed |= ImGui::InputDouble("Primary pitch (pixels)", &requestedSettings.heterodyne.primaryPitch);
      ImGui::TextDisabled("Two frequencies per axis; the second adds one period");
      break;
    case CodecType::PhaseShiftSpatialUnwrap:
      changed |= ImGui::InputInt("Phase steps", &requestedSettings.spatialUnwrap.steps);
      changed |= ImGui::InputInt("Fringe periods", &requestedSettings.spatialUnwrap.periods);
      break;
    case CodecType::PhaseShiftMicro:
      changed |= ImGui::InputInt("Phase steps", &requestedSettings.micro.steps);
      ImGui::TextDisabled("Fixed, jointly designed frequency code; nine additional patterns");
      break;
    case CodecType::PhaseShiftModulated:
      changed |= ImGui::InputInt("Signal steps", &requestedSettings.modulated.signalSteps);
      changed |= ImGui::InputInt("Carrier steps", &requestedSettings.modulated.carrierSteps);
      changed |= ImGui::InputInt("Cue steps", &requestedSettings.modulated.cueSteps);
      changed |= ImGui::InputInt("Signal periods", &requestedSettings.modulated.signalPeriods);
      changed |= ImGui::InputInt("Carrier periods", &requestedSettings.modulated.carrierPeriods);
      break;
    default:
      ImGui::TextDisabled("This algorithm uses a fixed three-pattern layout");
      break;
    }
    if (changed) attempt([&] { context.configureCodec(settings.reconstruction.coding.codec, requestedSettings); });
  }
  ImGui::Text("Total patterns: %u", codecPatternCount(settings.reconstruction.coding.codec,
      settings.reconstruction.coding.direction, settings.reconstruction.coding.algorithmSettings));
  if (settings.reconstruction.coding.codec == CodecType::PhaseShiftMotionCompensated) {
    ImGui::Checkbox("Motion compensation", &settings.reconstruction.motionCompensation);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Estimate translation from successive shading frames and align the two phase images.");
  }
  ImGui::EndDisabled();
  ImGui::Spacing();
  ImGui::SeparatorText("Camera");
  ImGui::BeginDisabled(context.busy);
  bool cameraChanged = context.cameraSettingsControls();
  ImGui::PushID("Camera crop");
  cameraChanged |= cropDimension("Crop width", settings.cameraWidth);
  cameraChanged |= cropDimension("Crop height", settings.cameraHeight);
  ImGui::PopID();
  cameraChanged |= context.cameraTriggerControls();
  if (cameraChanged) {
    context.invalidateGeometricCalibration();
    if (context.cameraRequested) attempt([&] { context.startCamera(); });
  }
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::SeparatorText("Projector");
  ImGui::BeginDisabled(busy || projector.has_value());
  if (ImGui::Button("Refresh projectors")) attempt([&] { context.refreshOutputs(); });
  if (ImGui::BeginCombo("Projector", outputs.empty() ? "None" : outputs.at(settings.outputIndex).name.c_str())) {
    for (std::size_t index = 0; index < outputs.size(); ++index)
      if (ImGui::Selectable(outputs[index].name.c_str(), settings.outputIndex == static_cast<int>(index)))
        settings.outputIndex = static_cast<int>(index);
    ImGui::EndCombo();
  }
  if (!outputs.empty() && outputs.at(settings.outputIndex).backend == slstudio::OutputBackend::LightCrafter3000)
    ImGui::InputText("LC3000 IPv4", settings.projectorAddress.data(), settings.projectorAddress.size());
  ImGui::BeginDisabled(outputs.empty());
  if (ImGui::Button("Open projector")) attempt([&] { context.openOutput(); });
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  ImGui::BeginDisabled(busy);
  ImGui::PushID("Projector crop");
  bool sizeChanged = cropDimension("Crop width", settings.projectorWidth);
  sizeChanged |= cropDimension("Crop height", settings.projectorHeight);
  ImGui::PopID();
  if (sizeChanged) {
    if (projector) {
      if (!settings.projectorWidth) settings.projectorWidth = projector->resolution().width;
      if (!settings.projectorHeight) settings.projectorHeight = projector->resolution().height;
    }
    context.invalidateGeometricCalibration();
    attempt([&] { context.validateProjectorCrop(); });
  }
  ImGui::EndDisabled();
  ImGui::BeginDisabled(!projector || busy);
  if (ImGui::Button("Black")) attempt([&] { projectorJob = context.blankProjector(false); }, true);
  ImGui::SameLine();
  if (ImGui::Button("White")) attempt([&] { projectorJob = context.blankProjector(true); }, true);
  ImGui::SameLine();
  if (ImGui::Button("Close projector")) attempt([&] { context.closeOutput(); }, true);
  ImGui::EndDisabled();
  if (projector && projector->capabilities().persistentPatterns) {
    ImGui::BeginDisabled(busy);
    const bool lightCrafter4500 = !outputs.empty() &&
        outputs.at(settings.outputIndex).backend == slstudio::OutputBackend::LightCrafter4500;
    const std::string details = lightCrafter4500 ?
        "Reads device metadata first; no firmware file needed.\nFirst-time region relocation may take about 2 minutes.\nKeep projector power connected." : "";
    if (ImGui::Button("Upload scan patterns...")) {
      calibrationPatterns_ = false;
      uploadConfirmation.request("Upload projector patterns?",
          "Overwrite the projector's stored pattern images?", "Upload patterns", details, "!");
    }
    if (ImGui::Button("Upload calibration patterns...")) {
      calibrationPatterns_ = true;
      uploadConfirmation.request("Upload projector patterns?",
          "Overwrite the projector's stored pattern images?", "Upload patterns", details, "!");
    }
    ImGui::EndDisabled();
  }
  uploadConfirmation.draw([&] {
    attempt([&] { context.startProjectorProgramming(calibrationPatterns_); });
  });
  if (projector && projector->capabilities().hardwareSequence) {
    ImGui::BeginDisabled(busy);
    ImGui::InputInt("Projector exposure (us)", &settings.projectorExposureUs);
    ImGui::InputInt("Frame period (us)", &settings.projectorPeriodUs);
    ImGui::EndDisabled();
  }
  ImGui::BeginDisabled(busy);
  ImGui::BeginDisabled(!settings.intensityLut);
  ImGui::Checkbox("Intensity compensation", &settings.intensityCompensationEnabled);
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Curve...")) intensityCurveWindow.open = true;
  ImGui::BeginDisabled(busy);
  ImGui::Checkbox("Correct projector lens distortion", &settings.reconstruction.correctProjectorDistortion);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Refine depth using calibrated projector distortion. Non-convergent points are rejected.");
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::SeparatorText("Reconstruction");
  ImGui::BeginDisabled(busy);
  ImGui::SliderInt("Min. modulation", &settings.reconstruction.minimumModulation, 0, 255);
  if (settings.reconstruction.coding.codec == CodecType::PhaseShiftSpatialUnwrap) {
    int order = settings.reconstruction.centerFringeOrder;
    if (ImGui::InputInt("Center fringe order", &order))
      attempt([&] {
        auto candidate = settings.reconstruction;
        candidate.centerFringeOrder = order;
        validateReconstructionSettings(candidate);
        settings.reconstruction = candidate;
      });
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Assumed zero-based fringe order at the seed selected near the image center.\n"
                        "A wrong assumption can produce plausible geometry at the wrong depth.");
  }
  ImGui::BeginDisabled(sparseGray && !settings.reconstruction.lisPhaseFilter);
  ImGui::Checkbox("LIS phase filter", &settings.reconstruction.lisPhaseFilter);
  ImGui::EndDisabled();
  if (sparseGray && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("LIS filtering requires organized phase-shift coordinate maps.");
  ImGui::InputFloat("Near depth", &settings.reconstruction.points.minDepth, 1, 10, "%.3f");
  ImGui::InputFloat("Far depth", &settings.reconstruction.points.maxDepth, 1, 10, "%.3f");
  bool components = settings.reconstruction.minimumComponentPixels > 0;
  if (ImGui::Checkbox("Reject small components", &components))
    settings.reconstruction.minimumComponentPixels = components ? 100 : 0;
  if (components) {
    ImGui::InputInt("Minimum component pixels", &settings.reconstruction.minimumComponentPixels);
    settings.reconstruction.minimumComponentPixels = std::max(1, settings.reconstruction.minimumComponentPixels);
  }
  ImGui::BeginDisabled(sparseGray && !settings.reconstruction.rejectDepthSpikes);
  ImGui::Checkbox("Reject depth spikes", &settings.reconstruction.rejectDepthSpikes);
  ImGui::EndDisabled();
  if (sparseGray && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Depth-spike filtering requires an organized camera-grid cloud.");
  if (settings.reconstruction.rejectDepthSpikes) {
    ImGui::InputFloat("Max. deviation from median", &settings.reconstruction.maximumDepthDeviation, 0, 0, "%.4f");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Reject a point if abs(depth - median neighboring depth) exceeds this threshold.\n"
          "Uses the eight neighbors in a fixed 3x3 window; at least three must be finite.\n"
          "Uses calibration spacing units (e.g. mm). Smaller values reject more points.\n"
          "This is not the far-depth limit; no depths are replaced or smoothed.");
    ImGui::TextDisabled("Depth values use calibration spacing units.");
  }
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::SeparatorText("Scan mode");
  ImGui::BeginDisabled(context.busy);
  int captureMode = settings.continuousCapture ? 1 : 0;
  const char *captureModes[] = {"Single sequence", "Continuous"};
  if (ImGui::Combo("Mode", &captureMode, captureModes, IM_ARRAYSIZE(captureModes)))
    settings.continuousCapture = captureMode == 1;
  ImGui::EndDisabled();
}
}
