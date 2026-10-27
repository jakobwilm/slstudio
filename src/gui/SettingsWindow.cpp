#include "SettingsWindow.h"
#include "GuiLayout.h"
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
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + uiPixels(20), viewport->WorkPos.y + uiPixels(40)), ImGuiCond_FirstUseEver);
  const float width = std::min(uiPixels(420), viewport->WorkSize.x);
  ImGui::SetNextWindowSizeConstraints(ImVec2(width, std::min(uiPixels(300), viewport->WorkSize.y)),
      ImVec2(width, viewport->WorkSize.y));
  ImGui::SetNextWindowSize(ImVec2(width,
      std::min(uiPixels(720), viewport->WorkSize.y)), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Settings", &open)) {
    ImGui::PushItemWidth(-uiPixels(150));
    drawControls(context);
    ImGui::PopItemWidth();
  }
  ImGui::End();
}

void SettingsWindow::drawControls(SettingsWindowContext &context) {
  auto &settings = context.settings;
  auto &outputs = context.outputs;
  auto &projector = context.projector;
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
  if (settings.reconstruction.coding.codec == CodecType::PhaseShiftSpatialUnwrap) {
    int order = settings.reconstruction.centerFringeOrder;
    if (ImGui::InputInt("Central period", &order))
      attempt([&] {
        auto candidate = settings.reconstruction;
        candidate.centerFringeOrder = order;
        validateReconstructionSettings(candidate);
        settings.reconstruction = candidate;
      });
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Zero-based fringe period at the valid seed selected near the camera image center.\n"
                        "Must be between 0 and Fringe periods minus one.\n"
                        "A wrong assumption can produce plausible geometry at the wrong depth.");
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
  ImGui::SeparatorText("Projector");
  ImGui::BeginDisabled(busy || projector.has_value());
  if (ImGui::Button("Refresh projectors")) attempt([&] { context.refreshOutputs(); });
  const slstudio::ProjectorOutputInfo *output = settings.outputIndex >= 0 &&
      static_cast<std::size_t>(settings.outputIndex) < outputs.size() ? &outputs[settings.outputIndex] : nullptr;
  if (ImGui::BeginCombo("Projector", output ? output->name.c_str() :
      settings.outputId.empty() ? "None" : "Saved projector unavailable")) {
    for (std::size_t index = 0; index < outputs.size(); ++index)
      if (ImGui::Selectable(outputs[index].name.c_str(), settings.outputIndex == static_cast<int>(index))) {
        settings.outputIndex = static_cast<int>(index);
        output = &outputs[index];
        settings.outputId = output->selectionId();
      }
    ImGui::EndCombo();
  }
  if (output && output->backend == slstudio::OutputBackend::LightCrafter3000)
    ImGui::InputText("LC3000 connection IPv4", settings.projectorAddress.data(), settings.projectorAddress.size());
  ImGui::EndDisabled();
  const bool lightCrafter = output &&
      (output->backend == slstudio::OutputBackend::LightCrafter3000 ||
       output->backend == slstudio::OutputBackend::LightCrafter4500);
  if (lightCrafter) {
    ImGui::BeginDisabled(busy);
    ImGui::InputFloat("Projector exposure (ms)", &settings.projectorExposure, 0.0f, 0.0f, "%.3f");
    ImGui::InputFloat("Frame period (ms)", &settings.projectorPeriod, 0.0f, 0.0f, "%.3f");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("LC4500 flash patterns use at least 0.231 ms between exposures for reliable camera triggers. Longer sequences also share the image-loading budget across each buffer's patterns.");
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
  if (lightCrafter) {
    ImGui::BeginDisabled(busy);
    const bool lightCrafter4500 = output->backend == slstudio::OutputBackend::LightCrafter4500;
    const std::string details = lightCrafter4500 ?
        "Takes approximately 30 seconds." : "";
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
    attempt([&] { context.startProjectorProgramming(calibrationPatterns_); }, true);
  });

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
  ImGui::SeparatorText("Reconstruction");
  ImGui::BeginDisabled(busy);
  ImGui::SliderInt("Min. modulation", &settings.reconstruction.minimumModulation, 0, 255);
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
