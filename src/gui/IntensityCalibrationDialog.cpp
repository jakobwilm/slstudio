#include "IntensityCalibrationDialog.h"
#include "IntensityCurveWindow.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace slstudio::gui {
void IntensityCalibrationDialog::prepare(CaptureWorker &capture) {
  if (worker) throw std::logic_error("Stop intensity measurement before preparing another");
  worker = &capture;
  preview.release();
  measuring = frameRequested = measureRequested = false;
  after = Clock::now();
}

void IntensityCalibrationDialog::start(bool compensationEnabled, int inputLevels) {
  if (inputLevels < 5 || inputLevels > 255)
    throw std::invalid_argument("Use 5-255 intensity levels");
  if (!worker || !worker->ready() || measuring || frameRequested || preview.empty())
    throw std::runtime_error("Wait for camera initialization before measuring");
  sampleIntensityFrame(preview, {roi[0], roi[1], roi[2], roi[3]}, 0);
  levels = inputLevels;
  measureRequested = false;
  compensated = compensationEnabled;
  samples.clear();
  result.reset();
  measuring = true;
  levelIndex = 0;
  after = Clock::now();
}

int IntensityCalibrationDialog::level() const {
  return (levelIndex * 255 + (levels - 1) / 2) / (levels - 1);
}

IntensityCalibrationDialog::Progress IntensityCalibrationDialog::consume(cv::Mat frame) {
  if (frame.empty() || frame.dims != 2 || frame.type() != CV_8UC1)
    throw std::runtime_error("Intensity Calibration capture requires Mono8 images");
  frameRequested = false;
  preview = std::move(frame);
  if (roi[2] == 0 && roi[3] == 0)
    roi = {preview.cols / 4, preview.rows / 4, std::max(1, preview.cols / 2), std::max(1, preview.rows / 2)};
  if (!measuring) {
    after = Clock::now() + std::chrono::milliseconds(100);
    return Progress::Waiting;
  }
  const auto sample = sampleIntensityFrame(preview, {roi[0], roi[1], roi[2], roi[3]}, level());
  samples.push_back(sample);
  if (++levelIndex == levels) return Progress::Complete;
  after = Clock::now();
  return Progress::NextLevel;
}

void IntensityCalibrationDialog::requestIfDue() {
  if (!worker) throw std::logic_error("Prepare intensity capture before requesting a frame");
  if (!frameRequested && worker->ready() && Clock::now() >= after)
    frameRequested = worker->requestFrame();
}

void IntensityCalibrationDialog::cancel() {
  measureRequested = measuring = frameRequested = false;
  worker = nullptr;
}

void IntensityCalibrationDialog::draw(const IntensityCalibrationDialogContext &context) {
  const auto *worker = this->worker;
  const auto &preview = this->preview;
  const auto &samples = this->samples;
  const auto &result = this->result;
  const bool compensated = this->compensated;
  const bool measuring = this->measuring || this->measureRequested;
  if (!open) {
    if (worker) {
      context.cancel();
    }
    return;
  }
  constexpr const char *title = "Intensity Calibration";
  const auto *viewport = ImGui::GetMainViewport();
  const cv::Size viewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
  const auto placement = !this->viewportSize.empty() && this->viewportSize != viewportSize ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
  this->viewportSize = viewportSize;
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), placement, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(std::min(uiPixels(1000), viewport->WorkSize.x), std::min(uiPixels(700), viewport->WorkSize.y)), placement);
  ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(uiPixels(560), viewport->WorkSize.x), std::min(uiPixels(560), viewport->WorkSize.y)), viewport->WorkSize);
  if (!ImGui::Begin(title, &open, ImGuiWindowFlags_NoCollapse)) {
    ImGui::End();
    if (!open && worker) context.cancel();
    return;
  }
  const bool otherBusy = (context.processing && !worker) || context.scanActive;
  context.drawCameraProjectorHeader();
  const char *message = this->measuring ? "Measuring intensity" : result ? "Intensity Calibration ready for review" :
      context.cameraWorker && context.cameraReady ? "Camera active" :
          context.cameraRequested ? "Waiting for camera" : "Camera inactive";
  const auto available = ImGui::GetContentRegionAvail();
  const float footerHeight = ImGui::CalcTextSize(message, nullptr, false, available.x).y +
      ImGui::GetStyle().ItemSpacing.y;
  const float contentHeight = std::max(uiPixels(100), available.y - footerHeight -
      2 * (ImGui::GetStyle().CellPadding.y + ImGui::GetStyle().ItemSpacing.y));
  if (ImGui::BeginTable("Intensity Calibration layout", 2, ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Camera", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Controls", ImGuiTableColumnFlags_WidthFixed, uiPixels(300));
    ImGui::TableNextColumn();
    ImGui::BeginChild("Intensity Calibration image", ImVec2(0, contentHeight), ImGuiChildFlags_Borders);
    ImGui::TextWrapped("Live camera - drag to select the measurement region");
    if (!context.cameraPreview.empty()) {
      context.cameraPreviewTexture.update(context.cameraPreview, context.cameraPreviewDirty);
      const int slot = (context.cameraNextPreviewFrame + context.cameraPreviewFrameCount - 1) % context.cameraPreviewFrameCount;
      const auto image = context.cameraPreview(cv::Rect(slot * context.cameraPreviewFrameSize.width, 0,
          context.cameraPreviewFrameSize.width, context.cameraPreviewFrameSize.height));
      if (roi[2] == 0 && roi[3] == 0)
        roi = {image.cols / 4, image.rows / 4, std::max(1, image.cols / 2), std::max(1, image.rows / 2)};
      const auto imageArea = ImGui::GetContentRegionAvail();
      const float imageHeight = std::max(1.0f, imageArea.y - 3 * ImGui::GetTextLineHeightWithSpacing());
      const float scale = std::min(imageArea.x / image.cols, imageHeight / image.rows);
      const auto cursor = ImGui::GetCursorPos();
      ImGui::SetCursorPos(ImVec2(cursor.x + (imageArea.x - image.cols * scale) / 2,
          cursor.y + (imageHeight - image.rows * scale) / 2));
      const auto origin = ImGui::GetCursorScreenPos();
      const ImVec2 end(origin.x + image.cols * scale, origin.y + image.rows * scale);
      ImGui::InvisibleButton("##intensity-image", ImVec2(image.cols * scale, image.rows * scale));
      ImGui::GetWindowDrawList()->AddImage(static_cast<ImTextureID>(context.cameraPreviewTexture.texture()), origin, end,
          ImVec2(static_cast<float>(slot) / context.cameraPreviewFrameCount, 0),
          ImVec2(static_cast<float>(slot + 1) / context.cameraPreviewFrameCount, 1));
      if (!measuring && ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const auto mouse = ImGui::GetMousePos();
        dragOrigin = {std::clamp(static_cast<int>((mouse.x - origin.x) / scale), 0, image.cols - 1),
                      std::clamp(static_cast<int>((mouse.y - origin.y) / scale), 0, image.rows - 1)};
      }
      if (!measuring && ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        const auto mouse = ImGui::GetMousePos();
        const int endX = std::clamp(static_cast<int>((mouse.x - origin.x) / scale), 0, image.cols - 1);
        const int endY = std::clamp(static_cast<int>((mouse.y - origin.y) / scale), 0, image.rows - 1);
        roi = {std::min(dragOrigin.x, endX), std::min(dragOrigin.y, endY),
            std::abs(endX - dragOrigin.x) + 1, std::abs(endY - dragOrigin.y) + 1};
        clearResult();
        context.status = "ROI changed";
      }
      ImGui::SetCursorPos(ImVec2(cursor.x, cursor.y + imageHeight + ImGui::GetStyle().ItemSpacing.y));
      if (roi[0] >= 0 && roi[1] >= 0 && roi[2] > 0 && roi[3] > 0 &&
          roi[2] <= image.cols && roi[3] <= image.rows &&
          roi[0] <= image.cols - roi[2] && roi[1] <= image.rows - roi[3]) {
        ImGui::GetWindowDrawList()->AddRect(ImVec2(origin.x + roi[0] * scale, origin.y + roi[1] * scale),
            ImVec2(origin.x + (roi[0] + roi[2]) * scale, origin.y + (roi[1] + roi[3]) * scale), IM_COL32(20, 180, 110, 255), 0, 0, 2);
        const auto sample = sampleIntensityFrame(image, {roi[0], roi[1], roi[2], roi[3]}, 0);
        ImGui::TextWrapped("ROI mean: %.2f DN | saturated: %.2f%%", sample.brightness, sample.saturatedFraction * 100);
      } else ImGui::TextUnformatted("ROI outside camera image");
    } else ImGui::TextUnformatted(context.cameraRequested ? "Waiting for camera frames" : "Camera inactive");
    ImGui::EndChild();
    ImGui::TableNextColumn();
    ImGui::BeginChild("Intensity Calibration controls", ImVec2(0, contentHeight));
    ImGui::PushItemWidth(-uiPixels(105));
    ImGui::SeparatorText("Measurement");
    ImGui::BeginDisabled(measuring);
    ImGui::SliderInt("Input levels", &context.settings.intensityLevels, 5, 255);
    ImGui::TextDisabled("One image per input level");
    ImGui::SliderInt("Settle (ms)", &context.settings.settleMilliseconds, 0, 1000);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Delay after changing the projected level before capture. Applies to software triggering only; intensity measurement always uses software triggering.");
    if (ImGui::InputInt4("ROI (x,y,w,h)", roi.data())) {
      clearResult();
      context.status = "ROI changed";
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(otherBusy || measuring || !context.projectorOpen || context.cameraPreview.empty() ||
        (worker && (frameRequested || preview.empty())));
    if (ImGui::Button("Measure")) context.start();
    ImGui::EndDisabled();
    if (worker) {
      ImGui::SameLine();
      if (ImGui::Button("Cancel")) context.cancel();
    } else if (!context.projectorOpen) {
      ImGui::SameLine();
      ImGui::TextDisabled("Projector unavailable; check Settings and reopen calibration");
    }
    if (measuring) ImGui::ProgressBar(progress(), ImVec2(-1, 0));

    ImGui::SeparatorText("Estimated intensity compensation");
    if (result) {
      std::array<float, 256> correction{};
      for (std::size_t index = 0; index < correction.size(); ++index)
        correction[index] = result->inverseLut[index];
      drawIntensityPlot("##estimated-lut", correction.data(), static_cast<int>(correction.size()),
          ImVec2(-1, std::clamp(contentHeight * 0.25f, uiPixels(180), uiPixels(220))));
    } else ImGui::TextDisabled("Measure to estimate a correction");
    ImGui::SeparatorText("Measured intensity");
    if (!samples.empty()) {
      std::vector<float> values;
      std::vector<int> levels;
      for (const auto &sample : samples) {
        values.push_back(static_cast<float>(sample.brightness));
        levels.push_back(sample.level);
      }
      ImGui::TextUnformatted(compensated ? "Compensated measurement" : "Uncompensated measurement");
      drawIntensityPlot("##intensity-curve", values.data(), static_cast<int>(values.size()),
          ImVec2(-1, std::clamp(contentHeight * 0.25f, uiPixels(180), uiPixels(220))),
          projectorInputAxis, cameraIntensityAxis, levels.data());
    } else ImGui::TextDisabled("No measurement samples");
    if (result) {
      const double range = result->whiteLevel - result->blackLevel;
      ImGui::TextWrapped("Linearity RMS: %.2f%% | max: %.2f%%", 100 * result->linearityRmsError / range,
          100 * result->linearityMaxError / range);
      ImGui::Text("Black: %.2f DN | white: %.2f DN", result->blackLevel, result->whiteLevel);
      if (compensated) ImGui::TextWrapped("Verification only; keep the stored curve.");
    }
    ImGui::BeginDisabled(worker != nullptr || otherBusy);
    ImGui::BeginDisabled(!result || compensated);
    ImGui::BeginDisabled(context.processing || context.scanActive);
    if (ImGui::Button("Accept")) {
      if (context.settings.intensityLut)
        replaceCurveDialog.request("Replace projector/camera intensity curve?",
            "Replace the stored projector/camera intensity curve with this measurement?", "Replace", {}, "?");
      else context.accept();
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::EndTable();
  }
  ImGui::TextUnformatted(message);
  replaceCurveDialog.draw(context.replace);
  ImGui::End();
  if (!open && worker) {
    context.cancel();
  }
}
}
