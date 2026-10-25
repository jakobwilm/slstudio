#include "IntensityCurveWindow.h"
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
void drawIntensityValues(const char *label, const float *values, int count, ImVec2 size, const int *levels) {
  ImGui::PlotLines(label, values, count, 0, nullptr, 0, 255, size);
  if (count == 0 || !ImGui::IsItemHovered()) return;
  const auto padding = ImGui::GetStyle().FramePadding;
  const auto minimum = ImGui::GetItemRectMin();
  const auto maximum = ImGui::GetItemRectMax();
  const auto mouse = ImGui::GetIO().MousePos;
  const float width = maximum.x - minimum.x - 2 * padding.x;
  if (width <= 0 || mouse.x < minimum.x + padding.x || mouse.x > maximum.x - padding.x ||
      mouse.y < minimum.y + padding.y || mouse.y > maximum.y - padding.y) return;
  const float position = (mouse.x - minimum.x - padding.x) / width;
  const int index = std::clamp(static_cast<int>(std::lround(position * (count - 1))), 0, count - 1);
  ImGui::SetTooltip("%d : %.4g", levels ? levels[index] : index, values[index]);
}
}

void drawIntensityPlot(const char *label, const float *values, int count, ImVec2 size,
    const char *xLabel, const char *yLabel, const int *levels) {
  const auto origin = ImGui::GetCursorScreenPos();
  const float width = size.x > 0 ? size.x : ImGui::GetContentRegionAvail().x + size.x;
  const float labelWidth = ImGui::GetTextLineHeight() + ImGui::GetStyle().ItemSpacing.x;
  const ImVec2 plotSize(std::max(1.0f, width - labelWidth),
      std::max(1.0f, size.y - ImGui::GetTextLineHeightWithSpacing()));
  ImGui::SetCursorScreenPos(ImVec2(origin.x + labelWidth, origin.y));
  drawIntensityValues(label, values, count, plotSize, levels);
  ImGui::SetCursorScreenPos(ImVec2(origin.x + labelWidth +
      (plotSize.x - ImGui::CalcTextSize(xLabel).x) * 0.5f,
      origin.y + plotSize.y + ImGui::GetStyle().ItemSpacing.y));
  ImGui::TextUnformatted(xLabel);
  auto *drawList = ImGui::GetWindowDrawList();
  const int firstVertex = drawList->VtxBuffer.Size;
  // Emit glyphs inside the clip rectangle before rotating them into the plot margin.
  const auto textOrigin = drawList->GetClipRectMin();
  drawList->AddText(textOrigin, ImGui::GetColorU32(ImGuiCol_Text), yLabel);
  const auto textSize = ImGui::CalcTextSize(yLabel);
  for (int index = firstVertex; index < drawList->VtxBuffer.Size; ++index) {
    auto &position = drawList->VtxBuffer[index].pos;
    const float x = position.x - textOrigin.x;
    const float y = position.y - textOrigin.y;
    position = ImVec2(origin.x + y, origin.y + (plotSize.y + textSize.x) * 0.5f - x);
  }
}

void IntensityCurveWindow::draw(const ApplicationSettings &settings) {
    if (!open) return;
    const auto *viewport = ImGui::GetMainViewport();
    const cv::Size currentViewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
    const auto placement = !viewportSize.empty() && viewportSize != currentViewportSize ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
    viewportSize = currentViewportSize;
    ImGui::SetNextWindowSize(ImVec2(std::min(uiPixels(560), viewport->WorkSize.x), std::min(uiPixels(280), viewport->WorkSize.y)), placement);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
        viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), placement, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(uiPixels(400), viewport->WorkSize.x), std::min(uiPixels(240), viewport->WorkSize.y)), viewport->WorkSize);
    if (!ImGui::Begin("Projector/camera intensity curve", &open)) { ImGui::End(); return; }
    if (settings.intensityLut) {
      std::array<float, 256> curve{};
      for (std::size_t index = 0; index < curve.size(); ++index)
        curve[index] = settings.intensityLut->inverseLut[index];
      const auto available = ImGui::GetContentRegionAvail();
      drawIntensityPlot("##intensity-compensation", curve.data(), static_cast<int>(curve.size()), available);
    } else {
      ImGui::TextUnformatted("No stored projector/camera intensity curve");
    }
    ImGui::End();
}
}
