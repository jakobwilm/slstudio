#include "IntensityCurveWindow.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void IntensityCurveWindow::draw(const ApplicationSettings &settings) {
    if (!open) return;
    const auto *viewport = ImGui::GetMainViewport();
    const cv::Size currentViewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
    const auto placement = !viewportSize.empty() && viewportSize != currentViewportSize ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
    viewportSize = currentViewportSize;
    ImGui::SetNextWindowSize(ImVec2(std::min(560.0f, viewport->WorkSize.x), std::min(280.0f, viewport->WorkSize.y)), placement);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
        viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), placement, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(400.0f, viewport->WorkSize.x), std::min(240.0f, viewport->WorkSize.y)), viewport->WorkSize);
    if (!ImGui::Begin("Projector/camera intensity curve", &open)) { ImGui::End(); return; }
    if (settings.intensityLut) {
      std::array<float, 256> curve{};
      for (std::size_t index = 0; index < curve.size(); ++index)
        curve[index] = settings.intensityLut->inverseLut[index];
      ImGui::TextUnformatted("Projector input (0-255)");
      ImGui::PlotLines("##intensity-compensation", curve.data(), static_cast<int>(curve.size()), 0, nullptr, 0, 255,
          ImVec2(-1, std::max(1.0f, ImGui::GetContentRegionAvail().y - ImGui::GetTextLineHeightWithSpacing())));
      ImGui::TextUnformatted("Desired intensity (0-255)");
    } else {
      ImGui::TextUnformatted("No stored projector/camera intensity curve");
    }
    ImGui::End();
}
}
