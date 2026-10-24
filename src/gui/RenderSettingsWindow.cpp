#include "RenderSettingsWindow.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void RenderSettingsWindow::draw(ApplicationSettings &settings, const PointCloudConstPtr &cloud) {
  if (!settings.renderSettingsOpen) return;
  const auto *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + std::max(0.0f, viewport->WorkSize.x - 320),
      viewport->WorkPos.y + 40), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(std::min(300.0f, viewport->WorkSize.x),
      0), ImGuiCond_Always);
  if (ImGui::Begin("Render Settings", &settings.renderSettingsOpen,
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::PushItemWidth(-110);
    int colorMode = static_cast<int>(settings.rendering.colorMode);
    const char *colorModes[] = {"Modulation", "Depth", "Solid"};
    if (ImGui::Combo("Color Mode", &colorMode, colorModes, IM_ARRAYSIZE(colorModes)))
      settings.rendering.colorMode = static_cast<PointCloudRenderer::ColorMode>(colorMode);
    if (settings.rendering.colorMode == PointCloudRenderer::ColorMode::Solid)
      ImGui::ColorEdit3("Solid Color", settings.rendering.solidColor.val);
    ImGui::BeginDisabled(cloud && cloud->pixelIndices.empty());
    ImGui::Checkbox("Surface Shading", &settings.rendering.surfaceShading);
    ImGui::EndDisabled();
    if (cloud && cloud->pixelIndices.empty() && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      ImGui::SetTooltip("Surface shading requires organized image-grid neighbors.");
    ImGui::SliderFloat("Point Size", &settings.rendering.pointSize, 1, 8, "%.1f");
    ImGui::PopItemWidth();
  }
  ImGui::End();
}
}
