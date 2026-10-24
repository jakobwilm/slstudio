#include "CameraProjectorHeader.h"
#include <imgui.h>

namespace slstudio::gui {
void CameraProjectorHeader::draw(cv::Size cameraSize, cv::Size projectorSize,
    const std::function<void()> &openSettings) {
  const char *label = "Camera/Projector Settings";
  const float buttonWidth = ImGui::CalcTextSize(label).x + 2 * ImGui::GetStyle().FramePadding.x;
  if (ImGui::BeginTable("Camera/projector header", 2, ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Resolutions", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Settings", ImGuiTableColumnFlags_WidthFixed, buttonWidth);
    ImGui::TableNextColumn();
    ImGui::AlignTextToFramePadding();
    ImGui::TextWrapped("Camera: %d x %d | Projector: %d x %d", cameraSize.width, cameraSize.height,
        projectorSize.width, projectorSize.height);
    ImGui::TableNextColumn();
    if (ImGui::Button(label, ImVec2(buttonWidth, 0))) {
      openSettings();
      ImGui::SetWindowCollapsed("Settings", false);
      ImGui::SetWindowFocus("Settings");
    }
    ImGui::EndTable();
  }
}
}
