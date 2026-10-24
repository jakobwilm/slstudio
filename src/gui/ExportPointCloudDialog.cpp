#include "ExportPointCloudDialog.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void ExportPointCloudDialog::draw(ApplicationSettings &settings, const PointCloudConstPtr &cloud,
    std::string &status, const std::function<void()> &savePointCloud) {
  if (requested) {
    requested = false;
    ImGui::OpenPopup("Export Point Cloud");
  }
  ImGui::SetNextWindowSize(ImVec2(540, 0), ImGuiCond_FirstUseEver);
  if (!ImGui::BeginPopupModal("Export Point Cloud", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  ImGui::SetNextItemWidth(-1);
  ImGui::InputText("##Export Path", settings.exportPath.data(), settings.exportPath.size());
  ImGui::TextUnformatted("Destination file (.ply or .pcd)");
  ImGui::BeginDisabled(!cloud || cloud->points.empty() || !settings.exportPath[0]);
  const bool saveRequested = ImGui::Button("Export");
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
  else if (saveRequested) {
    try {
      savePointCloud();
      ImGui::CloseCurrentPopup();
    } catch (const std::exception &error) {
      this->error = status = error.what();
    }
  }
  ImGui::EndPopup();
}
}
