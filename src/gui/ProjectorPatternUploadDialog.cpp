#include "ProjectorPatternUploadDialog.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void ProjectorPatternUploadDialog::draw(bool uploadActive) {
  constexpr const char *title = "Uploading projector patterns";
  if (progress && !ImGui::IsPopupOpen(title)) ImGui::OpenPopup(title);
  const auto *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(std::min(uiPixels(440), viewport->WorkSize.x), 0), ImGuiCond_Always);
  if (ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
    if (!uploadActive) {
      progress.reset();
      ImGui::CloseCurrentPopup();
    } else if (progress) {
      std::lock_guard<std::mutex> lock(progress->mutex);
      ImGui::TextWrapped("%s", progress->stage.c_str());
      ImGui::ProgressBar(progress->fraction, ImVec2(-1, 0));
      ImGui::TextWrapped("Keep projector power connected. Do not unplug USB.\n"
          "Progress reflects programming stages, not time remaining.");
    }
    ImGui::EndPopup();
  }
}
}
