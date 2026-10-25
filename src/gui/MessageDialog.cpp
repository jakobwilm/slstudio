#include "MessageDialog.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <utility>

namespace slstudio::gui {
void MessageDialog::request(std::string title, std::string question, std::string confirmLabel,
    std::string details, std::string icon) {
  message_ = Message{std::move(title), std::move(question), std::move(confirmLabel),
      std::move(details), std::move(icon)};
  opened_ = false;
}

void MessageDialog::draw(const std::function<void()> &confirm) {
  if (!message_) return;
  if (!opened_) {
    ImGui::OpenPopup(message_->title.c_str());
    opened_ = true;
  }
  const auto *viewport = ImGui::GetMainViewport();
  const float width = std::min(uiPixels(560), viewport->WorkSize.x);
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0), ImVec2(width, viewport->WorkSize.y));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(uiPixels(16), uiPixels(12)));
  const bool visible = ImGui::BeginPopupModal(message_->title.c_str(), nullptr,
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove);
  ImGui::PopStyleVar();
  if (!visible) return;

  if (!message_->icon.empty()) {
    ImGui::TextUnformatted(message_->icon.c_str());
    ImGui::SameLine();
  }
  ImGui::TextWrapped("%s", message_->question.c_str());
  if (!message_->details.empty()) {
    ImGui::Spacing();
    ImGui::TextWrapped("%s", message_->details.c_str());
  }
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();
  const float confirmWidth = std::max(uiPixels(120),
      ImGui::CalcTextSize(message_->confirmLabel.c_str()).x + 2 * ImGui::GetStyle().FramePadding.x);
  const float cancelWidth = std::max(uiPixels(100),
      ImGui::CalcTextSize("Cancel").x + 2 * ImGui::GetStyle().FramePadding.x);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f,
      ImGui::GetContentRegionAvail().x - confirmWidth - cancelWidth - ImGui::GetStyle().ItemSpacing.x));

  if (ImGui::Button(message_->confirmLabel.c_str(), ImVec2(confirmWidth, 0))) {
    confirm();
    ImGui::CloseCurrentPopup();
    message_.reset();
    opened_ = false;
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(cancelWidth, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    ImGui::CloseCurrentPopup();
    message_.reset();
    opened_ = false;
  }
  ImGui::EndPopup();
}
}
