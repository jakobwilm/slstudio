#include "MessageDialog.h"
#include <imgui.h>
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
  if (!ImGui::BeginPopupModal(message_->title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    return;

  if (!message_->icon.empty()) {
    ImGui::TextUnformatted(message_->icon.c_str());
    ImGui::SameLine();
  }
  ImGui::TextWrapped("%s", message_->question.c_str());
  if (!message_->details.empty()) ImGui::TextWrapped("%s", message_->details.c_str());

  if (ImGui::Button(message_->confirmLabel.c_str())) {
    confirm();
    ImGui::CloseCurrentPopup();
    message_.reset();
    opened_ = false;
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    ImGui::CloseCurrentPopup();
    message_.reset();
    opened_ = false;
  }
  ImGui::EndPopup();
}
}
