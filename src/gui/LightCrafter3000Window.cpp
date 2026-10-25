#include "LightCrafter3000Window.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void LightCrafter3000Window::draw(ApplicationSettings &settings, bool available, bool busy,
    bool projectorOpen, const std::function<void()> &apply) {
  if (!open) return;
  const auto *viewport = ImGui::GetMainViewport();
  const cv::Size viewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
  const auto placement = !this->viewportSize.empty() && this->viewportSize != viewportSize ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
  this->viewportSize = viewportSize;
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), placement, ImVec2(0.5f, 0.5f));
  const float width = std::min(uiPixels(440), viewport->WorkSize.x);
  ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0), ImVec2(width, viewport->WorkSize.y));
  ImGui::SetNextWindowSize(ImVec2(width, 0), ImGuiCond_Always);
  if (!ImGui::Begin("LightCrafter3000 Configuration", &open,
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) { ImGui::End(); return; }
  auto &requestedSettings = settings.lc3000Settings;
  ImGui::BeginDisabled(busy || !available);
  ImGui::PushItemWidth(-uiPixels(190));
  ImGui::InputText("IPv4 address", settings.projectorAddress.data(), settings.projectorAddress.size());
  ImGui::Checkbox("Standby", &requestedSettings.standby);
  ImGui::BeginDisabled(requestedSettings.standby);
  ImGui::SeparatorText("HDMI Video");
  ImGui::TextUnformatted("608 x 684 | 8-bit");
  ImGui::SliderInt("Frame rate (Hz)", &requestedSettings.frameRate, 1, 60);
  ImGui::SeparatorText("Illumination");
  ImGui::SliderInt("Red LED", &requestedSettings.ledCurrent[0], 0, 255);
  ImGui::SliderInt("Green LED", &requestedSettings.ledCurrent[1], 0, 255);
  ImGui::SliderInt("Blue LED", &requestedSettings.ledCurrent[2], 0, 255);
  ImGui::SeparatorText("Orientation");
  ImGui::Checkbox("Rotate", &requestedSettings.rotate);
  ImGui::Checkbox("Flip long axis", &requestedSettings.flipLongAxis);
  ImGui::Checkbox("Flip short axis", &requestedSettings.flipShortAxis);
  ImGui::SeparatorText("Camera Trigger Output");
  ImGui::Checkbox("Trigger enabled", &requestedSettings.triggerEnabled);
  ImGui::BeginDisabled(!requestedSettings.triggerEnabled);
  ImGui::Checkbox("Inverted trigger", &requestedSettings.triggerInverted);
  ImGui::InputInt("Trigger delay (us)", &requestedSettings.triggerDelayUs);
  ImGui::InputInt("Pulse width (us)", &requestedSettings.triggerPulseUs);
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  if (ImGui::Button("Apply settings")) apply();
  ImGui::PopItemWidth();
  ImGui::EndDisabled();
  if (!available) ImGui::TextUnformatted("LC3000 backend unavailable");
  if (projectorOpen) ImGui::TextUnformatted("Projector output open");
  ImGui::End();
}
}
