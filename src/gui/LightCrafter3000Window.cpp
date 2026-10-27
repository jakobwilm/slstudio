#include "LightCrafter3000Window.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void LightCrafter3000Window::populateControls(ApplicationSettings &settings,
    const slstudio::LightCrafter3000DeviceState &state) {
  if (state.triggerDelayUs > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      state.triggerPulseUs > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
    throw std::runtime_error("LC3000 trigger timing exceeds the control range");
  if (state.displayMode == 1 &&
      (state.testPattern < 0 || state.testPattern >= static_cast<int>(slstudio::lightCrafter3000TestPatterns.size())))
    throw std::runtime_error("LC3000 returned an unknown test pattern");
  auto &requested = settings.lc3000Settings;
  requested.standby = false;
  requested.ledCurrent = state.ledCurrent;
  requested.rotate = state.rotate;
  requested.flipLongAxis = state.flipLongAxis;
  requested.flipShortAxis = state.flipShortAxis;
  requested.frameRate = state.frameRate;
  requested.triggerEnabled = state.triggerEnabled;
  requested.triggerInverted = state.triggerInverted;
  requested.triggerDelayUs = static_cast<int>(state.triggerDelayUs);
  requested.triggerPulseUs = static_cast<int>(state.triggerPulseUs);
  manualControl.timing.exposure = std::chrono::microseconds(state.exposureUs);
  manualControl.timing.framePeriod = std::chrono::microseconds(state.periodUs);
  if (state.inputTrigger < 0 ||
      state.inputTrigger >= static_cast<int>(slstudio::lightCrafter3000SequenceTriggers.size()))
    throw std::runtime_error("LC3000 returned an unknown sequence trigger");
  manualControl.sequenceTrigger = static_cast<slstudio::LightCrafter3000SequenceTrigger>(state.inputTrigger);
  manualControl.inputTriggerDelayUs = state.inputTriggerDelayUs;
  manualControl.timing.repeat = state.repeat;
  if (state.displayMode == 1) manualControl.testPattern = state.testPattern;
  // TI display modes: 0 static image, 1 test pattern, 2 HDMI video, 4 pattern sequence.
  switch (state.displayMode) {
    case 0: mode = Mode::SolidColor; break;
    case 1: mode = Mode::TestPattern; break;
    case 4: mode = Mode::PatternSequence; break;
    default: mode = Mode::Hdmi; break;
  }
}

void LightCrafter3000Window::draw(ApplicationSettings &settings, bool available,
    const std::optional<bool> &connected, bool connectionCheckPaused,
    const std::array<std::string, 3> &firmware, const std::string &connectionError, bool busy,
    bool projectorOpen, const std::function<void()> &apply,
    const std::function<void(const std::string &)> &changeIp,
    const std::function<void(const slstudio::LightCrafter3000Control &)> &control) {
  if (!open) return;
  const auto *viewport = ImGui::GetMainViewport();
  const cv::Size viewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
  const auto placement = !this->viewportSize.empty() && this->viewportSize != viewportSize ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
  this->viewportSize = viewportSize;
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), placement, ImVec2(0.5f, 0.5f));
  const float width = std::min(uiPixels(840), viewport->WorkSize.x);
  const float height = std::min(uiPixels(530), viewport->WorkSize.y);
  ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
  if (!ImGui::Begin("LightCrafter3000 Configuration", &open,
      ImGuiWindowFlags_NoResize)) { ImGui::End(); return; }
  auto &requestedSettings = settings.lc3000Settings;
  const std::string address = settings.projectorAddress.data();
  connectionIndicator(!available || !connected ? ConnectionState::Pending :
      *connected ? ConnectionState::Connected : ConnectionState::Disconnected,
      (!available ? std::string("LC3000 support not built") :
      connectionCheckPaused ? "Busy" :
      !connected ? "Checking " + address + "..." :
      *connected ? "LC3000 responding at " + address : "No LC3000 response at " + address).c_str());
  // Fixed height keeps the controls from jumping while checks pause during commands.
  ImGui::BeginChild("LC3000 firmware", ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 2),
      ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  if (connected != false && !firmware[0].empty()) {
    ImGui::Text("DM365 %s | FPGA %s", firmware[0].c_str(), firmware[1].c_str());
    ImGui::Text("MSP430 %s", firmware[2].c_str());
  } else if (!connectionCheckPaused && !connectionError.empty())
    ImGui::TextWrapped("%s", connectionError.c_str());
  ImGui::EndChild();
  ImGui::BeginDisabled(busy || !available);
  ImGui::PushItemWidth(-uiPixels(190));
  ImGui::InputText("Connect to IPv4", settings.projectorAddress.data(), settings.projectorAddress.size());
  if (ImGui::Button("Change device IP...")) {
    std::snprintf(newIpAddress.data(), newIpAddress.size(), "%s", settings.projectorAddress.data());
    ImGui::OpenPopup("Change LC3000 IP address");
  }
  ImGui::PopItemWidth();
  auto command = [&](slstudio::LightCrafter3000Action action) {
    auto request = manualControl;
    request.action = action;
    control(request);
  };
  ImGui::BeginChild("LC3000 controls", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() * 2));
  const int columns = ImGui::GetContentRegionAvail().x >= uiPixels(700) ? 2 : 1;
  if (ImGui::BeginTable("LC3000 layout", columns, ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableNextColumn();
    ImGui::PushItemWidth(-uiPixels(160));
    if (requestedSettings.standby)
      ImGui::Checkbox("Standby (unsupported; uncheck to apply)", &requestedSettings.standby);
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
    if (ImGui::Button("Apply settings")) {
      mode = Mode::Hdmi;
      apply();
    }
    ImGui::TextDisabled("Apply selects HDMI video.");
    ImGui::PopItemWidth();
    if (columns == 2) ImGui::TableNextColumn();
    ImGui::PushItemWidth(-uiPixels(160));
    ImGui::SeparatorText("Display Mode");
    int selectedMode = static_cast<int>(mode);
    const char *modes[] = {"HDMI video", "Test pattern", "Solid colour", "Pattern sequence"};
    if (ImGui::Combo("Mode", &selectedMode, modes, IM_ARRAYSIZE(modes))) mode = static_cast<Mode>(selectedMode);
    switch (mode) {
    case Mode::Hdmi: {
      ImGui::TextUnformatted("608 x 684 | 8-bit");
      const std::string rate = std::to_string(requestedSettings.frameRate) + " Hz";
      if (ImGui::BeginCombo("Frame rate", rate.c_str())) {
        for (const int value : slstudio::lightCrafter3000FrameRates) {
          const std::string label = std::to_string(value) + " Hz";
          if (ImGui::Selectable(label.c_str(), requestedSettings.frameRate == value))
            requestedSettings.frameRate = value;
        }
        ImGui::EndCombo();
      }
      ImGui::TextDisabled("Apply settings to show HDMI video.");
      break;
    }
    case Mode::TestPattern: {
      const auto &patterns = slstudio::lightCrafter3000TestPatterns;
      if (ImGui::BeginCombo("Test pattern", patterns[manualControl.testPattern])) {
        for (int index = 0; index < static_cast<int>(patterns.size()); ++index)
          if (ImGui::Selectable(patterns[index], manualControl.testPattern == index))
            manualControl.testPattern = index;
        ImGui::EndCombo();
      }
      if (ImGui::Button("Show test pattern")) command(slstudio::LightCrafter3000Action::TestPattern);
      break;
    }
    case Mode::SolidColor:
      ImGui::SliderInt("Solid red", &manualControl.color[0], 0, 255);
      ImGui::SliderInt("Solid green", &manualControl.color[1], 0, 255);
      ImGui::SliderInt("Solid blue", &manualControl.color[2], 0, 255);
      if (ImGui::Button("Show solid colour")) command(slstudio::LightCrafter3000Action::SolidColor);
      ImGui::SameLine();
      if (ImGui::Button("Black")) {
        auto request = manualControl;
        request.action = slstudio::LightCrafter3000Action::SolidColor;
        request.color = {0, 0, 0};
        control(request);
      }
      ImGui::SameLine();
      if (ImGui::Button("White")) {
        auto request = manualControl;
        request.action = slstudio::LightCrafter3000Action::SolidColor;
        request.color = {255, 255, 255};
        control(request);
      }
      break;
    case Mode::PatternSequence:
      ImGui::TextDisabled("Stored patterns; zero-based indices.");
      if (deviceState) ImGui::Text("Patterns: %d", deviceState->patternCount);
      ImGui::InputInt("Pattern index", &manualControl.patternIndex);
      if (ImGui::Button("Show pattern")) command(slstudio::LightCrafter3000Action::SelectPattern);
      ImGui::SameLine();
      ImGui::BeginDisabled(manualControl.patternIndex <= 0);
      if (ImGui::Button("Previous")) {
        --manualControl.patternIndex;
        command(slstudio::LightCrafter3000Action::SelectPattern);
      }
      ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::BeginDisabled(!deviceState || manualControl.patternIndex < 0 ||
          manualControl.patternIndex >= deviceState->patternCount - 1);
      if (ImGui::Button("Next")) {
        ++manualControl.patternIndex;
        command(slstudio::LightCrafter3000Action::SelectPattern);
      }
      ImGui::EndDisabled();
      inputMilliseconds("Exposure (ms)", manualControl.timing.exposure);
      int sequenceTrigger = static_cast<int>(manualControl.sequenceTrigger);
      if (ImGui::Combo("Input trigger", &sequenceTrigger, slstudio::lightCrafter3000SequenceTriggers.data(),
          static_cast<int>(slstudio::lightCrafter3000SequenceTriggers.size())))
        manualControl.sequenceTrigger = static_cast<slstudio::LightCrafter3000SequenceTrigger>(sequenceTrigger);
      ImGui::InputScalar("Input delay (us)", ImGuiDataType_U32, &manualControl.inputTriggerDelayUs);
      if (manualControl.sequenceTrigger == slstudio::LightCrafter3000SequenceTrigger::Auto) {
        inputMilliseconds("Frame period (ms)", manualControl.timing.framePeriod);
        ImGui::TextDisabled("Auto trigger ignores repeat; playback continues until stopped.");
      } else {
        ImGui::Checkbox("Repeat sequence", &manualControl.timing.repeat);
        if (manualControl.sequenceTrigger == slstudio::LightCrafter3000SequenceTrigger::Software)
          ImGui::TextDisabled("Use Advance pattern to step the sequence.");
        else
          ImGui::TextDisabled("Requires the selected physical trigger signal.");
      }
      if (ImGui::Button("Start sequence")) command(slstudio::LightCrafter3000Action::Start);
      ImGui::SameLine();
      if (ImGui::Button("Stop sequence")) command(slstudio::LightCrafter3000Action::Stop);
      if (manualControl.sequenceTrigger == slstudio::LightCrafter3000SequenceTrigger::Software &&
          ImGui::Button("Advance pattern"))
        command(slstudio::LightCrafter3000Action::Advance);
      ImGui::TextDisabled("Stop holds the current pattern.");
      break;
    }
    ImGui::PopItemWidth();
    ImGui::EndTable();
  }
  ImGui::EndChild();
  ImGui::EndDisabled();
  if (!diagnosticsStatus.empty()) ImGui::TextWrapped("%s", diagnosticsStatus.c_str());
  if (!available) ImGui::TextUnformatted("LC3000 backend unavailable");
  if (projectorOpen) ImGui::TextUnformatted("Controlled by acquisition");
  if (!ipChangeStatus.empty()) ImGui::TextWrapped("%s", ipChangeStatus.c_str());
  if (ImGui::BeginPopupModal("Change LC3000 IP address", nullptr,
      ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Current connection target: %s", settings.projectorAddress.data());
    ImGui::InputText("New device IPv4", newIpAddress.data(), newIpAddress.size());
    ImGui::TextWrapped("Host network settings are unchanged. If reconnection fails, try either address.");
    ImGui::BeginDisabled(busy || !available || projectorOpen);
    if (ImGui::Button("Change IP")) {
      changeIp(newIpAddress.data());
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
    ImGui::EndDisabled();
    ImGui::EndPopup();
  }
  ImGui::End();
}
}
