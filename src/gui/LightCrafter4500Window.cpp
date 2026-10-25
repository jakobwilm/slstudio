#include "LightCrafter4500Window.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void LightCrafter4500Window::draw(ApplicationSettings &settings, bool available,
    const std::optional<bool> &connected, const std::optional<slstudio::LightCrafter4500Firmware> &firmware,
    bool firmwareQueryActive, const std::string &firmwareError, bool busy, bool projectorOpen,
    const std::function<void()> &apply, const std::function<void()> &retry,
    const std::optional<slstudio::LightCrafter4500Status> &playback, const std::string &playbackError,
    const std::function<void(slstudio::LightCrafter4500Action)> &control,
    const std::function<void()> &refreshPlayback) {
  if (!open) return;
  const auto *viewport = ImGui::GetMainViewport();
  const cv::Size viewportSize(static_cast<int>(viewport->WorkSize.x), static_cast<int>(viewport->WorkSize.y));
  const auto placement = !this->viewportSize.empty() && this->viewportSize != viewportSize ? ImGuiCond_Always : ImGuiCond_FirstUseEver;
  this->viewportSize = viewportSize;
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), placement, ImVec2(0.5f, 0.5f));
  const float width = std::min(uiPixels(440), viewport->WorkSize.x);
  const float height = std::min(uiPixels(660), viewport->WorkSize.y);
  ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
  if (!ImGui::Begin("LightCrafter4500 Configuration", &open,
      ImGuiWindowFlags_NoResize)) { ImGui::End(); return; }
    const auto indicator = ImGui::GetCursorScreenPos();
    const float diameter = ImGui::GetTextLineHeight();
    const ImU32 color = !available || !connected ? IM_COL32(220, 160, 20, 255) :
      *connected ? IM_COL32(25, 160, 75, 255) : IM_COL32(210, 50, 50, 255);
    ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(indicator.x + diameter * 0.5f,
      indicator.y + diameter * 0.5f), diameter * 0.35f, color);
    ImGui::Dummy(ImVec2(diameter, diameter));
    ImGui::SameLine();
    ImGui::TextUnformatted(!available ? "LC4500 support not built" : !connected ? "Checking USB..." :
      *connected ? "USB connected" : "USB disconnected");
  if (firmware) {
    const auto &versions = *firmware;
    ImGui::Text("Firmware %s | API %s", slstudio::lightCrafter4500VersionString(versions[0]).c_str(),
        slstudio::lightCrafter4500VersionString(versions[1]).c_str());
    ImGui::Text("SW config %s | Sequence %s", slstudio::lightCrafter4500VersionString(versions[2]).c_str(),
        slstudio::lightCrafter4500VersionString(versions[3]).c_str());
  } else if (firmwareQueryActive) ImGui::TextDisabled("Reading firmware versions...");
  else if (connected.value_or(false) && projectorOpen) ImGui::TextDisabled("Close projector output to read firmware versions");
  if (projectorOpen) ImGui::TextDisabled("Playback controlled by acquisition");
  else if (!connected.value_or(false)) ImGui::TextDisabled("Playback status unavailable");
  else if (playback) {
    const char *state = playback->standby ? "Standby" : !playback->patternMode ? "Video / test pattern" :
        playback->playback == slstudio::LightCrafter4500Playback::Running ? "Running" :
        playback->playback == slstudio::LightCrafter4500Playback::Paused ? "Paused" : "Stopped";
    ImGui::Text("Playback: %s%s", state, playback->ledsDisabled ? " (LEDs off)" : "");
    if (playback->patternMode) {
      if (!playback->validationReady) ImGui::TextDisabled("Sequence validation pending");
      else if (playback->validationFlags != 0) {
        ImGui::Text("Sequence validation %s: 0x%02X",
            playback->validationPassed() ? "warnings" : "errors", playback->validationFlags);
        const char *errors[] = {"Invalid exposure / period", "Invalid pattern number",
            "Continuous trigger / overlapping black intervals", "No black vector before an external trigger",
            "Exposure / period difference below 230 us"};
        for (unsigned int bit = 0; bit < IM_ARRAYSIZE(errors); ++bit)
          if (playback->validationFlags & (1u << bit)) ImGui::TextWrapped("%s", errors[bit]);
      }
    }
  } else ImGui::TextDisabled("Playback status not read");
  if (!playbackError.empty()) ImGui::TextWrapped("%s", playbackError.c_str());
  auto &requestedSettings = settings.lc4500Settings;
  ImGui::BeginDisabled(busy || !available || projectorOpen);
  ImGui::PushItemWidth(-uiPixels(190));
  ImGui::Checkbox("Standby", &requestedSettings.standby);
  ImGui::BeginDisabled(requestedSettings.standby);
  int mode = static_cast<int>(requestedSettings.displayMode);
  const char *modes[] = {"HDMI patterns (8-bit)", "Stored flash patterns", "HDMI video", "Internal test pattern"};
  if (ImGui::Combo("Display mode", &mode, modes, IM_ARRAYSIZE(modes))) {
    requestedSettings.displayMode = static_cast<slstudio::LightCrafter4500Mode>(mode);
    if (requestedSettings.displayMode == slstudio::LightCrafter4500Mode::HdmiPatterns) {
      const slstudio::LightCrafter4500Settings defaults;
      requestedSettings.exposure = defaults.exposure;
      requestedSettings.framePeriod = defaults.framePeriod;
    }
  }
  const bool flash = requestedSettings.displayMode == slstudio::LightCrafter4500Mode::FlashPatterns;
  const bool patterns = flash || requestedSettings.displayMode == slstudio::LightCrafter4500Mode::HdmiPatterns;
  if (requestedSettings.displayMode == slstudio::LightCrafter4500Mode::TestPattern) {
    const char *tests[] = {"Solid field", "Horizontal ramp", "Vertical ramp", "Horizontal lines", "Diagonal lines",
        "Vertical lines", "Grid", "Checkerboard", "RGB ramp", "Color bars", "Step bars"};
    ImGui::Combo("Test pattern", &requestedSettings.testPattern, tests, IM_ARRAYSIZE(tests));
  }
  if (flash) {
    ImGui::InputInt("Stored pattern count", &requestedSettings.flashPatternCount);
    ImGui::TextWrapped("Plays consecutive 8-bit patterns from flash image 0; three patterns per image. No flash programming.");
  }
  if (requestedSettings.displayMode == slstudio::LightCrafter4500Mode::HdmiPatterns)
    ImGui::TextWrapped("Repeats the same eight-bit HDMI pattern twice per frame. "
        "Default: 60 Hz HDMI, two 8333 us exposures. Output LEDs are selected below.");
  if (patterns) {
    ImGui::SeparatorText("Pattern Timing");
    int sequenceExposure = static_cast<int>(requestedSettings.exposure.count());
    int period = static_cast<int>(requestedSettings.framePeriod.count());
    if (ImGui::InputInt("Exposure (us)", &sequenceExposure)) requestedSettings.exposure = std::chrono::microseconds(sequenceExposure);
    if (ImGui::InputInt("Pattern period (us)", &period)) requestedSettings.framePeriod = std::chrono::microseconds(period);
    ImGui::TextWrapped("Frame period must equal exposure or exceed it by more than 230 us.");
    if (flash)
      ImGui::TextWrapped("Flash playback automatically keeps at least 231 us between exposures for camera triggering.");
    ImGui::TextWrapped("Eight-bit patterns require at least 8333 us exposure.");
  }
  ImGui::SeparatorText("Illumination");
  const char *leds[] = {"Red", "Green", "Red + green", "Blue", "Red + blue", "Green + blue", "All LEDs"};
  int selection = requestedSettings.ledSelection - 1;
  if (ImGui::Combo("Enabled LEDs", &selection, leds, IM_ARRAYSIZE(leds))) requestedSettings.ledSelection = selection + 1;
  ImGui::SliderInt("Red LED", &requestedSettings.ledCurrent[0], 0, 255);
  ImGui::SliderInt("Green LED", &requestedSettings.ledCurrent[1], 0, 255);
  ImGui::SliderInt("Blue LED", &requestedSettings.ledCurrent[2], 0, 255);
  ImGui::SeparatorText("Orientation");
  ImGui::Checkbox("Flip long axis", &requestedSettings.flipLongAxis);
  ImGui::Checkbox("Flip short axis", &requestedSettings.flipShortAxis);
  if (patterns) {
    ImGui::SeparatorText("Trigger Outputs");
    ImGui::Checkbox("Inverted triggers", &requestedSettings.triggerInverted);
  }
  ImGui::EndDisabled();
  std::string settingsError;
  try { slstudio::validateLightCrafterSettings(requestedSettings); }
  catch (const std::invalid_argument &error) { settingsError = error.what(); }
  if (!settingsError.empty()) ImGui::TextWrapped("%s", settingsError.c_str());
  ImGui::SeparatorText("Playback");
  ImGui::TextWrapped("Apply configures patterns without starting them. Start, Pause and Stop are available in both pattern modes. Blank disables LEDs in any mode.");
  ImGui::BeginDisabled(!connected.value_or(false) || projectorOpen);
  const bool patternPlayback = playback && playback->patternMode && !playback->standby;
  const bool running = patternPlayback && playback->playback == slstudio::LightCrafter4500Playback::Running;
  ImGui::BeginDisabled(!patternPlayback || !playback->validationPassed() ||
      (running && !playback->ledsDisabled));
  if (ImGui::Button("Start")) control(slstudio::LightCrafter4500Action::Start);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!running);
  if (ImGui::Button("Pause")) control(slstudio::LightCrafter4500Action::Pause);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!patternPlayback || playback->playback == slstudio::LightCrafter4500Playback::Stopped);
  if (ImGui::Button("Stop")) control(slstudio::LightCrafter4500Action::Stop);
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Blank")) control(slstudio::LightCrafter4500Action::Blank);
  ImGui::SameLine();
  if (ImGui::Button("Refresh status")) refreshPlayback();
  ImGui::EndDisabled();
  ImGui::BeginDisabled(!settingsError.empty());
  if (ImGui::Button("Apply settings")) apply();
  ImGui::EndDisabled();
  ImGui::PopItemWidth();
  ImGui::EndDisabled();
  if (!available) ImGui::TextUnformatted("LC4500 backend unavailable");
  if (!firmwareError.empty()) {
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Retry firmware query")) retry();
    ImGui::EndDisabled();
  }
  ImGui::End();
}
}
