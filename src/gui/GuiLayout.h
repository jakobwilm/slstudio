#pragma once

#include <imgui.h>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace slstudio::gui {
inline constexpr float baseFontSize = 13.0f;

// GLFW reports HiDPI in window coordinates on some platforms and framebuffer pixels on others.
inline float uiPixels(float pixels) {
  return pixels * ImGui::GetFontSize() / baseFontSize;
}

enum class ConnectionState { Pending, Connected, Disconnected };

// Edits a microsecond value in milliseconds; storage stays in microseconds.
inline bool inputMilliseconds(const char *label, std::chrono::microseconds &value) {
  float milliseconds = static_cast<float>(value.count()) / 1000.0f;
  if (!ImGui::InputFloat(label, &milliseconds, 0.0f, 0.0f, "%.3f")) return false;
  value = std::chrono::microseconds(std::lround(std::max(milliseconds, 0.0f) * 1000.0f));
  return true;
}

inline void connectionIndicator(ConnectionState state, const char *label) {
  const auto position = ImGui::GetCursorScreenPos();
  const float diameter = ImGui::GetTextLineHeight();
  const ImU32 color = state == ConnectionState::Connected ? IM_COL32(25, 160, 75, 255) :
      state == ConnectionState::Disconnected ? IM_COL32(210, 50, 50, 255) : IM_COL32(220, 160, 20, 255);
  ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(position.x + diameter * 0.5f,
      position.y + diameter * 0.5f), diameter * 0.35f, color);
  ImGui::Dummy(ImVec2(diameter, diameter));
  ImGui::SameLine();
  ImGui::TextUnformatted(label);
}
}
