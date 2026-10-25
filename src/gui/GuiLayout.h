#pragma once

#include <imgui.h>

namespace slstudio::gui {
inline constexpr float baseFontSize = 13.0f;

// GLFW reports HiDPI in window coordinates on some platforms and framebuffer pixels on others.
inline float uiPixels(float pixels) {
  return pixels * ImGui::GetFontSize() / baseFontSize;
}
}
