#pragma once

#include <filesystem>
#include <functional>

struct GLFWwindow;

namespace slstudio::gui {
struct ApplicationController;

struct GuiContext {
  GLFWwindow *window;
  std::function<void()> updateFonts;

  void prepareFrame() const;
  void renderDrawData() const;
  void saveScreenshot(const std::filesystem::path &path) const;
};

using GuiLoop = std::function<int(ApplicationController &, const GuiContext &)>;
int runGui(bool isolated, const GuiLoop &loop);
}
