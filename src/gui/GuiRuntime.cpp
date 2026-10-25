#include "GuiRuntime.h"
#include "GuiLayout.h"
#include "ApplicationController.h"
#include "AppIcon.h"
#include <imgui.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

namespace slstudio::gui {
namespace {
template <typename... Args>
void requestRedraw(GLFWwindow *window, Args...) {
  *static_cast<int *>(glfwGetWindowUserPointer(window)) = 2;
}
}

void registerDesktopLauncher() {
#ifdef __linux__
  if (const auto *image = std::getenv("APPIMAGE"); image && *image) return;
  const auto *xdg = std::getenv("XDG_DATA_HOME");
  std::filesystem::path dataHome;
  if (xdg && *xdg && std::filesystem::path(xdg).is_absolute()) dataHome = xdg;
  else {
    const auto *home = std::getenv("HOME");
    if (!home || !*home) throw std::runtime_error("No home directory for desktop launcher");
    dataHome = std::filesystem::path(home) / ".local/share";
  }
  const auto desktop = dataHome / "applications/slstudio.desktop";
  std::string existing;
  if (!std::filesystem::is_symlink(desktop) && std::filesystem::exists(desktop)) {
    std::ifstream input(desktop);
    if (!input) throw std::runtime_error("Cannot read desktop launcher: " + desktop.string());
    existing.assign(std::istreambuf_iterator<char>(input), {});
    if (input.bad()) throw std::runtime_error("Cannot read desktop launcher: " + desktop.string());
  }
  if (std::filesystem::is_symlink(desktop) ||
      (std::filesystem::exists(desktop) &&
       ("\n" + existing).find("\nX-SLStudio-LocalBuild=true\n") == std::string::npos &&
       ("\n" + existing).find("\nX-SLStudio-AppImage=true\n") == std::string::npos)) {
    std::cerr << "SLStudio: leaving the existing desktop launcher unchanged: " << desktop << '\n';
    return;
  }
  auto escape = [](const std::string &value, bool command) {
    std::string result;
    for (const char c : value) {
      switch (c) {
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      case '\\': result += command ? "\\\\\\\\" : "\\\\"; break;
      case '"': case '`': case '$':
        if (command) result += "\\\\";
        result += c;
        break;
      case '%': result += command ? "%%" : "%"; break;
      default: result += c; break;
      }
    }
    return result;
  };
  const auto icon = dataHome / "SLStudio/slstudio.png";
  if (std::filesystem::is_symlink(icon))
    throw std::runtime_error("Refusing to replace a symbolic-link icon: " + icon.string());
  const auto executable = std::filesystem::read_symlink("/proc/self/exe");
  std::string entry;
  std::istringstream source(slstudioDesktopEntry);
  for (std::string line; std::getline(source, line);)
    if (line.rfind("Exec=", 0) != 0 && line.rfind("Icon=", 0) != 0) entry += line + '\n';
  entry += "Exec=/usr/bin/env \"" + escape(executable.string(), true) + "\"\nIcon=" +
      escape(icon.string(), false) + "\nX-SLStudio-LocalBuild=true\n";
  auto write = [](const auto &path, const char *data, std::size_t size) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(path, std::ios::binary);
    output.write(data, static_cast<std::streamsize>(size));
    output.close();
  };
  write(icon, reinterpret_cast<const char *>(slstudioIconPng), sizeof(slstudioIconPng));
  if (entry != existing) write(desktop, entry.data(), entry.size());
#endif
}

void GuiContext::prepareFrame() const {
  updateFonts();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
}

void GuiContext::renderDrawData() const {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glClearColor(0.9f, 0.9f, 0.9f, 1);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GuiContext::saveScreenshot(const std::filesystem::path &path) const {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  cv::Mat image(height, width, CV_8UC3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, image.data);
  cv::flip(image, image, 0);
  if (!cv::imwrite(path.string(), image)) throw std::runtime_error("Cannot write screenshot: " + path.string());
}

int runGui(bool isolated, const GuiLoop &loop) {
  glfwSetErrorCallback([](int, const char *message) { std::cerr << message << '\n'; });
  if (!glfwInit()) throw std::runtime_error("Cannot initialize GLFW");
  struct GLFWGuard { ~GLFWGuard() { glfwTerminate(); } } glfwGuard;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_VISIBLE, isolated ? GLFW_FALSE : GLFW_TRUE);
  glfwWindowHintString(GLFW_X11_CLASS_NAME, "slstudio");
  glfwWindowHintString(GLFW_X11_INSTANCE_NAME, "slstudio");
#ifdef GLFW_WAYLAND_APP_ID
  glfwWindowHintString(GLFW_WAYLAND_APP_ID, "slstudio");
#endif
  auto *window = glfwCreateWindow(1280, 800, "SLStudio", nullptr, nullptr);
  if (!window) throw std::runtime_error("OpenGL 3.3 Core is unavailable");
  struct WindowGuard { GLFWwindow *window; ~WindowGuard() { glfwDestroyWindow(window); } } windowGuard{window};
  int redrawFrames = 2;
  glfwSetWindowUserPointer(window, &redrawFrames);
  glfwSetKeyCallback(window, requestRedraw);
  glfwSetCharCallback(window, requestRedraw);
  glfwSetMouseButtonCallback(window, requestRedraw);
  glfwSetCursorPosCallback(window, requestRedraw);
  glfwSetCursorEnterCallback(window, requestRedraw);
  glfwSetScrollCallback(window, requestRedraw);
  glfwSetWindowFocusCallback(window, requestRedraw);
  glfwSetWindowRefreshCallback(window, requestRedraw);
  glfwSetWindowSizeCallback(window, requestRedraw);
  glfwSetFramebufferSizeCallback(window, requestRedraw);
  glfwSetWindowContentScaleCallback(window, requestRedraw);
#ifndef __APPLE__
#ifdef GLFW_PLATFORM_WAYLAND
  const bool windowIconSupported = glfwGetPlatform() != GLFW_PLATFORM_WAYLAND;
#else
  const bool windowIconSupported = std::string(glfwGetVersionString()).find("Wayland") == std::string::npos;
#endif
  if (windowIconSupported) {
    auto icon = cv::imdecode(std::vector<unsigned char>(std::begin(slstudioIconPng), std::end(slstudioIconPng)),
        cv::IMREAD_UNCHANGED);
    if (icon.empty() || icon.type() != CV_8UC4) throw std::runtime_error("Cannot decode the embedded application icon");
    cv::cvtColor(icon, icon, cv::COLOR_BGRA2RGBA);
    const GLFWimage image{icon.cols, icon.rows, icon.data};
    glfwSetWindowIcon(window, 1, &image);
  }
#endif
  glfwSetWindowSizeLimits(window, 900, 600, GLFW_DONT_CARE, GLFW_DONT_CARE);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glewExperimental = GL_TRUE;
  const auto loaderStatus = glewInit();
  if (!GLEW_VERSION_3_3)
    throw std::runtime_error(reinterpret_cast<const char *>(glewGetErrorString(loaderStatus)));
  glGetError();
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  struct ImGuiGuard { ~ImGuiGuard() {
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext();
  } } imguiGuard;
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsLight();
  auto &colors = ImGui::GetStyle().Colors;
  const ImVec4 blue(42.0f / 255, 179.0f / 255, 247.0f / 255, 1.0f);
  const ImVec4 highlight(blue.x * 0.85f, blue.y * 0.85f, blue.z * 0.85f, blue.w);
  auto accent = [&](ImGuiCol slot, ImVec4 color, float alpha = 1.0f) {
    color.w = alpha;
    colors[slot] = color;
  };
  for (const auto slot : {ImGuiCol_Button, ImGuiCol_Header, ImGuiCol_Tab, ImGuiCol_TabDimmedSelected})
    accent(slot, blue, 0.35f);
  for (const auto slot : {ImGuiCol_ButtonHovered, ImGuiCol_HeaderHovered, ImGuiCol_FrameBgHovered,
       ImGuiCol_TabHovered, ImGuiCol_ResizeGripHovered, ImGuiCol_SeparatorHovered})
    accent(slot, highlight, 0.55f);
  for (const auto slot : {ImGuiCol_ButtonActive, ImGuiCol_HeaderActive, ImGuiCol_SliderGrabActive,
       ImGuiCol_ResizeGripActive, ImGuiCol_SeparatorActive, ImGuiCol_TabSelectedOverline})
    accent(slot, highlight);
  for (const auto slot : {ImGuiCol_CheckMark, ImGuiCol_SliderGrab,
       ImGuiCol_PlotLines, ImGuiCol_PlotHistogram, ImGuiCol_TextLink, ImGuiCol_TabDimmedSelectedOverline})
    accent(slot, blue);
  accent(ImGuiCol_NavCursor, highlight);
  accent(ImGuiCol_FrameBgActive, highlight, 0.45f);
  accent(ImGuiCol_TabSelected, highlight, 0.35f);
  accent(ImGuiCol_TextSelectedBg, highlight, 0.40f);
  accent(ImGuiCol_ResizeGrip, blue, 0.25f);
  accent(ImGuiCol_PlotLinesHovered, highlight);
  accent(ImGuiCol_PlotHistogramHovered, highlight);
  accent(ImGuiCol_DragDropTarget, highlight);
  ImGui::GetStyle().FrameRounding = 2.0f;
  ImGui::GetStyle().WindowRounding = 0.0f;
  const ImGuiStyle baseStyle = ImGui::GetStyle();
  float fontUiScale = 0, fontDensity = 0;
  auto updateFonts = [&] {
    int windowWidth = 0, windowHeight = 0, framebufferWidth = 0, framebufferHeight = 0;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    if (windowWidth <= 0 || windowHeight <= 0 || framebufferWidth <= 0 || framebufferHeight <= 0) return;
    float contentX = 1, contentY = 1;
    glfwGetWindowContentScale(window, &contentX, &contentY);
    const float density = std::max({1.0f, static_cast<float>(framebufferWidth) / windowWidth,
                                   static_cast<float>(framebufferHeight) / windowHeight});
    const float uiScale = std::max(1.0f, std::max(contentX, contentY) / density);
    if (std::abs(fontUiScale - uiScale) < 0.01f && std::abs(fontDensity - density) < 0.01f) return;
    ImGui_ImplOpenGL3_DestroyFontsTexture();
    auto &io = ImGui::GetIO();
    io.Fonts->Clear();
    ImFontConfig font;
    font.SizePixels = std::floor(baseFontSize * uiScale);
    font.RasterizerDensity = density;
    font.OversampleH = font.OversampleV = 2;
    io.FontDefault = io.Fonts->AddFontDefault(&font);
    ImGui::GetStyle() = baseStyle;
    ImGui::GetStyle().ScaleAllSizes(uiScale);
    if (!ImGui_ImplOpenGL3_CreateFontsTexture()) throw std::runtime_error("Cannot create HiDPI font atlas");
    fontUiScale = uiScale;
    fontDensity = density;
  };
  if (!ImGui_ImplGlfw_InitForOpenGL(window, true) || !ImGui_ImplOpenGL3_Init("#version 330 core"))
    throw std::runtime_error("Cannot initialize ImGui backends");
  auto waitForEvents = [&](bool active) {
    if (active || redrawFrames > 0) {
      glfwPollEvents();
    } else {
      const auto deadline = Clock::now() + std::chrono::milliseconds(100);
      // Ignore incidental wakeups, but keep input and window changes responsive.
      while (!redrawFrames && !glfwWindowShouldClose(window)) {
        const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
        if (remaining <= 0) break;
        glfwWaitEventsTimeout(remaining);
      }
    }
    if (redrawFrames > 0) --redrawFrames;
  };
  ApplicationController application(isolated);
  return loop(application, GuiContext{window, updateFonts, waitForEvents});
}
}
