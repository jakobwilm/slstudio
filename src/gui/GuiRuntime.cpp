#include "GuiRuntime.h"
#include "ApplicationController.h"
#include "AppIcon.h"
#include <imgui.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

namespace slstudio::gui {
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
    font.SizePixels = std::floor(13.0f * uiScale);
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
  ApplicationController application(isolated);
  return loop(application, GuiContext{window, updateFonts});
}
}
