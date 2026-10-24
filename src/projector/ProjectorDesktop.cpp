#include "ProjectorFactory.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace slstudio {
namespace {

struct WindowDeleter {
  void operator()(GLFWwindow *window) const { glfwDestroyWindow(window); }
};

class DesktopProjector final : public Projector {
public:
  explicit DesktopProjector(GLFWmonitor *monitor) {
    const auto *mode = glfwGetVideoMode(monitor);
    if (!mode)
      throw std::runtime_error("Cannot read projector video mode");
    size = {mode->width, mode->height};
    auto *previous = glfwGetCurrentContext();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
    window.reset(glfwCreateWindow(size.width, size.height, "SLStudio projector", monitor, nullptr));
    if (!window)
      throw std::runtime_error("Cannot create desktop projector window");
    glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
    glfwMakeContextCurrent(window.get());
    glewExperimental = GL_TRUE;
    glewInit();
    if (!GLEW_VERSION_3_3) {
      glfwMakeContextCurrent(previous);
      throw std::runtime_error("Cannot load projector OpenGL functions");
    }
    glfwSwapInterval(1);
    glGenFramebuffers(1, &framebuffer);
    glfwMakeContextCurrent(previous);
    blank();
  }

  ~DesktopProjector() override {
    auto *previous = glfwGetCurrentContext();
    glfwMakeContextCurrent(window.get());
    clearTextures();
    glDeleteFramebuffers(1, &framebuffer);
    glfwMakeContextCurrent(previous == window.get() ? nullptr : previous);
  }

  cv::Size resolution() const override { return size; }

  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    for (const auto &pattern : patterns)
      if (pattern.size() != size || (pattern.type() != CV_8UC3 && pattern.type() != CV_8UC1))
        throw std::invalid_argument("Projector patterns must match output pixels");
    auto *previous = glfwGetCurrentContext();
    glfwMakeContextCurrent(window.get());
    clearTextures();
    textures.resize(patterns.size());
    glGenTextures(static_cast<GLsizei>(textures.size()), textures.data());
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    for (std::size_t index = 0; index < patterns.size(); ++index) {
      cv::Mat rgb;
      if (patterns[index].channels() == 1)
        cv::cvtColor(patterns[index], rgb, cv::COLOR_GRAY2RGB);
      else
        rgb = patterns[index].clone();
      glBindTexture(GL_TEXTURE_2D, textures[index]);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, size.width, size.height, 0,
                   GL_RGB, GL_UNSIGNED_BYTE, rgb.data);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glfwMakeContextCurrent(previous);
  }

  void present(std::size_t index) override {
    if (index >= textures.size())
      throw std::out_of_range("Projector pattern index");
    if (glfwWindowShouldClose(window.get()))
      throw std::runtime_error("Projector window was closed");
    auto *previous = glfwGetCurrentContext();
    glfwMakeContextCurrent(window.get());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, textures[index], 0);
    int width, height;
    glfwGetFramebufferSize(window.get(), &width, &height);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_DITHER);
    glBlitFramebuffer(0, size.height, size.width, 0, 0, 0, width, height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glfwSwapBuffers(window.get());
    glfwMakeContextCurrent(previous);
  }

  void blank(bool white = false) override {
    auto *previous = glfwGetCurrentContext();
    glfwMakeContextCurrent(window.get());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    const float value = white ? 1.0f : 0.0f;
    glClearColor(value, value, value, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window.get());
    glfwMakeContextCurrent(previous);
  }

private:
  void clearTextures() {
    glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
    textures.clear();
  }
  std::unique_ptr<GLFWwindow, WindowDeleter> window;
  cv::Size size;
  GLuint framebuffer = 0;
  std::vector<GLuint> textures;
};
}

std::vector<ProjectorOutputInfo> desktopProjectorOutputs() {
  int count = 0;
  auto **monitors = glfwGetMonitors(&count);
  std::vector<ProjectorOutputInfo> outputs;
  for (int index = 0; index < count; ++index) {
    if (monitors[index] == glfwGetPrimaryMonitor())
      continue;
    outputs.push_back({std::to_string(index), std::string(glfwGetMonitorName(monitors[index])) + " (Fullscreen window)",
                       OutputBackend::Desktop, false});
  }
  return outputs;
}

std::unique_ptr<Projector> openDesktopProjector(const ProjectorOutputInfo &output) {
  int count = 0;
  auto **monitors = glfwGetMonitors(&count);
  const auto index = std::stoul(output.id);
  if (output.backend != OutputBackend::Desktop || index >= static_cast<unsigned int>(count) ||
      monitors[index] == glfwGetPrimaryMonitor())
    throw std::runtime_error("Secondary desktop projector is no longer available");
  return std::make_unique<DesktopProjector>(monitors[index]);
}
}