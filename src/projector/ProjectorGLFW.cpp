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

struct CurrentContext {
  GLFWwindow *previous = glfwGetCurrentContext();
  explicit CurrentContext(GLFWwindow *window) { glfwMakeContextCurrent(window); }
  ~CurrentContext() { glfwMakeContextCurrent(previous); }
};

class ProjectorGLFW final : public Projector {
public:
  explicit ProjectorGLFW(GLFWmonitor *monitor, cv::Size windowSize = {}) {
    const auto *mode = monitor ? glfwGetVideoMode(monitor) : nullptr;
    if (monitor && !mode)
      throw std::runtime_error("Cannot read projector video mode");
    size = mode ? cv::Size(mode->width, mode->height) : windowSize;
    if (size.width <= 0 || size.height <= 0)
      throw std::invalid_argument("Projector window dimensions must be positive");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
    glfwWindowHint(GLFW_RED_BITS, 8);
    glfwWindowHint(GLFW_GREEN_BITS, 8);
    glfwWindowHint(GLFW_BLUE_BITS, 8);
    glfwWindowHint(GLFW_ALPHA_BITS, 8);
    glfwWindowHint(GLFW_DEPTH_BITS, 0);
    glfwWindowHint(GLFW_STENCIL_BITS, 0);
    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_REFRESH_RATE, mode ? mode->refreshRate : GLFW_DONT_CARE);
    window.reset(glfwCreateWindow(size.width, size.height, "SLStudio projector", monitor, nullptr));
    if (!window)
      throw std::runtime_error("Cannot create desktop projector window");
    glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
    CurrentContext context(window.get());
    glewExperimental = GL_TRUE;
    const auto loaderStatus = glewInit();
    if (!GLEW_VERSION_3_3)
      throw std::runtime_error("Cannot load projector OpenGL functions: " +
          std::string(reinterpret_cast<const char *>(glewGetErrorString(loaderStatus))));
    glGetError();
    glfwGetFramebufferSize(window.get(), &size.width, &size.height);
    if (size.width <= 0 || size.height <= 0)
      throw std::runtime_error("Projector window has no usable pixel framebuffer");
    GLint red = 0, green = 0, blue = 0, samples = 0;
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_BACK_LEFT,
        GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE, &red);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_BACK_LEFT,
        GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE, &green);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_BACK_LEFT,
        GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE, &blue);
    glGetIntegerv(GL_SAMPLES, &samples);
    if (red != 8 || green != 8 || blue != 8 || samples != 0)
      throw std::runtime_error("Projector requires an RGB8 framebuffer without multisampling");
    glfwSwapInterval(1);
    glGenFramebuffers(1, &framebuffer);
    blank();
  }

  ~ProjectorGLFW() override {
    CurrentContext context(window.get());
    clearTextures();
    glDeleteFramebuffers(1, &framebuffer);
    if (context.previous == window.get()) context.previous = nullptr;
  }

  cv::Size resolution() const override { return size; }

#ifdef SLSTUDIO_GUI_SMOKE_TESTS
  GLFWwindow *testWindow() const { return window.get(); }
  std::function<void()> inspectFramebuffer;
#endif

  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    for (const auto &pattern : patterns)
      if (pattern.size() != size || (pattern.type() != CV_8UC3 && pattern.type() != CV_8UC1))
        throw std::invalid_argument("Projector patterns must match output pixels");
    CurrentContext context(window.get());
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
      glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
      glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
          GL_TEXTURE_2D, textures[index], 0);
      if (glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("Cannot prepare projector pattern framebuffer");
    }
    glFinish();
  }

  void present(std::size_t index) override {
    if (index >= textures.size())
      throw std::out_of_range("Projector pattern index");
    checkOutputSize();
    CurrentContext context(window.get());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, textures[index], 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_DITHER);
    glDisable(GL_SCISSOR_TEST);
    glBlitFramebuffer(0, size.height, size.width, 0, 0, 0, size.width, size.height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
#ifdef SLSTUDIO_GUI_SMOKE_TESTS
    if (inspectFramebuffer) inspectFramebuffer();
#endif
    glfwSwapBuffers(window.get());
  }

  void blank(bool white = false) override {
    checkOutputSize();
    CurrentContext context(window.get());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_DITHER);
    glDisable(GL_SCISSOR_TEST);
    const float value = white ? 1.0f : 0.0f;
    glClearColor(value, value, value, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window.get());
  }

private:
  void checkOutputSize() const {
    if (glfwWindowShouldClose(window.get()))
      throw std::runtime_error("Projector window was closed");
    int width = 0, height = 0;
    glfwGetFramebufferSize(window.get(), &width, &height);
    if (width != size.width || height != size.height)
      throw std::runtime_error("Projector framebuffer size changed; reopen output instead of scaling patterns");
  }

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

std::vector<ProjectorOutputInfo> glfwProjectorOutputs() {
  int count = 0;
  auto **monitors = glfwGetMonitors(&count);
  std::vector<ProjectorOutputInfo> outputs;
  for (int index = 0; index < count; ++index) {
    if (monitors[index] == glfwGetPrimaryMonitor())
      continue;
    outputs.push_back({std::to_string(index), std::string(glfwGetMonitorName(monitors[index])) + " (Fullscreen window) (not exclusive)",
                       OutputBackend::Desktop, false});
  }
  return outputs;
}

std::unique_ptr<Projector> openGLFWProjector(const ProjectorOutputInfo &output) {
  int count = 0;
  auto **monitors = glfwGetMonitors(&count);
  const auto index = std::stoul(output.id);
  if (output.backend != OutputBackend::Desktop || index >= static_cast<unsigned int>(count) ||
      monitors[index] == glfwGetPrimaryMonitor())
    throw std::runtime_error("Secondary desktop projector is no longer available");
  return std::make_unique<ProjectorGLFW>(monitors[index]);
}

#ifdef SLSTUDIO_GUI_SMOKE_TESTS
std::unique_ptr<Projector> openWindowedGLFWProjectorForTest(cv::Size size, GLFWwindow *&window,
    std::function<void()> inspectFramebuffer) {
  auto projector = std::make_unique<ProjectorGLFW>(nullptr, size);
  window = projector->testWindow();
  projector->inspectFramebuffer = std::move(inspectFramebuffer);
  return projector;
}
#endif
}