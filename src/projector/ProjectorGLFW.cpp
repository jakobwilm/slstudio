#include "ProjectorFactory.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3native.h>
#include <CoreGraphics/CoreGraphics.h>
#import <AppKit/AppKit.h>
#endif
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <thread>

namespace slstudio {
namespace {

struct WindowDeleter {
  std::function<void(std::function<void()>)> mainThread;
  void operator()(GLFWwindow *window) const {
    if (mainThread) mainThread([window] { glfwDestroyWindow(window); });
    else glfwDestroyWindow(window);
  }
};

struct CurrentContext {
  GLFWwindow *previous = glfwGetCurrentContext();
  explicit CurrentContext(GLFWwindow *window) { glfwMakeContextCurrent(window); }
  ~CurrentContext() { glfwMakeContextCurrent(previous); }
};

class ProjectorGLFW final : public Projector {
public:
  explicit ProjectorGLFW(cv::Size windowSize,
      std::function<void(std::function<void()>)> dispatch = {}, const std::string &outputId = {})
      : mainThread(std::move(dispatch)), window(nullptr, WindowDeleter{mainThread}) {
    cv::Size nativeSize;
    GLFWmonitor *monitor = nullptr;
    onMainThread([&] {
      if (!outputId.empty()) {
        int count = 0;
        auto **monitors = glfwGetMonitors(&count);
        const auto index = std::stoul(outputId);
        if (index >= static_cast<unsigned int>(count) || monitors[index] == glfwGetPrimaryMonitor())
          throw std::runtime_error("Secondary desktop projector is no longer available");
        monitor = monitors[index];
      }
      const auto *mode = monitor ? glfwGetVideoMode(monitor) : nullptr;
      if (monitor && !mode)
        throw std::runtime_error("Cannot read projector video mode");
      size = mode ? cv::Size(mode->width, mode->height) : windowSize;
      if (size.width <= 0 || size.height <= 0)
        throw std::invalid_argument("Projector window dimensions must be positive");
      nativeSize = size;
#ifdef __APPLE__
      if (monitor) {
        CGDisplayModeRef displayMode = CGDisplayCopyDisplayMode(glfwGetCocoaMonitor(monitor));
        if (!displayMode)
          throw std::runtime_error("Cannot read projector display pixel dimensions");
        const auto width = CGDisplayModeGetPixelWidth(displayMode);
        const auto height = CGDisplayModeGetPixelHeight(displayMode);
        CGDisplayModeRelease(displayMode);
        if (!width || !height || width > std::numeric_limits<int>::max() ||
            height > std::numeric_limits<int>::max())
          throw std::runtime_error("Invalid projector display pixel dimensions");
        nativeSize = {static_cast<int>(width), static_cast<int>(height)};
      }
#endif
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
#ifdef __APPLE__
      if (monitor) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
#endif
      window.reset(glfwCreateWindow(size.width, size.height, "SLStudio projector", nullptr, nullptr));
#ifdef __APPLE__
      if (monitor) glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
#endif
      if (!window)
        throw std::runtime_error("Cannot create desktop projector window");
      glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
      glfwGetFramebufferSize(window.get(), &size.width, &size.height);
      if (size.width <= 0 || size.height <= 0)
        throw std::runtime_error("Projector window has no usable pixel framebuffer");
      if (monitor) {
#ifdef __APPLE__
        // GLFW fullscreen can replace custom modes absent from its advertised mode list.
        // Cover the display without changing its selected mode.
        int x = 0, y = 0;
        glfwGetMonitorPos(monitor, &x, &y);
        glfwSetWindowAttrib(window.get(), GLFW_DECORATED, GLFW_FALSE);
        glfwSetWindowPos(window.get(), x, y);
        glfwSetWindowSize(window.get(), mode->width, mode->height);
        NSWindow *nativeWindow = glfwGetCocoaWindow(window.get());
        nativeWindow.level = NSMainMenuWindowLevel + 1;
        nativeWindow.hidesOnDeactivate = NO;
        nativeWindow.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces |
                                          NSWindowCollectionBehaviorStationary |
                                          NSWindowCollectionBehaviorIgnoresCycle;
        glfwShowWindow(window.get());
#else
        glfwSetWindowMonitor(window.get(), monitor, 0, 0, nativeSize.width, nativeSize.height, mode->refreshRate);
#endif
      }
    });
    if (monitor) {
      // Wait on acquisition, not the GUI, while the compositor configures native pixels.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      do {
        if (mainThread)
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        else
          glfwWaitEventsTimeout(0.01);
        onMainThread([&] { glfwGetFramebufferSize(window.get(), &size.width, &size.height); });
      } while (size != nativeSize && std::chrono::steady_clock::now() < deadline);
      if (size != nativeSize)
        throw std::runtime_error("Desktop did not configure the projector at its native pixel resolution: expected " +
            std::to_string(nativeSize.width) + "x" + std::to_string(nativeSize.height) +
            ", framebuffer is " + std::to_string(size.width) + "x" + std::to_string(size.height));
    }
    // GLEW stores process-global function pointers; initialize while GUI rendering is paused.
    onMainThread([&] {
      CurrentContext context(window.get());
      glewExperimental = GL_TRUE;
      const auto loaderStatus = glewInit();
      if (!GLEW_VERSION_3_3)
        throw std::runtime_error("Cannot load projector OpenGL functions: " +
            std::string(reinterpret_cast<const char *>(glewGetErrorString(loaderStatus))));
      glGetError();
    });
    CurrentContext context(window.get());
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
  void onMainThread(std::function<void()> action) const {
    if (mainThread)
      mainThread(std::move(action));
    else
      action();
  }

  void checkOutputSize() const {
    onMainThread([&] {
      if (glfwWindowShouldClose(window.get()))
        throw std::runtime_error("Projector window was closed");
      int width = 0, height = 0;
      glfwGetFramebufferSize(window.get(), &width, &height);
      if (width != size.width || height != size.height)
        throw std::runtime_error("Projector framebuffer size changed; reopen output instead of scaling patterns");
    });
  }

  void clearTextures() {
    glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
    textures.clear();
  }
  std::function<void(std::function<void()>)> mainThread;
  std::unique_ptr<GLFWwindow, WindowDeleter> window;
  cv::Size size;
  GLuint framebuffer = 0;
  std::vector<GLuint> textures;
};
}

std::vector<ProjectorOutputInfo> glfwProjectorOutputs() {
  glfwPollEvents();
  int count = 0;
  auto **monitors = glfwGetMonitors(&count);
  std::vector<ProjectorOutputInfo> outputs;
  for (int index = 0; index < count; ++index) {
    if (monitors[index] == glfwGetPrimaryMonitor())
      continue;
    outputs.push_back({std::to_string(index), "HDMI " + std::to_string(index) + " (Fullscreen window)",
                       OutputBackend::Desktop, false, {}, glfwGetMonitorName(monitors[index])});
  }
  return outputs;
}

std::unique_ptr<Projector> openGLFWProjector(const ProjectorOutputInfo &output,
    const ProjectorOpenSettings &settings) {
  if (output.backend != OutputBackend::Desktop || output.id.empty())
    throw std::invalid_argument("Not a secondary desktop projector output");
  return std::make_unique<ProjectorGLFW>(cv::Size{}, settings.mainThread, output.id);
}

#ifdef SLSTUDIO_GUI_SMOKE_TESTS
std::unique_ptr<Projector> openWindowedGLFWProjectorForTest(cv::Size size, GLFWwindow *&window,
    std::function<void()> inspectFramebuffer) {
  auto projector = std::make_unique<ProjectorGLFW>(size);
  window = projector->testWindow();
  projector->inspectFramebuffer = std::move(inspectFramebuffer);
  return projector;
}
#endif
}