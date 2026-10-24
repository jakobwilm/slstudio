#include "ApplicationController.h"
#include "GuiRuntime.h"
#ifdef SLSTUDIO_GUI_SMOKE_TESTS
#include "gui_smoke_tests.h"
#include "ros2_output_tests.h"
#endif
#include <imgui.h>
#include <GLFW/glfw3.h>
#include <iostream>

namespace slstudio::gui {
int run(bool cameraCheck, const char *screenshot) {
  return runGui(false, [&](ApplicationController &application, const GuiContext &gui) {
    if (cameraCheck) {
      application.settingsEnabled = false;
      application.geometricCalibrationDialog.open = true;
      application.startCamera();
    }
    const auto deadline = Clock::now() + std::chrono::seconds(45);
    while (!glfwWindowShouldClose(gui.window)) {
      glfwPollEvents();
      application.attempt([&] { application.tick(); }, ApplicationController::FailurePolicy::StopAcquisition);
      if (cameraCheck && Clock::now() >= deadline)
        throw std::runtime_error("Live camera check timed out: " + application.status);
      gui.prepareFrame();
      ImGui::NewFrame();
      application.draw();
      ImGui::Render();
      gui.renderDrawData();
      if (cameraCheck && application.cameraPreviewSampleCount() >= 3) {
        if (!application.cameraPreviewTexture.texture() || glGetError() != GL_NO_ERROR)
          throw std::runtime_error("Live camera frames were not rendered");
        if (screenshot) gui.saveScreenshot(screenshot);
        std::cout << "Live camera check passed: " << application.cameraPreviewFrameSize.width << " x " <<
            application.cameraPreviewFrameSize.height << "; 3 frames acquired and rendered using saved settings\n";
        break;
      }
      glfwSwapBuffers(gui.window);
    }
    return 0;
  });
}
}

int main(int argc, char **argv) {
  try {
    if (argc > 1 && std::string(argv[1]) == "--ros2-output-smoke-test") {
#ifdef SLSTUDIO_GUI_SMOKE_TESTS
      if (!slstudio::Ros2Output::available()) throw std::runtime_error("ROS 2 support was not built");
      return runRos2OutputTests();
#else
      throw std::runtime_error("ROS 2 output smoke tests require BUILD_TESTING=ON");
#endif
    }
    const bool intensityCalibrationSmoke = argc > 1 && std::string(argv[1]) == "--intensity-calibration-smoke-test";
    const bool projectorSmoke = argc > 1 && std::string(argv[1]) == "--projector-smoke-test";
    const bool packageSmoke = argc > 1 && std::string(argv[1]) == "--package-smoke-test";
    const bool cameraCheck = argc > 1 && std::string(argv[1]) == "--camera-check";
    const bool smoke = packageSmoke || projectorSmoke || intensityCalibrationSmoke || (argc > 1 && std::string(argv[1]) == "--smoke-test");
    if (argc > 1 && !smoke && !cameraCheck) {
      std::cout << "Usage: SLStudio [--smoke-test|--intensity-calibration-smoke-test|--projector-smoke-test|--package-smoke-test|--ros2-output-smoke-test|--camera-check [screenshot.png]]\n";
      return std::string(argv[1]) == "--help" ? 0 : 1;
    }
    const char *screenshot = argc > 2 ? argv[2] : nullptr;
    if (smoke) {
#ifdef SLSTUDIO_GUI_SMOKE_TESTS
      using slstudio::gui::GuiSmokeTest;
      const auto mode = packageSmoke ? GuiSmokeTest::Package :
          intensityCalibrationSmoke ? GuiSmokeTest::IntensityCalibration : projectorSmoke ? GuiSmokeTest::Projector : GuiSmokeTest::General;
      return slstudio::gui::runGuiSmokeTest(mode, screenshot);
#else
      throw std::runtime_error("GUI smoke tests require BUILD_TESTING=ON");
#endif
    }
    return slstudio::gui::run(cameraCheck, screenshot);
  } catch (const std::exception &error) {
    std::cerr << "SLStudio: " << error.what() << '\n';
    return 1;
  }
}