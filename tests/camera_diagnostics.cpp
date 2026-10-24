#include "CameraFactory.h"
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  try {
    require(argc == 2 && std::string(argv[1]) == "--ximea", "Usage: slstudio_camera_diagnostics --ximea");
    require(slstudio::cameraBackendAvailable(slstudio::CameraBackend::Ximea), "XIMEA backend is unavailable");
    auto camera = slstudio::openCamera(slstudio::CameraBackend::Ximea, 0);
    std::cout << camera->descriptor().model << " camera resolution: " <<
        camera->resolution().width << " x " << camera->resolution().height << '\n';
    const auto nativeSize = camera->resolution();
    camera.reset();
    slstudio::CameraSettings requestedSettings;
    requestedSettings.region = cv::Rect({}, nativeSize);
    camera = slstudio::openCamera(slstudio::CameraBackend::Ximea, 0, requestedSettings);
    camera->start();
    for (int index = 0; index < 3; ++index) {
      camera->softwareTrigger();
      const auto frame = camera->read(std::chrono::milliseconds(2000));
      require(frame.status == slstudio::CameraReadStatus::Complete &&
          frame.frame.image.size() == camera->resolution(), "XIMEA did not return a complete full-resolution frame");
    }
    camera->stop();
    std::cout << "XIMEA software-triggered acquisition passed: 3 frames\n";
    camera.reset();
    ++requestedSettings.region.height;
    bool rejected = false;
    try { slstudio::openCamera(slstudio::CameraBackend::Ximea, 0, requestedSettings); }
    catch (const slstudio::CameraError &error) {
      const std::string message = error.what();
      rejected = message.find("height " + std::to_string(requestedSettings.region.height)) != std::string::npos &&
          message.find(".." + std::to_string(nativeSize.height)) != std::string::npos;
      std::cout << message << '\n';
    }
    require(rejected, "Unsupported XIMEA height did not report the device limits");
    std::cout << "XIMEA unsupported-resolution validation passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
