#include "CameraFactory.h"
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  try {
    require(argc == 2 && (std::string(argv[1]) == "--ximea" || std::string(argv[1]) == "--iidc"),
        "Usage: slstudio_camera_diagnostics --ximea|--iidc");
    const bool iidc = std::string(argv[1]) == "--iidc";
    const auto backend = iidc ? slstudio::CameraBackend::IIDC : slstudio::CameraBackend::Ximea;
    const std::string name = iidc ? "IIDC" : "XIMEA";
    require(slstudio::cameraBackendAvailable(backend), "Requested camera backend is unavailable");
    auto camera = slstudio::openCamera(backend, 0);
    std::cout << camera->descriptor().model << " camera resolution: " <<
        camera->resolution().width << " x " << camera->resolution().height << '\n';
    const auto nativeSize = camera->resolution();
    if (iidc) {
      std::cout << "IIDC external trigger inputs:";
      for (const auto &source : slstudio::cameraFrameTriggerSources(backend, 0)) std::cout << ' ' << source;
      std::cout << '\n';
    }
    camera.reset();
    slstudio::CameraSettings requestedSettings;
    requestedSettings.region = cv::Rect({}, nativeSize);
    camera = slstudio::openCamera(backend, 0, requestedSettings);
    if (iidc && camera->descriptor().model == "Firefly MV FMVU-03MTC") {
      camera.reset();
      const auto originalExposure = requestedSettings.exposure;
      requestedSettings.exposure = std::chrono::milliseconds(100);
      camera = slstudio::openCamera(backend, 0, requestedSettings);
      camera->start();
      camera->softwareTrigger();
      const auto frame = camera->read(std::chrono::milliseconds(2000));
      if (frame.status != slstudio::CameraReadStatus::Complete)
        std::cerr << "IIDC extended-exposure read status: " << static_cast<int>(frame.status) << '\n';
      require(frame.status == slstudio::CameraReadStatus::Complete &&
          frame.frame.image.size() == nativeSize, "IIDC extended-exposure acquisition failed");
      camera->stop();
      camera->setTrigger(slstudio::CameraTrigger::FreeRunning);
      camera->start();
      const auto freeRunning = camera->read(std::chrono::milliseconds(2000));
      require(freeRunning.status == slstudio::CameraReadStatus::Complete &&
          freeRunning.frame.image.size() == nativeSize, "IIDC extended-exposure free-running acquisition failed");
      camera->stop();
      camera.reset();
      requestedSettings.exposure = originalExposure;
      camera = slstudio::openCamera(backend, 0, requestedSettings);
      std::cout << "IIDC Firefly MV 100 ms extended-exposure software-triggered and free-running acquisition passed\n";
    }
    camera->start();
    cv::Mat retained, expected;
    const int frameCount = iidc ? requestedSettings.bufferCount + 2 : 3;
    for (int index = 0; index < frameCount; ++index) {
      camera->softwareTrigger();
      const auto frame = camera->read(std::chrono::milliseconds(2000));
      if (frame.status != slstudio::CameraReadStatus::Complete)
        std::cerr << name << " frame " << index << " read status: " << static_cast<int>(frame.status) << '\n';
      require(frame.status == slstudio::CameraReadStatus::Complete &&
          frame.frame.image.size() == camera->resolution(), "Camera did not return a complete full-resolution frame");
      if (iidc) {
        require(!frame.frame.metadata.frameId && !frame.frame.metadata.deviceTimestamp &&
            !frame.frame.metadata.sequenceStart, "IIDC fabricated unavailable frame metadata");
        if (index == 0) {
          retained = frame.frame.image;
          expected = retained.clone();
        }
      }
    }
    if (iidc) require(cv::norm(retained, expected, cv::NORM_INF) == 0, "IIDC recycled an owned image");
    camera->stop();
    std::cout << name << " software-triggered acquisition passed: " << frameCount << " frames\n";
    if (iidc) {
      camera->start();
      require(camera->read(std::chrono::milliseconds(20)).status == slstudio::CameraReadStatus::Timeout,
          "IIDC restart retained an old frame or failed to honor a bounded timeout");
      camera->stop();
      camera->setTrigger(slstudio::CameraTrigger::FreeRunning);
      camera->start();
      require(camera->read(std::chrono::milliseconds(2000)).status == slstudio::CameraReadStatus::Complete,
          "IIDC free-running preview failed");
      camera->stop();
      camera->stop();
      std::cout << "IIDC timeout, restart, free-running preview and repeated stop passed\n";
    }
    camera.reset();
    ++requestedSettings.region.height;
    bool rejected = false;
    try { slstudio::openCamera(backend, 0, requestedSettings); }
    catch (const slstudio::CameraError &error) {
      const std::string message = error.what();
      rejected = iidc ? message.find("requested ROI exceeds") != std::string::npos :
          message.find("height " + std::to_string(requestedSettings.region.height)) != std::string::npos &&
          message.find(".." + std::to_string(nativeSize.height)) != std::string::npos;
      std::cout << message << '\n';
    }
    require(rejected, "Unsupported camera height did not report the device limits");
    std::cout << name << " unsupported-resolution validation passed\n";
    if (iidc) {
      requestedSettings.region = {};
      requestedSettings.sequenceMarker.policy = slstudio::SequenceMarkerPolicy::Required;
      rejected = false;
      try { slstudio::openCamera(backend, 0, requestedSettings); }
      catch (const slstudio::CameraError &error) {
        rejected = std::string(error.what()).find("sequence input metadata is unavailable") != std::string::npos;
      }
      require(rejected, "IIDC accepted unavailable required sequence metadata");
      requestedSettings.sequenceMarker.policy = slstudio::SequenceMarkerPolicy::Preferred;
      requestedSettings.region = {0, 0, 640, 240};
      camera = slstudio::openCamera(backend, 0, requestedSettings);
      require(camera->resolution() == requestedSettings.region.size(), "IIDC ignored the requested Format7 crop");
      camera->start();
      camera->softwareTrigger();
      const auto frame = camera->read(std::chrono::milliseconds(2000));
      if (frame.status != slstudio::CameraReadStatus::Complete)
        std::cerr << "IIDC cropped read status: " << static_cast<int>(frame.status) << '\n';
      require(frame.status == slstudio::CameraReadStatus::Complete &&
          frame.frame.image.size() == requestedSettings.region.size(),
          "IIDC cropped acquisition failed");
      camera->stop();
      std::cout << "IIDC required-metadata rejection and 640 x 240 cropped acquisition passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
