#include "CameraFactory.h"

#include <algorithm>
#include <iostream>
#include <limits>
#ifdef SLSTUDIO_TEST_ARAVIS
#include <arv.h>
#endif

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  try {
    if (argc > 1 && std::string(argv[1]) == "--sdk") {
      for (const auto backend : {slstudio::CameraBackend::Ximea, slstudio::CameraBackend::Spinnaker,
          slstudio::CameraBackend::IIDC}) {
        if (!slstudio::cameraBackendAvailable(backend)) {
          require(slstudio::cameraDevices(backend).empty(), "Unavailable backend discovered devices");
          bool rejected = false;
          try { slstudio::openCamera(backend, 0); }
          catch (const slstudio::CameraError &) { rejected = true; }
          require(rejected, "Unavailable camera backend accepted an open request");
          rejected = false;
          try { slstudio::cameraFrameTriggerSources(backend, 0); }
          catch (const slstudio::CameraError &) { rejected = true; }
          require(rejected, "Unavailable camera backend accepted a trigger-source query");
          continue;
        }
        const auto devices = slstudio::cameraDevices(backend);
        for (const auto &device : devices) require(device.backend == backend, "Vendor discovery returned another backend");
        bool rejected = false;
        try { slstudio::openCamera(backend, std::numeric_limits<std::size_t>::max()); }
        catch (const slstudio::CameraError &) { rejected = true; }
        require(rejected, "Vendor camera accepted an invalid device index");
        rejected = false;
        try { slstudio::cameraFrameTriggerSources(backend, std::numeric_limits<std::size_t>::max()); }
        catch (const slstudio::CameraError &) { rejected = true; }
        require(rejected, "Vendor trigger-source query accepted an invalid device index");
        std::cout << (backend == slstudio::CameraBackend::Ximea ? "XIMEA" :
            backend == slstudio::CameraBackend::IIDC ? "IIDC" : "Spinnaker")
                  << " discovery passed: " << devices.size() << " cameras\n";
      }
      return 0;
    }
#ifdef SLSTUDIO_TEST_ARAVIS
  if (argc > 1 && std::string(argv[1]) == "--aravis") {
    arv_enable_interface("Fake");
    auto devices = slstudio::cameraDevices();
    const auto found = std::find_if(devices.begin(), devices.end(), [](const auto &device) {
    return device.id == "Fake_1";
    });
    require(found != devices.end() && found->backend == slstudio::CameraBackend::Aravis,
      "Aravis fake device was not discovered");
    slstudio::CameraSettings requestedSettings;
    requestedSettings.region = {0, 0, 64, 48};
    requestedSettings.exposure = std::chrono::microseconds(1000);
    const auto deviceIndex = static_cast<std::size_t>(std::distance(devices.begin(), found));
    const auto sources = slstudio::cameraFrameTriggerSources(slstudio::CameraBackend::Aravis, deviceIndex);
    require(sources == std::vector<std::string>{"Line0"}, "Aravis trigger sources did not match available FrameStart inputs");
    auto camera = slstudio::openCamera(slstudio::CameraBackend::Aravis, deviceIndex, requestedSettings);
    require(camera->resolution() == cv::Size(64, 48) && camera->capabilities().softwareTrigger &&
      !camera->capabilities().sequenceStart, "Aravis capabilities or ROI mismatch");
    camera->start();
    require(camera->isCapturing(), "Aravis did not start");
      bool timedOut = false;
      for (int attempt = 0; attempt < 32 && !timedOut; ++attempt)
        timedOut = camera->read(std::chrono::milliseconds(1)).status == slstudio::CameraReadStatus::Timeout;
      require(timedOut, "Aravis did not distinguish a bounded read timeout");
    camera->softwareTrigger();
    auto result = camera->read(std::chrono::milliseconds(1000));
    require(result.status == slstudio::CameraReadStatus::Complete && result.frame.image.size() == cv::Size(64, 48) &&
      result.frame.metadata.frameId && result.frame.metadata.deviceTimestamp && !result.frame.metadata.sequenceStart,
      "Aravis frame ownership or metadata mismatch");
    const auto snapshot = result.frame.image.clone();
    for (int index = 0; index < 12; ++index) {
    camera->softwareTrigger();
    auto next = camera->read(std::chrono::milliseconds(1000));
    require(next.status == slstudio::CameraReadStatus::Complete, "Aravis software capture failed");
    }
    camera->stop();
    camera->stop();
    require(!camera->isCapturing() && cv::countNonZero(snapshot != result.frame.image) == 0,
      "Aravis recycled a returned image or failed to stop");
    camera->start();
    camera->softwareTrigger();
    require(camera->read(std::chrono::milliseconds(1000)).status == slstudio::CameraReadStatus::Complete,
      "Aravis restart failed");
    camera.reset();
    auto fullCamera = slstudio::openCamera(slstudio::CameraBackend::Aravis, deviceIndex);
    require(fullCamera->resolution() == cv::Size(ARV_FAKE_CAMERA_SENSOR_WIDTH, ARV_FAKE_CAMERA_SENSOR_HEIGHT),
        "Empty Aravis ROI did not restore full sensor dimensions");
    fullCamera.reset();
    requestedSettings.region = {0, 0, 0, 48};
    auto widthReset = slstudio::openCamera(slstudio::CameraBackend::Aravis, deviceIndex, requestedSettings);
    require(widthReset->resolution() == cv::Size(ARV_FAKE_CAMERA_SENSOR_WIDTH, 48),
        "Resetting crop width changed crop height");
    widthReset.reset();
    requestedSettings.region = {0, 0, 64, 0};
    auto heightReset = slstudio::openCamera(slstudio::CameraBackend::Aravis, deviceIndex, requestedSettings);
    require(heightReset->resolution() == cv::Size(64, ARV_FAKE_CAMERA_SENSOR_HEIGHT),
        "Resetting crop height changed crop width");
    heightReset.reset();
    requestedSettings.region = {0, 0, 64, 48};
    requestedSettings.sequenceMarker.policy = slstudio::SequenceMarkerPolicy::Required;
    bool rejected = false;
    try { slstudio::openCamera(slstudio::CameraBackend::Aravis, deviceIndex, requestedSettings); }
    catch (const slstudio::CameraError &) { rejected = true; }
    require(rejected, "Aravis accepted unavailable required chunk metadata");
    arv_shutdown();
    std::cout << "Aravis fake-camera discovery, acquisition, ownership, timeout, restart and marker checks passed\n";
    return 0;
  }
#endif
    require(argc == 1, "Unknown camera test argument");
    slstudio::CameraSettings requestedSettings;
    slstudio::validateCameraSettings(requestedSettings);
    const auto &marker = requestedSettings.sequenceMarker;
    require(!slstudio::cameraSequenceMarker(std::nullopt, marker), "Unavailable input metadata became a false marker");
    require(slstudio::cameraSequenceMarker(0, marker) == false, "An unasserted sequence input is not unavailable");
    require(slstudio::cameraSequenceMarker(1, marker) == true, "Sequence-start input was lost");
    requestedSettings.sequenceMarker.activeHigh = false;
    require(slstudio::cameraSequenceMarker(0, marker) == true, "Active-low sequence input failed");
    requestedSettings.sequenceMarker.policy = slstudio::SequenceMarkerPolicy::Disabled;
    require(!slstudio::cameraSequenceMarker(1, marker), "Disabled sequence metadata was used");
    requestedSettings.sequenceMarker.policy = slstudio::SequenceMarkerPolicy::Required;
    bool rejected = false;
    try { slstudio::cameraSequenceMarker(std::nullopt, marker); }
    catch (const slstudio::CameraError &) { rejected = true; }
    require(rejected, "Missing required sequence metadata was accepted");
    cv::Mat1b borrowed(4, 7, static_cast<unsigned char>(42));
    const auto region = borrowed(cv::Rect(1, 1, 3, 2));
    slstudio::CameraFrameMetadata metadata;
    metadata.frameId = std::numeric_limits<std::uint64_t>::max();
    auto frame = slstudio::cameraImageSnapshot(region, metadata);
    borrowed.setTo(0);
    require(frame.image(0, 0) == 42 && frame.image.isContinuous() &&
        frame.metadata.frameId == metadata.frameId &&
        frame.metadata.hostTimestamp != std::chrono::steady_clock::time_point{},
        "Camera snapshots lost ownership, stride handling or metadata");
    rejected = false;
    try { slstudio::cameraImageSnapshot(cv::Mat(2, 3, CV_32F)); }
    catch (const slstudio::CameraError &) { rejected = true; }
    require(rejected, "Camera silently converted float pixels");
    requestedSettings.region = {0, 0, -1, 10};
    rejected = false;
    try { slstudio::validateCameraSettings(requestedSettings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Invalid camera ROI accepted");
    rejected = false;
    try { slstudio::openCamera(static_cast<slstudio::CameraBackend>(99), 0); }
    catch (const slstudio::CameraError &) { rejected = true; }
    require(rejected, "Unknown backend accepted");
    rejected = false;
    try { slstudio::cameraFrameTriggerSources(static_cast<slstudio::CameraBackend>(99), 0); }
    catch (const slstudio::CameraError &) { rejected = true; }
    require(rejected, "Unknown trigger-source backend accepted");
    rejected = false;
    try { slstudio::validateCameraTimeout(std::chrono::milliseconds(0)); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Unbounded camera timeout accepted");
    std::cout << "Camera ownership, optional sequence metadata, settings and factory contracts passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}