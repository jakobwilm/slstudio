#include "CameraFactory.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

namespace slstudio {

#ifdef SLSTUDIO_CAMERA_ARAVIS
std::vector<CameraDescriptor> aravisCameraDevices();
std::unique_ptr<Camera> openAravisCamera(std::size_t, const CameraSettings &);
std::vector<std::string> aravisFrameTriggerSources(std::size_t);
#endif
#ifdef SLSTUDIO_CAMERA_XIMEA
std::vector<CameraDescriptor> ximeaCameraDevices();
std::unique_ptr<Camera> openXimeaCamera(std::size_t, const CameraSettings &);
std::vector<std::string> ximeaFrameTriggerSources(std::size_t);
#endif
#ifdef SLSTUDIO_CAMERA_SPINNAKER
std::vector<CameraDescriptor> spinnakerCameraDevices();
std::unique_ptr<Camera> openSpinnakerCamera(std::size_t, const CameraSettings &);
std::vector<std::string> spinnakerFrameTriggerSources(std::size_t);
#endif

void validateCameraSettings(const CameraSettings &requestedSettings) {
  const auto &region = requestedSettings.region;
  if ((region.x < 0 || region.y < 0 || region.width < 0 || region.height < 0 ||
      (region.width == 0 && region.x != 0) || (region.height == 0 && region.y != 0) ||
      region.x > std::numeric_limits<int>::max() - region.width ||
      region.y > std::numeric_limits<int>::max() - region.height) ||
      requestedSettings.exposure.count() <= 0 || requestedSettings.exposure > std::chrono::seconds(60) ||
      !std::isfinite(requestedSettings.gain) || requestedSettings.bufferCount < 2 || requestedSettings.bufferCount > 64)
    throw std::invalid_argument("Invalid camera ROI, exposure, gain or buffer count");
  switch (requestedSettings.trigger) {
  case CameraTrigger::FreeRunning:
  case CameraTrigger::Software: break;
  case CameraTrigger::External:
    if (requestedSettings.frameTriggerSource.empty()) throw std::invalid_argument("Select a camera frame trigger input");
    break;
  default: throw std::invalid_argument("Unknown camera trigger mode");
  }
  switch (requestedSettings.sequenceMarker.policy) {
  case SequenceMarkerPolicy::Disabled: break;
  case SequenceMarkerPolicy::Preferred:
  case SequenceMarkerPolicy::Required:
    if (!requestedSettings.sequenceMarker.mask) throw std::invalid_argument("Sequence input mask must be nonzero");
    break;
  default: throw std::invalid_argument("Unknown sequence marker policy");
  }
}

void validateCameraTimeout(std::chrono::milliseconds timeout) {
  if (timeout.count() <= 0 || timeout > std::chrono::minutes(1))
    throw std::invalid_argument("Camera read timeout must be in (0, 60000] ms");
}

std::optional<bool> cameraSequenceMarker(std::optional<std::uint64_t> lineStatus,
                                        const SequenceMarkerSettings &requestedSettings) {
  if (requestedSettings.policy == SequenceMarkerPolicy::Disabled) return std::nullopt;
  if (!lineStatus) {
    if (requestedSettings.policy == SequenceMarkerPolicy::Required)
      throw CameraError("Required per-frame sequence input metadata is missing");
    return std::nullopt;
  }
  if (!requestedSettings.mask) throw std::invalid_argument("Sequence input mask must be nonzero");
  return ((*lineStatus & requestedSettings.mask) != 0) == requestedSettings.activeHigh;
}

CameraImage cameraImageSnapshot(const cv::Mat &image, CameraFrameMetadata metadata) {
  if (image.empty() || image.dims != 2 || image.type() != CV_8UC1)
    throw CameraError("Camera must provide a two-dimensional Mono8 image");
  if (metadata.hostTimestamp == std::chrono::steady_clock::time_point{})
    metadata.hostTimestamp = std::chrono::steady_clock::now();
  return {cv::Mat1b(image.clone()), std::move(metadata)};
}

std::vector<CameraDescriptor> cameraDevices(CameraBackend backend) {
  switch (backend) {
  case CameraBackend::Aravis:
#ifdef SLSTUDIO_CAMERA_ARAVIS
    return aravisCameraDevices();
#else
    break;
#endif
  case CameraBackend::Ximea:
#ifdef SLSTUDIO_CAMERA_XIMEA
    return ximeaCameraDevices();
#else
    break;
#endif
  case CameraBackend::Spinnaker:
#ifdef SLSTUDIO_CAMERA_SPINNAKER
    return spinnakerCameraDevices();
#else
    break;
#endif
  }
  return {};
}

bool cameraBackendAvailable(CameraBackend backend) {
  switch (backend) {
  case CameraBackend::Aravis:
#ifdef SLSTUDIO_CAMERA_ARAVIS
    return true;
#else
    break;
#endif
  case CameraBackend::Ximea:
#ifdef SLSTUDIO_CAMERA_XIMEA
    return true;
#else
    break;
#endif
  case CameraBackend::Spinnaker:
#ifdef SLSTUDIO_CAMERA_SPINNAKER
    return true;
#else
    break;
#endif
  }
  return false;
}

std::vector<std::string> cameraFrameTriggerSources(CameraBackend backend, std::size_t deviceIndex) {
  static_cast<void>(deviceIndex);
  std::vector<std::string> sources;
  switch (backend) {
  case CameraBackend::Aravis:
#ifdef SLSTUDIO_CAMERA_ARAVIS
    sources = aravisFrameTriggerSources(deviceIndex);
    break;
#else
    throw CameraError("Aravis backend is unavailable in this build");
#endif
  case CameraBackend::Ximea:
#ifdef SLSTUDIO_CAMERA_XIMEA
    sources = ximeaFrameTriggerSources(deviceIndex);
    break;
#else
    throw CameraError("XIMEA backend is unavailable in this build");
#endif
  case CameraBackend::Spinnaker:
#ifdef SLSTUDIO_CAMERA_SPINNAKER
    sources = spinnakerFrameTriggerSources(deviceIndex);
    break;
#else
    throw CameraError("Spinnaker backend is unavailable in this build");
#endif
  default:
    throw CameraError("Unknown camera backend");
  }
  sources.erase(std::remove(sources.begin(), sources.end(), "Software"), sources.end());
  return sources;
}

std::unique_ptr<Camera> openCamera(CameraBackend backend, std::size_t deviceIndex, const CameraSettings &requestedSettings) {
  validateCameraSettings(requestedSettings);
  static_cast<void>(deviceIndex);
  switch (backend) {
  case CameraBackend::Aravis:
#ifdef SLSTUDIO_CAMERA_ARAVIS
    return openAravisCamera(deviceIndex, requestedSettings);
#else
    break;
#endif
  case CameraBackend::Ximea:
#ifdef SLSTUDIO_CAMERA_XIMEA
    return openXimeaCamera(deviceIndex, requestedSettings);
#else
    break;
#endif
  case CameraBackend::Spinnaker:
#ifdef SLSTUDIO_CAMERA_SPINNAKER
    return openSpinnakerCamera(deviceIndex, requestedSettings);
#else
    break;
#endif
  }
  throw CameraError("Requested camera backend is unavailable in this build");
}

}