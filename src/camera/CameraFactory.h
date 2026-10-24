#pragma once

#include "Camera.h"
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace slstudio {

enum class CameraBackend { Aravis, Ximea, Spinnaker };
enum class CameraTrigger { FreeRunning, Software, External };
enum class SequenceMarkerPolicy { Disabled, Preferred, Required };

struct CameraDescriptor {
  CameraBackend backend = CameraBackend::Aravis;
  std::string id, vendor, model, serial, transport;
};

struct SequenceMarkerSettings {
  SequenceMarkerPolicy policy = SequenceMarkerPolicy::Preferred;
  std::string inputLine;
  std::string chunkSelector = "LineStatusAll";
  std::string chunkFeature = "ChunkLineStatusAll";
  std::uint64_t mask = 1;
  bool activeHigh = true;
};

struct CameraSettings {
  // A zero width or height restores that sensor dimension at zero offset.
  cv::Rect region;
  std::chrono::microseconds exposure{16667};
  double gain = 0;
  CameraTrigger trigger = CameraTrigger::Software;
  std::string frameTriggerSource = "Line1";
  bool risingEdge = true;
  std::size_t bufferCount = 8;
  SequenceMarkerSettings sequenceMarker;
};

void validateCameraSettings(const CameraSettings &requestedSettings);
void validateCameraTimeout(std::chrono::milliseconds timeout);
std::optional<bool> cameraSequenceMarker(std::optional<std::uint64_t> lineStatus,
                                         const SequenceMarkerSettings &requestedSettings);
std::vector<CameraDescriptor> cameraDevices(CameraBackend backend = CameraBackend::Aravis);
// External FrameStart sources; software triggering is configured separately.
std::vector<std::string> cameraFrameTriggerSources(CameraBackend backend, std::size_t deviceIndex);
bool cameraBackendAvailable(CameraBackend backend);
std::unique_ptr<Camera> openCamera(CameraBackend backend, std::size_t deviceIndex,
                                   const CameraSettings &requestedSettings = {});

}
