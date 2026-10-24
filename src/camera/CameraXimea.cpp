#include "CameraFactory.h"

#include <xiApi.h>
#include <algorithm>
#include <limits>

namespace slstudio {
namespace {

void xiCheck(XI_RETURN result, const char *operation) {
  if (result != XI_OK) throw CameraError(std::string("XIMEA ") + operation + " failed (" + std::to_string(result) + ")");
}

std::string xiInfo(DWORD index, const char *name) {
  char value[256]{};
  xiCheck(xiGetDeviceInfoString(index, name, value, sizeof(value)), name);
  return value;
}

std::vector<std::string> xiFrameTriggerSources(HANDLE handle) {
  int minimum = 0, maximum = 0;
  xiCheck(xiGetParamInt(handle, XI_PRM_GPI_SELECTOR XI_PRM_INFO_MIN, &minimum), "read first trigger input");
  xiCheck(xiGetParamInt(handle, XI_PRM_GPI_SELECTOR XI_PRM_INFO_MAX, &maximum), "read last trigger input");
  if (maximum == 0 && minimum == 0) return {};
  if (minimum < 1 || maximum < minimum) throw CameraError("XIMEA reported an invalid trigger input range");
  std::vector<std::string> sources;
  for (std::int64_t line = minimum; line <= maximum; ++line) sources.push_back("Line" + std::to_string(line));
  return sources;
}

class CameraXimea final : public Camera {
public:
  CameraXimea(std::size_t index, const CameraSettings &requestedSettings) : settings(requestedSettings) {
    if (index > std::numeric_limits<DWORD>::max()) throw CameraError("XIMEA camera index is out of range");
    const auto devices = ximeaDevices();
    if (index >= devices.size()) throw CameraError("XIMEA camera index is out of range");
    info = devices[index];
    xiCheck(xiOpenDevice(static_cast<DWORD>(index), &handle), "open");
    try {
      set(XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
      set(XI_PRM_BUFFERS_QUEUE_SIZE, static_cast<int>(settings.bufferCount));
      set(XI_PRM_RECENT_FRAME, 0);
      set(XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
      set(XI_PRM_OFFSET_X, 0);
      set(XI_PRM_OFFSET_Y, 0);
      if (!settings.region.width) settings.region.width = get(XI_PRM_WIDTH XI_PRM_INFO_MAX);
      if (!settings.region.height) settings.region.height = get(XI_PRM_HEIGHT XI_PRM_INFO_MAX);
      validateDimension(XI_PRM_WIDTH, settings.region.width);
      validateDimension(XI_PRM_HEIGHT, settings.region.height);
      set(XI_PRM_WIDTH, settings.region.width);
      set(XI_PRM_HEIGHT, settings.region.height);
      set(XI_PRM_OFFSET_X, settings.region.x);
      set(XI_PRM_OFFSET_Y, settings.region.y);
      size = {get(XI_PRM_WIDTH), get(XI_PRM_HEIGHT)};
      if (size != settings.region.size()) throw CameraError("XIMEA ROI mismatch");
      set(XI_PRM_AEAG, XI_OFF);
      set(XI_PRM_EXPOSURE, static_cast<int>(settings.exposure.count()));
      xiCheck(xiSetParamFloat(handle, XI_PRM_GAIN, static_cast<float>(settings.gain)), "set gain");
      if (settings.trigger == CameraTrigger::External) {
        const auto sources = xiFrameTriggerSources(handle);
        if (std::find(sources.begin(), sources.end(), settings.frameTriggerSource) == sources.end())
          throw CameraError("Requested XIMEA frame trigger source is unavailable");
        set(XI_PRM_GPI_SELECTOR, std::stoi(settings.frameTriggerSource.substr(4)));
        set(XI_PRM_GPI_MODE, XI_GPI_TRIGGER);
      }
      set(XI_PRM_TRG_SELECTOR, XI_TRG_SEL_FRAME_START);
      set(XI_PRM_TRG_SOURCE, settings.trigger == CameraTrigger::External ?
          (settings.risingEdge ? XI_TRG_EDGE_RISING : XI_TRG_EDGE_FALLING) :
          settings.trigger == CameraTrigger::Software ? XI_TRG_SOFTWARE : XI_TRG_OFF);
    } catch (...) {
      xiCloseDevice(handle);
      handle = nullptr;
      throw;
    }
  }
  ~CameraXimea() override {
    try { stop(); } catch (...) {}
    if (handle) xiCloseDevice(handle);
  }
  const CameraDescriptor &descriptor() const override { return info; }
  CameraCapabilities capabilities() const override { return {true, true, settings.sequenceMarker.policy != SequenceMarkerPolicy::Disabled}; }
  cv::Size resolution() const override { return size; }
  bool isCapturing() const override { return capturing; }
  void start() override {
    if (capturing) return;
    xiCheck(xiStartAcquisition(handle), "start acquisition");
    capturing = true;
  }
  void stop() override {
    if (!capturing) return;
    xiCheck(xiStopAcquisition(handle), "stop acquisition");
    capturing = false;
  }
  void softwareTrigger() override {
    if (!capturing || settings.trigger != CameraTrigger::Software) throw CameraError("XIMEA software trigger is not armed");
    set(XI_PRM_TRG_SOFTWARE, 0);
  }
  CameraReadResult read(std::chrono::milliseconds timeout) override {
    validateCameraTimeout(timeout);
    if (!capturing) throw CameraError("Start XIMEA acquisition before reading");
    XI_IMG image{};
    image.size = sizeof(image);
    const auto result = xiGetImage(handle, static_cast<DWORD>(timeout.count()), &image);
    if (result == XI_TIMEOUT) return {};
    xiCheck(result, "get image");
    const auto stride = static_cast<std::size_t>(image.width) + image.padding_x;
    if (!image.bp || !image.width || !image.height || image.width > INT_MAX || image.height > INT_MAX ||
        cv::Size(static_cast<int>(image.width), static_cast<int>(image.height)) != size ||
        stride > std::numeric_limits<std::size_t>::max() / image.height ||
        image.bp_size < stride * (image.height - 1) + image.width)
      return {CameraReadStatus::Incomplete, {}};
    CameraFrameMetadata metadata;
    metadata.frameId = image.nframe;
    metadata.frameIdMaximum = UINT32_MAX;
    metadata.deviceTimestamp = std::chrono::seconds(image.tsSec) + std::chrono::microseconds(image.tsUSec);
    metadata.lineStatus = image.GPI_level;
    metadata.sequenceStart = cameraSequenceMarker(metadata.lineStatus, settings.sequenceMarker);
    return {CameraReadStatus::Complete, cameraImageSnapshot(
        cv::Mat(size.height, size.width, CV_8UC1, image.bp, stride), std::move(metadata))};
  }
  static std::vector<CameraDescriptor> ximeaDevices() {
    DWORD count = 0;
    xiCheck(xiGetNumberDevices(&count), "enumerate devices");
    std::vector<CameraDescriptor> devices;
    for (DWORD index = 0; index < count; ++index)
      devices.push_back({CameraBackend::Ximea, {}, "XIMEA", xiInfo(index, XI_PRM_DEVICE_NAME), xiInfo(index, XI_PRM_DEVICE_SN), "xiAPI"});
    return devices;
  }
private:
  CameraSettings settings;
  CameraDescriptor info;
  HANDLE handle = nullptr;
  cv::Size size;
  bool capturing = false;
  void validateDimension(const char *name, int value) {
    const int minimum = get((std::string(name) + XI_PRM_INFO_MIN).c_str());
    const int maximum = get((std::string(name) + XI_PRM_INFO_MAX).c_str());
    const int increment = get((std::string(name) + XI_PRM_INFO_INCREMENT).c_str());
    if (minimum < 0 || maximum < minimum || increment <= 0)
      throw CameraError(std::string("XIMEA reported invalid limits for ") + name);
    if (value < minimum || value > maximum || (value - minimum) % increment != 0)
      throw CameraError(std::string("XIMEA ") + name + " " + std::to_string(value) +
          " is unsupported; allowed " + std::to_string(minimum) + ".." + std::to_string(maximum) +
          " in steps of " + std::to_string(increment));
  }
  void set(const char *name, int value) { xiCheck(xiSetParamInt(handle, name, value), name); }
  int get(const char *name) {
    int value = 0;
    xiCheck(xiGetParamInt(handle, name, &value), name);
    return value;
  }
};

}
std::vector<CameraDescriptor> ximeaCameraDevices() { return CameraXimea::ximeaDevices(); }
std::unique_ptr<Camera> openXimeaCamera(std::size_t index, const CameraSettings &requestedSettings) {
  return std::make_unique<CameraXimea>(index, requestedSettings);
}
std::vector<std::string> ximeaFrameTriggerSources(std::size_t index) {
  const auto devices = ximeaCameraDevices();
  if (index >= devices.size()) throw CameraError("XIMEA camera index is out of range");
  HANDLE handle = nullptr;
  xiCheck(xiOpenDevice(static_cast<DWORD>(index), &handle), "inspect trigger inputs");
  struct HandleGuard {
    HANDLE handle;
    ~HandleGuard() { xiCloseDevice(handle); }
  } guard{handle};
  return xiFrameTriggerSources(handle);
}
}