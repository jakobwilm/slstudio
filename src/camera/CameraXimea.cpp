#include "CameraFactory.h"

#include <xiApi.h>
#include <algorithm>
#include <limits>
#include <vector>

namespace slstudio {
namespace {

const char *xiErrorDescription(XI_RETURN result) {
  switch (result) {
  case XI_INVALID_HANDLE: return "invalid device handle";
  case XI_READREG: return "device register read failed";
  case XI_WRITEREG: return "device register write failed";
  case XI_FREE_RESOURCES: return "could not release device resources";
  case XI_FREE_CHANNEL: return "could not release the transfer channel";
  case XI_FREE_BANDWIDTH: return "could not release reserved bandwidth";
  case XI_READBLK: return "device block read failed";
  case XI_WRITEBLK: return "device block write failed";
  case XI_NO_IMAGE: return "no image available";
  case XI_TIMEOUT: return "timed out waiting for an image";
  case XI_INVALID_ARG: return "invalid argument";
  case XI_NOT_SUPPORTED: return "operation is not supported";
  case XI_ISOCH_ATTACH_BUFFERS: return "could not attach transfer buffers";
  case XI_GET_OVERLAPPED_RESULT: return "asynchronous transfer failed";
  case XI_MEMORY_ALLOCATION: return "memory allocation failed";
  case XI_DLLCONTEXTISNULL: return "SDK context is missing";
  case XI_DLLCONTEXTISNONZERO: return "SDK context is unexpectedly initialized";
  case XI_DLLCONTEXTEXIST: return "SDK context already exists";
  case XI_TOOMANYDEVICES: return "too many connected devices";
  case XI_ERRORCAMCONTEXT: return "invalid camera context";
  case XI_UNKNOWN_HARDWARE: return "unrecognized camera hardware";
  case XI_INVALID_TM_FILE: return "invalid camera configuration file";
  case XI_INVALID_TM_TAG: return "invalid camera configuration tag";
  case XI_INCOMPLETE_TM: return "incomplete camera configuration";
  case XI_BUS_RESET_FAILED: return "device bus reset failed";
  case XI_NOT_IMPLEMENTED: return "operation is not implemented";
  case XI_SHADING_TOOBRIGHT: return "shading calibration image is too bright";
  case XI_SHADING_TOODARK: return "shading calibration image is too dark";
  case XI_TOO_LOW_GAIN: return "gain is too low";
  case XI_INVALID_BPL: return "invalid sensor defect correction list";
  case XI_BPL_REALLOC: return "could not resize the sensor defect correction list";
  case XI_INVALID_PIXEL_LIST: return "invalid pixel correction list";
  case XI_INVALID_FFS: return "invalid camera flash filesystem";
  case XI_INVALID_PROFILE: return "invalid camera profile";
  case XI_INVALID_CALIBRATION: return "invalid camera calibration";
  case XI_INVALID_BUFFER: return "invalid image buffer";
  case XI_INVALID_DATA: return "invalid image data";
  case XI_TGBUSY: return "camera timing generator is busy";
  case XI_IO_WRONG: return "invalid device I/O operation";
  case XI_ACQUISITION_ALREADY_UP: return "acquisition is already running";
  case XI_OLD_DRIVER_VERSION: return "installed device driver is too old";
  case XI_GET_LAST_ERROR: return "operating-system error; consult the SDK diagnostics";
  case XI_CANT_PROCESS: return "image data could not be processed";
  case XI_ACQUISITION_STOPED: return "acquisition is stopped";
  case XI_ACQUISITION_STOPED_WERR: return "acquisition stopped because of an error";
  case XI_INVALID_INPUT_ICC_PROFILE: return "invalid input color profile";
  case XI_INVALID_OUTPUT_ICC_PROFILE: return "invalid output color profile";
  case XI_DEVICE_NOT_READY: return "camera is not ready";
  case XI_SHADING_TOOCONTRAST: return "shading calibration has excessive contrast";
  case XI_ALREADY_INITIALIZED: return "SDK module is already initialized";
  case XI_NOT_ENOUGH_PRIVILEGES: return "insufficient permission to access the camera";
  case XI_NOT_COMPATIBLE_DRIVER: return "installed device driver is incompatible";
  case XI_TM_INVALID_RESOURCE: return "camera configuration resource could not be loaded";
  case XI_DEVICE_HAS_BEEN_RESETED: return "camera was reset; reopen the device";
  case XI_NO_DEVICES_FOUND: return "no cameras found";
  case XI_RESOURCE_OR_FUNCTION_LOCKED: return "device or function is locked";
  case XI_BUFFER_SIZE_TOO_SMALL: return "supplied image buffer is too small";
  case XI_COULDNT_INIT_PROCESSOR: return "image processor initialization failed";
  case XI_NOT_INITIALIZED: return "SDK module is not initialized";
  case XI_RESOURCE_NOT_FOUND: return "required resource was not found";
  case XI_UNKNOWN_PARAM: return "unknown parameter";
  case XI_WRONG_PARAM_VALUE: return "invalid parameter value";
  case XI_WRONG_PARAM_TYPE: return "incorrect parameter type";
  case XI_WRONG_PARAM_SIZE: return "incorrect parameter size";
  case XI_BUFFER_TOO_SMALL: return "parameter buffer is too small";
  case XI_NOT_SUPPORTED_PARAM: return "parameter is not supported by this camera";
  case XI_NOT_SUPPORTED_PARAM_INFO: return "requested parameter information is unavailable";
  case XI_NOT_SUPPORTED_DATA_FORMAT: return "image data format is not supported";
  case XI_READ_ONLY_PARAM: return "parameter is read-only";
  case XI_BANDWIDTH_NOT_SUPPORTED: return "available transfer bandwidth is insufficient";
  case XI_INVALID_FFS_FILE_NAME: return "invalid camera flash filename";
  case XI_FFS_FILE_NOT_FOUND: return "camera flash file was not found";
  case XI_PARAM_NOT_SETTABLE: return "parameter cannot be set to the requested value";
  case XI_SAFE_POLICY_NOT_SUPPORTED: return "safe image buffering is not supported";
  case XI_GPUDIRECT_NOT_AVAILABLE: return "GPUDirect image transfer is unavailable";
  case XI_INCORRECT_SENS_ID_CHECK: return "sensor identifier checksum is invalid";
  case XI_INCORRECT_FPGA_TYPE: return "camera FPGA firmware is unsupported";
  case XI_PARAM_CONDITIONALLY_NOT_AVAILABLE: return "parameter is unavailable with the current camera settings";
  case XI_ERR_FRAME_BUFFER_RAM_INIT: return "camera frame-buffer memory initialization failed";
  case XI_NOT_SUPPORTED_WHILE_ACQUISITION: return "stop acquisition before changing this parameter";
  case XI_PROC_OTHER_ERROR: return "image processing failed";
  case XI_PROC_PROCESSING_ERROR: return "image processing failed";
  case XI_PROC_INPUT_FORMAT_UNSUPPORTED: return "image processor does not support the input format";
  case XI_PROC_OUTPUT_FORMAT_UNSUPPORTED: return "image processor does not support the output format";
  case XI_OUT_OF_RANGE: return "parameter value is out of range";
  default: return "unrecognized SDK error";
  }
}

void xiCheck(XI_RETURN result, const char *operation) {
  if (result != XI_OK) throw CameraError(std::string("XIMEA ") + operation + " failed: " +
      xiErrorDescription(result) + " (code " + std::to_string(result) + ")");
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
      // Expected image timeouts are not failures; checked calls report other errors below.
      set(XI_PRM_DEBUG_LEVEL, XI_DL_FATAL);
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
      configureTrigger(settings.trigger);
      const int payloadSize = get(XI_PRM_IMAGE_PAYLOAD_SIZE);
      if (payloadSize <= 0) throw CameraError("XIMEA reported an invalid image payload size");
      imageBuffer.resize(static_cast<std::size_t>(payloadSize));
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
  void setTrigger(CameraTrigger trigger) override {
    if (capturing) throw CameraError("Stop XIMEA acquisition before changing the trigger source");
    configureTrigger(trigger);
    settings.trigger = trigger;
  }
  CameraReadResult read(std::chrono::milliseconds timeout) override {
    validateCameraTimeout(timeout);
    if (!capturing) throw CameraError("Start XIMEA acquisition before reading");
    XI_IMG image{};
    image.size = sizeof(image);
    // Safe mode must reuse caller-owned storage instead of allocating on every read.
    image.bp = imageBuffer.data();
    image.bp_size = static_cast<DWORD>(imageBuffer.size());
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
  std::vector<unsigned char> imageBuffer;
  cv::Size size;
  bool capturing = false;
  void configureTrigger(CameraTrigger trigger) {
    if (trigger == CameraTrigger::External) {
      const auto sources = xiFrameTriggerSources(handle);
      if (std::find(sources.begin(), sources.end(), settings.frameTriggerSource) == sources.end())
        throw CameraError("Requested XIMEA frame trigger source is unavailable");
      set(XI_PRM_GPI_SELECTOR, std::stoi(settings.frameTriggerSource.substr(4)));
      set(XI_PRM_GPI_MODE, XI_GPI_TRIGGER);
    }
    set(XI_PRM_TRG_SELECTOR, XI_TRG_SEL_FRAME_START);
    set(XI_PRM_TRG_SOURCE, trigger == CameraTrigger::External ?
        (settings.risingEdge ? XI_TRG_EDGE_RISING : XI_TRG_EDGE_FALLING) :
        trigger == CameraTrigger::Software ? XI_TRG_SOFTWARE : XI_TRG_OFF);
  }
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