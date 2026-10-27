#include "CameraFactory.h"

#include <dc1394/dc1394.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <climits>
#include <iostream>
#include <thread>

namespace slstudio {
namespace {

void check(dc1394error_t error, const char *operation) {
  if (error != DC1394_SUCCESS)
    throw CameraError(std::string("IIDC: ") + operation + ": " +
        dc1394_error_get_string(error) + " [" + std::to_string(error) + "]");
}

void logCleanup(dc1394error_t error, const char *operation) {
  if (error != DC1394_SUCCESS)
    std::cerr << "IIDC cleanup: " << operation << ": " << dc1394_error_get_string(error) << '\n';
}

using Context = std::unique_ptr<dc1394_t, decltype(&dc1394_free)>;
using CameraPtr = std::unique_ptr<dc1394camera_t, decltype(&dc1394_camera_free)>;
using CameraList = std::unique_ptr<dc1394camera_list_t, decltype(&dc1394_camera_free_list)>;

Context createContext() {
  Context context(dc1394_new(), dc1394_free);
  if (!context) throw CameraError("IIDC: cannot create libdc1394 context");
  return context;
}

CameraList enumerate(dc1394_t *context) {
  dc1394camera_list_t *list = nullptr;
  const auto error = dc1394_camera_enumerate(context, &list);
  CameraList cameras(list, dc1394_camera_free_list);
  check(error, "enumerate cameras");
  if (!cameras) throw CameraError("IIDC: enumeration returned no camera list");
  return cameras;
}

CameraPtr openDevice(dc1394_t *context, const dc1394camera_id_t &id) {
  CameraPtr camera(dc1394_camera_new_unit(context, id.guid, id.unit), dc1394_camera_free);
  if (!camera)
    throw CameraDisconnected("IIDC: cannot open camera " + std::to_string(id.guid) +
        "; check its connection and USB/FireWire access permissions");
  return camera;
}

struct Device {
  Context context = createContext();
  CameraPtr camera{nullptr, dc1394_camera_free};

  explicit Device(std::size_t index) {
    const auto cameras = enumerate(context.get());
    if (index >= cameras->num) throw CameraError("IIDC camera index is out of range");
    camera = openDevice(context.get(), cameras->ids[index]);
  }
};

CameraDescriptor iidcDescriptor(const dc1394camera_t &camera) {
  return {CameraBackend::IIDC, std::to_string(camera.guid) + ":" + std::to_string(camera.unit),
      camera.vendor ? camera.vendor : "", camera.model ? camera.model : "",
      std::to_string(camera.guid), "IIDC"};
}

dc1394feature_info_t triggerInfo(dc1394camera_t *camera) {
  dc1394feature_info_t feature{};
  feature.id = DC1394_FEATURE_TRIGGER;
  check(dc1394_feature_get(camera, &feature), "inspect trigger capabilities");
  return feature;
}

std::vector<std::string> triggerSources(const dc1394feature_info_t &feature) {
  std::vector<std::string> sources;
  if (feature.available != DC1394_TRUE) return sources;
  if (std::find(feature.trigger_modes.modes, feature.trigger_modes.modes + feature.trigger_modes.num,
      DC1394_TRIGGER_MODE_0) == feature.trigger_modes.modes + feature.trigger_modes.num)
    return sources;
  for (std::uint32_t index = 0; index < feature.trigger_sources.num; ++index) {
    const auto source = feature.trigger_sources.sources[index];
    if (source >= DC1394_TRIGGER_SOURCE_0 && source <= DC1394_TRIGGER_SOURCE_3)
      sources.push_back("Line" + std::to_string(source - DC1394_TRIGGER_SOURCE_0));
    else if (source == DC1394_TRIGGER_SOURCE_SOFTWARE)
      sources.push_back("Software");
  }
  return sources;
}

int codingPriority(dc1394color_coding_t coding) {
  switch (coding) {
  case DC1394_COLOR_CODING_MONO8: return 5;
  case DC1394_COLOR_CODING_RGB8: return 4;
  case DC1394_COLOR_CODING_RAW8: return 3;
  case DC1394_COLOR_CODING_YUV422: return 2;
  case DC1394_COLOR_CODING_YUV411: return 1;
  case DC1394_COLOR_CODING_YUV444: return 0;
  default: return -1;
  }
}

class CameraIIDC final : public Camera {
public:
  CameraIIDC(std::size_t index, const CameraSettings &requestedSettings)
      : device(index), info(iidcDescriptor(*device.camera)), settings(requestedSettings) {
    if (settings.sequenceMarker.policy == SequenceMarkerPolicy::Required)
      throw CameraError("IIDC: frame-associated sequence input metadata is unavailable");
    check(dc1394_video_set_transmission(camera(), DC1394_OFF), "stop transmission before configuration");
    trigger = triggerInfo(camera());
    const auto sources = triggerSources(trigger);
    support.softwareTrigger = std::find(sources.begin(), sources.end(), "Software") != sources.end();
    support.externalTrigger = std::any_of(sources.begin(), sources.end(),
        [](const auto &source) { return source != "Software"; });
    configureTrigger(CameraTrigger::FreeRunning);
    configureImage();
    configureExposure();
    setAbsolute(DC1394_FEATURE_GAIN, settings.gain, "gain (dB)");
    synchronizeStream();
    configureTrigger(settings.trigger);
  }

  ~CameraIIDC() override {
    try { stop(); }
    catch (const std::exception &error) { std::cerr << "IIDC camera shutdown: " << error.what() << '\n'; }
  }

  const CameraDescriptor &descriptor() const override { return info; }
  CameraCapabilities capabilities() const override { return support; }
  cv::Size resolution() const override { return size; }
  bool isCapturing() const override { return capturing; }

  void start() override {
    if (capturing) return;
    if (captureAllocated) stop();
    check(dc1394_capture_setup(camera(), static_cast<std::uint32_t>(settings.bufferCount),
        DC1394_CAPTURE_FLAGS_DEFAULT), "allocate capture buffers");
    captureAllocated = true;
    const auto error = dc1394_video_set_transmission(camera(), DC1394_ON);
    if (error != DC1394_SUCCESS) {
      logCleanup(dc1394_video_set_transmission(camera(), DC1394_OFF), "stop transmission after failed start");
      const auto cleanup = dc1394_capture_stop(camera());
      captureAllocated = cleanup != DC1394_SUCCESS;
      logCleanup(cleanup, "release capture buffers after failed start");
      check(error, "start transmission");
    }
    capturing = true;
  }

  void stop() override {
    if (!captureAllocated) return;
    const auto transmission = dc1394_video_set_transmission(camera(), DC1394_OFF);
    const auto capture = dc1394_capture_stop(camera());
    captureAllocated = capture != DC1394_SUCCESS;
    capturing = captureAllocated;
    if (transmission != DC1394_SUCCESS) {
      logCleanup(capture, "release capture buffers");
      check(transmission, "stop transmission");
    }
    check(capture, "release capture buffers");
  }

  void setTrigger(CameraTrigger requested) override {
    if (capturing || captureAllocated) throw CameraError("Stop IIDC acquisition before changing the trigger source");
    configureTrigger(requested);
    settings.trigger = requested;
  }

  void softwareTrigger() override {
    if (!capturing || settings.trigger != CameraTrigger::Software || !support.softwareTrigger)
      throw CameraError("IIDC software trigger requires active software-triggered capture");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    for (;;) {
      dc1394switch_t pending = DC1394_OFF;
      check(dc1394_software_trigger_get_power(camera(), &pending), "inspect software trigger");
      if (pending == DC1394_OFF) break;
      if (std::chrono::steady_clock::now() >= deadline)
        throw CameraError("IIDC: camera did not become ready for a software trigger within one second");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(dc1394_software_trigger_set_power(camera(), DC1394_ON), "software trigger");
  }

  CameraReadResult read(std::chrono::milliseconds timeout) override {
    validateCameraTimeout(timeout);
    if (!capturing) throw CameraError("Start IIDC acquisition before reading");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      dc1394video_frame_t *frame = nullptr;
      check(dc1394_capture_dequeue(camera(), DC1394_CAPTURE_POLICY_POLL, &frame), "dequeue image");
      if (frame) {
        struct ReturnFrame {
          dc1394camera_t *camera;
          dc1394video_frame_t *frame;
          ~ReturnFrame() { if (frame) logCleanup(dc1394_capture_enqueue(camera, frame), "return image buffer"); }
        } returned{camera(), frame};
        CameraReadResult result{CameraReadStatus::Incomplete, {}};
        if (dc1394_capture_is_frame_corrupt(camera(), frame) != DC1394_TRUE)
          result = {CameraReadStatus::Complete, cameraImageSnapshot(grayscale(*frame))};
        const auto error = dc1394_capture_enqueue(camera(), frame);
        returned.frame = nullptr;
        check(error, "return image buffer");
        return result;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        dc1394switch_t transmission = DC1394_OFF;
        check(dc1394_video_get_transmission(camera(), &transmission), "check camera after capture timeout");
        if (transmission != DC1394_ON) throw CameraDisconnected("IIDC camera stopped transmitting");
        return {};
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

private:
  Device device;
  CameraDescriptor info;
  CameraSettings settings;
  CameraCapabilities support;
  dc1394feature_info_t trigger{};
  cv::Size size;
  dc1394color_coding_t coding = DC1394_COLOR_CODING_MONO8;
  bool captureAllocated = false, capturing = false;

  dc1394camera_t *camera() const { return device.camera.get(); }

  // Point Grey USB firmware keeps its bulk data toggle when libdc1394 reconfigures the device on open,
  // while the host resets its own. The first packet of the next stream is then dropped and every image
  // is circularly shifted. Any transferred packet resynchronizes both sides, so stream one frame first.
  void synchronizeStream() {
    start();
    try {
      read(std::min<std::chrono::milliseconds>(std::chrono::minutes(1),
          std::chrono::duration_cast<std::chrono::milliseconds>(settings.exposure) + std::chrono::seconds(1)));
    } catch (...) {
      stop();
      throw;
    }
    stop();
  }


  void configureTrigger(CameraTrigger requested) {
    if (trigger.available != DC1394_TRUE) {
      if (requested != CameraTrigger::FreeRunning) throw CameraError("IIDC camera does not support triggering");
      return;
    }
    check(dc1394_external_trigger_set_power(camera(), DC1394_OFF), "disable external trigger");
    if (requested == CameraTrigger::FreeRunning) return;
    const auto sources = triggerSources(trigger);
    const auto source = requested == CameraTrigger::Software ? std::string("Software") : settings.frameTriggerSource;
    if (std::find(sources.begin(), sources.end(), source) == sources.end())
      throw CameraError("IIDC: trigger source is unavailable: " + source);
    check(dc1394_external_trigger_set_mode(camera(), DC1394_TRIGGER_MODE_0), "select frame-start trigger mode");
    const auto selected = source == "Software" ? DC1394_TRIGGER_SOURCE_SOFTWARE :
        static_cast<dc1394trigger_source_t>(DC1394_TRIGGER_SOURCE_0 + source.back() - '0');
    check(dc1394_external_trigger_set_source(camera(), selected), "select trigger source");
    if (requested == CameraTrigger::External) {
      const auto polarity = settings.risingEdge ? DC1394_TRIGGER_ACTIVE_HIGH : DC1394_TRIGGER_ACTIVE_LOW;
      if (trigger.polarity_capable == DC1394_TRUE)
        check(dc1394_external_trigger_set_polarity(camera(), polarity), "set trigger polarity");
      else if (trigger.trigger_polarity != polarity)
        throw CameraError("IIDC: requested trigger polarity is not supported");
    }
    check(dc1394_external_trigger_set_power(camera(), DC1394_ON), "enable frame-start trigger");
  }

  dc1394feature_info_t absoluteFeature(dc1394feature_t id, const char *name) {
    dc1394feature_info_t feature{};
    feature.id = id;
    check(dc1394_feature_get(camera(), &feature), name);
    if (feature.available != DC1394_TRUE || feature.absolute_capable != DC1394_TRUE)
      throw CameraError(std::string("IIDC: absolute control is unavailable for ") + name);
    return feature;
  }

  void configureExposure() {
    const double seconds = static_cast<double>(settings.exposure.count()) / 1000000.0;
    auto shutter = absoluteFeature(DC1394_FEATURE_SHUTTER, "exposure (seconds)");
    dc1394video_mode_t mode{};
    check(dc1394_video_get_mode(camera(), &mode), "query exposure video mode");
    if (seconds > shutter.abs_max && dc1394_is_video_mode_scalable(mode)) {
      // The maximum shutter scales with the Format7 transfer rate; smaller packets allow longer exposures.
      std::uint32_t unit = 0, maximum = 0, packet = 0;
      check(dc1394_format7_get_packet_parameters(camera(), mode, &unit, &maximum), "query packet size limits");
      check(dc1394_format7_get_packet_size(camera(), mode, &packet), "query packet size");
      while (unit && seconds > shutter.abs_max && packet > unit) {
        packet = std::max(unit, packet / 2 / unit * unit);
        check(dc1394_format7_set_packet_size(camera(), mode, packet), "reduce packet size for exposure");
        shutter = absoluteFeature(DC1394_FEATURE_SHUTTER, "exposure (seconds)");
      }
    }
    setAbsolute(DC1394_FEATURE_SHUTTER, seconds, "exposure (seconds)");
  }

  void setAbsolute(dc1394feature_t id, double value, const char *name) {
    const auto feature = absoluteFeature(id, name);
    if (value < feature.abs_min || value > feature.abs_max)
      throw CameraError(std::string("IIDC: ") + name + " must be within " +
          std::to_string(feature.abs_min) + ".." + std::to_string(feature.abs_max));
    if (feature.on_off_capable == DC1394_TRUE)
      check(dc1394_feature_set_power(camera(), id, DC1394_ON), name);
    check(dc1394_feature_set_mode(camera(), id, DC1394_FEATURE_MODE_MANUAL), name);
    check(dc1394_feature_set_absolute_control(camera(), id, DC1394_ON), name);
    check(dc1394_feature_set_absolute_value(camera(), id, static_cast<float>(value)), name);
  }

  void configureImage() {
    dc1394video_modes_t modes{};
    check(dc1394_video_get_supported_modes(camera(), &modes), "query video modes");
    dc1394video_mode_t chosen = DC1394_VIDEO_MODE_MIN;
    dc1394format7mode_t scalable{};
    std::uint64_t bestArea = 0;
    int bestCoding = -1;
    for (std::uint32_t index = 0; index < modes.num; ++index) {
      const auto mode = modes.modes[index];
      dc1394format7mode_t format{};
      dc1394color_codings_t codings{};
      std::uint32_t width = 0, height = 0;
      if (dc1394_is_video_mode_scalable(mode)) {
        // libdc1394 requires presence as input even for an enumerated mode.
        format.present = DC1394_TRUE;
        check(dc1394_format7_get_mode_info(camera(), mode, &format), "query Format7 mode");
        width = format.max_size_x;
        height = format.max_size_y;
        codings = format.color_codings;
      } else {
        check(dc1394_get_image_size_from_video_mode(camera(), mode, &width, &height), "query fixed image dimensions");
        codings.num = 1;
        check(dc1394_get_color_coding_from_video_mode(camera(), mode, &codings.codings[0]), "query fixed pixel format");
      }
      if (!width || !height || width > INT_MAX / 3 || height > INT_MAX) continue;
      const auto area = static_cast<std::uint64_t>(width) * height;
      for (std::uint32_t color = 0; color < codings.num; ++color) {
        const auto priority = codingPriority(codings.codings[color]);
        if (priority >= 0 && (area > bestArea || (area == bestArea && (priority > bestCoding ||
            (priority == bestCoding && dc1394_is_video_mode_scalable(mode) &&
                !dc1394_is_video_mode_scalable(chosen)))))) {
          bestArea = area;
          bestCoding = priority;
          chosen = mode;
          coding = codings.codings[color];
          scalable = format;
          size = {static_cast<int>(width), static_cast<int>(height)};
        }
      }
    }
    if (!bestArea) throw CameraError("IIDC: no supported 8-bit image mode");
    const auto &region = settings.region;
    const cv::Size requested(region.width ? region.width : size.width, region.height ? region.height : size.height);
    if (requested.width > size.width - region.x || requested.height > size.height - region.y)
      throw CameraError("IIDC: requested ROI exceeds the sensor dimensions " +
          std::to_string(size.width) + " x " + std::to_string(size.height));
    check(dc1394_video_set_mode(camera(), chosen), "select video mode");
    if (dc1394_is_video_mode_scalable(chosen)) {
      if (!scalable.unit_size_x || !scalable.unit_size_y ||
          requested.width % scalable.unit_size_x || requested.height % scalable.unit_size_y ||
          (scalable.unit_pos_x ? region.x % scalable.unit_pos_x : region.x != 0) ||
          (scalable.unit_pos_y ? region.y % scalable.unit_pos_y : region.y != 0))
        throw CameraError("IIDC: ROI does not match the camera's Format7 size/offset increments");
      check(dc1394_format7_set_roi(camera(), chosen, coding, DC1394_USE_MAX_AVAIL,
          region.x, region.y, requested.width, requested.height), "configure Format7 ROI");
    } else {
      if (region.x || region.y || requested != size)
        throw CameraError("IIDC: cropping requires a Format7 image mode");
      dc1394framerates_t rates{};
      check(dc1394_video_get_supported_framerates(camera(), chosen, &rates), "query frame rates");
      if (!rates.num) throw CameraError("IIDC: video mode has no frame rates");
      check(dc1394_video_set_framerate(camera(), *std::max_element(rates.framerates,
          rates.framerates + rates.num)), "select frame rate");
    }
    size = requested;
  }

  cv::Mat grayscale(const dc1394video_frame_t &frame) const {
    if (!frame.image || frame.size[0] != static_cast<std::uint32_t>(size.width) ||
        frame.size[1] != static_cast<std::uint32_t>(size.height) || frame.color_coding != coding)
      throw CameraError("IIDC: unexpected image dimensions or pixel format");
    std::uint32_t bits = 0;
    check(dc1394_get_color_coding_bit_size(coding, &bits), "query pixel size");
    const auto rowBits = static_cast<std::uint64_t>(size.width) * bits;
    const auto rowBytes = rowBits / 8;
    if (rowBits % 8 || !rowBytes || frame.stride < rowBytes ||
        frame.total_bytes < static_cast<std::uint64_t>(frame.stride) * (size.height - 1) + rowBytes)
      throw CameraError("IIDC: truncated image payload or invalid row stride");
    if (coding == DC1394_COLOR_CODING_MONO8)
      return cv::Mat(size.height, size.width, CV_8UC1, frame.image, frame.stride);
    if ((coding == DC1394_COLOR_CODING_YUV411 && size.width % 4) ||
        (coding == DC1394_COLOR_CODING_YUV422 && size.width % 2))
      throw CameraError("IIDC: image width is incompatible with the YUV pixel grouping");
    auto packed = cv::Mat(size.height, static_cast<int>(rowBytes), CV_8UC1, frame.image, frame.stride);
    if (!packed.isContinuous()) packed = packed.clone();
    cv::Mat rgb(size, CV_8UC3), gray;
    if (coding == DC1394_COLOR_CODING_RAW8) {
      if (size.width < 2 || size.height < 2 || size.width % 2 || size.height % 2)
        throw CameraError("IIDC: Bayer images require even dimensions of at least two rows and columns");
      check(dc1394_bayer_decoding_8bit(packed.data, rgb.data, size.width, size.height,
          frame.color_filter, DC1394_BAYER_METHOD_BILINEAR), "decode Bayer image");
    } else {
      check(dc1394_convert_to_RGB8(packed.data, rgb.data, size.width, size.height,
          frame.yuv_byte_order, coding, 8), "convert image to RGB8");
    }
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
  }
};
}

std::vector<CameraDescriptor> iidcCameraDevices() {
  auto context = createContext();
  const auto list = enumerate(context.get());
  std::vector<CameraDescriptor> devices;
  for (std::uint32_t index = 0; index < list->num; ++index) {
    const auto camera = openDevice(context.get(), list->ids[index]);
    devices.push_back(iidcDescriptor(*camera));
  }
  return devices;
}

std::vector<std::string> iidcFrameTriggerSources(std::size_t index) {
  Device device(index);
  return triggerSources(triggerInfo(device.camera.get()));
}

std::unique_ptr<Camera> openIIDCCamera(std::size_t index, const CameraSettings &settings) {
  return std::make_unique<CameraIIDC>(index, settings);
}
}
