#include "CameraFactory.h"

#include <arv.h>
#include <atomic>
#include <limits>
#include <mutex>
#include <type_traits>
#include <utility>

namespace slstudio {
namespace {

struct ObjectRelease {
  template <typename Object> void operator()(Object *object) const { g_object_unref(object); }
};
template <typename Object> using ObjectPtr = std::unique_ptr<Object, ObjectRelease>;

template <typename Function> auto checked(const char *operation, Function function) {
  GError *error = nullptr;
  using Result = decltype(function(&error));
  auto check = [&] {
    if (!error) return;
    std::string message = std::string(operation) + ": " + error->message;
    g_error_free(error);
    throw CameraError(message);
  };
  if constexpr (std::is_void_v<Result>) {
    function(&error);
    check();
  } else {
    auto result = function(&error);
    check();
    return result;
  }
}

std::mutex discoveryMutex;
std::string text(const char *value) { return value ? value : ""; }

class CameraAravis final : public Camera {
public:
  CameraAravis(CameraDescriptor descriptor, CameraSettings requestedSettings)
      : info(std::move(descriptor)), settings(std::move(requestedSettings)) {
    {
      std::lock_guard<std::mutex> lock(discoveryMutex);
      camera.reset(checked("Open Aravis camera", [&](GError **error) {
        return arv_camera_new(info.id.c_str(), error);
      }));
    }
    if (!camera) throw CameraDisconnected("Aravis camera was not found: " + info.id);
    auto *device = arv_camera_get_device(camera.get());
    arv_device_set_range_check_policy(device, ARV_RANGE_CHECK_POLICY_ENABLE);
    arv_device_set_access_check_policy(device, ARV_ACCESS_CHECK_POLICY_ENABLE);
    controlLost = g_signal_connect(device, "control-lost", G_CALLBACK(lostControl), &disconnected);
    try {
      configure();
    } catch (...) {
      g_signal_handler_disconnect(device, controlLost);
      throw;
    }
  }

  ~CameraAravis() override {
    try { stop(); } catch (...) {}
    g_signal_handler_disconnect(arv_camera_get_device(camera.get()), controlLost);
  }

  const CameraDescriptor &descriptor() const override { return info; }
  CameraCapabilities capabilities() const override { return support; }
  cv::Size resolution() const override { return size; }
  bool isCapturing() const override { return capturing; }

  void start() override {
    ensureConnected();
    if (capturing) return;
    stream.reset(checked("Create Aravis stream", [&](GError **error) {
      return arv_camera_create_stream(camera.get(), nullptr, nullptr, error);
    }));
    if (!stream) throw CameraError("Aravis did not create a stream");
    for (std::size_t index = 0; index < settings.bufferCount; ++index) {
      auto *buffer = arv_buffer_new_allocate(payload);
      if (!buffer) throw CameraError("Cannot allocate Aravis image buffer");
      arv_stream_push_buffer(stream.get(), buffer);
    }
    checked("Start Aravis acquisition", [&](GError **error) { arv_camera_start_acquisition(camera.get(), error); });
    capturing = true;
  }

  void stop() override {
    if (!capturing) { stream.reset(); return; }
    capturing = false;
    try {
      checked("Stop Aravis acquisition", [&](GError **error) { arv_camera_stop_acquisition(camera.get(), error); });
    } catch (...) {
      stream.reset();
      throw;
    }
    stream.reset();
  }

  void softwareTrigger() override {
    ensureConnected();
    if (!capturing || settings.trigger != CameraTrigger::Software)
      throw CameraError("Aravis software trigger requires active software-triggered capture");
    checked("Trigger Aravis camera", [&](GError **error) { arv_camera_software_trigger(camera.get(), error); });
  }

  CameraReadResult read(std::chrono::milliseconds timeout) override {
    validateCameraTimeout(timeout);
    ensureConnected();
    if (!capturing) throw CameraError("Start Aravis acquisition before reading");
    auto *buffer = arv_stream_timeout_pop_buffer(stream.get(),
        static_cast<guint64>(std::chrono::duration_cast<std::chrono::microseconds>(timeout).count()));
    if (!buffer) { ensureConnected(); return {}; }
    struct ReturnBuffer {
      ArvStream *stream;
      ArvBuffer *buffer;
      ~ReturnBuffer() { arv_stream_push_buffer(stream, buffer); }
    } returnBuffer{stream.get(), buffer};
    ensureConnected();
    if (arv_buffer_get_status(buffer) != ARV_BUFFER_STATUS_SUCCESS)
      return {CameraReadStatus::Incomplete, {}};
    if (arv_buffer_get_image_pixel_format(buffer) != ARV_PIXEL_FORMAT_MONO_8)
      throw CameraError("Aravis returned a non-Mono8 frame");
    const int width = arv_buffer_get_image_width(buffer), height = arv_buffer_get_image_height(buffer);
    gint paddingX = 0, paddingY = 0;
    arv_buffer_get_image_padding(buffer, &paddingX, &paddingY);
    std::size_t bytes = 0;
    const auto *data = arv_buffer_get_image_data(buffer, &bytes);
    if (!data || width <= 0 || height <= 0 || paddingX < 0 || cv::Size(width, height) != size)
      throw CameraError("Aravis returned invalid image geometry");
    const auto stride = static_cast<std::size_t>(width) + static_cast<std::size_t>(paddingX);
    if (stride > std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(height) ||
        bytes < stride * static_cast<std::size_t>(height - 1) + static_cast<std::size_t>(width))
      throw CameraError("Aravis image payload is truncated");
    CameraFrameMetadata metadata;
    metadata.frameId = arv_buffer_get_frame_id(buffer);
    const auto timestamp = arv_buffer_get_timestamp(buffer);
    if (timestamp <= static_cast<guint64>(std::numeric_limits<std::int64_t>::max()))
      metadata.deviceTimestamp = std::chrono::nanoseconds(timestamp);
    metadata.hostTimestamp = std::chrono::steady_clock::now();
    if (support.sequenceStart && arv_buffer_has_chunks(buffer)) {
      GError *error = nullptr;
      const auto status = arv_chunk_parser_get_integer_value(parser.get(), buffer,
          settings.sequenceMarker.chunkFeature.c_str(), &error);
      if (error) {
        std::string message = error->message;
        g_error_free(error);
        if (settings.sequenceMarker.policy == SequenceMarkerPolicy::Required)
          throw CameraError("Read required Aravis sequence input: " + message);
      } else {
        metadata.lineStatus = static_cast<std::uint64_t>(status);
      }
    }
    metadata.sequenceStart = cameraSequenceMarker(metadata.lineStatus, settings.sequenceMarker);
    return {CameraReadStatus::Complete,
        cameraImageSnapshot(cv::Mat(height, width, CV_8UC1, const_cast<void *>(data), stride), std::move(metadata))};
  }

private:
  CameraDescriptor info;
  CameraSettings settings;
  ObjectPtr<ArvCamera> camera;
  std::atomic<bool> disconnected{false};
  ObjectPtr<ArvChunkParser> parser;
  ObjectPtr<ArvStream> stream;
  CameraCapabilities support;
  cv::Size size;
  guint payload = 0;
  gulong controlLost = 0;
  bool capturing = false;

  static void lostControl(ArvDevice *, gpointer state) {
    static_cast<std::atomic<bool> *>(state)->store(true);
  }
  void ensureConnected() const {
    if (disconnected.load()) throw CameraDisconnected("Lost control of Aravis camera: " + info.id);
  }
  bool feature(const char *name) {
    return checked("Inspect Aravis feature", [&](GError **error) {
      return arv_camera_is_feature_available(camera.get(), name, error);
    });
  }
  bool entry(const char *featureName, const char *value) {
    if (!feature(featureName)) return false;
    return checked("Inspect Aravis enumeration", [&](GError **error) {
      return arv_camera_is_enumeration_entry_available(camera.get(), featureName, value, error);
    });
  }
  void set(const char *name, const char *value) {
    checked(name, [&](GError **error) { arv_camera_set_string(camera.get(), name, value, error); });
  }
  void configure() {
    checked("Select Mono8", [&](GError **error) { arv_camera_set_pixel_format(camera.get(), ARV_PIXEL_FORMAT_MONO_8, error); });
    auto region = settings.region;
    if (!region.width || !region.height) {
      int width = 0, height = 0;
      checked("Read Aravis sensor size", [&](GError **error) {
        arv_camera_get_sensor_size(camera.get(), &width, &height, error);
      });
      if (!region.width) region.width = width;
      if (!region.height) region.height = height;
    }
    checked("Set Aravis ROI", [&](GError **error) {
      arv_camera_set_region(camera.get(), region.x, region.y, region.width, region.height, error);
    });
    int offsetX = 0, offsetY = 0;
    checked("Read Aravis ROI", [&](GError **error) {
      arv_camera_get_region(camera.get(), &offsetX, &offsetY, &size.width, &size.height, error);
    });
    if (size.width <= 0 || size.height <= 0 || cv::Rect(offsetX, offsetY, size.width, size.height) != region)
      throw CameraError("Aravis ROI does not match the requested geometry");
    if (checked("Inspect auto exposure", [&](GError **error) { return arv_camera_is_exposure_auto_available(camera.get(), error); }))
      checked("Disable auto exposure", [&](GError **error) { arv_camera_set_exposure_time_auto(camera.get(), ARV_AUTO_OFF, error); });
    checked("Set exposure in microseconds", [&](GError **error) {
      arv_camera_set_exposure_time(camera.get(), static_cast<double>(settings.exposure.count()), error);
    });
    if (checked("Inspect auto gain", [&](GError **error) { return arv_camera_is_gain_auto_available(camera.get(), error); }))
      checked("Disable auto gain", [&](GError **error) { arv_camera_set_gain_auto(camera.get(), ARV_AUTO_OFF, error); });
    if (checked("Inspect gain", [&](GError **error) { return arv_camera_is_gain_available(camera.get(), error); }))
      checked("Set gain", [&](GError **error) { arv_camera_set_gain(camera.get(), settings.gain, error); });
    else if (settings.gain != 0) throw CameraError("This Aravis camera does not expose gain control");
    checked("Select continuous acquisition", [&](GError **error) {
      arv_camera_set_acquisition_mode(camera.get(), ARV_ACQUISITION_MODE_CONTINUOUS, error);
    });
    if (feature("TriggerSelector")) {
      checked("Disable existing triggers", [&](GError **error) { arv_camera_clear_triggers(camera.get(), error); });
      if (entry("TriggerSelector", "FrameStart")) set("TriggerSelector", "FrameStart");
      else if (settings.trigger != CameraTrigger::FreeRunning)
        throw CameraError("Aravis camera does not expose the FrameStart trigger selector");
    }
    support.softwareTrigger = entry("TriggerSource", "Software") && feature("TriggerSoftware");
    support.externalTrigger = entry("TriggerSource", settings.frameTriggerSource.c_str());
    if (settings.trigger != CameraTrigger::FreeRunning) {
      if (!(settings.trigger == CameraTrigger::Software ? support.softwareTrigger : support.externalTrigger))
        throw CameraError("Requested Aravis frame trigger source is unavailable");
      set("TriggerSource", settings.trigger == CameraTrigger::Software ? "Software" : settings.frameTriggerSource.c_str());
      if (settings.trigger == CameraTrigger::External) {
        const char *activation = settings.risingEdge ? "RisingEdge" : "FallingEdge";
        if (!entry("TriggerActivation", activation)) throw CameraError("Requested Aravis trigger edge is unavailable");
        set("TriggerActivation", activation);
      }
      set("TriggerMode", "On");
    }
    configureMarker();
    payload = checked("Read Aravis payload size", [&](GError **error) { return arv_camera_get_payload(camera.get(), error); });
    if (!payload || static_cast<std::uint64_t>(payload) * settings.bufferCount > 512ull * 1024 * 1024)
      throw CameraError("Aravis buffer pool is empty or exceeds 512 MiB");
  }
  void configureMarker() {
    const auto &marker = settings.sequenceMarker;
    if (marker.policy == SequenceMarkerPolicy::Disabled) return;
    const bool available = !marker.chunkSelector.empty() && !marker.chunkFeature.empty() &&
        feature("ChunkModeActive") && entry("ChunkSelector", marker.chunkSelector.c_str()) &&
        feature(marker.chunkFeature.c_str());
    if (!available) {
      if (marker.policy == SequenceMarkerPolicy::Required)
        throw CameraError("Camera does not expose the required latched sequence-input chunk; configure the camera-specific chunk names");
      return;
    }
    if (!marker.inputLine.empty()) {
      if (!entry("LineSelector", marker.inputLine.c_str())) throw CameraError("Sequence input line is unavailable");
      set("LineSelector", marker.inputLine.c_str());
      if (entry("LineMode", "Input")) set("LineMode", "Input");
    }
    checked("Enable chunk metadata", [&](GError **error) { arv_camera_set_chunk_mode(camera.get(), TRUE, error); });
    checked("Enable sequence input chunk", [&](GError **error) {
      arv_camera_set_chunk_state(camera.get(), marker.chunkSelector.c_str(), TRUE, error);
    });
    parser.reset(arv_camera_create_chunk_parser(camera.get()));
    if (!parser) throw CameraError("Cannot create Aravis chunk parser");
    support.sequenceStart = true;
  }
};

}

std::vector<CameraDescriptor> aravisCameraDevices() {
  std::lock_guard<std::mutex> lock(discoveryMutex);
  arv_update_device_list();
  std::vector<CameraDescriptor> result;
  for (unsigned int index = 0; index < arv_get_n_devices(); ++index)
    result.push_back({CameraBackend::Aravis, text(arv_get_device_id(index)), text(arv_get_device_vendor(index)),
        text(arv_get_device_model(index)), text(arv_get_device_serial_nbr(index)), text(arv_get_device_protocol(index))});
  return result;
}

std::unique_ptr<Camera> openAravisCamera(std::size_t deviceIndex, const CameraSettings &requestedSettings) {
  const auto devices = aravisCameraDevices();
  if (deviceIndex >= devices.size()) throw CameraError("Aravis camera index is out of range");
  return std::make_unique<CameraAravis>(devices[deviceIndex], requestedSettings);
}

std::vector<std::string> aravisFrameTriggerSources(std::size_t deviceIndex) {
  const auto devices = aravisCameraDevices();
  if (deviceIndex >= devices.size()) throw CameraError("Aravis camera index is out of range");
  ObjectPtr<ArvCamera> camera;
  {
    std::lock_guard<std::mutex> lock(discoveryMutex);
    camera.reset(checked("Inspect Aravis camera", [&](GError **error) {
      return arv_camera_new(devices[deviceIndex].id.c_str(), error);
    }));
  }
  if (!camera) throw CameraDisconnected("Aravis camera was not found: " + devices[deviceIndex].id);
  const auto available = [&](const char *feature) {
    return checked("Inspect Aravis trigger feature", [&](GError **error) {
      return arv_camera_is_feature_available(camera.get(), feature, error);
    });
  };
  if (!available("TriggerSource")) return {};
  if (available("TriggerSelector")) {
    if (!checked("Inspect Aravis FrameStart trigger", [&](GError **error) {
      return arv_camera_is_enumeration_entry_available(camera.get(), "TriggerSelector", "FrameStart", error);
    })) return {};
    checked("Select Aravis FrameStart trigger", [&](GError **error) {
      arv_camera_set_string(camera.get(), "TriggerSelector", "FrameStart", error);
    });
  }
  guint count = 0;
  std::unique_ptr<const char *, decltype(&g_free)> sources(
      checked("Read Aravis trigger sources", [&](GError **error) {
        return arv_camera_dup_available_trigger_sources(camera.get(), &count, error);
      }), g_free);
  std::vector<std::string> result;
  for (guint index = 0; index < count; ++index) result.emplace_back(sources.get()[index]);
  return result;
}

}