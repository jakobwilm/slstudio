#include "CameraFactory.h"

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <limits>
#include <iostream>
#include <mutex>

namespace slstudio {
namespace {
using namespace Spinnaker;
using namespace Spinnaker::GenApi;

struct SpinSystem {
  SystemPtr system = System::GetInstance();
  ~SpinSystem() { try { system->ReleaseInstance(); } catch (...) {} }
};

std::shared_ptr<SpinSystem> spinSystem() {
  static std::mutex mutex;
  static std::weak_ptr<SpinSystem> current;
  std::lock_guard<std::mutex> lock(mutex);
  auto system = current.lock();
  if (!system) { system = std::make_shared<SpinSystem>(); current = system; }
  return system;
}

template <typename Function> auto spinChecked(Function function) {
  try { return function(); }
  catch (const Spinnaker::Exception &error) { throw CameraError(std::string("Spinnaker: ") + error.what()); }
}

std::string spinText(INodeMap &nodes, const char *name) {
  CStringPtr node = nodes.GetNode(name);
  return IsReadable(node) ? node->GetValue().c_str() : "";
}

CameraDescriptor spinDescriptor(CameraPtr camera) {
  auto &nodes = camera->GetTLDeviceNodeMap();
  return {CameraBackend::Spinnaker, {}, spinText(nodes, "DeviceVendorName"), spinText(nodes, "DeviceModelName"),
      spinText(nodes, "DeviceSerialNumber"), "Spinnaker"};
}

class CameraSpinnaker final : public Camera {
public:
  CameraSpinnaker(std::size_t index, const CameraSettings &requestedSettings)
      : owner(spinSystem()), settings(requestedSettings) {
    auto list = owner->system->GetCameras();
    if (index >= list.GetSize()) throw CameraError("Spinnaker camera index is out of range");
    camera = list.GetByIndex(static_cast<unsigned int>(index));
    list.Clear();
    info = spinDescriptor(camera);
    camera->Init();
    try { configure(); }
    catch (...) { camera->DeInit(); camera = nullptr; throw; }
  }
  ~CameraSpinnaker() override {
    try { if (capturing) camera->EndAcquisition(); } catch (...) {}
    try { camera->DeInit(); } catch (...) {}
    camera = nullptr;
  }
  const CameraDescriptor &descriptor() const override { return info; }
  CameraCapabilities capabilities() const override { return support; }
  cv::Size resolution() const override { return size; }
  bool isCapturing() const override { return capturing; }
  void start() override {
    if (capturing) return;
    spinChecked([&] { camera->BeginAcquisition(); });
    capturing = true;
  }
  void stop() override {
    if (!capturing) return;
    spinChecked([&] { camera->EndAcquisition(); });
    capturing = false;
  }
  void softwareTrigger() override {
    if (!capturing || settings.trigger != CameraTrigger::Software) throw CameraError("Spinnaker software trigger is not armed");
    spinChecked([&] {
      CCommandPtr command = camera->GetNodeMap().GetNode("TriggerSoftware");
      if (!IsWritable(command)) throw CameraError("Spinnaker software trigger is unavailable");
      command->Execute();
    });
  }
  CameraReadResult read(std::chrono::milliseconds timeout) override {
    validateCameraTimeout(timeout);
    if (!capturing) throw CameraError("Start Spinnaker acquisition before reading");
    try {
      auto image = camera->GetNextImage(static_cast<std::uint64_t>(timeout.count()));
      struct ReleaseImage { ImagePtr image; ~ReleaseImage() { try { image->Release(); } catch (...) {} } } release{image};
      if (image->IsIncomplete()) return {CameraReadStatus::Incomplete, {}};
      const auto width = image->GetWidth(), height = image->GetHeight(), stride = image->GetStride();
      if (image->GetPixelFormat() != PixelFormat_Mono8 || width != static_cast<std::size_t>(size.width) ||
          height != static_cast<std::size_t>(size.height) || !image->GetData() || stride < width ||
          stride > std::numeric_limits<std::size_t>::max() / height || image->GetBufferSize() < stride * (height - 1) + width)
        throw CameraError("Spinnaker returned invalid Mono8 geometry or payload");
      CameraFrameMetadata metadata;
      metadata.frameId = image->GetFrameID();
      const auto timestamp = image->GetTimeStamp();
      if (timestamp <= static_cast<std::uint64_t>(INT64_MAX)) metadata.deviceTimestamp = std::chrono::nanoseconds(timestamp);
      if (support.sequenceStart) metadata.lineStatus = static_cast<std::uint64_t>(image->GetChunkData().GetLineStatusAll());
      metadata.sequenceStart = cameraSequenceMarker(metadata.lineStatus, settings.sequenceMarker);
      return {CameraReadStatus::Complete, cameraImageSnapshot(
          cv::Mat(size.height, size.width, CV_8UC1, image->GetData(), stride), std::move(metadata))};
    } catch (const Spinnaker::Exception &error) {
      if (error.GetError() == SPINNAKER_ERR_TIMEOUT) return {};
      throw CameraError(std::string("Spinnaker read: ") + error.what());
    }
  }
private:
  std::shared_ptr<SpinSystem> owner;
  CameraPtr camera;
  CameraSettings settings;
  CameraDescriptor info;
  CameraCapabilities support;
  cv::Size size;
  bool capturing = false;

  bool entry(INodeMap &nodes, const char *name, const char *value) {
    CEnumerationPtr node = nodes.GetNode(name);
    return IsReadable(node) && IsReadable(node->GetEntryByName(value));
  }
  void set(INodeMap &nodes, const char *name, const char *value) {
    CEnumerationPtr node = nodes.GetNode(name);
    if (!IsWritable(node) || !entry(nodes, name, value)) throw CameraError(std::string("Spinnaker feature unavailable: ") + name + "=" + value);
    node->SetIntValue(node->GetEntryByName(value)->GetValue());
  }
  void integer(INodeMap &nodes, const char *name, std::int64_t value) {
    CIntegerPtr node = nodes.GetNode(name);
    if (!IsWritable(node) || value < node->GetMin() || value > node->GetMax() ||
        (value - node->GetMin()) % node->GetInc() != 0) throw CameraError(std::string("Invalid Spinnaker setting: ") + name);
    node->SetValue(value);
  }
  void number(INodeMap &nodes, const char *name, double value) {
    CFloatPtr node = nodes.GetNode(name);
    if (!IsWritable(node) || value < node->GetMin() || value > node->GetMax()) throw CameraError(std::string("Invalid Spinnaker setting: ") + name);
    node->SetValue(value);
  }
  void configure() {
    auto &nodes = camera->GetNodeMap();
    set(nodes, "TriggerMode", "Off");
    set(nodes, "PixelFormat", "Mono8");
    integer(nodes, "OffsetX", 0);
    integer(nodes, "OffsetY", 0);
    CIntegerPtr width = nodes.GetNode("Width"), height = nodes.GetNode("Height");
    if (!IsReadable(width) || !IsReadable(height)) throw CameraError("Spinnaker sensor size is unavailable");
    integer(nodes, "Width", settings.region.width ? settings.region.width : width->GetMax());
    integer(nodes, "Height", settings.region.height ? settings.region.height : height->GetMax());
    integer(nodes, "OffsetX", settings.region.x);
    integer(nodes, "OffsetY", settings.region.y);
    if (!IsReadable(width) || !IsReadable(height) || width->GetValue() > INT_MAX || height->GetValue() > INT_MAX)
      throw CameraError("Spinnaker image geometry is unavailable");
    size = {static_cast<int>(width->GetValue()), static_cast<int>(height->GetValue())};
    if (size.width <= 0 || size.height <= 0) throw CameraError("Invalid Spinnaker image size");
    if (entry(nodes, "ExposureAuto", "Off")) set(nodes, "ExposureAuto", "Off");
    if (entry(nodes, "GainAuto", "Off")) set(nodes, "GainAuto", "Off");
    if (entry(nodes, "ExposureMode", "Timed")) set(nodes, "ExposureMode", "Timed");
    number(nodes, "ExposureTime", static_cast<double>(settings.exposure.count()));
    number(nodes, "Gain", settings.gain);
    set(nodes, "AcquisitionMode", "Continuous");
    set(nodes, "TriggerSelector", "FrameStart");
    support.softwareTrigger = entry(nodes, "TriggerSource", "Software");
    support.externalTrigger = entry(nodes, "TriggerSource", settings.frameTriggerSource.c_str());
    if (settings.trigger != CameraTrigger::FreeRunning) {
      set(nodes, "TriggerSource", settings.trigger == CameraTrigger::Software ? "Software" : settings.frameTriggerSource.c_str());
      if (settings.trigger == CameraTrigger::External) set(nodes, "TriggerActivation", settings.risingEdge ? "RisingEdge" : "FallingEdge");
      set(nodes, "TriggerMode", "On");
    }
    auto &stream = camera->GetTLStreamNodeMap();
    set(stream, "StreamBufferHandlingMode", "OldestFirst");
    set(stream, "StreamBufferCountMode", "Manual");
    integer(stream, "StreamBufferCountManual", static_cast<std::int64_t>(settings.bufferCount));
    const auto &marker = settings.sequenceMarker;
    if (marker.policy != SequenceMarkerPolicy::Disabled) {
      CBooleanPtr active = nodes.GetNode("ChunkModeActive");
      if (IsWritable(active) && marker.chunkFeature == "ChunkLineStatusAll" && entry(nodes, "ChunkSelector", marker.chunkSelector.c_str())) {
        active->SetValue(true);
        set(nodes, "ChunkSelector", marker.chunkSelector.c_str());
        CBooleanPtr enabled = nodes.GetNode("ChunkEnable");
        if (!IsWritable(enabled)) throw CameraError("Spinnaker sequence input chunk is not writable");
        enabled->SetValue(true);
        support.sequenceStart = true;
      } else if (marker.policy == SequenceMarkerPolicy::Required) {
        throw CameraError("Spinnaker camera does not expose required per-frame LineStatusAll metadata");
      }
    }
  }
};
}

std::vector<CameraDescriptor> spinnakerCameraDevices() {
  return spinChecked([] {
    auto owner = spinSystem();
    auto list = owner->system->GetCameras();
    std::vector<CameraDescriptor> devices;
    for (unsigned int index = 0; index < list.GetSize(); ++index) devices.push_back(spinDescriptor(list.GetByIndex(index)));
    list.Clear();
    return devices;
  });
}
std::unique_ptr<Camera> openSpinnakerCamera(std::size_t index, const CameraSettings &requestedSettings) {
  return spinChecked([&]() -> std::unique_ptr<Camera> { return std::make_unique<CameraSpinnaker>(index, requestedSettings); });
}
std::vector<std::string> spinnakerFrameTriggerSources(std::size_t index) {
  return spinChecked([&] {
    auto owner = spinSystem();
    auto list = owner->system->GetCameras();
    if (index >= list.GetSize()) throw CameraError("Spinnaker camera index is out of range");
    CameraPtr camera = list.GetByIndex(static_cast<unsigned int>(index));
    list.Clear();
    camera->Init();
    struct CameraGuard {
      CameraPtr camera;
      ~CameraGuard() {
        try { camera->DeInit(); }
        catch (const Spinnaker::Exception &error) { std::cerr << "Spinnaker trigger inspection cleanup: " << error.what() << '\n'; }
        camera = nullptr;
      }
    } guard{camera};
    auto &nodes = camera->GetNodeMap();
    CEnumerationPtr selector = nodes.GetNode("TriggerSelector");
    if (IsAvailable(selector)) {
      CEnumEntryPtr frameStart = selector->GetEntryByName("FrameStart");
      if (!IsAvailable(frameStart) || !IsReadable(frameStart)) return std::vector<std::string>{};
      if (!IsWritable(selector)) throw CameraError("Spinnaker FrameStart selector is not writable");
      selector->SetIntValue(frameStart->GetValue());
    }
    CEnumerationPtr source = nodes.GetNode("TriggerSource");
    if (!IsAvailable(source)) return std::vector<std::string>{};
    if (!IsReadable(source)) throw CameraError("Spinnaker trigger sources are not readable");
    NodeList_t entries;
    source->GetEntries(entries);
    std::vector<std::string> sources;
    for (const auto &node : entries) {
      CEnumEntryPtr entry = node;
      if (IsAvailable(entry) && IsReadable(entry)) sources.emplace_back(entry->GetSymbolic().c_str());
    }
    return sources;
  });
}
}