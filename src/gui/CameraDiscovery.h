#pragma once

#include "CameraFactory.h"
#include <future>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace slstudio::gui {
struct CameraDiscoveryResult {
  std::vector<slstudio::CameraDescriptor> devices;
  int index = 0;
  std::vector<std::string> triggerSources;
  std::string triggerError;
  bool triggersInspected = false;
};

class CameraDiscovery {
public:
  using Probe = std::function<CameraDiscoveryResult()>;

  void initializeBackends();
  int ensureBackend(slstudio::CameraBackend backend);
  bool nativeSelected(int cameraInterface) const;
  bool busy() const { return job_.valid(); }
  bool ready() const;
  void wait();
  void begin(std::optional<slstudio::CameraBackend> backend, int cameraInterface, int cameraIndex,
      bool enumerate, bool inspect, std::future<void> previousCamera, Probe probe = {});
  bool matches(int cameraInterface, int cameraIndex) const;
  CameraDiscoveryResult takeReady();
  int apply(CameraDiscoveryResult result);
  void clearTriggerSources();
  void setTriggerError(std::string error) { triggerError_ = std::move(error); }

  std::vector<slstudio::CameraBackend> &backends() { return backends_; }
  const std::vector<slstudio::CameraBackend> &backends() const { return backends_; }
  std::vector<slstudio::CameraDescriptor> &devices() { return devices_; }
  const std::vector<slstudio::CameraDescriptor> &devices() const { return devices_; }
  std::optional<slstudio::CameraDescriptor> &triggerDevice() { return triggerDevice_; }
  const std::optional<slstudio::CameraDescriptor> &triggerDevice() const { return triggerDevice_; }
  std::vector<std::string> &triggerSources() { return triggerSources_; }
  const std::vector<std::string> &triggerSources() const { return triggerSources_; }
  std::string &triggerError() { return triggerError_; }
  const std::string &triggerError() const { return triggerError_; }

private:
  std::vector<slstudio::CameraBackend> backends_;
  std::vector<slstudio::CameraDescriptor> devices_;
  std::future<CameraDiscoveryResult> job_;
  int requestedInterface_ = 0;
  int requestedIndex_ = 0;
  std::optional<slstudio::CameraDescriptor> triggerDevice_;
  std::vector<std::string> triggerSources_;
  std::string triggerError_;
};
}