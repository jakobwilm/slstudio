#include "CameraDiscovery.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace slstudio::gui {
void CameraDiscovery::initializeBackends() {
  for (const auto backend : {slstudio::CameraBackend::Aravis, slstudio::CameraBackend::Ximea,
      slstudio::CameraBackend::Spinnaker, slstudio::CameraBackend::IIDC})
    if (slstudio::cameraBackendAvailable(backend)) backends_.push_back(backend);
}

int CameraDiscovery::ensureBackend(slstudio::CameraBackend backend) {
  const auto found = std::find(backends_.begin(), backends_.end(), backend);
  if (found != backends_.end()) return static_cast<int>(found - backends_.begin());
  backends_.push_back(backend);
  return static_cast<int>(backends_.size() - 1);
}

bool CameraDiscovery::nativeSelected(int cameraInterface) const {
  return cameraInterface >= 0 && static_cast<std::size_t>(cameraInterface) < backends_.size();
}

bool CameraDiscovery::ready() const {
  return job_.valid() && job_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

void CameraDiscovery::wait() {
  if (job_.valid()) job_.wait();
}

void CameraDiscovery::begin(std::optional<slstudio::CameraBackend> backend, int cameraInterface,
    int cameraIndex, bool enumerate, bool inspect, std::future<void> previousCamera, Probe probe) {
  if (busy()) throw std::logic_error("Wait for camera discovery to finish");
  requestedInterface_ = cameraInterface;
  requestedIndex_ = cameraIndex;
  clearTriggerSources();
  auto devices = enumerate ? std::vector<slstudio::CameraDescriptor>{} : devices_;
  if (enumerate) devices_.clear();
  job_ = std::async(std::launch::async,
      [backend, index = cameraIndex, enumerate, inspect, devices = std::move(devices),
       previousCamera = std::move(previousCamera), probe = std::move(probe)]() mutable {
        if (previousCamera.valid()) previousCamera.get();
        if (probe) return probe();
        CameraDiscoveryResult result;
        result.index = index;
        if (!backend) return result;
        result.devices = enumerate ? slstudio::cameraDevices(*backend) : std::move(devices);
        result.index = std::clamp(index, 0, std::max(0, static_cast<int>(result.devices.size()) - 1));
        if (inspect && !result.devices.empty()) {
          result.triggersInspected = true;
          try {
            result.triggerSources = slstudio::cameraFrameTriggerSources(*backend,
                static_cast<std::size_t>(result.index));
          } catch (const std::exception &error) {
            result.triggerError = error.what();
          }
        }
        return result;
      });
}

bool CameraDiscovery::matches(int cameraInterface, int cameraIndex) const {
  return cameraInterface == requestedInterface_ && cameraIndex == requestedIndex_;
}

CameraDiscoveryResult CameraDiscovery::takeReady() {
  if (!ready()) throw std::logic_error("Camera discovery is not ready");
  return job_.get();
}

int CameraDiscovery::apply(CameraDiscoveryResult result) {
  devices_ = std::move(result.devices);
  triggerSources_ = std::move(result.triggerSources);
  triggerError_ = std::move(result.triggerError);
  if (result.triggersInspected && triggerError_.empty() && result.index >= 0 &&
      static_cast<std::size_t>(result.index) < devices_.size())
    triggerDevice_ = devices_[result.index];
  return result.index;
}

void CameraDiscovery::clearTriggerSources() {
  triggerDevice_.reset();
  triggerSources_.clear();
  triggerError_.clear();
}
}