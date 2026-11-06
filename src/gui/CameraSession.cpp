#include "CameraSession.h"

#include <stdexcept>
#include <utility>

namespace slstudio::gui {
bool CameraSession::requestFrame() {
  if (!worker_) throw std::logic_error("Start the camera before requesting a frame");
  return worker_->requestFrame();
}

std::optional<cv::Mat> CameraSession::tryTakeFrame() {
  if (!worker_) throw std::logic_error("Start the camera before taking a frame");
  auto frame = worker_->tryTake();
  if (frame && (resolution_.width <= 0 || resolution_.height <= 0))
    resolution_ = frame->size();
  return frame;
}

std::future<void> CameraSession::open(CaptureWorker::CameraFactory factory, cv::Size resolution,
    bool rememberPreviewFactory) {
  if (!factory) throw std::invalid_argument("Camera factory is required");
  if (rememberPreviewFactory) previewFactory_ = factory;
  worker_ = &acquisition_;
  resolution_ = resolution;
  return acquisition_.openCamera(std::move(factory));
}

std::future<void> CameraSession::openPreview(cv::Size resolution) {
  if (!previewFactory_) throw std::logic_error("No preview camera factory is available");
  return open(previewFactory_, resolution);
}

std::future<void> CameraSession::close() {
  auto *previous = std::exchange(worker_, nullptr);
  resolution_ = {};
  return previous ? previous->closeCamera() : std::future<void>{};
}
}