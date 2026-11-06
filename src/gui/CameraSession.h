#pragma once

#include "CaptureWorker.h"
#include <future>
#include <utility>

namespace slstudio::gui {
class CameraSession {
public:
  explicit CameraSession(CaptureWorker &acquisition) : acquisition_(acquisition) {}

  CaptureWorker *worker() const { return worker_; }
  bool active() const { return worker_ != nullptr; }
  bool requested() const { return requested_; }
  void setRequested(bool requested) { requested_ = requested; }
  cv::Size resolution() const { return resolution_; }
  bool ready() const { return worker_ && worker_->ready(); }
  bool framePending() const { return worker_ && worker_->framePending(); }
  bool requestFrame();
  std::optional<cv::Mat> tryTakeFrame();
  std::future<void> open(CaptureWorker::CameraFactory factory, cv::Size resolution,
      bool rememberPreviewFactory = false);
  std::future<void> openPreview(cv::Size resolution);
  std::future<void> close();
  void setPreviewFactory(CaptureWorker::CameraFactory factory) { previewFactory_ = std::move(factory); }
  bool hasPreviewFactory() const { return static_cast<bool>(previewFactory_); }

private:
  CaptureWorker &acquisition_;
  CaptureWorker *worker_ = nullptr;
  cv::Size resolution_;
  bool requested_ = false;
  CaptureWorker::CameraFactory previewFactory_;
};
}