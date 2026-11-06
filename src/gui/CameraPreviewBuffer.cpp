#include "CameraPreviewBuffer.h"

#include <algorithm>
#include <stdexcept>

namespace slstudio::gui {
void CameraPreviewBuffer::clear() {
  image_.release();
  frameSize_ = {};
  nextFrame_ = 0;
  dirty_ = false;
  times_.clear();
}

void CameraPreviewBuffer::setFrameCount(int count, int configuredWidth, int maximumTextureSize) {
  const int width = frameSize_.empty() ? std::max(1, configuredWidth) : frameSize_.width;
  if (count < 1 || maximumTextureSize <= 0 || count > maximumTextureSize / width)
    throw std::invalid_argument("Preview frame count exceeds the supported image width");
  if (frameCount_ != count) {
    frameCount_ = count;
    image_.release();
    nextFrame_ = 0;
    dirty_ = false;
  }
}

void CameraPreviewBuffer::append(const cv::Mat &frame, int maximumTextureSize, Clock::time_point now) {
  if (frame.empty() || frame.dims != 2 || frame.type() != CV_8UC1)
    throw std::invalid_argument("Camera preview requires a nonempty Mono8 frame");
  if (image_.empty() || frame.size() != frameSize_) {
    if (maximumTextureSize <= 0 || frameCount_ > maximumTextureSize / frame.cols || frame.rows > maximumTextureSize)
      throw std::invalid_argument("Camera preview dimensions exceed the supported texture size; reduce the frame count or camera resolution");
    image_ = cv::Mat::zeros(frame.rows, frame.cols * frameCount_, CV_8UC1);
    frameSize_ = frame.size();
    nextFrame_ = 0;
  }
  frame.copyTo(image_(cv::Rect(nextFrame_ * frame.cols, 0, frame.cols, frame.rows)));
  nextFrame_ = (nextFrame_ + 1) % frameCount_;
  dirty_ = true;
  times_.push_back(now);
  if (times_.size() > 31) times_.pop_front();
}

cv::Mat CameraPreviewBuffer::latestFrame() const {
  if (image_.empty()) return {};
  const int index = latestFrameIndex();
  return image_(cv::Rect(index * frameSize_.width, 0, frameSize_.width, frameSize_.height));
}

int CameraPreviewBuffer::latestFrameIndex() const {
  return (nextFrame_ + frameCount_ - 1) % frameCount_;
}

double CameraPreviewBuffer::fps(Clock::time_point now) const {
  if (times_.size() < 2 || now - times_.back() >= std::chrono::seconds(1)) return 0;
  const double elapsed = std::chrono::duration<double>(times_.back() - times_.front()).count();
  return elapsed > 0 ? (times_.size() - 1) / elapsed : 0;
}
}