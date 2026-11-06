#pragma once

#include <opencv2/core.hpp>
#include <chrono>
#include <cstddef>
#include <deque>

namespace slstudio::gui {
class CameraPreviewBuffer {
public:
  using Clock = std::chrono::steady_clock;

  void clear();
  void setFrameCount(int count, int configuredWidth, int maximumTextureSize);
  void append(const cv::Mat &frame, int maximumTextureSize, Clock::time_point now = Clock::now());

  bool empty() const { return image_.empty(); }
  const cv::Mat &image() const { return image_; }
  cv::Mat latestFrame() const;
  cv::Size frameSize() const { return frameSize_; }
  int frameCount() const { return frameCount_; }
  int nextWriteIndex() const { return nextFrame_; }
  int latestFrameIndex() const;
  bool dirty() const { return dirty_; }
  void markUploaded() { dirty_ = false; }
  std::size_t sampleCount() const { return times_.size(); }
  double fps(Clock::time_point now = Clock::now()) const;

private:
  cv::Mat image_;
  cv::Size frameSize_;
  int frameCount_ = 1;
  int nextFrame_ = 0;
  bool dirty_ = false;
  std::deque<Clock::time_point> times_;
};
}