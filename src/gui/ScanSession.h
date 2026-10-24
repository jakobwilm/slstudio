#pragma once

#include "CaptureWorker.h"

namespace slstudio::gui {
class ScanSession {
public:
  bool active() const { return capture_ != nullptr; }
  int index() const { return active() ? static_cast<int>(capture_->sequenceIndex()) : -1; }
  unsigned int count() const { return count_; }
  bool geometricCalibration() const { return geometricCalibration_; }
  bool continuous() const { return continuous_; }
  CaptureWorker *capture() const { return capture_; }
  SequenceCaptureStatistics statistics() const {
    return capture_ ? capture_->statistics() : statistics_;
  }

  void begin(unsigned int count, bool geometricCalibration, bool continuous,
             CaptureWorker &capture) {
    if (active()) throw std::logic_error("Stop capture before starting another scan");
    if (!count || (geometricCalibration && continuous)) throw std::invalid_argument("Invalid scan settings");
    count_ = count;
    geometricCalibration_ = geometricCalibration;
    continuous_ = continuous;
    capture_ = &capture;
    statistics_ = {};
  }

  void stop() {
    statistics_ = statistics();
    capture_ = nullptr;
    continuous_ = false;
  }

private:
  CaptureWorker *capture_ = nullptr;
  SequenceCaptureStatistics statistics_;
  unsigned int count_ = 0;
  bool geometricCalibration_ = false, continuous_ = false;
};
}
