#pragma once

#include "CodecPhaseShift.h"

class EncoderCalibration : public EncoderPhaseShift {
public:
  EncoderCalibration(unsigned int columns, unsigned int rows, CodecDir direction)
      : EncoderPhaseShift(columns, rows, direction, {3, 16}) {}
};

class DecoderCalibration : public DecoderPhaseShift {
public:
  DecoderCalibration(unsigned int columns, unsigned int rows, CodecDir direction)
      : DecoderPhaseShift(columns, rows, direction, {3, 16}) {}

  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override {
    decodeFramesWithThreshold(frames, up, vp, mask, shading, 10);
  }
};
