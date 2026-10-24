#pragma once

#include "Codec.h"

class EncoderPhaseShiftHeterodyne : public Encoder {
public:
  EncoderPhaseShiftHeterodyne(unsigned int columns, unsigned int rows, CodecDir direction,
                             const PhaseShiftHeterodyneSettings &settings = {});
  cv::Mat getEncodingPattern(unsigned int index) const override;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShiftHeterodyne : public Decoder {
public:
  DecoderPhaseShiftHeterodyne(unsigned int columns, unsigned int rows, CodecDir direction,
                             const PhaseShiftHeterodyneSettings &settings = {});
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;

private:
  PhaseShiftHeterodyneSettings settings;
};
