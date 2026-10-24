#pragma once

#include "Codec.h"

class EncoderPhaseShift : public Encoder {
public:
  EncoderPhaseShift(unsigned int columns, unsigned int rows, CodecDir direction,
                    const PhaseShiftSettings &settings = {});
  cv::Mat getEncodingPattern(unsigned int index) const override;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift : public Decoder {
public:
  DecoderPhaseShift(unsigned int columns, unsigned int rows, CodecDir direction,
                    const PhaseShiftSettings &settings = {});
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;

protected:
  void decodeFramesWithThreshold(const std::vector<cv::Mat> &frames,
                                 cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading,
                                 double threshold) const;

private:
  const PhaseShiftSettings settings;
};
