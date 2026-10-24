#pragma once

#include "Codec.h"

class EncoderPhaseShiftMotionCompensated : public Encoder {
public:
  EncoderPhaseShiftMotionCompensated(unsigned int _screenCols, unsigned int _screenRows,
                       CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderPhaseShiftMotionCompensated : public Decoder {
public:
  DecoderPhaseShiftMotionCompensated(unsigned int _screenCols, unsigned int _screenRows,
                       CodecDir _dir, bool motionCompensation = true);
  void setMotionCompensation(bool enabled);
  // Decoding
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;

private:
  bool motionCompensation;
  cv::Mat1f lastShading;
};
