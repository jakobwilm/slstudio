#pragma once

#include "Codec.h"
#include "ProjectorTransition.h"

class EncoderGrayCode : public Encoder {
public:
  EncoderGrayCode(unsigned int _screenCols, unsigned int _screenRows,
                  CodecDir _dir, const GrayCodeSettings &settings = {});
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderGrayCode : public Decoder {
public:
  DecoderGrayCode(unsigned int _screenCols, unsigned int _screenRows,
                  CodecDir _dir, const GrayCodeSettings &settings = {});
  // Decoding
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                    cv::Mat &mask, cv::Mat &shading) override;
  std::vector<ProjectorTransition> decodeTransitions(const std::vector<cv::Mat> &frames,
                                                    const cv::Mat1f &up, const cv::Mat1f &vp,
                                                    const cv::Mat1b &mask, const cv::Mat1b &shading) const;

private:
  const GrayCodeSettings settings;
};
