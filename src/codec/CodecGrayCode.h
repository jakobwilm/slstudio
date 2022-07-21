#pragma once

#include "Codec.h"

class EncoderGrayCode : public Encoder {
public:
  EncoderGrayCode(unsigned int _screenCols, unsigned int _screenRows,
                  CodecDir _dir);
  // Encoding
  cv::Mat getEncodingPattern(unsigned int depth) const;

private:
  std::vector<cv::Mat> patterns;
};

class DecoderGrayCode : public Decoder {
public:
  DecoderGrayCode(unsigned int _screenCols, unsigned int _screenRows,
                  CodecDir _dir);
  // Decoding
  void setFrame(unsigned int depth, const cv::Mat frame);
  void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                    cv::Mat &shading) const;

private:
  std::vector<cv::Mat> frames;
};
