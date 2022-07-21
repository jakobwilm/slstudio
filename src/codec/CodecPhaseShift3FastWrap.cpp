#include "CodecPhaseShift3FastWrap.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"

#ifndef M_PI
#define M_PI 3.14159265359
#endif

// Encoder
EncoderPhaseShift3FastWrap::EncoderPhaseShift3FastWrap(unsigned int _screenCols,
                                                       unsigned int _screenRows,
                                                       CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir) {

  // Set N
  N = 3;

  // Precompute encoded patterns
  const float pi = M_PI;
  for (unsigned int i = 0; i < N; i++) {
    float phase = 2.0 * pi / float(N) * i;
    float pitch = screenCols;
    cv::Mat patternI = pstools::computePhaseVector(screenCols, phase, pitch);
    patterns.push_back(patternI.t());
  }
}

cv::Mat
EncoderPhaseShift3FastWrap::getEncodingPattern(unsigned int depth) const {
  return patterns[depth];
}

// Decoder
DecoderPhaseShift3FastWrap::DecoderPhaseShift3FastWrap(unsigned int _screenCols,
                                                       unsigned int _screenRows,
                                                       CodecDir _dir)
    : Decoder(_screenCols, _screenRows) {
  N = 3;
  frames.resize(N);
}

void DecoderPhaseShift3FastWrap::setFrame(unsigned int depth, cv::Mat frame) {
  frames[depth] = frame;
}

void DecoderPhaseShift3FastWrap::decodeFrames(cv::Mat &up, cv::Mat &vp,
                                              cv::Mat &mask,
                                              cv::Mat &shading) const {

  const float pi = M_PI;

  up.create(frames[0].size(), CV_32FC1);

  // Wrap phase using Zhang's intensity ratio (w/o correction)
  for (int r = 0; r < up.rows; r++) {
    for (int c = 0; c < up.cols; c++) {

      float f0 = frames[0].at<unsigned short>(r, c);
      float f1 = frames[1].at<unsigned short>(r, c);
      float f2 = frames[2].at<unsigned short>(r, c);

      if (f0 > f1) {
        if (f1 > f2) {
          // f0 > f1 > f2
          up.at<float>(r, c) = (f1 - f2) / (f0 - f2);
        } else if (f0 > f2) {
          // f0 > f2 > f1
          up.at<float>(r, c) = 6.0 - (f2 - f1) / (f0 - f1);
        } else {
          // f2 > f0 > f1
          up.at<float>(r, c) = 4.0 + (f0 - f1) / (f2 - f1);
        }
      } else {
        if (f0 > f2) {
          // f1 > f0 > f2
          up.at<float>(r, c) = 2.0 - (f0 - f2) / (f1 - f2);
        } else if (f1 > f2) {
          // f1 > f2 > f0
          up.at<float>(r, c) = 2.0 + (f2 - f0) / (f1 - f0);
        } else {
          // f2 > f1 > f0
          up.at<float>(r, c) = 4.0 - (f1 - f0) / (f2 - f0);
        }
      }
    }
  }

  //    cvtools::writeMat(up, "up.mat");

  up *= screenCols / (2 * pi);

  cv::GaussianBlur(up, up, cv::Size(0, 0), 3, 3);

  //    shading = pstools::getMagnitude(frames[0], frames[1], frames[2]);

  shading = cv::max(frames[0], frames[1]);
  shading = cv::max(shading, frames[2]);

  // Create mask from modulation image and erode
  mask.create(shading.size(), cv::DataType<bool>::type);
  mask = (shading > 15) & (shading < 254);

  //    cv::Mat edges;
  //    cv::Sobel(up, edges, -1, 1, 1, 7);
  //    edges = abs(edges) < 500;
  //    cv::erode(edges, edges, cv::Mat());

  //    mask = mask & edges;
}
