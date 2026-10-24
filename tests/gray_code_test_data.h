#pragma once

#include "GeometricCalibrationData.h"
#include "CodecGrayCode.h"
#include <algorithm>
#include <cmath>

inline GeometricCalibrationData grayCodeCalibration() {
  GeometricCalibrationData calibration;
  calibration.frameWidth = 64;
  calibration.frameHeight = 48;
  calibration.screenResX = 128;
  calibration.screenResY = 96;
  calibration.Kc = {80, 0, 32, 0, 80, 24, 0, 0, 1};
  calibration.Kp = {80, 0, 64, 0, 80, 48, 0, 0, 1};
  calibration.kc = calibration.kp = cv::Vec<float, 5>::all(0);
  calibration.Rp = cv::Matx33f::eye();
  calibration.Tp = {-0.153f, -0.107f, 0};
  return calibration;
}

// Parallel pinhole rig viewing a frontoparallel plane. A two-camera-pixel box
// footprint produces a linear edge ramp with an independently known crossing.
inline std::vector<cv::Mat> grayCodeFrames(const GeometricCalibrationData &calibration, CodecDir direction,
                                          double depth = 1, bool varyingReflectance = false) {
  EncoderGrayCode encoder(calibration.screenResX, calibration.screenResY, direction);
  std::vector<cv::Mat> frames;
  for (unsigned int index = 0; index < encoder.getNPatterns(); ++index) {
    const auto pattern = encoder.getEncodingPattern(index);
    cv::Mat1b frame(calibration.frameHeight, calibration.frameWidth);
    for (int row = 0; row < frame.rows; ++row)
      for (int column = 0; column < frame.cols; ++column) {
        double coverage = pattern.at<cv::Vec3b>(0, 0)[0] / 255.0;
        if (pattern.rows > 1 || pattern.cols > 1) {
          const int axis = pattern.cols > 1 ? 0 : 1;
          const double scale = calibration.Kp(axis, axis) / calibration.Kc(axis, axis);
          const double camera = axis == 0 ? column : row;
          const double projector = (camera - calibration.Kc(axis, 2)) * scale +
              calibration.Kp(axis, 2) + calibration.Kp(axis, axis) * calibration.Tp[axis] / depth;
          const double left = projector - scale, right = projector + scale;
          coverage = 0;
          for (int pixel = static_cast<int>(std::floor(left + 0.5));
               pixel <= static_cast<int>(std::floor(right + 0.5)); ++pixel) {
            const double overlap = std::max(0.0, std::min(right, pixel + 0.5) - std::max(left, pixel - 0.5));
            if (pixel >= 0 && pixel < (axis == 0 ? pattern.cols : pattern.rows))
              coverage += overlap * pattern.at<cv::Vec3b>(axis == 0 ? 0 : pixel, axis == 0 ? pixel : 0)[0] /
                  (255.0 * (right - left));
          }
        }
        const double contrast = varyingReflectance ? 120 + column : 200;
        frame(row, column) = cv::saturate_cast<std::uint8_t>(20 + contrast * coverage);
      }
    frames.push_back(frame);
  }
  return frames;
}
