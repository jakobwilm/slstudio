#pragma once

#include "GeometricCalibrationData.h"
#include <opencv2/calib3d.hpp>
#include <cstddef>
#include <string>
#include <vector>
#include <limits>

enum class GeometricCalibrationInterpolation { LocalHomography, RBF };

struct GeometricCalibrationSettings {
  cv::Size boardCorners{10, 10};
  float spacing = 10.0f;
  bool circles = false;
  bool partialCircles = false;
  int neighborhoodRadius = 10;
  int cameraFlags = cv::CALIB_FIX_K3;
  int projectorFlags = cv::CALIB_FIX_K3;
  GeometricCalibrationInterpolation interpolation = GeometricCalibrationInterpolation::LocalHomography;
};

struct GeometricCalibrationPoseDiagnostics {
  std::vector<cv::Point2f> corners;
  std::size_t correspondenceCount = 0;
  bool accepted = false;
  std::string reason;
  std::vector<cv::Point2f> acceptedCorners, projectorCorners;
  std::vector<cv::Point2f> cameraResiduals, projectorResiduals;
  double cameraRms = std::numeric_limits<double>::quiet_NaN();
  double projectorRms = std::numeric_limits<double>::quiet_NaN();
};

GeometricCalibrationData calibrateGeometry(const std::vector<std::vector<cv::Mat>> &sequences,
                                cv::Size projectorSize,
                                const GeometricCalibrationSettings &settings = {});

GeometricCalibrationData calibrateGeometry(const std::vector<std::vector<cv::Mat>> &sequences,
                                cv::Size projectorSize, const GeometricCalibrationSettings &settings,
                                std::vector<GeometricCalibrationPoseDiagnostics> *poseDiagnostics);