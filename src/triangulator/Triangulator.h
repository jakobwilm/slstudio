#pragma once

#include "GeometricCalibrationData.h"
#include "PointCloud.h"
#include "ProjectorTransition.h"

#include <opencv2/opencv.hpp>
#include <vector>

struct TriangulationResult {
  cv::Mat3f xyz;
  cv::Mat1b shading;
};

class TriangulatorOrganized {
public:
  explicit TriangulatorOrganized(const GeometricCalibrationData &calibration, bool correctProjectorDistortion = false);
  GeometricCalibrationData getGeometricCalibration() const { return calibration; }
  TriangulationResult triangulate(const cv::Mat1f &up, const cv::Mat1f &vp,
                                 const cv::Mat1b &mask, const cv::Mat1b &shading) const;
  // Reconstruction
  void triangulate(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading,
                   cv::Mat &pointCloud) const;

private:
  void triangulateFromUp(const cv::Mat &up, cv::Mat &xyz) const;
  void triangulateFromVp(const cv::Mat &vp, cv::Mat &xyz) const;
  void triangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp,
                           const cv::Mat &mask, cv::Mat &xyz) const;
  void refineProjectorDistortion(const cv::Mat &up, const cv::Mat &vp,
                                const cv::Mat &mask, cv::Mat &xyz) const;
  GeometricCalibrationData calibration;
  bool correctProjectorDistortion;
  cv::Mat determinantTensor;
  cv::Mat uc, vc;
  cv::Mat lensMap1, lensMap2;
  std::vector<cv::Mat> xyzwPrecomputeOffset;
  std::vector<cv::Mat> xyzwPrecomputeFactor;
};

class TriangulatorUnorganized {
public:
  explicit TriangulatorUnorganized(const GeometricCalibrationData &calibration,
                                  bool correctProjectorDistortion = false);
  PointCloudPtr triangulate(const std::vector<ProjectorTransition> &transitions,
                            const PointCloudSettings &settings = {}) const;

private:
  GeometricCalibrationData calibration;
  bool correctProjectorDistortion;
};
