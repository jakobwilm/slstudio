#pragma once

#include <opencv2/core.hpp>
#include <vector>

class RBFInterpolator {
public:
  explicit RBFInterpolator(double regularization = 100.0);
  void setDataPoints(const std::vector<cv::Point2f> &points,
                     const std::vector<cv::Point2f> &values);
  cv::Point2f interpolate(cv::Point2f point) const;

private:
  double regularization;
  cv::Point2d origin;
  std::vector<cv::Point2d> knots;
  cv::Mat coefficients;
};
