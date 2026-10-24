#include "RBFInterpolator.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
double gaussian(cv::Point2d first, cv::Point2d second) {
  const auto delta = first - second;
  return std::exp(-0.2 * delta.dot(delta));
}

bool finite(cv::Point2f point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}
}

RBFInterpolator::RBFInterpolator(double smoothing) : regularization(smoothing) {
  if (!std::isfinite(regularization) || regularization < 0)
    throw std::invalid_argument("RBF regularization must be finite and nonnegative");
}

void RBFInterpolator::setDataPoints(const std::vector<cv::Point2f> &points,
                                    const std::vector<cv::Point2f> &values) {
  if (points.size() != values.size() || points.size() < 3 ||
      points.size() > static_cast<std::size_t>(std::numeric_limits<int>::max() - 3))
    throw std::invalid_argument("RBF interpolation requires matching sets of at least three points");
  const auto center = cv::Point2d(points.front());
  std::vector<cv::Point2d> centered;
  centered.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!finite(points[index]) || !finite(values[index]))
      throw std::invalid_argument("RBF coordinates must be finite");
    centered.push_back(cv::Point2d(points[index]) - center);
  }
  const int count = static_cast<int>(points.size());
  cv::Mat1d system(count + 3, count + 3, 0.0), rhs(count + 3, 2, 0.0);
  for (int row = 0; row < count; ++row) {
    for (int col = 0; col <= row; ++col)
      system(row, col) = system(col, row) = gaussian(centered[row], centered[col]);
    system(row, row) += regularization;
    system(row, count) = system(count, row) = 1;
    system(row, count + 1) = system(count + 1, row) = centered[row].x;
    system(row, count + 2) = system(count + 2, row) = centered[row].y;
    rhs(row, 0) = values[row].x;
    rhs(row, 1) = values[row].y;
  }
  cv::Mat fitted;
  if (!cv::solve(system, rhs, fitted, cv::DECOMP_LU) || !cv::checkRange(fitted))
    throw std::runtime_error("Cannot fit RBF interpolation: degenerate point neighborhood");
  knots = std::move(centered);
  origin = center;
  coefficients = std::move(fitted);
}

cv::Point2f RBFInterpolator::interpolate(cv::Point2f point) const {
  if (coefficients.empty()) throw std::logic_error("RBF interpolation has not been fitted");
  if (!finite(point)) throw std::invalid_argument("RBF query coordinates must be finite");
  const auto query = cv::Point2d(point) - origin;
  const int count = static_cast<int>(knots.size());
  cv::Vec2d value;
  for (int axis = 0; axis < 2; ++axis) {
    double result = coefficients.at<double>(count, axis) +
        query.x * coefficients.at<double>(count + 1, axis) +
        query.y * coefficients.at<double>(count + 2, axis);
    for (int index = 0; index < count; ++index)
      result += gaussian(knots[index], query) * coefficients.at<double>(index, axis);
    value[axis] = result;
  }
  const cv::Point2f result(static_cast<float>(value[0]), static_cast<float>(value[1]));
  if (!finite(result)) throw std::runtime_error("RBF interpolation produced non-finite coordinates");
  return result;
}
