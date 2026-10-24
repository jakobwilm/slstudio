#include "cvtools.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>

namespace cvtools {
namespace {
std::vector<std::size_t> findNearestNeighborsAngleSorted(const cv::KeyPoint &queryPoint,
                                                        const std::vector<cv::KeyPoint> &searchPoints,
                                                        std::size_t count) {
  struct Neighbor {
    std::size_t index;
    float distanceSquared;
    float angle;
  };
  std::vector<Neighbor> neighbors;
  for (std::size_t index = 0; index < searchPoints.size(); ++index) {
    if (queryPoint.pt == searchPoints[index].pt) continue;
    const auto distanceSquared = static_cast<float>(
        cv::norm(cv::Vec2f(queryPoint.pt), cv::Vec2f(searchPoints[index].pt), cv::NORM_L2SQR));
    const Neighbor neighbor{index, distanceSquared, 0};
    if (neighbors.size() < count)
      neighbors.push_back(neighbor);
    else if (neighbor.distanceSquared < neighbors.back().distanceSquared)
      neighbors.back() = neighbor;
    std::sort(neighbors.begin(), neighbors.end(), [](const Neighbor &first, const Neighbor &second) {
      return first.distanceSquared < second.distanceSquared;
    });
  }
  for (auto &neighbor : neighbors) {
    const auto delta = searchPoints[neighbor.index].pt - queryPoint.pt;
    neighbor.angle = std::atan2(delta.x, delta.y);
  }
  std::sort(neighbors.begin(), neighbors.end(), [](const Neighbor &first, const Neighbor &second) {
    return first.angle < second.angle;
  });
  std::vector<std::size_t> indices;
  indices.reserve(neighbors.size());
  for (const auto &neighbor : neighbors) indices.push_back(neighbor.index);
  return indices;
}
}

bool findPartialCirclesGrid(const cv::Mat &im, std::vector<cv::Point2f> &q,
                            std::vector<cv::Point3f> &Q,
                            const float circleSpacing) {
  q.clear();
  Q.clear();
  if (im.empty() || im.type() != CV_8UC1 || !std::isfinite(circleSpacing) || circleSpacing <= 0)
    throw std::invalid_argument("Partial circle detection requires a Mono8 image and positive spacing");
  cv::SimpleBlobDetector::Params parameters;
  parameters.filterByArea = true;
  parameters.filterByColor = true;
  parameters.filterByCircularity = true;
  parameters.filterByInertia = true;
  parameters.filterByConvexity = true;
  parameters.minArea = 25;
  parameters.maxArea = std::max(1000.0f, static_cast<float>(im.total()) * 0.02f);
  parameters.minCircularity = 0.65f;
  parameters.minInertiaRatio = 0.2f;
  parameters.minConvexity = 0.85f;
  std::vector<std::pair<cv::Point, cv::Point2f>> corrBest;
  for (const auto color : {0, 255}) {
    parameters.blobColor = static_cast<unsigned char>(color);
    std::vector<cv::KeyPoint> detected, keypoints;
    cv::SimpleBlobDetector::create(parameters)->detect(im, detected);
    for (const auto &keypoint : detected) {
      const float margin = keypoint.size / 2 + 3;
      if (keypoint.pt.x > margin && keypoint.pt.x < im.cols - margin &&
          keypoint.pt.y > margin && keypoint.pt.y < im.rows - margin)
        keypoints.push_back(keypoint);
    }
    if (keypoints.size() < 11) continue;
    for (const auto &anchor : keypoints) {
      const auto neighbors = findNearestNeighborsAngleSorted(anchor, keypoints, 4);
      std::vector<cv::Point2f> imagePoints{anchor.pt};
      for (const auto index : neighbors) imagePoints.push_back(keypoints[index].pt);
      const std::vector<cv::Point2f> gridPoints{{0, 0}, {-1, 0}, {0, 1}, {1, 0}, {0, -1}};
      const auto homography = cv::findHomography(imagePoints, gridPoints);
      if (homography.empty() || !cv::checkRange(homography)) continue;
      std::vector<cv::Point2f> fitted;
      cv::perspectiveTransform(imagePoints, fitted, homography);
      if (cv::norm(fitted, gridPoints) > 0.1) continue;
      std::vector<cv::Point2f> points;
      for (const auto &keypoint : keypoints) points.push_back(keypoint.pt);
      std::vector<cv::Point2f> lattice;
      cv::perspectiveTransform(points, lattice, homography);
      // Match detections to the lattice directly, avoiding unbounded image-corner extrapolation.
      std::map<std::pair<int, int>, std::pair<cv::Point2f, double>> unique;
      for (std::size_t i = 0; i < lattice.size(); ++i) {
        const auto p = lattice[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 10000 || std::abs(p.y) > 10000)
          continue;
        const int x = cvRound(p.x), y = cvRound(p.y);
        const double error = cv::norm(p - cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
        if (error > 0.15) continue;
        const auto key = std::make_pair(x, y);
        const auto previous = unique.find(key);
        if (previous == unique.end() || error < previous->second.second)
          unique[key] = {points[i], error};
      }
      if (unique.size() <= corrBest.size()) continue;
      std::vector<cv::Point> indices;
      for (const auto &entry : unique) indices.emplace_back(entry.first.first, entry.first.second);
      const auto bounds = cv::boundingRect(indices);
      if (bounds.width < 3 || bounds.height < 3) continue;
      corrBest.clear();
      for (const auto &entry : unique)
        corrBest.emplace_back(cv::Point(entry.first.first, entry.first.second), entry.second.first);
    }
  }
  if (corrBest.size() < 11) return false;
  std::vector<cv::Point> idealPoints(corrBest.size());
  q.resize(corrBest.size());
  for (std::size_t i = 0; i < corrBest.size(); ++i) {
    idealPoints[i] = corrBest[i].first;
    q[i] = corrBest[i].second;
  }
  const auto bounds = cv::boundingRect(idealPoints);
  Q.resize(corrBest.size());
  for (std::size_t i = 0; i < corrBest.size(); ++i)
    Q[i] = circleSpacing * cv::Point3f(idealPoints[i].x - bounds.x, idealPoints[i].y - bounds.y, 0);
  return true;
}

} // namespace cvtools
