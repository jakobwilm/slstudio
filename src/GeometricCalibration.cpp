#include "GeometricCalibration.h"
#include "CodecCalibration.h"
#include "RBFInterpolator.h"
#include "cvtools.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <stdexcept>

// Calibration references:
//
// Wilm, Jakob, Oline V. Olesen, and Rasmus Larsen.
// "Accurate and simple calibration of DLP projector systems."
// Emerging Digital Micromirror Device Based Systems and Applications VI.
// Vol. 8979. SPIE, 2014.
//
// Moreno, Daniel, and Gabriel Taubin.
// "Simple, accurate, and robust projector-camera calibration."
// 2012 Second International Conference on 3D Imaging, Modeling, Processing,
// Visualization & Transmission. IEEE, 2012.
GeometricCalibrationData calibrateGeometry(const std::vector<std::vector<cv::Mat>> &sequences,
                                cv::Size projectorSize, const GeometricCalibrationSettings &settings,
                                std::vector<GeometricCalibrationPoseDiagnostics> *poseDiagnostics) {
  if (poseDiagnostics) poseDiagnostics->clear();
  if (settings.interpolation != GeometricCalibrationInterpolation::LocalHomography &&
      settings.interpolation != GeometricCalibrationInterpolation::RBF)
    throw std::invalid_argument("Unknown calibration interpolation method");
  if (sequences.size() < 3 || projectorSize.width < 1 || projectorSize.height < 1 ||
      (!settings.partialCircles && (settings.boardCorners.width < 2 || settings.boardCorners.height < 2)) ||
      (settings.partialCircles && !settings.circles) ||
      !std::isfinite(settings.spacing) || settings.spacing <= 0 ||
      settings.neighborhoodRadius < 2 || settings.neighborhoodRadius > 100)
    throw std::invalid_argument("Geometric Calibration needs at least three poses and valid board/output dimensions");

  DecoderCalibration decoder(projectorSize.width, projectorSize.height, CodecDirBoth);
  std::vector<std::vector<cv::Point2f>> cameraPoints, projectorPoints;
  std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<std::size_t> acceptedPoses;
  cv::Size cameraSize;
  std::size_t detectedBoards = 0, bestCorrespondences = 0;
  for (const auto &frames : sequences) {
    if (frames.size() != decoder.getNPatterns())
      throw std::invalid_argument("Geometric Calibration requires twelve horizontal/vertical phase-shift frames per pose");
    if (cameraSize.empty()) cameraSize = frames.front().size();
    for (const auto &frame : frames) {
      if (frame.empty() || frame.type() != CV_8UC1 || frame.size() != cameraSize)
        throw std::invalid_argument("Geometric Calibration camera frames must be matching grayscale images");
    }
    cv::Mat up, vp, mask, shading;
    decoder.decodeFrames(frames, up, vp, mask, shading);
    cv::Mat boardImage = cv::Mat::zeros(cameraSize, CV_32F);
    for (const auto &frame : frames) {
      cv::Mat floating;
      frame.convertTo(floating, CV_32F);
      boardImage += floating;
    }
    boardImage.convertTo(boardImage, CV_8U, 1.0 / frames.size());
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> boardPoints;
    bool found;
    if (settings.partialCircles) {
      found = cvtools::findPartialCirclesGrid(boardImage, corners, boardPoints, settings.spacing);
    } else if (settings.circles) {
      found = cv::findCirclesGrid(boardImage, settings.boardCorners, corners, cv::CALIB_CB_SYMMETRIC_GRID);
    } else {
      found = cv::findChessboardCorners(shading, settings.boardCorners, corners);
      if (!found)
        found = cv::findChessboardCornersSB(boardImage, settings.boardCorners, corners,
                                            cv::CALIB_CB_NORMALIZE_IMAGE);
    }
    if (found && !settings.partialCircles)
      for (std::size_t index = 0; index < corners.size(); ++index)
        boardPoints.emplace_back((index % settings.boardCorners.width) * settings.spacing,
                                 (index / settings.boardCorners.width) * settings.spacing, 0);
    if (poseDiagnostics) {
      GeometricCalibrationPoseDiagnostics diagnostics;
      if (found) diagnostics.corners = corners;
      diagnostics.reason = found ? "Too few reliable projector correspondences" : "Board not detected";
      poseDiagnostics->push_back(std::move(diagnostics));
    }
    if (!found) continue;
    ++detectedBoards;
    std::vector<cv::Point2f> acceptedCamera, acceptedProjector;
    std::vector<cv::Point3f> acceptedObject;
    for (std::size_t index = 0; index < corners.size(); ++index) {
      const auto corner = corners[index];
      const int radius = settings.neighborhoodRadius;
      const auto roi = cv::Rect(cvRound(corner.x) - radius, cvRound(corner.y) - radius,
                                 2 * radius + 1, 2 * radius + 1) & cv::Rect({}, cameraSize);
      std::vector<cv::Point2f> neighborhood, projected;
      for (int row = roi.y; row < roi.y + roi.height; ++row) {
        for (int col = roi.x; col < roi.x + roi.width; ++col) {
          const float deltaX = col - corner.x, deltaY = row - corner.y;
          if (deltaX * deltaX + deltaY * deltaY >= radius * radius) continue;
          if (!mask.at<unsigned char>(row, col)) continue;
          const float horizontal = up.at<float>(row, col), vertical = vp.at<float>(row, col);
          if (!std::isfinite(horizontal) || !std::isfinite(vertical)) continue;
          neighborhood.emplace_back(col, row);
          projected.emplace_back(horizontal, vertical);
        }
      }
      if (neighborhood.size() < 10) continue;
      cv::Point2f mapped;
      if (settings.interpolation == GeometricCalibrationInterpolation::RBF) {
        RBFInterpolator interpolator;
        interpolator.setDataPoints(neighborhood, projected);
        mapped = interpolator.interpolate(corner);
      } else {
        const auto homography = cv::findHomography(neighborhood, projected, cv::LMEDS);
        if (homography.empty()) continue;
        std::vector<cv::Point2f> fitted;
        cv::perspectiveTransform(neighborhood, fitted, homography);
        std::vector<double> residuals;
        residuals.reserve(fitted.size());
        for (std::size_t sample = 0; sample < fitted.size(); ++sample) {
          const auto delta = fitted[sample] - projected[sample];
          residuals.push_back(delta.dot(delta));
        }
        if (!std::all_of(residuals.begin(), residuals.end(),
            [](double residual) { return std::isfinite(residual); })) continue;
        auto sortedResiduals = residuals;
        const auto median = sortedResiduals.begin() + sortedResiduals.size() / 2;
        std::nth_element(sortedResiduals.begin(), median, sortedResiduals.end());
        // Preserve LMedS's adaptive inlier scale; OpenCV 4.14 changed its returned mask to a fixed threshold.
        const double sigma = std::max(0.001, 2.5 * 1.4826 *
            (1 + 5.0 / (residuals.size() - 4)) * std::sqrt(*median));
        const auto inliers = std::count_if(residuals.begin(), residuals.end(),
            [sigma](double residual) { return residual <= sigma * sigma; });
        if (inliers < 0.9 * neighborhood.size()) continue;
        std::vector<cv::Point2f> transformed;
        cv::perspectiveTransform(std::vector<cv::Point2f>{corner}, transformed, homography);
        mapped = transformed.front();
      }
      if (!std::isfinite(mapped.x) || !std::isfinite(mapped.y) ||
          mapped.x < 0 || mapped.y < 0 ||
          mapped.x >= projectorSize.width || mapped.y >= projectorSize.height) continue;
      acceptedCamera.push_back(corner);
      acceptedProjector.push_back(mapped);
      acceptedObject.push_back(boardPoints[index]);
    }
    bestCorrespondences = std::max(bestCorrespondences, acceptedObject.size());
    if (poseDiagnostics) {
      auto &diagnostics = poseDiagnostics->back();
      diagnostics.correspondenceCount = acceptedObject.size();
      diagnostics.accepted = acceptedObject.size() >= 6;
      diagnostics.acceptedCorners = acceptedCamera;
      diagnostics.projectorCorners = acceptedProjector;
      if (diagnostics.accepted) diagnostics.reason = "Accepted";
    }
    if (acceptedObject.size() < 6) continue;
    cameraPoints.push_back(std::move(acceptedCamera));
    projectorPoints.push_back(std::move(acceptedProjector));
    objectPoints.push_back(std::move(acceptedObject));
    acceptedPoses.push_back(poseDiagnostics ? poseDiagnostics->size() - 1 : 0);
  }
  if (objectPoints.size() < 3)
    throw std::runtime_error("Fewer than three usable calibration poses: " +
      std::to_string(detectedBoards) + " boards detected, " +
      std::to_string(objectPoints.size()) + " usable, best pose has " +
      std::to_string(bestCorrespondences) + " projector correspondences");
  GeometricCalibrationData result;
  cv::Mat cameraMatrix, cameraDistortion, projectorMatrix, projectorDistortion;
  std::vector<cv::Mat> cameraRotations, cameraTranslations, projectorRotations, projectorTranslations;
  result.cam_error = cv::calibrateCamera(objectPoints, cameraPoints, cameraSize,
      cameraMatrix, cameraDistortion, cameraRotations, cameraTranslations,
      result.cam_stdint, result.cam_stdext, result.cam_pve, settings.cameraFlags);
  result.proj_error = cv::calibrateCamera(objectPoints, projectorPoints, projectorSize,
      projectorMatrix, projectorDistortion, projectorRotations, projectorTranslations,
      result.proj_stdint, result.proj_stdext, result.proj_pve, settings.projectorFlags);
  cv::Mat rotation, translation, essential, fundamental;
  result.stereo_error = cv::stereoCalibrate(objectPoints, cameraPoints, projectorPoints,
      cameraMatrix, cameraDistortion, projectorMatrix, projectorDistortion, cameraSize,
      rotation, translation, essential, fundamental, cv::CALIB_FIX_INTRINSIC);
  result.Kc = cameraMatrix;
  result.kc = cameraDistortion;
  result.Kp = projectorMatrix;
  result.kp = projectorDistortion;
  result.Rp = rotation;
  result.Tp = translation;
  result.frameWidth = cameraSize.width;
  result.frameHeight = cameraSize.height;
  result.screenResX = projectorSize.width;
  result.screenResY = projectorSize.height;
  result.calibrationDateTime = std::to_string(std::time(nullptr));
  if (poseDiagnostics) {
    for (std::size_t pose = 0; pose < objectPoints.size(); ++pose) {
      auto &diagnostics = poseDiagnostics->at(acceptedPoses[pose]);
      diagnostics.cameraRms = result.cam_pve.at(pose);
      diagnostics.projectorRms = result.proj_pve.at(pose);
      std::vector<cv::Point2f> projectedCamera, projectedProjector;
      cv::projectPoints(objectPoints[pose], cameraRotations[pose], cameraTranslations[pose],
          cameraMatrix, cameraDistortion, projectedCamera);
      cv::projectPoints(objectPoints[pose], projectorRotations[pose], projectorTranslations[pose],
          projectorMatrix, projectorDistortion, projectedProjector);
      for (std::size_t index = 0; index < projectedCamera.size(); ++index) {
        diagnostics.cameraResiduals.push_back(cameraPoints[pose][index] - projectedCamera[index]);
        diagnostics.projectorResiduals.push_back(projectorPoints[pose][index] - projectedProjector[index]);
      }
    }
  }
  return result;
}

GeometricCalibrationData calibrateGeometry(const std::vector<std::vector<cv::Mat>> &sequences,
                                cv::Size projectorSize, const GeometricCalibrationSettings &settings) {
  return calibrateGeometry(sequences, projectorSize, settings, nullptr);
}