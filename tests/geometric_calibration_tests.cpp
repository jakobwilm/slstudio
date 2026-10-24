#include "GeometricCalibration.h"
#include "FrameSequence.h"
#include "CodecCalibration.h"
#include "RBFInterpolator.h"
#include "cvtools.h"
#include "pstools.h"
#include <set>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

template <typename Function> void rejects(Function function) {
  bool rejected = false;
  try { function(); } catch (const std::exception &) { rejected = true; }
  require(rejected, "Invalid calibration input was accepted");
}

static void testCalibrationCodec() {
  constexpr unsigned int columns = 127, rows = 93;
  const float pi = static_cast<float>(CV_PI);
  for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
    EncoderCalibration calibrationEncoder(columns, rows, direction);
    EncoderPhaseShift encoder(columns, rows, direction);
    DecoderCalibration calibrationDecoder(columns, rows, direction);
    DecoderPhaseShift decoder(columns, rows, direction);
    const unsigned int count = direction == CodecDirBoth ? 12 : 6;
    require(calibrationEncoder.getNPatterns() == count && encoder.getNPatterns() == count &&
        calibrationDecoder.getNPatterns() == count, "Geometric Calibration pattern counts changed");
    std::vector<cv::Mat> frames;
    unsigned int index = 0;
    for (const auto axis : {CodecDirHorizontal, CodecDirVertical}) {
      if (!(direction & axis)) continue;
      const auto length = axis == CodecDirHorizontal ? columns : rows;
      for (const auto periods : {16, 1}) {
        for (unsigned int phase = 0; phase < 3; ++phase, ++index) {
          cv::Mat expected = pstools::computePhaseVector(length, 2.0 * pi / 3.0 * phase,
                                                        static_cast<float>(length) / periods);
          if (axis == CodecDirHorizontal) expected = expected.t();
          const auto actual = calibrationEncoder.getEncodingPattern(index);
          require(actual.size() == expected.size() && actual.type() == expected.type() &&
              cv::norm(actual, expected, cv::NORM_INF) == 0 &&
              cv::norm(actual, encoder.getEncodingPattern(index), cv::NORM_INF) == 0,
              "Geometric Calibration pattern pixels or axis/frequency/phase ordering changed");
          cv::Mat gray;
          cv::cvtColor(cv::repeat(actual, rows / actual.rows, columns / actual.cols), gray, cv::COLOR_RGB2GRAY);
          gray.convertTo(gray, CV_8U, 0.1);
          frames.push_back(gray);
        }
      }
    }
    cv::Mat up, vp, mask, shading, reconstructionUp, reconstructionVp, reconstructionMask, reconstructionShading;
    calibrationDecoder.setMinimumModulation(255);
    calibrationDecoder.decodeFrames(frames, up, vp, mask, shading);
    decoder.decodeFrames(frames, reconstructionUp, reconstructionVp, reconstructionMask, reconstructionShading);
    const auto expectedShading = pstools::getMagnitude(frames[0], frames[1], frames[2]);
    require(cv::norm(shading, expectedShading, cv::NORM_INF) == 0 &&
        cv::norm(mask, expectedShading > 10, cv::NORM_INF) == 0 &&
        cv::countNonZero(mask) > cv::countNonZero(reconstructionMask) &&
        cv::countNonZero(reconstructionMask) == 0, "Geometric Calibration and reconstruction thresholds were conflated");
    index = 0;
    for (const auto axis : {CodecDirHorizontal, CodecDirVertical}) {
      if (!(direction & axis)) continue;
      const auto phase = pstools::getPhase(frames[index], frames[index + 1], frames[index + 2]);
      const auto cue = pstools::getPhase(frames[index + 3], frames[index + 4], frames[index + 5]);
      cv::Mat expected = pstools::unwrapWithCue(phase, cue, 16);
      expected *= (axis == CodecDirHorizontal ? columns : rows) / (2 * pi);
      require(cv::norm(axis == CodecDirHorizontal ? up : vp, expected, cv::NORM_INF) < 1e-4,
              "Shared decoding differs from the original calibration algorithm");
      index += 6;
    }
    decoder.setMinimumModulation(10);
    decoder.decodeFrames(frames, reconstructionUp, reconstructionVp, reconstructionMask, reconstructionShading);
    require(cv::norm(mask, reconstructionMask, cv::NORM_INF) == 0,
            "Reconstruction modulation option no longer matches the selected threshold");
  }
}

void testRBFInterpolation() {
  rejects([] { RBFInterpolator invalid(-1); });
  rejects([] { RBFInterpolator invalid(std::numeric_limits<double>::quiet_NaN()); });
  RBFInterpolator interpolator;
  rejects([&] { interpolator.interpolate({0, 0}); });
  rejects([&] { interpolator.setDataPoints({{0, 0}, {1, 0}}, {{0, 0}, {1, 0}}); });
  rejects([&] { interpolator.setDataPoints({{0, 0}, {1, 0}, {0, 1}}, {{0, 0}}); });
  rejects([&] { interpolator.setDataPoints({{0, 0}, {1, 0}, {2, 0}}, {{0, 0}, {1, 0}, {2, 0}}); });
  rejects([&] {
    interpolator.setDataPoints({{0, 0}, {1, 0}, {0, 1}},
        {{0, 0}, {1, 0}, {0, std::numeric_limits<float>::quiet_NaN()}});
  });
  std::vector<cv::Point2f> points, values;
  auto affine = [](cv::Point2f point) {
    return cv::Point2f(2 * point.x + 0.5f * point.y + 3, -0.25f * point.x + point.y - 7);
  };
  for (int row = -2; row <= 2; ++row)
    for (int col = -2; col <= 2; ++col) {
      const cv::Point2f point(1000.0f + col, 2000.0f + row);
      points.push_back(point);
      values.push_back(affine(point));
    }
  interpolator.setDataPoints(points, values);
  const cv::Point2f query(1000.3f, 1999.6f);
  require(cv::norm(interpolator.interpolate(query) - affine(query)) < 0.001,
          "Regularized RBF did not reproduce an affine camera-to-projector mapping");
  rejects([&] { interpolator.interpolate({std::numeric_limits<float>::infinity(), 0}); });

  auto curved = [](cv::Point2f point) {
    return cv::Point2f(point.x + 0.15f * point.x * point.x + 0.1f * std::sin(point.y),
                       point.y + 0.2f * point.x * point.y);
  };
  points.clear();
  values.clear();
  for (int row = -2; row <= 2; ++row)
    for (int col = -2; col <= 2; ++col) {
      const cv::Point2f point(static_cast<float>(col), static_cast<float>(row));
      points.push_back(point);
      values.push_back(curved(point));
    }
  RBFInterpolator exact(0);
  exact.setDataPoints(points, values);
  for (std::size_t index = 0; index < points.size(); ++index)
    require(cv::norm(exact.interpolate(points[index]) - values[index]) < 1e-5,
            "Unregularized RBF did not interpolate nonlinear knot values");
  require(cv::norm(exact.interpolate({0.3f, -0.4f}) - curved({0.3f, -0.4f})) < 0.03,
          "RBF interpolation between nonlinear samples is inaccurate");
}

void testPartialGrid() {
  const float spacing = 4;
  cv::Mat1b image(330, 440, std::uint8_t(0));
  std::vector<cv::Point2f> expected;
  for (int row = 0; row < 7; ++row)
    for (int col = 0; col < 9; ++col) {
      if ((row == 0 && col < 3) || (row == 3 && col == 4)) continue;
      const cv::Point2f point(50.0f + col * 36, 45.0f + row * 36);
      expected.push_back(point);
      cv::circle(image, point, 7, cv::Scalar(255), -1, cv::LINE_AA);
    }
  const cv::Matx33f transform(1, 0.08f, 3, 0.04f, 1, 2, 0.00012f, -0.0001f, 1);
  cv::warpPerspective(image.clone(), image, transform, image.size());
  cv::perspectiveTransform(expected, expected, transform);
  for (const bool inverted : {false, true}) {
    std::vector<cv::Point2f> detected;
    std::vector<cv::Point3f> object;
    const cv::Mat input = inverted ? cv::Mat(255 - image) : cv::Mat(image);
    require(cvtools::findPartialCirclesGrid(input, detected, object, spacing), "Partial circle grid was not detected");
    require(detected.size() == expected.size() && object.size() == expected.size(),
            "Partial grid lost or duplicated correspondences");
    std::set<std::pair<int, int>> unique;
    for (std::size_t i = 0; i < detected.size(); ++i) {
      double best = 1000;
      for (const auto p : expected) best = std::min(best, cv::norm(p - detected[i]));
      require(best < 0.5, "Partial grid corner is not a true circle center");
      unique.emplace(cvRound(object[i].x / spacing), cvRound(object[i].y / spacing));
    }
    require(unique.size() == object.size(), "Partial grid contains duplicate lattice coordinates");
  }
  std::vector<cv::Point2f> detected{{1, 2}};
  std::vector<cv::Point3f> object{{1, 2, 3}};
  require(!cvtools::findPartialCirclesGrid(cv::Mat1b(80, 80, std::uint8_t(0)), detected, object, spacing) &&
      detected.empty() && object.empty(), "Failed grid detection left stale correspondences");
}

void testPartialCalibration() {
  const cv::Size cameraSize(480, 360), projectorSize(640, 480);
  const cv::Matx33d camera(500, 0, 240, 0, 500, 180, 0, 0, 1);
  const cv::Matx33d projector(480, 0, 320, 0, 480, 240, 0, 0, 1);
  const cv::Vec3d baseline(-0.06, -0.025, 0.002);
  constexpr double spacing = 0.014;
  std::vector<std::vector<cv::Mat>> poses;
  for (int pose = 0; pose < 6; ++pose) {
    cv::Matx33d rotation;
    cv::Rodrigues(cv::Vec3d(-0.25 + 0.09 * pose, 0.18 - 0.065 * pose, pose * 0.025), rotation);
    const cv::Vec3d translation(-0.06 + 0.003 * pose, -0.045 + 0.002 * pose, 0.36 + 0.025 * pose);
    const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
    std::vector<cv::Mat> frames;
    for (int index = 0; index < 12; ++index) frames.emplace_back(cameraSize, CV_8UC1);
    for (int row = 0; row < cameraSize.height; ++row) {
      for (int col = 0; col < cameraSize.width; ++col) {
        const cv::Vec3d ray((col - camera(0, 2)) / camera(0, 0), (row - camera(1, 2)) / camera(1, 1), 1);
        const auto point = ray * (normal.dot(translation) / normal.dot(ray));
        const auto board = rotation.t() * (point - translation);
        const int x = cvRound(board[0] / spacing), y = cvRound(board[1] / spacing);
        const bool missing = (y == 0 && x < 3) || (x == 4 && y == 3);
        const bool circle = x >= 0 && x < 9 && y >= 0 && y < 7 && !missing &&
            std::hypot(board[0] - x * spacing, board[1] - y * spacing) < 0.004;
        const double reflectance = circle ? 0.95 : 0.2;
        const auto p = point + baseline;
        const double u = projector(0, 0) * p[0] / p[2] + projector(0, 2);
        const double v = projector(1, 1) * p[1] / p[2] + projector(1, 2);
        for (int index = 0; index < 12; ++index) {
          const double position = index < 6 ? u / projectorSize.width : v / projectorSize.height;
          const double cycles = index % 6 < 3 ? 16 : 1;
          const double phase = 2 * CV_PI * (cycles * position - (index % 3) / 3.0);
          frames[index].at<std::uint8_t>(row, col) =
              cv::saturate_cast<std::uint8_t>(5 + reflectance * (127 + 110 * std::cos(phase)));
        }
      }
    }
    poses.push_back(std::move(frames));
  }
  // A rejected pose between accepted ones exercises diagnostic-to-view indexing.
  poses.insert(poses.begin() + 2, std::vector<cv::Mat>(12, cv::Mat1b(cameraSize, std::uint8_t(40))));
  GeometricCalibrationSettings settings;
  settings.circles = true;
  settings.partialCircles = true;
  settings.spacing = spacing;
  settings.cameraFlags = settings.projectorFlags = cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 |
      cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;
  std::vector<GeometricCalibrationPoseDiagnostics> diagnostics;
  const auto result = calibrateGeometry(poses, projectorSize, settings, &diagnostics);
  require(diagnostics.size() == poses.size() && !diagnostics[2].accepted &&
      diagnostics[2].cameraResiduals.empty(), "Rejected partial pose diagnostics were misindexed");
  std::size_t accepted = 0;
  for (const auto &pose : diagnostics) {
    if (!pose.accepted) continue;
    ++accepted;
    require(pose.correspondenceCount >= 11 && pose.cameraResiduals.size() == pose.correspondenceCount &&
        pose.projectorResiduals.size() == pose.correspondenceCount &&
        std::isfinite(pose.cameraRms) && std::isfinite(pose.projectorRms),
        "Partial calibration lacks reliable correspondences or residual diagnostics");
  }
  require(accepted >= 5 && result.cam_error < 0.5 && result.proj_error < 0.5 && result.stereo_error < 0.5,
          "Synthetic partial-grid calibration accuracy is insufficient");
  require(std::abs(cv::norm(result.Tp) - cv::norm(baseline)) < 0.005,
          "Partial grid calibration did not preserve physical spacing");
}

void testCalibrationPersistence() {
  const auto path = std::filesystem::temp_directory_path() /
      ("slstudio-calibration-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".xml");
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code error;
      std::filesystem::remove(path, error);
      if (error) std::cerr << "Cannot remove calibration test file: " << error.message() << '\n';
    }
  } cleanup{path};
  GeometricCalibrationData calibration;
  calibration.cam_stdint = {0.1, 0.2, 0.3};
  calibration.proj_stdint = {0.2, 0.3, 0.4};
  calibration.cam_pve = {0.25};
  calibration.proj_pve = {0.35};
  require(calibration.save(path), "Calibration save failed");
  GeometricCalibrationData restored;
  require(restored.load(path), "Calibration load failed");
  require(restored.frameWidth == calibration.frameWidth && restored.cam_stdint == calibration.cam_stdint &&
      restored.proj_stdint == calibration.proj_stdint && restored.cam_pve == calibration.cam_pve &&
      restored.proj_pve == calibration.proj_pve, "Calibration diagnostics did not survive XML round trip");
  {
    cv::FileStorage malformed(path.string(), cv::FileStorage::WRITE);
    malformed << "Kc" << cv::Mat::eye(2, 2, CV_32F);
  }
  const auto originalWidth = restored.frameWidth;
  require(!restored.load(path) && restored.frameWidth == originalWidth, "Failed calibration load changed its state");
  rejects([] { calibrateGeometry({}, cv::Size(1024, 768)); });
}

int main() {
  try {
    testCalibrationPersistence();
    testCalibrationCodec();
    testRBFInterpolation();
    testPartialGrid();
    testPartialCalibration();
    const std::filesystem::path source(SLSTUDIO_SOURCE_DIR);
    std::vector<std::vector<cv::Mat>> poses;
    for (int index = 1; index <= 6; ++index)
      poses.push_back(loadFrameSequence(source / "example_data/calibration" / ("calseq" + std::to_string(index))));
    GeometricCalibrationSettings settings;
    settings.boardCorners = cv::Size(9, 13);
    require(settings.interpolation == GeometricCalibrationInterpolation::LocalHomography,
            "Geometric Calibration no longer defaults to local homography");
    std::vector<GeometricCalibrationPoseDiagnostics> diagnostics;
    const auto result = calibrateGeometry(poses, cv::Size(1824, 1140), settings, &diagnostics);
    require(diagnostics.size() == poses.size(), "Missing per-pose calibration diagnostics");
    std::size_t acceptedPoses = 0;
    for (const auto &pose : diagnostics) {
      require(!pose.reason.empty(), "Missing calibration pose status");
      if (pose.accepted) {
        ++acceptedPoses;
        require(pose.corners.size() == 117 && pose.correspondenceCount >= 6,
                "Accepted calibration pose has invalid diagnostics");
        require(std::isfinite(pose.cameraRms) && std::isfinite(pose.projectorRms) &&
            pose.cameraResiduals.size() == pose.correspondenceCount &&
            pose.projectorResiduals.size() == pose.correspondenceCount &&
            pose.acceptedCorners.size() == pose.correspondenceCount,
            "Geometric Calibration residual diagnostics are missing or misaligned");
        double cameraSquared = 0, projectorSquared = 0;
        for (std::size_t i = 0; i < pose.correspondenceCount; ++i) {
          cameraSquared += pose.cameraResiduals[i].dot(pose.cameraResiduals[i]);
          projectorSquared += pose.projectorResiduals[i].dot(pose.projectorResiduals[i]);
        }
        require(std::abs(std::sqrt(cameraSquared / pose.correspondenceCount) - pose.cameraRms) < 1e-4 &&
            std::abs(std::sqrt(projectorSquared / pose.correspondenceCount) - pose.projectorRms) < 1e-4,
            "Reported per-view RMS does not match residual vectors");
      }
    }
    require(acceptedPoses >= 3, "Too few accepted calibration poses");
    require(result.cam_pve.size() == acceptedPoses && result.proj_pve.size() == acceptedPoses &&
        !result.cam_stdint.empty() && !result.proj_stdint.empty(), "Geometric Calibration uncertainty outputs were not populated");
          require(std::isfinite(result.cam_error) && result.cam_error < 1.0,
            "Example camera calibration reprojection error is invalid");
          require(std::isfinite(result.proj_error) && result.proj_error < 1.0,
            "Example projector calibration reprojection error is invalid");
          require(std::isfinite(result.stereo_error) && result.stereo_error < 1.0,
            "Example calibration reprojection error is invalid");
    std::cout << "Geometric Calibration RMS: " << result.stereo_error << " px\n";
    auto rbfSettings = settings;
    rbfSettings.interpolation = GeometricCalibrationInterpolation::RBF;
    std::vector<GeometricCalibrationPoseDiagnostics> rbfDiagnostics;
    const auto rbf = calibrateGeometry(poses, cv::Size(1824, 1140), rbfSettings, &rbfDiagnostics);
    require(rbfDiagnostics.size() == poses.size() &&
        std::count_if(rbfDiagnostics.begin(), rbfDiagnostics.end(),
            [](const auto &pose) { return pose.accepted; }) >= 3,
        "RBF calibration did not produce enough accepted poses");
    require(std::isfinite(rbf.cam_error) && rbf.cam_error < 1.0 &&
        std::isfinite(rbf.proj_error) && rbf.proj_error < 1.0 &&
        std::isfinite(rbf.stereo_error) && rbf.stereo_error < 1.0,
        "RBF calibration reprojection error is invalid");
    std::cout << "RBF calibration RMS: " << rbf.stereo_error << " px\n";
    rbfSettings.interpolation = static_cast<GeometricCalibrationInterpolation>(-1);
    bool rejectedInterpolation = false;
    try { calibrateGeometry(poses, cv::Size(1824, 1140), rbfSettings); }
    catch (const std::invalid_argument &) { rejectedInterpolation = true; }
    require(rejectedInterpolation, "Unknown calibration interpolation method was accepted");
    settings.cameraFlags = cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 |
        cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;
    settings.projectorFlags = settings.cameraFlags;
    const auto fixed = calibrateGeometry(poses, cv::Size(1824, 1140), settings);
    require(cv::norm(fixed.kc, cv::NORM_INF) == 0 && cv::norm(fixed.kp, cv::NORM_INF) == 0,
        "Disabled camera/projector distortion coefficients were estimated");
    require(std::abs(fixed.Kc(0, 2) - (fixed.frameWidth - 1) / 2.0) < 1e-6 &&
        std::abs(fixed.Kc(1, 2) - (fixed.frameHeight - 1) / 2.0) < 1e-6 &&
        std::abs(fixed.Kp(0, 2) - (fixed.screenResX - 1) / 2.0) < 1e-6 &&
        std::abs(fixed.Kp(1, 2) - (fixed.screenResY - 1) / 2.0) < 1e-6,
        "Disabled camera/projector principal points were not fixed at the image center");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}