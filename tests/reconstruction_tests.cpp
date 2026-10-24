#include "GeometricCalibration.h"
#include "DecoderWorker.h"
#include "TriangulatorWorker.h"
#include "FrameSequence.h"
#include <cstring>

#include <cmath>
#include <iostream>
#include <random>

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function> void rejects(Function function) {
  bool rejected = false;
  try { function(); } catch (const std::exception &) { rejected = true; }
  require(rejected, "Invalid reconstruction settings was accepted");
}

GeometricCalibrationData syntheticCalibration() {
  GeometricCalibrationData cal;
  cal.frameWidth = 32;
  cal.frameHeight = 24;
  cal.screenResX = 64;
  cal.screenResY = 48;
  cal.Kc = {80, 0, 16, 0, 80, 12, 0, 0, 1};
  cal.Kp = {75, 0, 32, 0, 78, 24, 0, 0, 1};
  cal.kp = {0.25f, -0.05f, 0.02f, -0.015f, 0.01f};
  cv::Rodrigues(cv::Vec3f(0.02f, 0.03f, -0.01f), cal.Rp);
  cal.Tp = {-0.08f, -0.06f, 0.005f};
  return cal;
}


struct PlaneProjection {
  cv::Mat1f up, vp;
  cv::Mat1b mask, shading;
};

PlaneProjection projectPlane(const GeometricCalibrationData &cal, float depth) {
  const cv::Size size(cal.frameWidth, cal.frameHeight);
  std::vector<cv::Point3f> points;
  for (int row = 0; row < size.height; ++row)
    for (int col = 0; col < size.width; ++col)
      points.emplace_back((col - cal.Kc(0, 2)) * depth / cal.Kc(0, 0),
                          (row - cal.Kc(1, 2)) * depth / cal.Kc(1, 1), depth);
  std::vector<cv::Point2f> projected;
  cv::Vec3d rotation;
  cv::Rodrigues(cv::Matx33d(cal.Rp), rotation);
  cv::projectPoints(points, rotation, cv::Vec3d(cal.Tp), cv::Matx33d(cal.Kp),
      cv::Vec<double, 5>(cal.kp), projected);
  PlaneProjection sequence{cv::Mat1f(size), cv::Mat1f(size),
      cv::Mat1b(size, std::uint8_t(255)), cv::Mat1b(size, std::uint8_t(150))};
  for (int row = 0; row < size.height; ++row)
    for (int col = 0; col < size.width; ++col) {
      sequence.up(row, col) = projected[row * size.width + col].x;
      sequence.vp(row, col) = projected[row * size.width + col].y;
    }
  return sequence;
}

void testTwoCoordinateTriangulation() {
  auto cal = syntheticCalibration();
  cal.kp = cv::Vec<float, 5>::all(0);
  for (const float depth : {0.2f, 0.4f, 2.0f}) {
    const auto sequence = projectPlane(cal, depth);
    const auto result = TriangulatorOrganized(cal).triangulate(
        sequence.up, sequence.vp, sequence.mask, sequence.shading);
    cv::Mat up = sequence.up, vp = sequence.vp, mask = sequence.mask, shading = sequence.shading, xyz;
    TriangulatorOrganized(cal).triangulate(up, vp, mask, shading, xyz);
    require(cv::countNonZero(mask) > sequence.mask.total() * 0.9,
            "Two-coordinate triangulation lost valid camera support");
    for (int row = 0; row < cal.frameHeight; ++row)
      for (int col = 0; col < cal.frameWidth; ++col) {
        if (!mask.at<std::uint8_t>(row, col)) {
          require(std::isnan(result.xyz(row, col)[2]), "Undistortion mask was ignored");
          continue;
        }
        const cv::Vec3f expected((col - cal.Kc(0, 2)) * depth / cal.Kc(0, 0),
                                 (row - cal.Kc(1, 2)) * depth / cal.Kc(1, 1), depth);
        if (!(cv::norm(result.xyz(row, col) - expected) < 2e-5))
          throw std::runtime_error("Two-coordinate triangulation failed at " +
              std::to_string(col) + "," + std::to_string(row) + " depth " + std::to_string(depth) +
              ": got " + std::to_string(result.xyz(row, col)[0]) + "," +
              std::to_string(result.xyz(row, col)[1]) + "," + std::to_string(result.xyz(row, col)[2]));
      }
    require(cv::norm(result.shading, sequence.shading, cv::NORM_INF) == 0,
            "Two-coordinate triangulation changed shading");
  }

  const auto sequence = projectPlane(cal, 0.4f);
  const TriangulatorOrganized triangulator(cal);
  const auto baseline = triangulator.triangulate(sequence.up, sequence.vp, sequence.mask, sequence.shading);
  for (const bool horizontal : {true, false}) {
    auto up = sequence.up.clone(), vp = sequence.vp.clone();
    (horizontal ? up : vp) += 0.25f;
    const auto changed = triangulator.triangulate(up, vp, sequence.mask, sequence.shading);
    require(std::isfinite(changed.xyz(12, 16)[2]) &&
        std::abs(changed.xyz(12, 16)[2] - baseline.xyz(12, 16)[2]) > 1e-4,
        "Two-coordinate triangulation ignored a projector coordinate");
  }

  auto noisyUp = sequence.up.clone(), noisyVp = sequence.vp.clone();
  cv::RNG noise(42);
  cv::Mat1f perturbation(sequence.up.size());
  noise.fill(perturbation, cv::RNG::NORMAL, 0, 0.01);
  noisyUp += perturbation;
  noise.fill(perturbation, cv::RNG::NORMAL, 0, 0.01);
  noisyVp += perturbation;
  const auto noisy = triangulator.triangulate(noisyUp, noisyVp, sequence.mask, sequence.shading);
  for (int row = 0; row < noisy.xyz.rows; ++row)
    for (int col = 0; col < noisy.xyz.cols; ++col) {
      if (!std::isfinite(baseline.xyz(row, col)[2])) continue;
      require(std::isfinite(noisy.xyz(row, col)[2]) && std::abs(noisy.xyz(row, col)[2] - 0.4f) < 0.002,
              "Noisy two-coordinate triangulation exceeded its depth error bound");
    }

  auto up = sequence.up.clone(), vp = sequence.vp.clone();
  auto mask = sequence.mask.clone();
  up(10, 10) = NAN;
  vp(12, 12) = std::numeric_limits<float>::infinity();
  mask(cv::Rect(13, 13, 3, 3)).setTo(0);
  const auto invalid = triangulator.triangulate(up, vp, mask, sequence.shading);
  for (const auto pixel : {cv::Point(10, 10), cv::Point(12, 12), cv::Point(14, 14)})
    for (int axis = 0; axis < 3; ++axis)
      require(std::isnan(invalid.xyz(pixel)[axis]), "Invalid correspondence produced finite geometry");
  const cv::Mat1b emptyMask(sequence.mask.size(), std::uint8_t(0));
  const auto empty = triangulator.triangulate(sequence.up, sequence.vp, emptyMask, sequence.shading);
  require(empty.xyz.size() == sequence.up.size() && !cv::checkRange(empty.xyz, true, nullptr),
          "Empty mask did not preserve invalid organized output");
  for (int row = 0; row < empty.xyz.rows; ++row)
    for (int col = 0; col < empty.xyz.cols; ++col)
      require(std::isnan(empty.xyz(row, col)[2]), "Empty mask produced a valid point");

  cal.Rp = cv::Matx33f::eye();
  for (const auto &translation : {cv::Vec3f(-0.08f, 0, 0), cv::Vec3f(0, -0.06f, 0)}) {
    cal.Tp = translation;
    const auto plane = projectPlane(cal, 0.4f);
    const auto result = TriangulatorOrganized(cal).triangulate(plane.up, plane.vp, plane.mask, plane.shading);
    require(std::abs(result.xyz(12, 16)[2] - 0.4f) < 2e-5,
            "Axis-aligned baseline broke two-coordinate triangulation");
  }
  auto parallel = projectPlane(cal, 0.4f);
  parallel.up(12, 16) = cal.Kp(0, 2);
  parallel.vp(12, 16) = cal.Kp(1, 2);
  require(std::isnan(TriangulatorOrganized(cal).triangulate(
      parallel.up, parallel.vp, parallel.mask, parallel.shading).xyz(12, 16)[2]),
      "Parallel rays produced finite geometry");
  cal.Tp = cv::Vec3f::all(0);
  require(std::isnan(TriangulatorOrganized(cal).triangulate(
      sequence.up, sequence.vp, sequence.mask, sequence.shading).xyz(12, 16)[2]),
      "Zero baseline produced finite geometry");

  for (const bool behindCamera : {true, false}) {
    cal.Tp = {-0.08f, -0.06f, behindCamera ? 0.0f : -1.0f};
    const auto plane = projectPlane(cal, behindCamera ? -0.4f : 0.4f);
    const auto result = TriangulatorOrganized(cal).triangulate(plane.up, plane.vp, plane.mask, plane.shading);
    for (int row = 0; row < result.xyz.rows; ++row)
      for (int col = 0; col < result.xyz.cols; ++col)
        require(std::isnan(result.xyz(row, col)[2]), "Geometry behind an optical center was accepted");
  }
}

void testDistortion() {
  auto cal = syntheticCalibration();
  for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
    auto sequence = projectPlane(cal, 0.4f);
    if (direction == CodecDirHorizontal) sequence.vp.release();
    if (direction == CodecDirVertical) sequence.up.release();
    const auto baseline = TriangulatorOrganized(cal).triangulate(sequence.up, sequence.vp, sequence.mask, sequence.shading);
    const auto corrected = TriangulatorOrganized(cal, true).triangulate(sequence.up, sequence.vp, sequence.mask, sequence.shading);
    double originalError = 0, correctedError = 0;
    std::size_t validPixels = 0;
    for (int row = 0; row < cal.frameHeight; ++row)
      for (int col = 0; col < cal.frameWidth; ++col) {
        const auto p = corrected.xyz(row, col);
        if (!std::isfinite(baseline.xyz(row, col)[2])) {
          require(!std::isfinite(p[2]), "Refinement filled an invalid baseline pixel");
          continue;
        }
        ++validPixels;
        if (!std::isfinite(p[2]))
          throw std::runtime_error("Distortion refinement rejected synthetic direction " + std::to_string(direction) +
              " at " + std::to_string(row) + "," + std::to_string(col) +
              " baseline depth " + std::to_string(baseline.xyz(row, col)[2]) +
              " phase " + std::to_string(sequence.up.empty() ? sequence.vp(row, col) : sequence.up(row, col)));
        require(std::abs(p[2] - 0.4f) < 2e-5f, "Distortion refinement did not recover known depth");
        require(std::abs(p[0] - (col - 16) * p[2] / 80) < 1e-6f &&
            std::abs(p[1] - (row - 12) * p[2] / 80) < 1e-6f, "Refinement moved off the camera ray");
        originalError += std::abs(baseline.xyz(row, col)[2] - 0.4f);
        correctedError += std::abs(p[2] - 0.4f);
      }
    require(validPixels > sequence.mask.total() * 0.9 && originalError > 0.001 && correctedError < originalError * 0.01,
            "Projector distortion correction did not materially improve accuracy");
    ReconstructionSettings settings;
    settings.correctProjectorDistortion = true;
    const auto cloud = TriangulationStage(cal, settings).triangulateDense(
        sequence.up, sequence.vp, sequence.mask, sequence.shading);
    require(cloud->points.size() == validPixels, "Reconstruction stage ignored distortion correction");
    for (const auto &point : cloud->points)
      require(std::abs(point.z - 0.4f) < 2e-5f, "Reconstruction stage returned uncorrected depth");
  }
  cal.kp = cv::Vec<float, 5>::all(0);
  const auto sequence = projectPlane(cal, 0.4f);
  const auto baseline = TriangulatorOrganized(cal).triangulate(sequence.up, {}, sequence.mask, sequence.shading);
  const auto corrected = TriangulatorOrganized(cal, true).triangulate(sequence.up, {}, sequence.mask, sequence.shading);
  require(cv::norm(baseline.xyz, corrected.xyz, cv::NORM_INF) == 0, "Zero distortion changed existing geometry");
  cal.kp[0] = NAN;
  rejects([&] { TriangulatorOrganized invalid(cal, true); });
  cal = syntheticCalibration();
  auto invalid = projectPlane(cal, 0.4f);
  invalid.up(10, 10) = NAN;
  const auto rejected = TriangulatorOrganized(cal, true).triangulate(invalid.up, {}, invalid.mask, invalid.shading);
  require(!std::isfinite(rejected.xyz(10, 10)[2]), "Invalid phase produced success-shaped geometry");
}


void testCleanup() {
  ReconstructionSettings settings;
  require(!settings.correctProjectorDistortion && !settings.minimumComponentPixels && !settings.rejectDepthSpikes,
          "Reconstruction corrections and filters must be opt-in");
  cv::Mat1b mask(10, 12, std::uint8_t(0));
  mask(cv::Rect(1, 1, 4, 4)).setTo(255);
  mask(8, 10) = 255;
  const auto original = mask.clone();
  filterSmallComponents(mask, 0);
  require(cv::countNonZero(mask != original) == 0, "Disabled component filter changed the mask");
  filterSmallComponents(mask, 16);
  require(cv::countNonZero(mask) == 16 && !mask(8, 10), "Component size threshold or isolated rejection failed");
  mask.setTo(0);
  filterSmallComponents(mask, 1);
  require(cv::countNonZero(mask) == 0, "Empty foreground handling failed");
  cv::Mat3f xyz(7, 7, cv::Vec3f(1, 2, 10));
  xyz(3, 3) = {1, 2, 20};
  xyz(2, 2) = {NAN, NAN, NAN};
  filterDepthSpikes(xyz, 1);
  require(!std::isfinite(xyz(3, 3)[2]) && !std::isfinite(xyz(2, 2)[2]),
          "Depth spike rejection failed or filled missing geometry");
  require(xyz(3, 2)[2] == 10 && xyz(0, 0)[2] == 10, "Depth spike filter modified valid neighbors");
  xyz.setTo(cv::Scalar(NAN, NAN, NAN));
  filterDepthSpikes(xyz, 1);
  require(!std::isfinite(xyz(3, 3)[2]), "Empty geometry was interpolated");
  xyz.setTo(cv::Scalar(1, 2, 10));
  xyz(cv::Rect(2, 2, 3, 3)).setTo(cv::Scalar(1, 2, 20));
  filterDepthSpikes(xyz, 1);
  require(xyz(3, 3)[2] == 20, "Depth spike filter used points outside its fixed 3x3 window");
  xyz.setTo(cv::Scalar(NAN, NAN, NAN));
  xyz(3, 3) = {1, 2, 20};
  xyz(3, 2) = xyz(3, 4) = {1, 2, 10};
  filterDepthSpikes(xyz, 1);
  require(xyz(3, 3)[2] == 20, "Depth spike filter rejected a point with fewer than three finite neighbors");
  rejects([&] { filterDepthSpikes(xyz, 0); });

  auto cal = syntheticCalibration();
  cal.kp = cv::Vec<float, 5>::all(0);
  auto plane = projectPlane(cal, 0.4f);
  const auto spike = projectPlane(cal, 0.2f);
  plane.up(12, 16) = spike.up(12, 16);
  plane.vp.release();
  const auto unfiltered = TriangulationStage(cal).triangulateDense(plane.up, plane.vp, plane.mask, plane.shading);
  settings.rejectDepthSpikes = true;
  settings.maximumDepthDeviation = 0.05f;
  const auto cloud = TriangulationStage(cal, settings).triangulateDense(plane.up, plane.vp, plane.mask, plane.shading);
  require(cloud->pixelIndices(12, 16) == -1 && cloud->points.size() == unfiltered->points.size() - 1,
          "Depth spike settings were not applied by triangulation stage");
  settings.maximumDepthDeviation = 0;
  rejects([&] { validateReconstructionSettings(settings); });
  settings.maximumDepthDeviation = NAN;
  rejects([&] { validateReconstructionSettings(settings); });
}



void testLisPhase() {
  auto check = [](const std::vector<float> &values, const std::vector<std::uint8_t> &valid) {
    std::vector<int> lengths(values.size(), 0);
    int longest = 0;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (!valid[index] || !std::isfinite(values[index])) continue;
      lengths[index] = 1;
      for (std::size_t previous = 0; previous < index; ++previous)
        if (lengths[previous] && values[previous] < values[index])
          lengths[index] = std::max(lengths[index], lengths[previous] + 1);
      longest = std::max(longest, lengths[index]);
    }
    for (const bool horizontal : {true, false}) {
      const int count = static_cast<int>(values.size());
      cv::Mat1f phase(horizontal ? 1 : count, horizontal ? count : 1);
      cv::Mat1b mask(phase.size());
      for (int index = 0; index < count; ++index) {
        phase(horizontal ? 0 : index, horizontal ? index : 0) = values[index];
        mask(horizontal ? 0 : index, horizontal ? index : 0) = valid[index];
      }
      const auto original = phase.clone();
      filterLisPhase(horizontal ? phase : cv::Mat1f{}, horizontal ? cv::Mat1f{} : phase, mask);
      int survivors = 0;
      float previous = -std::numeric_limits<float>::infinity();
      for (int index = 0; index < count; ++index) {
        if (!mask(horizontal ? 0 : index, horizontal ? index : 0)) continue;
        require(valid[index] && std::isfinite(values[index]) && previous < values[index],
                "LIS phase filter retained an invalid or non-increasing pixel");
        previous = values[index];
        ++survivors;
      }
      require(survivors == longest, "LIS phase filter did not retain a longest subsequence");
      require(std::memcmp(original.data, phase.data, original.total() * original.elemSize()) == 0,
              "LIS phase filter modified the phase map");
    }
  };
  check({1, 2, 3, 4}, {255, 255, 255, 255});
  check({4, 3, 2, 1}, {255, 255, 255, 255});
  check({2, 2, 2, 2}, {255, 255, 255, 255});
  check({1, 9, 2, 3, 4, 0, 5}, {255, 255, 255, 255, 255, 255, 255});
  check({1, 2, 3}, {0, 0, 0});
  check({4}, {255});
  check({1, std::numeric_limits<float>::quiet_NaN(), 2, std::numeric_limits<float>::infinity(), 3},
        {255, 255, 255, 255, 255});
  std::mt19937 random(173);
  for (int iteration = 0; iteration < 200; ++iteration) {
    std::vector<float> values(1 + random() % 30);
    std::vector<std::uint8_t> valid(values.size());
    for (std::size_t index = 0; index < values.size(); ++index) {
      values[index] = static_cast<float>(random() % 20);
      valid[index] = random() % 4 ? 255 : 0;
    }
    check(values, valid);
  }
  const cv::Mat1f bothUp = (cv::Mat1f(2, 3) << 1, 2, 3, 3, 1, 2);
  const cv::Mat1f bothVp = (cv::Mat1f(2, 3) << 2, 1, 1, 1, 2, 2);
  cv::Mat1b bothMask(2, 3, std::uint8_t(255));
  filterLisPhase(bothUp, bothVp, bothMask);
  const cv::Mat1b expected = (cv::Mat1b(2, 3) << 0, 255, 255, 0, 255, 255);
  require(cv::countNonZero(bothMask != expected) == 0, "Both-map filtering was order-dependent");
  cv::Mat1b invalid;
  rejects([&] { filterLisPhase({}, {}, invalid); });

  const std::filesystem::path source(SLSTUDIO_SOURCE_DIR);
  GeometricCalibrationData calibration;
  require(calibration.load(source / "example_data/calibration/calibration.xml"), "Missing test calibration");
  const auto frames = loadFrameSequence(source / "example_data/skull_2x3");
  ReconstructionSettings settings;
  require(!settings.lisPhaseFilter, "LIS phase filter must be disabled by default");
  cv::Mat1f up, vp;
  cv::Mat1b baselineMask, shading;
  DecodeStage(calibration, settings).decodeMaps(frames, up, vp, baselineMask, shading);
  filterLisPhase(up, vp, baselineMask);
  settings.lisPhaseFilter = true;
  cv::Mat1b filteredMask;
  DecodeStage(calibration, settings).decodeMaps(frames, up, vp, filteredMask, shading);
  require(cv::countNonZero(filteredMask != baselineMask) == 0, "Decode stage did not apply LIS phase filtering");
  const auto cloud = TriangulationStage(calibration, settings).triangulateDense(up, vp, filteredMask, shading);
  cv::Mat lensMap1, lensMap2, undistortedMask;
  cv::initUndistortRectifyMap(calibration.Kc, calibration.kc, cv::Matx33f::eye(), calibration.Kc,
      filteredMask.size(), CV_16SC2, lensMap1, lensMap2);
  cv::remap(filteredMask, undistortedMask, lensMap1, lensMap2, cv::INTER_NEAREST);
  for (int row = 0; row < cloud->pixelIndices.rows; ++row)
    for (int column = 0; column < cloud->pixelIndices.cols; ++column)
      require(undistortedMask.at<std::uint8_t>(row, column) || cloud->pixelIndices(row, column) == -1,
              "Triangulation retained a pixel discarded by LIS phase filtering");
}
}

int main() {
  try {
    testLisPhase();
    testTwoCoordinateTriangulation();
    testDistortion();
    testCleanup();
    std::cout << "Triangulation accuracy, distortion correction, reconstruction filters and integration passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
