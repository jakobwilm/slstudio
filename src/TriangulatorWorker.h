#pragma once

#include "GeometricCalibrationData.h"
#include "ReconstructionSettings.h"
#include "Triangulator.h"
#include <variant>
#include <memory>
#include <optional>

void filterDepthSpikes(cv::Mat3f &xyz, float maximumDeviation);

class TriangulationStage {
public:
  TriangulationStage(const GeometricCalibrationData &calibration, const PointCloudSettings &settings = {});
  TriangulationStage(const GeometricCalibrationData &calibration, const ReconstructionSettings &settings);
  PointCloudPtr triangulateDense(const cv::Mat1f &up, const cv::Mat1f &vp,
                                const cv::Mat1b &mask, const cv::Mat1b &shading) const;
  PointCloudPtr triangulateSparse(const std::vector<ProjectorTransition> &coordinates) const;

private:
  const ReconstructionSettings settings;
  std::variant<TriangulatorOrganized, TriangulatorUnorganized> triangulator;
};

struct ReconstructionResult {
  std::uint64_t sequence = 0;
  PointCloudConstPtr cloud;
};

struct TriangulatorStatistics {
  std::uint64_t submitted = 0, reconstructed = 0, dropped = 0;
};

class TriangulatorWorker {
public:
  TriangulatorWorker(const GeometricCalibrationData &calibration,
                     const ReconstructionSettings &settings = {});
  ~TriangulatorWorker();
  TriangulatorWorker(const TriangulatorWorker &) = delete;
  TriangulatorWorker &operator=(const TriangulatorWorker &) = delete;

  // Submissions replace pending work. Dense buffers are shared, not cloned; do not mutate them afterward.
  bool trySubmitDense(std::uint64_t sequence, cv::Mat1f up, cv::Mat1f vp,
                      cv::Mat1b mask, cv::Mat1b shading);
  bool trySubmitSparse(std::uint64_t sequence, std::vector<ProjectorTransition> coordinates);
  bool canSubmit() const;
  bool busy() const;
  TriangulatorStatistics statistics() const;
  // Takes the newest completed result, or rethrows a background reconstruction failure.
  std::optional<ReconstructionResult> tryTake();
  void cancel();

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};
