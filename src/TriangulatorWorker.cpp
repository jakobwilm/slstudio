#include "TriangulatorWorker.h"
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>
#include <functional>

void filterDepthSpikes(cv::Mat3f &xyz, float maximumDeviation) {
  if (xyz.empty() || !std::isfinite(maximumDeviation) || maximumDeviation <= 0)
    throw std::invalid_argument("Depth spike filter requires valid geometry and positive deviation");
  const auto source = xyz.clone();
  std::vector<float> neighbors;
  neighbors.reserve(8);
  for (int row = 0; row < xyz.rows; ++row) {
    for (int col = 0; col < xyz.cols; ++col) {
      if (!std::isfinite(source(row, col)[2])) continue;
      neighbors.clear();
      for (int y = std::max(0, row - 1); y <= std::min(xyz.rows - 1, row + 1); ++y)
        for (int x = std::max(0, col - 1); x <= std::min(xyz.cols - 1, col + 1); ++x)
          if ((x != col || y != row) && std::isfinite(source(y, x)[2]))
            neighbors.push_back(source(y, x)[2]);
      if (neighbors.size() < 3) continue;
      const auto middle = neighbors.begin() + neighbors.size() / 2;
      std::nth_element(neighbors.begin(), middle, neighbors.end());
      const float median = neighbors.size() % 2 ? *middle :
          0.5f * (*middle + *std::max_element(neighbors.begin(), middle));
      if (std::abs(source(row, col)[2] - median) > maximumDeviation)
        xyz(row, col) = {NAN, NAN, NAN};
    }
  }
}

TriangulationStage::TriangulationStage(const GeometricCalibrationData &calibration, const PointCloudSettings &requestedSettings)
    : TriangulationStage(calibration, [&] {
        ReconstructionSettings result;
        result.points = requestedSettings;
        return result;
      }()) {}

TriangulationStage::TriangulationStage(const GeometricCalibrationData &calibration, const ReconstructionSettings &requestedSettings)
    : settings(requestedSettings), triangulator([&]() -> std::variant<TriangulatorOrganized, TriangulatorUnorganized> {
        if (requestedSettings.coding.codec == CodecType::GrayCode)
          return TriangulatorUnorganized(calibration, requestedSettings.correctProjectorDistortion);
        return TriangulatorOrganized(calibration, requestedSettings.correctProjectorDistortion);
      }()) {
  validateReconstructionSettings(settings);
  validateReconstructionSampling(settings);
}

PointCloudPtr TriangulationStage::triangulateSparse(const std::vector<ProjectorTransition> &coordinates) const {
  if (const auto *unorganized = std::get_if<TriangulatorUnorganized>(&triangulator)) {
    return unorganized->triangulate(coordinates, settings.points);
  }
  throw std::invalid_argument("Sparse triangulation requires a sparse codec settings");
}

PointCloudPtr TriangulationStage::triangulateDense(const cv::Mat1f &up, const cv::Mat1f &vp,
                                                  const cv::Mat1b &mask, const cv::Mat1b &shading) const {
  const auto *organized = std::get_if<TriangulatorOrganized>(&triangulator);
  if (!organized) throw std::invalid_argument("Gray-code triangulation requires sparse coordinates");
  auto result = organized->triangulate(up, vp, mask, shading);
  if (settings.rejectDepthSpikes) filterDepthSpikes(result.xyz, settings.maximumDepthDeviation);
  return makePointCloud(result.xyz, result.shading, settings.points);
}

struct TriangulatorWorker::Impl {
  struct Input {
    std::uint64_t sequence;
    std::function<PointCloudPtr()> compute;
  };
  TriangulationStage triangulator;
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::optional<Input> pending;
  std::optional<ReconstructionResult> completed;
  std::exception_ptr error;
  TriangulatorStatistics counts;
  bool stopped = false, working = false;
  std::thread thread;

  Impl(const GeometricCalibrationData &calibration, const ReconstructionSettings &settings)
      : triangulator(calibration, settings), thread([this] { run(); }) {}
  ~Impl() {
    cancel();
    if (thread.joinable()) thread.join();
  }
  void cancel() {
    std::lock_guard<std::mutex> lock(mutex);
    stopped = true;
    pending.reset();
    completed.reset();
    changed.notify_all();
  }
  void run() {
    try {
      for (;;) {
        Input input;
        {
          std::unique_lock<std::mutex> lock(mutex);
          changed.wait(lock, [&] { return stopped || pending.has_value(); });
          if (stopped) return;
          input = std::move(*pending);
          pending.reset();
          working = true;
        }
        auto cloud = input.compute();
        std::lock_guard<std::mutex> lock(mutex);
        working = false;
        if (stopped) return;
        if (completed) ++counts.dropped;
        completed = ReconstructionResult{input.sequence, std::move(cloud)};
        ++counts.reconstructed;
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!stopped) error = std::current_exception();
      stopped = true;
      working = false;
      pending.reset();
      completed.reset();
    }
  }
  bool submit(std::uint64_t sequence, std::function<PointCloudPtr()> compute) {
    std::lock_guard<std::mutex> lock(mutex);
    if (stopped) return false;
    if (pending) ++counts.dropped;
    pending = Input{sequence, std::move(compute)};
    ++counts.submitted;
    changed.notify_one();
    return true;
  }
};

TriangulatorWorker::TriangulatorWorker(const GeometricCalibrationData &calibration,
                                       const ReconstructionSettings &settings)
    : impl(std::make_unique<Impl>(calibration, settings)) {}
TriangulatorWorker::~TriangulatorWorker() = default;

bool TriangulatorWorker::trySubmitDense(std::uint64_t sequence, cv::Mat1f up, cv::Mat1f vp,
                                       cv::Mat1b mask, cv::Mat1b shading) {
  return impl->submit(sequence, [target = impl.get(), up = std::move(up), vp = std::move(vp),
                                 mask = std::move(mask), shading = std::move(shading)] {
    return target->triangulator.triangulateDense(up, vp, mask, shading);
  });
}
bool TriangulatorWorker::trySubmitSparse(std::uint64_t sequence,
                                        std::vector<ProjectorTransition> coordinates) {
  return impl->submit(sequence, [target = impl.get(), coordinates = std::move(coordinates)] {
    return target->triangulator.triangulateSparse(coordinates);
  });
}
bool TriangulatorWorker::canSubmit() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return !impl->stopped;
}
bool TriangulatorWorker::busy() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return !impl->stopped && (impl->working || impl->pending.has_value() || impl->completed.has_value());
}
TriangulatorStatistics TriangulatorWorker::statistics() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->counts;
}
std::optional<ReconstructionResult> TriangulatorWorker::tryTake() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->error) std::rethrow_exception(std::exchange(impl->error, std::exception_ptr{}));
  return std::exchange(impl->completed, std::nullopt);
}
void TriangulatorWorker::cancel() { impl->cancel(); }
