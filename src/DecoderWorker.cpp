#include "DecoderWorker.h"
#include "CodecGrayCode.h"
#include <algorithm>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

void filterSmallComponents(cv::Mat1b &mask, int minimumPixels) {
  if (mask.empty() || minimumPixels < 0)
    throw std::invalid_argument("Component filter requires a nonempty mask and nonnegative size");
  if (minimumPixels == 0) return;
  cv::Mat1i labels, stats;
  cv::Mat centroids;
  cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
  for (int row = 0; row < mask.rows; ++row)
    for (int col = 0; col < mask.cols; ++col) {
      const int label = labels(row, col);
      if (!label || stats(label, cv::CC_STAT_AREA) < minimumPixels) mask(row, col) = 0;
    }
}

void filterLisPhase(const cv::Mat1f &up, const cv::Mat1f &vp, cv::Mat1b &mask) {
  if (mask.empty() || mask.dims != 2 ||
      (up.empty() && vp.empty()) ||
      (!up.empty() && (up.dims != 2 || up.size() != mask.size())) ||
      (!vp.empty() && (vp.dims != 2 || vp.size() != mask.size())))
    throw std::invalid_argument("LIS phase filter requires phase maps and a matching validity mask");
  const auto eligible = mask.clone();
  auto filterMap = [&](const cv::Mat1f &phase, bool horizontal) {
    if (phase.empty()) return;
    cv::Mat1b retained(phase.size(), std::uint8_t(0));
    const int lineCount = horizontal ? phase.rows : phase.cols;
    const int length = horizontal ? phase.cols : phase.rows;
    std::vector<float> tails;
    std::vector<int> tailIndices, predecessors(length);
    tails.reserve(length);
    tailIndices.reserve(length);
    for (int line = 0; line < lineCount; ++line) {
      tails.clear();
      tailIndices.clear();
      for (int position = 0; position < length; ++position) {
        const int row = horizontal ? line : position;
        const int column = horizontal ? position : line;
        const float value = phase(row, column);
        if (!eligible(row, column) || !std::isfinite(value)) continue;
        const auto tail = std::lower_bound(tails.begin(), tails.end(), value);
        const auto index = static_cast<std::size_t>(tail - tails.begin());
        predecessors[position] = index ? tailIndices[index - 1] : -1;
        if (tail == tails.end()) {
          tails.push_back(value);
          tailIndices.push_back(position);
        } else {
          *tail = value;
          tailIndices[index] = position;
        }
      }
      if (tailIndices.empty()) continue;
      for (int position = tailIndices.back(); position >= 0; position = predecessors[position])
        retained(horizontal ? line : position, horizontal ? position : line) = 255;
    }
    // Both maps are evaluated against the original mask, then their survivors intersect.
    mask &= retained;
  };
  filterMap(up, true);
  filterMap(vp, false);
}

DecodeStage::DecodeStage(const GeometricCalibrationData &calibration, const ReconstructionSettings &requestedSettings)
    : settings(requestedSettings), frameSize(calibration.frameWidth, calibration.frameHeight),
      decoder(DecoderFactory::NewDecoder(requestedSettings.coding.codec, calibration.screenResX,
                                          calibration.screenResY, requestedSettings.coding.direction,
                                          requestedSettings.motionCompensation, requestedSettings.coding.algorithmSettings,
                                          requestedSettings.centerFringeOrder)) {
  validateReconstructionSettings(requestedSettings);
  validateReconstructionSampling(requestedSettings);
  decoder->setMinimumModulation(requestedSettings.minimumModulation);
}

void DecodeStage::decodeMaps(const std::vector<cv::Mat> &frames, cv::Mat1f &up, cv::Mat1f &vp,
                             cv::Mat1b &mask, cv::Mat1b &shading) {
  if (settings.coding.codec == CodecType::GrayCode)
    throw std::invalid_argument("Gray-code decoding requires the sparse transition method");
  decodeFrames(frames, up, vp, mask, shading);
}

std::vector<ProjectorTransition> DecodeStage::decodeTransitions(const std::vector<cv::Mat> &frames) {
  if (settings.coding.codec != CodecType::GrayCode)
    throw std::invalid_argument("Transition decoding requires Gray code");
  cv::Mat1f up, vp;
  cv::Mat1b mask, shading;
  decodeFrames(frames, up, vp, mask, shading);
  const auto &gray = dynamic_cast<const DecoderGrayCode &>(*decoder);
  return gray.decodeTransitions(frames, up, vp, mask, shading);
}

void DecodeStage::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat1f &up, cv::Mat1f &vp,
                               cv::Mat1b &mask, cv::Mat1b &shading) {
  if (frames.size() != patternCount())
    throw std::invalid_argument("Frame count does not match the selected codec and direction");
  for (const auto &frame : frames) {
    if (frame.dims != 2 || frame.size() != frameSize || frame.type() != CV_8UC1)
      throw std::invalid_argument("Camera frame must be grayscale and match calibration dimensions");
  }
  up.release();
  vp.release();
  decoder->decodeFrames(frames, up, vp, mask, shading);
  const auto validPhase = [&](const cv::Mat &phase, bool required) {
    return phase.empty() ? !required : phase.dims == 2 && phase.type() == CV_32FC1 && phase.size() == frameSize;
  };
  if (!validPhase(up, settings.coding.direction & CodecDirHorizontal) ||
      !validPhase(vp, settings.coding.direction & CodecDirVertical) ||
      mask.dims != 2 || mask.type() != CV_8UC1 || mask.size() != frameSize ||
      shading.dims != 2 || shading.type() != CV_8UC1 || shading.size() != frameSize)
    throw std::runtime_error("Decoder returned invalid phase, mask or shading images");
  mask &= shading >= settings.minimumModulation;
  if (settings.lisPhaseFilter) filterLisPhase(up, vp, mask);
  if (settings.minimumComponentPixels) filterSmallComponents(mask, settings.minimumComponentPixels);
}

struct DecoderWorker::Impl {
  struct Input {
    std::uint64_t sequence;
    std::vector<cv::Mat> frames;
  };
  DecodeStage decoder;
  DenseHandler handleMaps;
  SparseHandler handleTransitions;
  const bool sparse;
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::optional<Input> pending;
  std::exception_ptr error;
  DecoderStatistics counts;
  std::uint64_t nextSequence = 1;
  bool stopped = false, working = false;
  std::thread thread;

  Impl(const GeometricCalibrationData &calibration, const ReconstructionSettings &settings,
       DenseHandler maps, SparseHandler transitions)
      : decoder(calibration, settings), handleMaps(std::move(maps)),
        handleTransitions(std::move(transitions)), sparse(settings.coding.codec == CodecType::GrayCode) {
    if (sparse ? !handleTransitions : !handleMaps)
      throw std::invalid_argument("Decoder requires a handler matching its output type");
    thread = std::thread([this] { run(); });
  }
  ~Impl() {
    cancel();
    if (thread.joinable()) thread.join();
  }
  void cancel() {
    std::lock_guard<std::mutex> lock(mutex);
    stopped = true;
    pending.reset();
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
        if (sparse) {
          auto coordinates = decoder.decodeTransitions(input.frames);
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopped) { working = false; return; }
          }
          handleTransitions(input.sequence, std::move(coordinates));
        } else {
          cv::Mat1f up, vp;
          cv::Mat1b mask, shading;
          decoder.decodeMaps(input.frames, up, vp, mask, shading);
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopped) { working = false; return; }
          }
          handleMaps(input.sequence, std::move(up), std::move(vp), std::move(mask), std::move(shading));
        }
        std::lock_guard<std::mutex> lock(mutex);
        working = false;
        if (!stopped) ++counts.decoded;
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!stopped) error = std::current_exception();
      stopped = true;
      working = false;
      pending.reset();
    }
  }
};

DecoderWorker::DecoderWorker(const GeometricCalibrationData &calibration, const ReconstructionSettings &settings,
                             DenseHandler handleMaps, SparseHandler handleTransitions)
    : impl(std::make_unique<Impl>(calibration, settings, std::move(handleMaps),
                                  std::move(handleTransitions))) {}
DecoderWorker::~DecoderWorker() = default;

bool DecoderWorker::trySubmit(const std::vector<cv::Mat> &frames) {
  {
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (impl->stopped) return false;
  }
  std::vector<cv::Mat> owned;
  owned.reserve(frames.size());
  for (const auto &frame : frames) owned.push_back(frame.clone());
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->stopped) return false;
  if (impl->pending) ++impl->counts.dropped;
  impl->pending = Impl::Input{impl->nextSequence++, std::move(owned)};
  ++impl->counts.submitted;
  impl->changed.notify_one();
  return true;
}
bool DecoderWorker::canSubmit() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return !impl->stopped;
}
bool DecoderWorker::busy() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return !impl->stopped && (impl->working || impl->pending.has_value());
}
DecoderStatistics DecoderWorker::statistics() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->counts;
}
void DecoderWorker::checkError() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->error) std::rethrow_exception(std::exchange(impl->error, std::exception_ptr{}));
}
void DecoderWorker::cancel() { impl->cancel(); }
