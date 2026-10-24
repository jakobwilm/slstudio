#pragma once

#include "GeometricCalibrationData.h"
#include "ProjectorTransition.h"
#include "ReconstructionSettings.h"
#include <functional>
#include <memory>

void filterSmallComponents(cv::Mat1b &mask, int minimumPixels);
void filterLisPhase(const cv::Mat1f &up, const cv::Mat1f &vp, cv::Mat1b &mask);

// Synchronous decoding; Gray code uses transitions instead of dense projector-coordinate maps.
class DecodeStage {
public:
  DecodeStage(const GeometricCalibrationData &calibration, const ReconstructionSettings &settings = {});
  void decodeMaps(const std::vector<cv::Mat> &frames, cv::Mat1f &up, cv::Mat1f &vp,
                  cv::Mat1b &mask, cv::Mat1b &shading);
  std::vector<ProjectorTransition> decodeTransitions(const std::vector<cv::Mat> &frames);
  unsigned int patternCount() const { return decoder->getNPatterns(); }

private:
  void decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat1f &up, cv::Mat1f &vp,
                    cv::Mat1b &mask, cv::Mat1b &shading);
  const ReconstructionSettings settings;
  const cv::Size frameSize;
  std::unique_ptr<Decoder> decoder;
};

struct DecoderStatistics {
  std::uint64_t submitted = 0, decoded = 0, dropped = 0;
};

class DecoderWorker {
public:
  // Runs on the decoder thread; its target must outlive this worker.
  using DenseHandler = std::function<void(std::uint64_t, cv::Mat1f, cv::Mat1f, cv::Mat1b, cv::Mat1b)>;
  using SparseHandler = std::function<void(std::uint64_t, std::vector<ProjectorTransition>)>;
  DecoderWorker(const GeometricCalibrationData &calibration, const ReconstructionSettings &settings,
                DenseHandler handleMaps, SparseHandler handleTransitions = {});
  ~DecoderWorker();
  DecoderWorker(const DecoderWorker &) = delete;
  DecoderWorker &operator=(const DecoderWorker &) = delete;

  // Copies frames and replaces any pending sequence; false means the worker has stopped.
  bool trySubmit(const std::vector<cv::Mat> &frames);
  bool canSubmit() const;
  bool busy() const;
  DecoderStatistics statistics() const;
  // Poll even when no result arrives: failures in decoding or handlers stop the worker.
  void checkError();
  void cancel();

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};
