#pragma once

#include "CodecFactory.h"
#include "PointCloud.h"
#include <cmath>
#include <stdexcept>

struct ReconstructionSettings {
  // Acquisition metadata is separate from filters and geometry used for reconstruction.
  CodecSettings coding;
  int minimumModulation = 55;
  PointCloudSettings points;
  bool lisPhaseFilter = false;
  bool correctProjectorDistortion = false;
  int minimumComponentPixels = 0;
  bool rejectDepthSpikes = false;
  float maximumDepthDeviation = 1.0f;
  bool motionCompensation = true;
  // Assumed seed fringe order for spatial unwrapping; a wrong value can shift depth without detection.
  int centerFringeOrder = DefaultCenterFringeOrder;
};

inline bool operator==(const ReconstructionSettings &a, const ReconstructionSettings &b) {
  return a.coding == b.coding &&
      a.minimumModulation == b.minimumModulation && a.motionCompensation == b.motionCompensation &&
      a.centerFringeOrder == b.centerFringeOrder &&
      a.lisPhaseFilter == b.lisPhaseFilter && a.correctProjectorDistortion == b.correctProjectorDistortion &&
      a.minimumComponentPixels == b.minimumComponentPixels && a.rejectDepthSpikes == b.rejectDepthSpikes &&
      a.maximumDepthDeviation == b.maximumDepthDeviation && a.points.minDepth == b.points.minDepth &&
      a.points.maxDepth == b.points.maxDepth;
}

// Apply a recorded sequence's coding without replacing the active reconstruction options.
inline ReconstructionSettings withCodecSettings(const ReconstructionSettings &settings,
                                                     const CodecSettings &coding) {
  auto result = settings;
  result.coding = coding;
  return result;
}

inline void validateReconstructionSettings(const ReconstructionSettings &settings) {
  validateCodecSettings(settings.coding.algorithmSettings);
  if (settings.centerFringeOrder < 0 ||
      (settings.coding.codec == CodecType::PhaseShiftSpatialUnwrap &&
       settings.centerFringeOrder >= settings.coding.algorithmSettings.spatialUnwrap.periods))
    throw std::invalid_argument("Invalid center fringe order for spatial unwrapping");
  if (settings.minimumModulation < 0 || settings.minimumModulation > 255 ||
      settings.minimumComponentPixels < 0 ||
      !std::isfinite(settings.maximumDepthDeviation) || settings.maximumDepthDeviation <= 0 ||
      !std::isfinite(settings.points.minDepth) ||
      !std::isfinite(settings.points.maxDepth) || settings.points.minDepth > settings.points.maxDepth)
    throw std::invalid_argument("Invalid reconstruction thresholds or depth range");
}

inline void validateReconstructionSampling(const ReconstructionSettings &settings) {
  if (settings.coding.codec == CodecType::GrayCode && (settings.lisPhaseFilter || settings.rejectDepthSpikes))
    throw std::invalid_argument("Gray-code transition clouds do not support LIS phase or image-grid depth-spike filtering");
}
