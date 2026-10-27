#include "ApplicationSettings.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <type_traits>

namespace slstudio::gui {
void ApplicationSettings::fields(cv::FileStorage &storage, bool reading) {
  auto node = storage.root();
  auto field = [&](const char *name, auto &value) {
    if constexpr (std::is_same_v<std::decay_t<decltype(value)>, bool>) {
      if (reading) { if (!node[name].empty()) value = static_cast<int>(node[name]) != 0; }
      else storage << name << static_cast<int>(value);
    } else {
      if (reading) { if (!node[name].empty()) node[name] >> value; }
      else storage << name << value;
    }
  };
  auto path = [&](const char *name, auto &buffer) {
    std::string value = buffer.data();
    field(name, value);
    if (reading) {
      if (value.size() >= buffer.size()) throw std::runtime_error(std::string("Saved path is too long: ") + name);
      std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
    }
  };
  if (reading) {
    if (!node["codec"].isNone()) {
      if (!node["codec"].isString()) throw std::runtime_error("Invalid saved codec name");
      reconstruction.coding.codec = codecTypeFromName(static_cast<std::string>(node["codec"]));
    }
    reconstruction.coding.algorithmSettings = CodecAlgorithmSettings{};
  } else {
    const auto name = codecName(reconstruction.coding.codec);
    storage << "codec" << name;
  }
  auto codecInteger = [&](const char *name, int &value) {
    if (reading && !node[name].isNone() && !node[name].isInt())
      throw std::runtime_error(std::string("Invalid saved codec parameter: ") + name);
    field(name, value);
  };
  codecInteger("phase_steps", reconstruction.coding.algorithmSettings.phaseShift.steps);
  codecInteger("fringe_periods", reconstruction.coding.algorithmSettings.phaseShift.periods);
  codecInteger("gray_horizontal_bits", reconstruction.coding.algorithmSettings.grayCode.horizontalBits);
  codecInteger("gray_vertical_bits", reconstruction.coding.algorithmSettings.grayCode.verticalBits);
  auto codecReal = [&](const char *name, double &value) {
    if (reading && !node[name].isNone() && !node[name].isInt() && !node[name].isReal())
      throw std::runtime_error(std::string("Invalid saved codec parameter: ") + name);
    field(name, value);
  };
  codecInteger("heterodyne_steps", reconstruction.coding.algorithmSettings.heterodyne.steps);
  codecReal("heterodyne_primary_pitch", reconstruction.coding.algorithmSettings.heterodyne.primaryPitch);
  codecInteger("spatial_unwrap_steps", reconstruction.coding.algorithmSettings.spatialUnwrap.steps);
  codecInteger("spatial_unwrap_periods", reconstruction.coding.algorithmSettings.spatialUnwrap.periods);
  codecInteger("micro_steps", reconstruction.coding.algorithmSettings.micro.steps);
  codecInteger("modulated_signal_steps", reconstruction.coding.algorithmSettings.modulated.signalSteps);
  codecInteger("modulated_carrier_steps", reconstruction.coding.algorithmSettings.modulated.carrierSteps);
  codecInteger("modulated_cue_steps", reconstruction.coding.algorithmSettings.modulated.cueSteps);
  codecInteger("modulated_signal_periods", reconstruction.coding.algorithmSettings.modulated.signalPeriods);
  codecInteger("modulated_carrier_periods", reconstruction.coding.algorithmSettings.modulated.carrierPeriods);
  validateCodecSettings(reconstruction.coding.algorithmSettings);
  field("direction", directionIndex);
  field("minimumModulation", reconstruction.minimumModulation);
  field("motionCompensation", reconstruction.motionCompensation);
  codecInteger("centerFringeOrder", reconstruction.centerFringeOrder);
  field("minDepth", reconstruction.points.minDepth);
  field("maxDepth", reconstruction.points.maxDepth);
  field("lis_phase_filter", reconstruction.lisPhaseFilter);
  field("correctProjectorDistortion", reconstruction.correctProjectorDistortion);
  field("minimumComponentPixels", reconstruction.minimumComponentPixels);
  field("rejectDepthSpikes", reconstruction.rejectDepthSpikes);
  field("maximumDepthDeviation", reconstruction.maximumDepthDeviation);
  field("camera", cameraIndex);
  field("cameraInterface", cameraInterface);
  field("exposure", exposure);
  field("gain", gain);
  field("autoExposure", autoExposure);
  field("hardwareTrigger", hardwareTrigger);
  field("triggerRisingEdge", triggerRisingEdge);
  path("frameTriggerSource", frameTriggerSource);
  field("continuous", continuousCapture);
  field("settleMs", settleMilliseconds);
  field("cameraWidth", cameraWidth);
  field("cameraHeight", cameraHeight);
  field("previewFrameCount", previewFrameCount);
  field("pointSize", rendering.pointSize);
  int colorMode = static_cast<int>(rendering.colorMode);
  field("colorMode", colorMode);
  if (reading && !node["solidColor"].empty() && (!node["solidColor"].isSeq() || node["solidColor"].size() != 3))
    throw std::runtime_error("Invalid saved solid color");
  field("solidColor", rendering.solidColor);
  if (reading) {
    if (colorMode < 0 || colorMode > 2)
      throw std::runtime_error("Invalid saved color mode");
    for (int channel = 0; channel < 3; ++channel)
      if (!std::isfinite(rendering.solidColor[channel]) || rendering.solidColor[channel] < 0 || rendering.solidColor[channel] > 1)
        throw std::runtime_error("Invalid saved solid color");
    rendering.colorMode = static_cast<RenderSettings::ColorMode>(colorMode);
  }
  field("surfaceShading", rendering.surfaceShading);
  field("showRenderSettings", renderSettingsOpen);
  field("showSettings", settingsWindowOpen);
  field("output", outputIndex);
  field("outputId", outputId);
  field("projectorExposure", projectorExposure);
  field("projectorPeriod", projectorPeriod);
  path("projectorAddress", projectorAddress);
  path("sequencePath", sequencePath);
  path("geometricCalibrationPath", geometricCalibrationPath);
  path("exportPath", exportPath);
  field("boardCorners", geometricCalibrationSettings.boardCorners);
  field("boardSpacing", geometricCalibrationSettings.spacing);
  field("boardCircles", geometricCalibrationSettings.circles);
  geometricCalibrationSettings.partialCircles = geometricCalibrationSettings.circles;
  field("neighborhoodRadius", geometricCalibrationSettings.neighborhoodRadius);
  std::string interpolation;
  switch (geometricCalibrationSettings.interpolation) {
  case GeometricCalibrationInterpolation::LocalHomography: interpolation = "homography"; break;
  case GeometricCalibrationInterpolation::RBF: interpolation = "rbf"; break;
  default: throw std::runtime_error("Invalid calibration interpolation method");
  }
  field("geometricCalibrationInterpolation", interpolation);
  if (reading) {
    if (interpolation == "homography") geometricCalibrationSettings.interpolation = GeometricCalibrationInterpolation::LocalHomography;
    else if (interpolation == "rbf") geometricCalibrationSettings.interpolation = GeometricCalibrationInterpolation::RBF;
    else throw std::runtime_error("Invalid saved calibration interpolation method");
  }
  field("cameraFlags", geometricCalibrationSettings.cameraFlags);
  field("projectorFlags", geometricCalibrationSettings.projectorFlags);
  field("intensityLevels", intensityLevels);
  field("intensityCompensationEnabled", intensityCompensationEnabled);
  cv::Mat lut;
  if (intensityLut) lut = cv::Mat(1, 256, CV_8UC1, intensityLut->inverseLut.data()).clone();
  field("intensityLut", lut);
  if (reading) {
    intensityLut.reset();
    if (!lut.empty()) {
      if (lut.type() != CV_8UC1 || lut.rows != 1 || lut.cols != 256)
        throw std::runtime_error("Invalid saved projector/camera intensity curve");
      intensityLut.emplace();
      std::copy_n(lut.ptr<std::uint8_t>(), 256, intensityLut->inverseLut.begin());
    }
    if (!intensityLut) intensityCompensationEnabled = false;
  }
  auto lightCrafter = [&](const char *name, auto &projectorSettings) {
    if (reading) node = storage[name];
    else storage << name << "{";
    field("standby", projectorSettings.standby);
    field("red", projectorSettings.ledCurrent[0]);
    field("green", projectorSettings.ledCurrent[1]);
    field("blue", projectorSettings.ledCurrent[2]);
    field("flipLongAxis", projectorSettings.flipLongAxis);
    field("flipShortAxis", projectorSettings.flipShortAxis);
    field("triggerInverted", projectorSettings.triggerInverted);
  };
  lightCrafter("lc3000", lc3000Settings);
  field("rotate", lc3000Settings.rotate);
  field("frameRate", lc3000Settings.frameRate);
  field("triggerEnabled", lc3000Settings.triggerEnabled);
  field("triggerDelayUs", lc3000Settings.triggerDelayUs);
  field("triggerPulseUs", lc3000Settings.triggerPulseUs);
  if (!reading) storage << "}";

  lightCrafter("lc4500", lc4500Settings);
  field("ledSelection", lc4500Settings.ledSelection);
  int displayMode = static_cast<int>(lc4500Settings.displayMode);
  field("displayMode", displayMode);
  field("testPattern", lc4500Settings.testPattern);
  field("flashPatternCount", lc4500Settings.flashPatternCount);
  if (reading) {
    if (displayMode < 0 || displayMode > static_cast<int>(slstudio::LightCrafter4500Mode::TestPattern) ||
        lc4500Settings.testPattern < 0 || lc4500Settings.testPattern > 10 ||
        lc4500Settings.flashPatternCount < 1 || lc4500Settings.flashPatternCount > 128)
      throw std::runtime_error("Invalid saved LC4500 display mode, test pattern or stored pattern count");
    lc4500Settings.displayMode = static_cast<slstudio::LightCrafter4500Mode>(displayMode);
  }
  int exposureUs = static_cast<int>(lc4500Settings.exposure.count());
  int periodUs = static_cast<int>(lc4500Settings.framePeriod.count());
  field("exposureUs", exposureUs);
  field("periodUs", periodUs);
  if (reading) {
    lc4500Settings.exposure = std::chrono::microseconds(exposureUs);
    lc4500Settings.framePeriod = std::chrono::microseconds(periodUs);
  } else storage << "}";
}

ApplicationSettings ApplicationSettings::read(cv::FileStorage &storage) const {
  auto result = *this;
  result.fields(storage, true);
  if (result.previewFrameCount < 1) throw std::runtime_error("Invalid saved preview frame count");
  auto thresholds = result.reconstruction;
  thresholds.minimumModulation = std::clamp(thresholds.minimumModulation, 0, 255);
  validateReconstructionSettings(thresholds);
  return result;
}

void ApplicationSettings::write(cv::FileStorage &storage) const {
  validateReconstructionSettings(reconstruction);
  auto snapshot = *this;
  snapshot.fields(storage, false);
}
}
