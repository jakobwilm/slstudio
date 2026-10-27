#pragma once

#include "GeometricCalibration.h"
#include "ReconstructionSettings.h"
#include "IntensityCalibrationLut.h"
#include "ProjectorLightCrafter3000.h"
#include "ProjectorLightCrafter4500.h"
#include "RenderSettings.h"
#include <array>
#include <optional>
#include <string>

namespace slstudio::gui {
struct ApplicationSettings {
  ReconstructionSettings reconstruction;
  GeometricCalibrationSettings geometricCalibrationSettings{cv::Size(20, 15), 17.0f, true, true};
  int directionIndex = 0, cameraIndex = 0, cameraInterface = 0, outputIndex = 0;
  std::string outputId;
  float exposure = -6, gain = 0;
  bool autoExposure = true, hardwareTrigger = false, triggerRisingEdge = true, continuousCapture = false;
  int settleMilliseconds = 100, cameraWidth = 640, cameraHeight = 480, previewFrameCount = 1;
  RenderSettings rendering;
  bool renderSettingsOpen = true, settingsWindowOpen = true;
  float projectorExposure = 16.667f, projectorPeriod = 16.667f;  // ms
  std::array<char, 64> frameTriggerSource{}, projectorAddress{};
  std::array<char, 1024> sequencePath{}, geometricCalibrationPath{}, exportPath{};
  int intensityLevels = 255;
  bool intensityCompensationEnabled = false;
  std::optional<IntensityCalibrationLut> intensityLut;
  slstudio::LightCrafter3000Settings lc3000Settings;
  slstudio::LightCrafter4500Settings lc4500Settings;

  ApplicationSettings read(cv::FileStorage &storage) const;
  void write(cv::FileStorage &storage) const;

private:
  void fields(cv::FileStorage &storage, bool reading);
};
}
