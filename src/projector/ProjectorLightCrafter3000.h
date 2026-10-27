#pragma once

#include "ProjectorFactory.h"
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace slstudio {

inline constexpr std::array<int, 3> lightCrafter3000FrameRates{15, 30, 60};
inline constexpr std::array<const char *, 14> lightCrafter3000TestPatterns{
    "Fine checkerboard", "Black", "White", "Green", "Blue", "Red",
    "Vertical lines", "Horizontal lines", "Fine vertical lines", "Fine horizontal lines",
    "Diagonal lines", "Vertical gray ramp", "Horizontal gray ramp", "ANSI 4x4 checkerboard"};

enum class LightCrafter3000SequenceTrigger {
  Software = 0, Auto = 1, ExternalPositive = 2, ExternalNegative = 3,
  CameraPositive = 4, CameraNegative = 5, ExternalExposure = 6
};

inline constexpr std::array<const char *, 7> lightCrafter3000SequenceTriggers{
    "Software / command", "Auto", "External (positive edge)", "External (negative edge)",
    "Camera (positive edge)", "Camera (negative edge)", "External + exposure (1-bit)"};

struct LightCrafter3000DeviceState {
  int displayMode = 0, testPattern = 0;
  std::array<int, 3> ledCurrent{};
  bool rotate = false, flipLongAxis = false, flipShortAxis = false;
  std::array<int, 6> videoGeometry{};
  int frameRate = 0, videoBitDepth = 0, videoColor = 0;
  bool triggerEnabled = false, triggerInverted = false;
  int triggerSource = 0;
  std::uint32_t triggerDelayUs = 0, triggerPulseUs = 0;
  int patternCount = 0, patternBitDepth = 0, patternType = 0, inputTrigger = 0, patternLed = 0;
  std::uint32_t inputTriggerDelayUs = 0, exposureUs = 0, periodUs = 0;
  bool repeat = false;
};

enum class LightCrafter3000Action { TestPattern, SolidColor, SelectPattern, Start, Advance, Stop };

struct LightCrafter3000Control {
  LightCrafter3000Action action = LightCrafter3000Action::Stop;
  int testPattern = 0, patternIndex = 0;
  std::array<int, 3> color{};
  ProjectorSequenceSettings timing;
  LightCrafter3000SequenceTrigger sequenceTrigger = LightCrafter3000SequenceTrigger::Auto;
  std::uint32_t inputTriggerDelayUs = 0;
};

struct LightCrafter3000Status {
  std::optional<bool> connected;
  std::array<std::string, 3> firmware;
  std::string error;
};

struct LightCrafter3000Settings {
  bool standby = false;
  std::array<int, 3> ledCurrent{255, 255, 255};
  bool flipLongAxis = false, flipShortAxis = true, rotate = false;
  int frameRate = 60;
  bool triggerEnabled = true, triggerInverted = false;
  int triggerDelayUs = 0, triggerPulseUs = 200;
};

void validateLightCrafterSettings(const LightCrafter3000Settings &settings);
void configureLightCrafter(const LightCrafter3000Settings &settings,
                          const ProjectorOpenSettings &openSettings = {});
void changeLightCrafter3000Ip(const std::string &currentAddress, const std::string &newAddress);
// Leaves connected unset when another LC3000 connection is active.
LightCrafter3000Status lightCrafter3000Status(const std::string &address, std::chrono::milliseconds timeout);
LightCrafter3000DeviceState readLightCrafter3000State(const ProjectorOpenSettings &settings = {});
void controlLightCrafter3000(const LightCrafter3000Control &control, const ProjectorOpenSettings &settings = {});

}
