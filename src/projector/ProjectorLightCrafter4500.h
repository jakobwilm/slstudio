#pragma once

#include "ProjectorFactory.h"
#include <array>

namespace slstudio {

enum class LightCrafter4500Mode { HdmiPatterns, FlashPatterns, HdmiVideo, TestPattern };
using LightCrafter4500Firmware = std::array<unsigned int, 4>;

enum class LightCrafter4500Playback { Stopped, Paused, Running };
enum class LightCrafter4500Action { Start, Pause, Stop, Blank };

struct LightCrafter4500Status {
  bool standby = false;
  bool patternMode = false;
  bool ledsDisabled = false;
  LightCrafter4500Playback playback = LightCrafter4500Playback::Stopped;
  bool validationReady = false;
  unsigned int validationFlags = 0;
  bool validationPassed() const { return validationReady && (validationFlags & ~0x1cu) == 0; }
};

struct LightCrafter4500Settings {
  bool standby = false;
  std::array<int, 3> ledCurrent{90, 0, 0};
  int ledSelection = 1;
  bool flipLongAxis = false, flipShortAxis = false;
  bool triggerInverted = false;
  LightCrafter4500Mode displayMode = LightCrafter4500Mode::HdmiPatterns;
  int testPattern = 7;
  int flashPatternCount = 6;
  std::chrono::microseconds exposure{8333}, framePeriod{8333};
};

bool lightCrafter4500Connected();
LightCrafter4500Firmware lightCrafter4500Firmware(const ProjectorOpenSettings &settings = {});
LightCrafter4500Status lightCrafter4500Status(const ProjectorOpenSettings &settings = {});
LightCrafter4500Status controlLightCrafter4500(LightCrafter4500Action action,
                                             const ProjectorOpenSettings &settings = {});
std::string lightCrafter4500VersionString(unsigned int version);
std::uint8_t lightCrafter4500LedPwm(std::uint8_t level);
void validateLightCrafter4500Sequence(const ProjectorSequenceSettings &settings);
void validateLightCrafterSettings(const LightCrafter4500Settings &settings);
void configureLightCrafter(const LightCrafter4500Settings &settings,
                          const ProjectorOpenSettings &openSettings = {});

}
