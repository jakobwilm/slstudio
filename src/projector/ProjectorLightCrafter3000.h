#pragma once

#include "ProjectorFactory.h"
#include <array>

namespace slstudio {

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

}
