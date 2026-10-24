#pragma once

namespace slstudio::gui {
enum class GuiSmokeTest { General, GeometricCalibration, IntensityCalibration, Projector, Package };
int runGuiSmokeTest(GuiSmokeTest mode, const char *screenshot);
}
