#include "ProjectorFactory.h"
#include "ProjectorLightCrafter3000.h"
#include "ProjectorLightCrafter4500.h"
#ifdef SLSTUDIO_PROJECTOR_LC4500
#include "LC4500API/dlpc350_usb.h"
#endif

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace slstudio {
#ifdef SLSTUDIO_PROJECTOR_GLFW
std::vector<ProjectorOutputInfo> glfwProjectorOutputs();
std::unique_ptr<Projector> openGLFWProjector(const ProjectorOutputInfo &output);
#endif
#ifdef SLSTUDIO_PROJECTOR_DRM_LEASE
std::vector<ProjectorOutputInfo> drmLeaseProjectorOutputs();
std::unique_ptr<Projector> openDrmLeaseProjector(const ProjectorOutputInfo &output);
#endif
#ifdef SLSTUDIO_PROJECTOR_DISPLAY_CORE
std::vector<ProjectorOutputInfo> displayCoreProjectorOutputs();
std::unique_ptr<Projector> openDisplayCoreProjector(const ProjectorOutputInfo &output);
#endif
#ifdef SLSTUDIO_PROJECTOR_CORE_GRAPHICS
std::vector<ProjectorOutputInfo> coreGraphicsProjectorOutputs();
std::unique_ptr<Projector> openCoreGraphicsProjector(const ProjectorOutputInfo &output);
#endif
#ifdef SLSTUDIO_PROJECTOR_LC3000
std::unique_ptr<Projector> openLightCrafter3000(const ProjectorOpenSettings &settings);
void configureLightCrafter3000(const LightCrafter3000Settings &requestedSettings, const ProjectorOpenSettings &settings);
#endif
#ifdef SLSTUDIO_PROJECTOR_LC4500
std::unique_ptr<Projector> openLightCrafter4500(const ProjectorOpenSettings &settings);
void configureLightCrafter4500(const LightCrafter4500Settings &requestedSettings, const ProjectorOpenSettings &settings);
#endif

bool lightCrafter4500Connected() {
#ifdef SLSTUDIO_PROJECTOR_LC4500
  return DLPC350_USB_IsPresent() != 0;
#else
  return false;
#endif
}

std::string lightCrafter4500VersionString(unsigned int version) {
  return std::to_string(version >> 24) + "." + std::to_string((version >> 16) & 255) + "." +
      std::to_string(version & 65535);
}

std::uint8_t lightCrafter4500LedPwm(std::uint8_t level) {
  // The LC4500 board's LED drivers use an inverted current scale.
  return static_cast<std::uint8_t>(255 - level);
}

#ifndef SLSTUDIO_PROJECTOR_LC4500
LightCrafter4500Firmware lightCrafter4500Firmware(const ProjectorOpenSettings &) {
  throw std::runtime_error("LC4500 support was not built");
}

LightCrafter4500Status lightCrafter4500Status(const ProjectorOpenSettings &) {
  throw std::runtime_error("LC4500 support was not built");
}

LightCrafter4500Status controlLightCrafter4500(LightCrafter4500Action, const ProjectorOpenSettings &) {
  throw std::runtime_error("LC4500 support was not built");
}
#endif

#ifndef SLSTUDIO_PROJECTOR_LC3000
LightCrafter3000DeviceState readLightCrafter3000State(const ProjectorOpenSettings &) {
  throw std::runtime_error("LC3000 backend is not available in this build");
}

void controlLightCrafter3000(const LightCrafter3000Control &, const ProjectorOpenSettings &) {
  throw std::runtime_error("LC3000 backend is not available in this build");
}

void changeLightCrafter3000Ip(const std::string &, const std::string &) {
  throw std::runtime_error("LC3000 backend is not available in this build");
}

LightCrafter3000Status lightCrafter3000Status(const std::string &, std::chrono::milliseconds) {
  return {false, {}, "LC3000 backend is not available in this build"};
}
#endif

bool lightCrafterAvailable(OutputBackend backend) {
#ifdef SLSTUDIO_PROJECTOR_LC3000
  if (backend == OutputBackend::LightCrafter3000) return true;
#endif
#ifdef SLSTUDIO_PROJECTOR_LC4500
  if (backend == OutputBackend::LightCrafter4500) return true;
#endif
  static_cast<void>(backend);
  return false;
}

namespace {
void validateLedCurrent(const std::array<int, 3> &currents) {
  for (const auto current : currents)
    if (current < 0 || current > 255) throw std::invalid_argument("LED levels must be in [0, 255]");
}

void validateProjectorOpenSettings(const ProjectorOpenSettings &settings) {
  if (settings.timeout.count() <= 0 || settings.timeout > std::chrono::minutes(1))
    throw std::invalid_argument("Projector timeout must be in (0, 60000] ms");
}

void validateLightCrafter4500PatternTiming(const ProjectorSequenceSettings &settings) {
  validateProjectorSequence(settings);
  if (settings.framePeriod > std::chrono::seconds(1))
    throw std::invalid_argument("LC4500 pattern frame period must be <= 1000000 us");
  const auto darkTime = settings.framePeriod - settings.exposure;
  if (darkTime.count() != 0 && darkTime <= std::chrono::microseconds(230))
    throw std::invalid_argument("LC4500 frame period must equal exposure or exceed it by more than 230 us");
}
}

void validateLightCrafter4500Sequence(const ProjectorSequenceSettings &settings) {
  validateLightCrafter4500PatternTiming(settings);
  if (settings.exposure < std::chrono::microseconds(8333))
    throw std::invalid_argument("LC4500 eight-bit patterns require exposure >= 8333 us");
}

void validateLightCrafterSettings(const LightCrafter3000Settings &settings) {
  if (settings.standby)
    throw std::invalid_argument("LC3000 standby is not supported by the network API; clear the standby setting");
  validateLedCurrent(settings.ledCurrent);
  if (std::find(lightCrafter3000FrameRates.begin(), lightCrafter3000FrameRates.end(),
      settings.frameRate) == lightCrafter3000FrameRates.end())
    throw std::invalid_argument("LC3000 eight-bit HDMI video requires 15, 30 or 60 Hz");
  if (settings.triggerDelayUs < 0 ||
      settings.triggerPulseUs < 1 || settings.triggerDelayUs > 1000000 || settings.triggerPulseUs > 1000000)
    throw std::invalid_argument("LC3000 trigger timing must be within [0, 1000000] us with a positive pulse width");
}

void validateLightCrafterSettings(const LightCrafter4500Settings &settings) {
  if (settings.standby) return;
  validateLedCurrent(settings.ledCurrent);
  if (settings.ledSelection < 1 || settings.ledSelection > 7)
    throw std::invalid_argument("Select at least one projector LED");
  switch (settings.displayMode) {
    case LightCrafter4500Mode::HdmiPatterns:
      validateLightCrafter4500Sequence({settings.exposure, settings.framePeriod});
      break;
    case LightCrafter4500Mode::FlashPatterns:
      if (settings.flashPatternCount < 1 || settings.flashPatternCount > 128)
        throw std::invalid_argument("LC4500 stored pattern count must be in [1, 128]");
      validateLightCrafter4500Sequence({settings.exposure, settings.framePeriod});
      break;
    case LightCrafter4500Mode::TestPattern:
      if (settings.testPattern < 0 || settings.testPattern > 10)
        throw std::invalid_argument("LC4500 test pattern must be in [0, 10]");
      break;
    case LightCrafter4500Mode::HdmiVideo:
      break;
    default:
      throw std::invalid_argument("Unknown LC4500 display mode");
    }
}

void configureLightCrafter(const LightCrafter3000Settings &settings, const ProjectorOpenSettings &openSettings) {
  validateLightCrafterSettings(settings);
  validateProjectorOpenSettings(openSettings);
#ifdef SLSTUDIO_PROJECTOR_LC3000
  configureLightCrafter3000(settings, openSettings);
#else
  throw std::runtime_error("LC3000 backend is not available in this build");
#endif
}

void configureLightCrafter(const LightCrafter4500Settings &settings, const ProjectorOpenSettings &openSettings) {
  validateLightCrafterSettings(settings);
  validateProjectorOpenSettings(openSettings);
#ifdef SLSTUDIO_PROJECTOR_LC4500
  configureLightCrafter4500(settings, openSettings);
#else
  throw std::runtime_error("LC4500 backend is not available in this build");
#endif
}

std::vector<ProjectorOutputInfo> projectorOutputs(bool allowDesktop) {
  std::vector<ProjectorOutputInfo> outputs;
#ifdef SLSTUDIO_PROJECTOR_DRM_LEASE
  outputs = drmLeaseProjectorOutputs();
#endif
#ifdef SLSTUDIO_PROJECTOR_GLFW
  if (allowDesktop) {
    auto desktop = glfwProjectorOutputs();
    outputs.insert(outputs.end(), desktop.begin(), desktop.end());
  }
#else
  static_cast<void>(allowDesktop);
#endif
#ifdef SLSTUDIO_PROJECTOR_LC3000
  outputs.push_back({"lc3000", "LightCrafter 3000", OutputBackend::LightCrafter3000, true});
#endif
#ifdef SLSTUDIO_PROJECTOR_LC4500
  outputs.push_back({"lc4500", "LightCrafter 4500", OutputBackend::LightCrafter4500, true});
#endif
#ifdef SLSTUDIO_PROJECTOR_DISPLAY_CORE
  auto displayCore = displayCoreProjectorOutputs();
  outputs.insert(outputs.end(), displayCore.begin(), displayCore.end());
#endif
#ifdef SLSTUDIO_PROJECTOR_CORE_GRAPHICS
  auto coreGraphics = coreGraphicsProjectorOutputs();
  outputs.insert(outputs.end(), coreGraphics.begin(), coreGraphics.end());
#endif
  return outputs;
}

std::unique_ptr<Projector> openProjector(const ProjectorOutputInfo &output,
                                             const ProjectorOpenSettings &settings) {
  if (output.id.empty() || settings.timeout.count() <= 0 || settings.timeout > std::chrono::minutes(1))
    throw std::invalid_argument("Opening a projector requires a device ID and timeout in (0, 60000] ms");
  switch (output.backend) {
  case OutputBackend::Desktop:
    if (!settings.allowDesktop)
      throw std::invalid_argument("Non-exclusive desktop projection requires explicit opt-in");
#ifdef SLSTUDIO_PROJECTOR_GLFW
    return openGLFWProjector(output);
#else
    break;
#endif
  case OutputBackend::WaylandLease:
  case OutputBackend::XRandRLease:
#ifdef SLSTUDIO_PROJECTOR_DRM_LEASE
    return openDrmLeaseProjector(output);
#else
    break;
#endif
  case OutputBackend::LightCrafter3000:
#ifdef SLSTUDIO_PROJECTOR_LC3000
    if (output.id != "lc3000") throw std::invalid_argument("Unknown LightCrafter 3000 device");
    return openLightCrafter3000(settings);
#else
    break;
#endif
  case OutputBackend::LightCrafter4500:
#ifdef SLSTUDIO_PROJECTOR_LC4500
    if (output.id != "lc4500") throw std::invalid_argument("Unknown LightCrafter 4500 device");
    return openLightCrafter4500(settings);
#else
    break;
#endif
  case OutputBackend::DisplayCore:
#ifdef SLSTUDIO_PROJECTOR_DISPLAY_CORE
    return openDisplayCoreProjector(output);
#else
    break;
#endif
  case OutputBackend::CoreGraphics:
#ifdef SLSTUDIO_PROJECTOR_CORE_GRAPHICS
    return openCoreGraphicsProjector(output);
#else
    break;
#endif
  }
  throw std::runtime_error("Requested projector backend is not available in this build");
}

}