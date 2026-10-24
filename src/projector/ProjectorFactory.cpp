#include "ProjectorFactory.h"
#include "ProjectorLightCrafter3000.h"
#include "ProjectorLightCrafter4500.h"
#ifdef SLSTUDIO_PROJECTOR_LC4500
#include "LC4500API/dlpc350_usb.h"
#endif

#include <cstdint>
#include <stdexcept>

namespace slstudio {
#ifdef SLSTUDIO_PROJECTOR_DESKTOP
std::vector<ProjectorOutputInfo> desktopProjectorOutputs();
std::unique_ptr<Projector> openDesktopProjector(const ProjectorOutputInfo &output);
#endif
#ifdef SLSTUDIO_WITH_DRM
std::vector<ProjectorOutputInfo> leasedProjectorOutputs();
std::unique_ptr<Projector> openLeasedProjector(const ProjectorOutputInfo &output);
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
  if (settings.standby) return;
  validateLedCurrent(settings.ledCurrent);
  if (settings.frameRate < 1 || settings.frameRate > 60 || settings.triggerDelayUs < 0 ||
      settings.triggerPulseUs < 1 || settings.triggerDelayUs > 1000000 || settings.triggerPulseUs > 1000000)
    throw std::invalid_argument("LC3000 requires 1-60 Hz video and trigger timing within [0, 1000000] us with a positive pulse width");
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
      validateLightCrafter4500PatternTiming({settings.exposure, settings.framePeriod});
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
#ifdef SLSTUDIO_WITH_DRM
  outputs = leasedProjectorOutputs();
#endif
#ifdef SLSTUDIO_PROJECTOR_DESKTOP
  if (allowDesktop) {
    auto desktop = desktopProjectorOutputs();
    outputs.insert(outputs.end(), desktop.begin(), desktop.end());
  }
#else
  static_cast<void>(allowDesktop);
#endif
#ifdef SLSTUDIO_PROJECTOR_LC3000
  outputs.push_back({"lc3000", "LightCrafter 3000 (configured network device)", OutputBackend::LightCrafter3000, true});
#endif
#ifdef SLSTUDIO_PROJECTOR_LC4500
  outputs.push_back({"lc4500", "LightCrafter 4500 (USB)", OutputBackend::LightCrafter4500, true});
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
#ifdef SLSTUDIO_PROJECTOR_DESKTOP
    return openDesktopProjector(output);
#else
    break;
#endif
  case OutputBackend::WaylandLease:
  case OutputBackend::XRandRLease:
#ifdef SLSTUDIO_WITH_DRM
    return openLeasedProjector(output);
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
  }
  throw std::runtime_error("Requested projector backend is not available in this build");
}

}