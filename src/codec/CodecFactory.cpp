#include "CodecFactory.h"
#include "CodecGrayCode.h"
#include "CodecPhaseShiftMotionCompensated.h"
#include "CodecPhaseShiftHeterodyne.h"
#include "CodecPhaseShiftSpatialUnwrap.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecPhaseShiftModulated.h"
#include "CodecPhaseShift.h"

#include <string>

const char *codecName(CodecType type) {
  for (const auto &codec : Codecs)
    if (codec.type == type) return codec.name;
  throw std::invalid_argument("Unknown codec type");
}

CodecType codecTypeFromName(std::string_view name) {
  for (const auto &codec : Codecs)
    if (codec.name == name) return codec.type;
  throw std::invalid_argument("Unknown codec: " + std::string(name));
}

std::unique_ptr<Encoder> EncoderFactory::NewEncoder(CodecType type, unsigned int screenResX,
                                                    unsigned int screenResY, CodecDir dir,
                                                    const CodecAlgorithmSettings &settings) {
  validateCodecSettings(settings);
  switch (type) {
  case CodecType::GrayCode:
    return std::make_unique<EncoderGrayCode>(screenResX, screenResY, dir, settings.grayCode);
  case CodecType::PhaseShift:
    return std::make_unique<EncoderPhaseShift>(screenResX, screenResY, dir, settings.phaseShift);
  case CodecType::PhaseShiftMotionCompensated:
    return std::make_unique<EncoderPhaseShiftMotionCompensated>(screenResX, screenResY, dir);
  case CodecType::PhaseShiftSpatialUnwrap:
    return std::make_unique<EncoderPhaseShiftSpatialUnwrap>(screenResX, screenResY, dir, settings.spatialUnwrap);
  case CodecType::PhaseShiftMicro:
    return std::make_unique<EncoderPhaseShiftMicro>(screenResX, screenResY, dir, settings.micro);
  case CodecType::PhaseShiftModulated:
    return std::make_unique<EncoderPhaseShiftModulated>(screenResX, screenResY, dir, settings.modulated);
  case CodecType::PhaseShiftHeterodyne:
    return std::make_unique<EncoderPhaseShiftHeterodyne>(screenResX, screenResY, dir, settings.heterodyne);
  }
  throw std::invalid_argument("Unknown encoder codec");
}

std::unique_ptr<Decoder> DecoderFactory::NewDecoder(CodecType type, unsigned int screenResX,
                                                    unsigned int screenResY, CodecDir dir,
                                                    bool motionCompensation, const CodecAlgorithmSettings &settings,
                                                    int centerFringeOrder) {
  validateCodecSettings(settings);
  switch (type) {
  case CodecType::GrayCode:
    return std::make_unique<DecoderGrayCode>(screenResX, screenResY, dir, settings.grayCode);
  case CodecType::PhaseShift:
    return std::make_unique<DecoderPhaseShift>(screenResX, screenResY, dir, settings.phaseShift);
  case CodecType::PhaseShiftMotionCompensated:
    return std::make_unique<DecoderPhaseShiftMotionCompensated>(screenResX, screenResY, dir, motionCompensation);
  case CodecType::PhaseShiftSpatialUnwrap:
    return std::make_unique<DecoderPhaseShiftSpatialUnwrap>(screenResX, screenResY, dir,
                                                          settings.spatialUnwrap, centerFringeOrder);
  case CodecType::PhaseShiftMicro:
    return std::make_unique<DecoderPhaseShiftMicro>(screenResX, screenResY, dir, settings.micro);
  case CodecType::PhaseShiftModulated:
    return std::make_unique<DecoderPhaseShiftModulated>(screenResX, screenResY, dir, settings.modulated);
  case CodecType::PhaseShiftHeterodyne:
    return std::make_unique<DecoderPhaseShiftHeterodyne>(screenResX, screenResY, dir, settings.heterodyne);
  }
  throw std::invalid_argument("Unknown decoder codec");
}

unsigned int codecPatternCount(CodecType type, CodecDir direction, const CodecAlgorithmSettings &settings) {
  validateCodecDimensions(1, 1, direction);
  validateCodecSettings(settings);
  if (type == CodecType::PhaseShift)
    return (settings.phaseShift.steps + (settings.phaseShift.periods > 1 ? 3 : 0)) *
        (direction == CodecDirBoth ? 2 : 1);
  if (type == CodecType::GrayCode)
    return 2 + (direction & CodecDirHorizontal ? settings.grayCode.horizontalBits : 0) +
        (direction & CodecDirVertical ? settings.grayCode.verticalBits : 0);
  if (type == CodecType::PhaseShiftHeterodyne)
    return 2 * settings.heterodyne.steps * (direction == CodecDirBoth ? 2 : 1);
  if (type == CodecType::PhaseShiftSpatialUnwrap) return settings.spatialUnwrap.steps;
  if (type == CodecType::PhaseShiftMicro) return settings.micro.steps + MicroPhasePitches.size() - 1;
  if (type == CodecType::PhaseShiftModulated)
    return settings.modulated.signalSteps * settings.modulated.carrierSteps + settings.modulated.cueSteps;
  for (const auto &codec : Codecs)
    if (codec.type == type) return codec.patterns * (codec.directional && direction == CodecDirBoth ? 2 : 1);
  throw std::invalid_argument("Unknown codec type");
}
