#pragma once

#include "Codec.h"
#include "spatialUnwrapping.h"

#include <array>
#include <memory>
#include <string_view>

enum class CodecType {
  PhaseShift,
  GrayCode,
  PhaseShiftMotionCompensated,
  PhaseShiftSpatialUnwrap,
  PhaseShiftMicro,
  PhaseShiftModulated,
  PhaseShiftHeterodyne = 7
};

struct CodecSettings {
  CodecType codec = CodecType::PhaseShift;
  CodecDir direction = CodecDirHorizontal;
  CodecAlgorithmSettings algorithmSettings;
};

inline bool operator==(const CodecSettings &a, const CodecSettings &b) {
  return a.codec == b.codec && a.direction == b.direction && a.algorithmSettings == b.algorithmSettings;
}

struct CodecInfo {
  const char *name;
  CodecType type;
  unsigned int patterns;
  bool directional = false;
};

inline constexpr std::array<CodecInfo, 7> Codecs{{
    {"PhaseShift", CodecType::PhaseShift, 0, true},
    {"GrayCode", CodecType::GrayCode, 0, true},
    {"PhaseShiftMotionCompensated", CodecType::PhaseShiftMotionCompensated, 3},
    {"PhaseShiftSpatialUnwrap", CodecType::PhaseShiftSpatialUnwrap, 0},
    {"PhaseShiftMicro", CodecType::PhaseShiftMicro, 0},
    {"PhaseShiftModulated", CodecType::PhaseShiftModulated, 0},
    {"PhaseShiftHeterodyne", CodecType::PhaseShiftHeterodyne, 0, true}}};

const char *codecName(CodecType type);
CodecType codecTypeFromName(std::string_view name);
unsigned int codecPatternCount(CodecType type, CodecDir direction, const CodecAlgorithmSettings &settings = {});

class EncoderFactory {
public:
  static std::unique_ptr<Encoder> NewEncoder(CodecType type, unsigned int screenResX,
                                            unsigned int screenResY,
                                            CodecDir dir = CodecDirHorizontal,
                                            const CodecAlgorithmSettings &settings = {});
};

class DecoderFactory {
public:
  // Motion compensation applies only to PhaseShiftMotionCompensated.
  // Center fringe order applies only to PhaseShiftSpatialUnwrap.
  static std::unique_ptr<Decoder> NewDecoder(CodecType type, unsigned int screenResX,
                                            unsigned int screenResY,
                                            CodecDir dir = CodecDirHorizontal,
                                            bool motionCompensation = true,
                                            const CodecAlgorithmSettings &settings = {},
                                            int centerFringeOrder = DefaultCenterFringeOrder);
};
