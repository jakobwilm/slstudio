#pragma once

#include "Codec.h"

#include "CodecFastRatio.h"
#include "CodecGrayCode.h"
#include "CodecPhaseShift2p1.h"
#include "CodecPhaseShift2x3.h"
#include "CodecPhaseShift3.h"
#include "CodecPhaseShift3FastWrap.h"
#include "CodecPhaseShift3Unwrap.h"
#include "CodecPhaseShift4.h"
#include "CodecPhaseShiftDescatter.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecPhaseShiftModulated.h"
#include "CodecPhaseShiftNStep.h"

enum class CodecType {
  FastRatio,
  GrayCode,
  PhaseShift2p1,
  PhaseShift2x3,
  PhaseShift3,
  PhaseShift3Unwrap,
  PhaseShift3FastWrap,
  PhaseShift4,
  PhaseShiftDescatter,
  PhaseShiftMicro,
  PhaseShiftModulated,
  PhaseShiftNStep
};

static const std::map<const std::string, CodecType> Codecs{
    {"FastRatio", CodecType::FastRatio},
    {"GrayCode", CodecType::GrayCode},
    {"PhaseShift2p1", CodecType::PhaseShift2p1},
    {"PhaseShift2x3", CodecType::PhaseShift2x3},
    {"PhaseShift3", CodecType::PhaseShift3},
    {"PhaseShift3Unwrap", CodecType::PhaseShift3Unwrap},
    {"PhaseShift3FastWrap", CodecType::PhaseShift3FastWrap},
    {"PhaseShift4", CodecType::PhaseShift4},
    {"PhaseShiftDescatter", CodecType::PhaseShiftDescatter},
    {"PhaseShiftMicro", CodecType::PhaseShiftMicro},
    {"PhaseShiftModulated", CodecType::PhaseShiftModulated},
    {"PhaseShiftNStep", CodecType::PhaseShiftNStep}};

// Base class for all encoders
class EncoderFactory {
public:
  static std::unique_ptr<Encoder> NewEncoder(const CodecType type,
                                             unsigned int screenResX,
                                             unsigned int screenResY,
                                             CodecDir dir = CodecDirHorizontal);
};

class DecoderFactory {
public:
  static std::unique_ptr<Decoder> NewDecoder(const CodecType type,
                                             unsigned int screenResX,
                                             unsigned int screenResY,
                                             CodecDir dir = CodecDirHorizontal);
};
