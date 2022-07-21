#include "CodecFactory.h"

#include <memory>

std::unique_ptr<Encoder> EncoderFactory::NewEncoder(const CodecType type,
                                                    unsigned int screenResX,
                                                    unsigned int screenResY,
                                                    CodecDir dir) {

  std::unique_ptr<Encoder> encoder;

  if (type == CodecType::PhaseShift3) {
    encoder = std::make_unique<EncoderPhaseShift3>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift4) {
    encoder = std::make_unique<EncoderPhaseShift4>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift2x3) {
    encoder =
        std::make_unique<EncoderPhaseShift2x3>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift3Unwrap) {
    encoder =
        std::make_unique<EncoderPhaseShift3Unwrap>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShiftNStep) {
    encoder =
        std::make_unique<EncoderPhaseShiftNStep>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift3FastWrap) {
    encoder = std::make_unique<EncoderPhaseShift3FastWrap>(screenResX,
                                                           screenResY, dir);
  } else if (type == CodecType::PhaseShift2p1) {
    encoder =
        std::make_unique<EncoderPhaseShift2p1>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShiftDescatter) {
    encoder = std::make_unique<EncoderPhaseShiftDescatter>(screenResX,
                                                           screenResY, dir);
  } else if (type == CodecType::PhaseShiftModulated) {
    encoder = std::make_unique<EncoderPhaseShiftModulated>(screenResX,
                                                           screenResY, dir);
  } else if (type == CodecType::PhaseShiftMicro) {
    encoder =
        std::make_unique<EncoderPhaseShiftMicro>(screenResX, screenResY, dir);
  } else if (type == CodecType::FastRatio) {
    encoder = std::make_unique<EncoderFastRatio>(screenResX, screenResY, dir);
  } else if (type == CodecType::GrayCode) {
    encoder = std::make_unique<EncoderGrayCode>(screenResX, screenResY, dir);
  }

  return encoder;
}

std::unique_ptr<Decoder> DecoderFactory::NewDecoder(const CodecType type,
                                                    unsigned int screenResX,
                                                    unsigned int screenResY,
                                                    CodecDir dir) {

  std::unique_ptr<Decoder> decoder;

  if (type == CodecType::PhaseShift3) {
    decoder = std::make_unique<DecoderPhaseShift3>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift4) {
    decoder = std::make_unique<DecoderPhaseShift4>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift2x3) {
    decoder =
        std::make_unique<DecoderPhaseShift2x3>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift3Unwrap) {
    decoder =
        std::make_unique<DecoderPhaseShift3Unwrap>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShiftNStep) {
    decoder =
        std::make_unique<DecoderPhaseShiftNStep>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShift3FastWrap) {
    decoder = std::make_unique<DecoderPhaseShift3FastWrap>(screenResX,
                                                           screenResY, dir);
  } else if (type == CodecType::PhaseShift2p1) {
    decoder =
        std::make_unique<DecoderPhaseShift2p1>(screenResX, screenResY, dir);
  } else if (type == CodecType::PhaseShiftDescatter) {
    decoder = std::make_unique<DecoderPhaseShiftDescatter>(screenResX,
                                                           screenResY, dir);
  } else if (type == CodecType::PhaseShiftModulated) {
    decoder = std::make_unique<DecoderPhaseShiftModulated>(screenResX,
                                                           screenResY, dir);
  } else if (type == CodecType::PhaseShiftMicro) {
    decoder =
        std::make_unique<DecoderPhaseShiftMicro>(screenResX, screenResY, dir);
  } else if (type == CodecType::FastRatio) {
    decoder = std::make_unique<DecoderFastRatio>(screenResX, screenResY, dir);
  } else if (type == CodecType::GrayCode) {
    decoder = std::make_unique<DecoderGrayCode>(screenResX, screenResY, dir);
  }

  return decoder;
}
