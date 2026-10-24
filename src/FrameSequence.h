#pragma once

#include "CodecFactory.h"
#include <filesystem>

struct FrameSequence {
  std::vector<cv::Mat> frames;
  // Records acquisition parameters only; combine with active ReconstructionSettings when decoding.
  CodecSettings coding;
};

std::vector<cv::Mat> loadFrameSequence(const std::filesystem::path &directory);

FrameSequence loadSequence(const std::filesystem::path &directory,
                           const CodecSettings &fallback = {});
void saveSequence(const std::filesystem::path &directory, const FrameSequence &sequence);
