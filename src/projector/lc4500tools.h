#pragma once

#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>

namespace slstudio {

constexpr std::uint32_t lc4500FlashBase = 0xf9000000;
constexpr std::uint32_t lc4500FlashSectorSize = 128 * 1024;
constexpr std::uint32_t lc4500PatternFlashLimit = 16 * 1024 * 1024;

struct LightCrafter4500FlashUpdate {
  std::uint32_t patternOffset;
  std::vector<unsigned char> tableSector;
  std::vector<unsigned char> patternData;
};

LightCrafter4500FlashUpdate prepareLightCrafter4500FlashUpdate(
    unsigned int firmwareVersion, const std::vector<unsigned char> &tableSector,
    const std::vector<cv::Mat3b> &images);

}
