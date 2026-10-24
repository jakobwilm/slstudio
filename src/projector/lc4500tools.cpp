#include "lc4500tools.h"
#include "LC4500API/dlpc350_firmware.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>

namespace slstudio {
namespace {
std::uint32_t word(const std::vector<unsigned char> &bytes, std::size_t offset) {
  return bytes[offset] | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
      (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
      (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
}
}

LightCrafter4500FlashUpdate prepareLightCrafter4500FlashUpdate(
    unsigned int firmwareVersion, const std::vector<unsigned char> &tableSector,
    const std::vector<cv::Mat3b> &images) {
  static_assert(sizeof(FLASH_TABLE) == 464, "Unexpected DLPC350 flash-table layout");
  if (firmwareVersion != 0x04040000)
    throw std::runtime_error("LC4500 pattern programming supports firmware 4.4.0 only");
  if ((tableSector.size() != sizeof(FLASH_TABLE) && tableSector.size() != lc4500FlashSectorSize) ||
      word(tableSector, 0) != FLASHTABLE_APP_SIGNATURE ||
      word(tableSector, 4) != lc4500FlashBase || word(tableSector, 8) != 0x13)
    throw std::runtime_error("Unsupported LC4500 flash table; refusing to erase flash");
  if (images.empty() || images.size() > 16)
    throw std::invalid_argument("LC4500 requires 1-16 packed pattern images");
  for (const auto &image : images)
    if (image.empty() || image.size() != images.front().size() || image.cols > 912 ||
        image.rows > 1140 || image.rows % 2 != 0)
      throw std::invalid_argument("LC4500 packed images must have matching dimensions up to 912x1140 and even height");

  const auto oldAddress = word(tableSector, offsetof(FLASH_TABLE, Splash_Data));
  if (oldAddress < lc4500FlashBase + 2 * lc4500FlashSectorSize ||
      oldAddress >= lc4500FlashBase + lc4500PatternFlashLimit)
    throw std::runtime_error("Invalid LC4500 pattern-region address");
  const auto offset = (oldAddress - lc4500FlashBase + lc4500FlashSectorSize - 1) /
      lc4500FlashSectorSize * lc4500FlashSectorSize;
  auto limit = lc4500PatternFlashLimit;
  for (std::size_t index = offsetof(FLASH_TABLE, AppCode); index < sizeof(FLASH_TABLE); index += sizeof(FLASH_BLOCK)) {
    const auto address = word(tableSector, index);
    const auto size = word(tableSector, index + 4);
    if ((address == 0 || address == 0xffffffff) && (size == 0 || size == 0xffffffff)) continue;
    if (address < lc4500FlashBase || address >= lc4500FlashBase + lc4500PatternFlashLimit ||
        size == 0 || size > lc4500PatternFlashLimit - (address - lc4500FlashBase))
      throw std::runtime_error("Invalid LC4500 flash-block bounds");
    const bool pattern = (index >= offsetof(FLASH_TABLE, Splash_Data) &&
        index < offsetof(FLASH_TABLE, APPL_OtherBinary)) ||
        (index >= offsetof(FLASH_TABLE, Splash_Data2) && index < offsetof(FLASH_TABLE, Batch_File));
    if (pattern) continue;
    const auto start = address - lc4500FlashBase;
    if (start < offset && start + size > offset)
      throw std::runtime_error("LC4500 pattern region overlaps protected firmware data");
    if (start >= offset)
      limit = std::min(limit, start / lc4500FlashSectorSize * lc4500FlashSectorSize);
  }
  if (offset >= limit)
    throw std::runtime_error("LC4500 has no safe sector-aligned pattern region");

  if (DLPC350_Frmw_SPLASH_InitBufferAtAddress(static_cast<int>(images.size()), lc4500FlashBase + offset) < 0)
    throw std::runtime_error("LC4500 pattern buffer initialization failed");
  for (const auto &image : images) {
    std::vector<unsigned char> bitmap;
    if (!cv::imencode(".bmp", image, bitmap)) throw std::runtime_error("LC4500 bitmap encoding failed");
    uint8 compression = SPLASH_NOCOMP_SPECIFIED;
    uint32 compressedSize = 0;
    if (DLPC350_Frmw_SPLASH_AddSplash(bitmap.data(), &compression, &compressedSize) < 0)
      throw std::runtime_error("LC4500 pattern image packing failed");
  }
  unsigned char *data = nullptr;
  uint32 size = 0;
  DLPC350_Frmw_Get_NewSplashBuffer(&data, &size);
  if (!data || size == 0 || size > limit - offset)
    throw std::runtime_error("LC4500 pattern images exceed the safe flash region");
  LightCrafter4500FlashUpdate update{offset, tableSector, {data, data + size}};
  const auto address = lc4500FlashBase + offset;
  for (unsigned int index = 0; index < 4; ++index)
    update.tableSector[offsetof(FLASH_TABLE, Splash_Data) + index] =
        static_cast<unsigned char>(address >> (index * 8));
  return update;
}
}
