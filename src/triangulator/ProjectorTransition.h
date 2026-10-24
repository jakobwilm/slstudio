#pragma once

#include "Codec.h"
#include <cstdint>

// Camera coordinates are distorted image coordinates; projector coordinates use
// integer pixel centers, so a boundary between pixels k and k+1 is at k+0.5.
struct ProjectorTransition {
  cv::Point2f camera;
  CodecDir direction = CodecDirHorizontal;
  float coordinate = 0;
  std::uint8_t shading = 0;
};
