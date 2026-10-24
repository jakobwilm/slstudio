#pragma once

#include <opencv2/core.hpp>

inline constexpr int DefaultCenterFringeOrder = 3;

// Multilevel quality-guided phase unwrapping (Zhang, Li and Yau, Applied Optics 46(1), 2007).
// Phase is in radians. The mask is updated to contain only reached pixels.
void spatialUnwrapping(cv::Mat &phase, cv::Mat &mask, int centerFringeOrder);
