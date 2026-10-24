#pragma once

#include <opencv2/core.hpp>
#include <array>
#include <cstdint>
#include <filesystem>
#include <vector>

struct IntensitySample {
  int level = 0;
  double brightness = 0;
  double saturatedFraction = 0;
};

struct IntensityCalibrationLut {
  double blackLevel = 0, whiteLevel = 255;
  double linearityRmsError = 0, linearityMaxError = 0;
  std::array<std::uint8_t, 256> inverseLut{};
};

IntensitySample sampleIntensityFrame(const cv::Mat &image, const cv::Rect &region, int level);
IntensityCalibrationLut analyzeIntensity(const std::vector<IntensitySample> &samples);
void saveIntensityLut(const std::filesystem::path &path, const std::vector<IntensitySample> &samples,
                     const IntensityCalibrationLut &intensity);
void saveIntensityLut(const std::filesystem::path &path, const IntensityCalibrationLut &intensity);
IntensityCalibrationLut loadIntensityLut(const std::filesystem::path &path);
void saveIntensityCsv(const std::filesystem::path &path, const std::vector<IntensitySample> &samples);
std::vector<IntensitySample> loadIntensitySamples(const std::filesystem::path &path);
cv::Mat compensateIntensity(const cv::Mat &pattern, const IntensityCalibrationLut &intensity);