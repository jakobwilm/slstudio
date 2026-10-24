#include "IntensityCalibrationLut.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>

namespace {
void writeLut(cv::FileStorage &file, const IntensityCalibrationLut &intensity) {
  if (intensity.inverseLut.front() != 0 || intensity.inverseLut.back() != 255 ||
      !std::is_sorted(intensity.inverseLut.begin(), intensity.inverseLut.end()))
    throw std::invalid_argument("LUT must be monotonic with endpoints 0 and 255");
  cv::Mat1b lut(1, 256);
  std::copy(intensity.inverseLut.begin(), intensity.inverseLut.end(), lut.ptr<std::uint8_t>());
  file << "inverse_lut" << lut;
}
}

void saveIntensityLut(const std::filesystem::path &path, const IntensityCalibrationLut &intensity) {
  cv::FileStorage file(path.string(), cv::FileStorage::WRITE);
  if (!file.isOpened()) throw std::runtime_error("Cannot save intensity LUT");
  file << "format_version" << 1;
  writeLut(file, intensity);
}

IntensityCalibrationLut loadIntensityLut(const std::filesystem::path &path) {
  cv::FileStorage file(path.string(), cv::FileStorage::READ);
  if (!file.isOpened()) throw std::runtime_error("Cannot read intensity LUT");
  if (!file["format_version"].empty() &&
      (!file["format_version"].isInt() || static_cast<int>(file["format_version"]) != 1))
    throw std::runtime_error("Unsupported intensity LUT format");
  cv::Mat lut;
  file["inverse_lut"] >> lut;
  if (lut.type() != CV_8UC1 || lut.rows != 1 || lut.cols != 256)
    throw std::runtime_error("Intensity Calibration LUT must contain a 1 x 256 byte inverse_lut");
  IntensityCalibrationLut intensity;
  std::copy_n(lut.ptr<std::uint8_t>(), 256, intensity.inverseLut.begin());
  if (intensity.inverseLut.front() != 0 || intensity.inverseLut.back() != 255 ||
      !std::is_sorted(intensity.inverseLut.begin(), intensity.inverseLut.end()))
    throw std::runtime_error("Invalid intensity LUT: expected monotonic values with endpoints 0 and 255");
  if (!file["samples"].empty()) {
    auto measured = analyzeIntensity(loadIntensitySamples(path));
    if (measured.inverseLut != intensity.inverseLut) throw std::runtime_error("Intensity Calibration LUT does not match its measurements");
    intensity = measured;
  }
  return intensity;
}

IntensitySample sampleIntensityFrame(const cv::Mat &image, const cv::Rect &region, int level) {
  if (image.empty() || image.dims != 2 || image.type() != CV_8UC1 || level < 0 || level > 255 ||
      region.x < 0 || region.y < 0 || region.width <= 0 || region.height <= 0 ||
      region.width > image.cols || region.height > image.rows ||
      region.x > image.cols - region.width || region.y > image.rows - region.height)
    throw std::invalid_argument("Intensity Calibration measurement requires a Mono8 image and an in-bounds ROI");
  const auto pixels = image(region);
  return {level, cv::mean(pixels)[0], static_cast<double>(cv::countNonZero(pixels >= 250)) /
      static_cast<double>(pixels.total())};
}

IntensityCalibrationLut analyzeIntensity(const std::vector<IntensitySample> &samples) {
  if (samples.size() < 5 || samples.front().level != 0 || samples.back().level != 255)
    throw std::invalid_argument("Measure at least five levels, including black and white");
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto &sample = samples[index];
    if (sample.level < 0 || sample.level > 255 || !std::isfinite(sample.brightness) ||
        sample.brightness < 0 || sample.brightness > 255 || !std::isfinite(sample.saturatedFraction) ||
        sample.saturatedFraction < 0 || sample.saturatedFraction > 1 ||
        (index && sample.level <= samples[index - 1].level))
      throw std::invalid_argument("Intensity Calibration samples must be finite and ordered by increasing input level");
    if (sample.saturatedFraction > 0.01)
      throw std::runtime_error("Saturated camera pixels: reduce exposure or gain and repeat");
  }
  IntensityCalibrationLut intensity;
  intensity.blackLevel = samples.front().brightness;
  intensity.whiteLevel = samples.back().brightness;
  const double range = intensity.whiteLevel - intensity.blackLevel;
  if (range < 10) throw std::runtime_error("Insufficient brightness range: check exposure and camera ROI");
  double maximum = intensity.blackLevel;
  std::vector<double> brightness;
  for (const auto &sample : samples) {
    if (sample.brightness + std::max(2.0, range * 0.03) < maximum)
      throw std::runtime_error("Non-monotonic intensity: check settling, ambient light and automatic camera controls");
    maximum = std::max(maximum, sample.brightness);
    brightness.push_back(std::clamp(maximum, intensity.blackLevel, intensity.whiteLevel));
    const double error = sample.brightness - intensity.blackLevel - range * sample.level / 255.0;
    intensity.linearityRmsError += error * error;
    intensity.linearityMaxError = std::max(intensity.linearityMaxError, std::abs(error));
  }
  intensity.linearityRmsError = std::sqrt(intensity.linearityRmsError / static_cast<double>(samples.size()));
  intensity.inverseLut.front() = 0;
  intensity.inverseLut.back() = 255;
  for (std::size_t index = 1; index + 1 < intensity.inverseLut.size(); ++index) {
    const double target = intensity.blackLevel + range * index / 255.0;
    const auto upper = static_cast<std::size_t>(std::lower_bound(brightness.begin(), brightness.end(), target) - brightness.begin());
    const double fraction = (target - brightness[upper - 1]) / (brightness[upper] - brightness[upper - 1]);
    intensity.inverseLut[index] = static_cast<std::uint8_t>(std::lround(samples[upper - 1].level +
        fraction * (samples[upper].level - samples[upper - 1].level)));
  }
  return intensity;
}

void saveIntensityLut(const std::filesystem::path &path, const std::vector<IntensitySample> &samples,
                     const IntensityCalibrationLut &intensity) {
  const auto verified = analyzeIntensity(samples);
  if (verified.blackLevel != intensity.blackLevel || verified.whiteLevel != intensity.whiteLevel ||
      verified.linearityRmsError != intensity.linearityRmsError || verified.linearityMaxError != intensity.linearityMaxError ||
      verified.inverseLut != intensity.inverseLut)
    throw std::invalid_argument("Intensity Calibration LUT does not match the measurements");
  cv::FileStorage file(path.string(), cv::FileStorage::WRITE);
  if (!file.isOpened()) throw std::runtime_error("Cannot save intensity LUT");
  file << "black_level" << intensity.blackLevel << "white_level" << intensity.whiteLevel
       << "linearity_rms_error_dn" << intensity.linearityRmsError << "linearity_max_error_dn" << intensity.linearityMaxError
       << "inverse_method" << "piecewise_linear_monotone_envelope";
  writeLut(file, intensity);
  file << "samples" << "[";
  for (const auto &sample : samples)
    file << "{" << "level" << sample.level << "brightness" << sample.brightness << "saturated_fraction" << sample.saturatedFraction << "}";
  file << "]";
}

void saveIntensityCsv(const std::filesystem::path &path, const std::vector<IntensitySample> &samples) {
  if (samples.empty()) throw std::invalid_argument("No intensity measurements to export");
  std::ofstream file(path);
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file.imbue(std::locale::classic());
  file << "level,brightness,saturated_fraction\n" << std::setprecision(12);
  for (const auto &sample : samples)
    file << sample.level << ',' << sample.brightness << ',' << sample.saturatedFraction << '\n';
  file.close();
}

std::vector<IntensitySample> loadIntensitySamples(const std::filesystem::path &path) {
  cv::FileStorage file(path.string(), cv::FileStorage::READ);
  if (!file.isOpened() || !file["samples"].isSeq()) throw std::runtime_error("Cannot read intensity measurements");
  std::vector<IntensitySample> samples;
  for (const auto &node : file["samples"]) {
    if (!node.isMap() || !node["level"].isInt() ||
        (!node["brightness"].isReal() && !node["brightness"].isInt()) ||
        (!node["saturated_fraction"].isReal() && !node["saturated_fraction"].isInt()))
      throw std::runtime_error("Invalid intensity measurement");
    samples.push_back({static_cast<int>(node["level"]), static_cast<double>(node["brightness"]),
        static_cast<double>(node["saturated_fraction"])});
  }
  analyzeIntensity(samples);
  return samples;
}

cv::Mat compensateIntensity(const cv::Mat &pattern, const IntensityCalibrationLut &intensity) {
  if (pattern.empty() || pattern.dims != 2 || pattern.depth() != CV_8U)
    throw std::invalid_argument("Intensity Calibration compensation requires an 8-bit pattern");
  cv::Mat1b lut(1, 256);
  std::copy(intensity.inverseLut.begin(), intensity.inverseLut.end(), lut.ptr<std::uint8_t>());
  cv::Mat compensated;
  cv::LUT(pattern, lut, compensated);
  return compensated;
}