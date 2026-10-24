#include "IntensityCalibrationLut.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

int main() {
  try {
    std::vector<IntensitySample> samples;
    for (int index = 0; index <= 16; ++index) {
      const int level = (index * 255 + 8) / 16;
      samples.push_back({level, 12 + 210 * std::pow(level / 255.0, 2.2), 0});
    }
    const auto intensity = analyzeIntensity(samples);
    const auto validSamples = samples;
          require(intensity.linearityRmsError > 10 && intensity.linearityMaxError > 30,
            "Nonlinear intensity was not detected");
          const std::vector<IntensitySample> arbitrary{{0, 10, 0}, {64, 30, 0}, {128, 100, 0}, {192, 150, 0}, {255, 230, 0}};
          const auto measured = analyzeIntensity(arbitrary);
          require(measured.inverseLut[32] == 71 && measured.inverseLut[128] == 154,
            "Inverse LUT does not interpolate the measured curve");
          cv::Mat1b inputs(1, 3);
          inputs(0, 0) = 0; inputs(0, 1) = 128; inputs(0, 2) = 255;
          const auto corrected = compensateIntensity(inputs, measured);
          require(corrected.at<std::uint8_t>(0, 0) == 0 && corrected.at<std::uint8_t>(0, 1) == 154 &&
        corrected.at<std::uint8_t>(0, 2) == 255 && inputs(0, 1) == 128, "Pattern compensation failed");
          auto linear = validSamples;
          for (auto &entry : linear) entry.brightness = 12 + 210 * entry.level / 255.0;
          const auto ideal = analyzeIntensity(linear);
          require(ideal.linearityRmsError < 1e-10 && ideal.linearityMaxError < 1e-10, "Linear intensity was not recognized");
          for (std::size_t index = 0; index < ideal.inverseLut.size(); ++index)
      require(ideal.inverseLut[index] == index, "Linear intensity compensation is not identity");
          require(intensity.blackLevel == 12 && intensity.whiteLevel == 222,
            "Intensity Calibration analysis failed to preserve the black and white endpoints");
    require(intensity.inverseLut.front() == 0 && intensity.inverseLut.back() == 255 &&
        std::is_sorted(intensity.inverseLut.begin(), intensity.inverseLut.end()), "Intensity Calibration inverse LUT is invalid");
    cv::Mat1b image(6, 8, static_cast<unsigned char>(40));
    image(2, 3) = 255;
    const auto sample = sampleIntensityFrame(image, {2, 1, 2, 2}, 128);
    require(sample.saturatedFraction == 0.25 && sample.brightness == 93.75, "Intensity Calibration ROI statistics are incorrect");
    bool rejected = false;
    try { sampleIntensityFrame(image, {7, 0, 2, 2}, 128); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Out-of-bounds intensity ROI accepted");
    samples.back().saturatedFraction = 0.2;
    rejected = false;
    try { analyzeIntensity(samples); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Saturated intensity data accepted");
    samples.back().saturatedFraction = 0;
    for (auto &entry : samples) entry.brightness = 40;
    rejected = false;
    try { analyzeIntensity(samples); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Flat intensity data accepted");
    samples = validSamples;
    samples.back().brightness = samples[samples.size() - 2].brightness - 20;
    rejected = false;
    try { analyzeIntensity(samples); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Non-monotonic white endpoint accepted");
    samples = validSamples;
    samples[8].brightness = samples[7].brightness - 20;
    rejected = false;
    try { analyzeIntensity(samples); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Non-monotonic midtone accepted");
    auto forged = intensity;
    forged.blackLevel += 1;
    rejected = false;
    try { saveIntensityLut({}, validSamples, forged); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Inconsistent intensity result could be exported");
    std::cout << "Intensity Calibration fit, inverse LUT, ROI, saturation and contrast checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}