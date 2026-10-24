#include "ProjectorFactory.h"
#include "ProjectorLightCrafter3000.h"
#include "ProjectorLightCrafter4500.h"

#include <iostream>
#include <cstddef>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

struct MemoryProjector : slstudio::Projector {
  std::vector<cv::Mat1b> patterns;
  cv::Mat displayed;
  cv::Size resolution() const override { return {6, 3}; }
  void setPatterns(const std::vector<cv::Mat> &input) override {
    patterns = slstudio::prepareMonochromeProjectorPatterns(input, geometry(), 12);
  }
  void present(std::size_t index) override { displayed = patterns.at(index).clone(); }
  void blank(bool white = false) override { showSolid(white ? 255 : 0); }
};

int main() {
  try {
    require(slstudio::lightCrafter4500VersionString(0x04040000) == "4.4.0" &&
        slstudio::lightCrafter4500VersionString(0x01020304) == "1.2.772",
        "LC4500 firmware version did not preserve the 16-bit patch number");
    for (int level = 0; level <= 255; ++level)
      require(slstudio::lightCrafter4500LedPwm(static_cast<std::uint8_t>(level)) == 255 - level,
          "LC4500 LED brightness must map to the inverted hardware PWM scale");
    for (int operation = 0; operation < 2; ++operation) {
      bool unavailable = false;
      try {
        if (operation == 0) slstudio::lightCrafter4500Status();
        else slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Blank);
      } catch (const std::runtime_error &error) {
        unavailable = std::string(error.what()) == "LC4500 support was not built";
      }
      require(unavailable, "LC4500 status/control must report unavailable backend support");
    }
    MemoryProjector projector;
    for (const auto level : {0, 128, 255}) {
      projector.showSolid(static_cast<std::uint8_t>(level));
      require(cv::countNonZero(projector.displayed != level) == 0,
          "Projector solid field did not preserve the requested intensity");
    }
    std::vector<cv::Mat> sequence;
    for (int index = 0; index < 12; ++index)
      sequence.emplace_back(projector.resolution(), CV_8UC1, cv::Scalar(index * 20));
    projector.setPatterns(sequence);
    for (auto &image : sequence) image.setTo(255);
    for (int repetition = 0; repetition < 2; ++repetition)
      for (std::size_t index = 0; index < projector.patterns.size(); ++index) {
        projector.present(index);
        require(cv::countNonZero(projector.displayed != static_cast<int>(index) * 20) == 0,
            "Projector cycling changed pattern order or borrowed caller memory");
      }
    const auto sequenceImages = slstudio::packLightCrafter4500Patterns(projector.patterns);
    require(sequenceImages.size() == 4, "Twelve projector patterns did not pack into four flash images");
    for (std::size_t index = 0; index < projector.patterns.size(); ++index)
      require(sequenceImages[index / 3](0, 0)[(index + 1) % 3] == static_cast<int>(index) * 20,
          "LC4500 packed pattern order changed");
    for (const bool white : {true, false, true, false}) {
      projector.blank(white);
      require(cv::countNonZero(projector.displayed != (white ? 255 : 0)) == 0,
          "Projector white/black cycling failed");
    }
    cv::Mat pixels(5, 10, CV_8UC3);
    for (int row = 0; row < pixels.rows; ++row)
      for (int column = 0; column < pixels.cols; ++column)
        pixels.at<cv::Vec3b>(row, column) = cv::Vec3b(row, column, row + column);
    const auto logical = pixels(cv::Rect(1, 1, 6, 3));
    const auto native = slstudio::prepareProjectorPattern(logical,
        {{6, 3}, {3, 3}, slstudio::PixelLayout::Diamond});
    for (int row = 0; row < native.rows; ++row)
      for (int column = 0; column < native.cols; ++column)
        require(native.at<cv::Vec3b>(row, column) == logical.at<cv::Vec3b>(row, column * 2 + row % 2),
                "Diamond mapping changed row, parity, orientation or channel order");
    const auto rectangular = slstudio::prepareProjectorPattern(logical, {{6, 3}, {6, 3}});
    require(cv::norm(rectangular, logical, cv::NORM_INF) == 0 && rectangular.data != logical.data,
            "Rectangular preparation must make an unchanged owned snapshot");
    cv::Mat gray;
    cv::extractChannel(logical, gray, 1);
    const auto grayNative = slstudio::prepareProjectorPattern(gray,
        {{6, 3}, {3, 3}, slstudio::PixelLayout::Diamond});
    require(grayNative.at<unsigned char>(1, 2) == gray.at<unsigned char>(1, 5),
            "Grayscale diamond mapping failed");
    const auto packed = slstudio::packLightCrafter4500Patterns({
      cv::Mat1b(2, 3, static_cast<unsigned char>(11)), cv::Mat1b(2, 3, static_cast<unsigned char>(22)),
      cv::Mat1b(2, 3, static_cast<unsigned char>(33)), cv::Mat1b(2, 3, static_cast<unsigned char>(44))});
    require(packed.size() == 2 && packed[0](0, 0) == cv::Vec3b(33, 11, 22) &&
      packed[1](1, 2) == cv::Vec3b(0, 44, 0), "LC4500 GRB packing or unused-channel clearing failed");
    bool rejected = false;
    try { slstudio::prepareProjectorPattern(logical, {{6, 3}, {6, 3}, slstudio::PixelLayout::Diamond}); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Invalid diamond geometry was accepted");
    rejected = false;
    try { slstudio::prepareProjectorPattern(cv::Mat(3, 6, CV_32F), {{6, 3}, {6, 3}}); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Float patterns were silently converted");
    rejected = false;
    try { slstudio::openProjector({"0", "Desktop", slstudio::OutputBackend::Desktop, true}); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "A forged exclusive flag bypassed the desktop opt-in");
    rejected = false;
    try { slstudio::openProjector({"missing", "Unknown", static_cast<slstudio::OutputBackend>(99), true}); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Unknown projector backend was accepted");
    slstudio::ProjectorSequenceSettings timing;
    if (!slstudio::lightCrafterAvailable(slstudio::OutputBackend::LightCrafter4500))
      require(!slstudio::lightCrafter4500Connected(), "Unavailable LC4500 backend reported a USB connection");
    slstudio::validateProjectorSequence(timing);
    timing.exposure = std::chrono::microseconds(0);
    rejected = false;
    try { slstudio::validateProjectorSequence(timing); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Zero exposure was accepted");
    timing.exposure = timing.framePeriod + std::chrono::microseconds(1);
    rejected = false;
    try { slstudio::validateProjectorSequence(timing); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Exposure longer than the frame period was accepted");
    timing.exposure = std::chrono::microseconds(8333);
    for (const int darkTime : {0, 1, 229, 230, 231, 1000}) {
      timing.framePeriod = timing.exposure + std::chrono::microseconds(darkTime);
      rejected = false;
      try { slstudio::validateLightCrafter4500Sequence(timing); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected == (darkTime > 0 && darkTime <= 230),
          "LC4500 dark time must be zero or strictly greater than 230 us");
    }
    for (const int exposure : {8332, 8333, 1000000, 1000001}) {
      timing.exposure = timing.framePeriod = std::chrono::microseconds(exposure);
      rejected = false;
      try { slstudio::validateLightCrafter4500Sequence(timing); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected == (exposure < 8333 || exposure > 1000000),
          "LC4500 eight-bit timing limits changed");
    }
    slstudio::LightCrafter3000Settings lc3000Settings;
    slstudio::LightCrafter4500Settings requestedSettings;
    slstudio::validateLightCrafterSettings(lc3000Settings);
    slstudio::validateLightCrafterSettings(requestedSettings);
    lc3000Settings.ledCurrent[0] = 256;
    rejected = false;
    try { slstudio::validateLightCrafterSettings(lc3000Settings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Out-of-range LED level accepted");
    lc3000Settings.ledCurrent[0] = 90;
    lc3000Settings.frameRate = 0;
    rejected = false;
    try { slstudio::validateLightCrafterSettings(lc3000Settings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Invalid LC3000 HDMI frame rate accepted");
    requestedSettings.ledCurrent[1] = -1;
    rejected = false;
    try { slstudio::validateLightCrafterSettings(requestedSettings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Out-of-range LC4500 LED level accepted");
    requestedSettings.ledCurrent[1] = 0;
    for (const int selection : {0, 8}) {
      requestedSettings.ledSelection = selection;
      rejected = false;
      try { slstudio::validateLightCrafterSettings(requestedSettings); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, "Invalid LC4500 LED selection accepted");
    }
    requestedSettings.ledSelection = 1;
    requestedSettings.exposure = std::chrono::microseconds(100);
    rejected = false;
    try { slstudio::validateLightCrafterSettings(requestedSettings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Unsupported eight-bit HDMI exposure accepted");
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::FlashPatterns;
    slstudio::validateLightCrafterSettings(requestedSettings);
    for (const auto mode : {slstudio::LightCrafter4500Mode::HdmiPatterns,
                           slstudio::LightCrafter4500Mode::FlashPatterns}) {
      requestedSettings.displayMode = mode;
      requestedSettings.exposure = std::chrono::microseconds(8333);
      for (const int darkTime : {0, 1, 230, 231}) {
        requestedSettings.framePeriod = requestedSettings.exposure + std::chrono::microseconds(darkTime);
        rejected = false;
        try { slstudio::validateLightCrafterSettings(requestedSettings); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected == (darkTime > 0 && darkTime <= 230),
            "LC4500 dialog modes must share the hardware dark-time constraint");
      }
    }
    requestedSettings.exposure = std::chrono::microseconds(-1);
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::HdmiVideo;
    slstudio::validateLightCrafterSettings(requestedSettings);
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::TestPattern;
    for (int pattern = 0; pattern <= 10; ++pattern) {
      requestedSettings.testPattern = pattern;
      slstudio::validateLightCrafterSettings(requestedSettings);
    }
    requestedSettings.testPattern = 11;
    rejected = false;
    try { slstudio::validateLightCrafterSettings(requestedSettings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Unknown LC4500 internal test pattern accepted");
    requestedSettings.displayMode = static_cast<slstudio::LightCrafter4500Mode>(99);
    rejected = false;
    try { slstudio::validateLightCrafterSettings(requestedSettings); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Unknown LC4500 display mode accepted");
    std::cout << "Projector geometry and pattern ownership passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}