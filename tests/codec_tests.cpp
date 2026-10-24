#include "CodecFactory.h"
#include "CodecPhaseShiftMotionCompensated.h"
#include <random>
#include "CodecGrayCode.h"
#include "CodecPhaseShift.h"
#include "CodecPhaseShiftMicro.h"
#include "CodecPhaseShiftSpatialUnwrap.h"
#include "pstools.h"
#include "DecoderWorker.h"
#include "TriangulatorWorker.h"
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function> void rejects(Function action) {
  bool rejected = false;
  try { action(); } catch (const std::exception &) { rejected = true; }
  require(rejected, "Invalid codec settings was accepted");
}

std::vector<cv::Mat> render(const Encoder &encoder, cv::Size size, double gain = 1) {
  std::vector<cv::Mat> frames;
  for (unsigned int index = 0; index < encoder.getNPatterns(); ++index) {
    const auto pattern = encoder.getEncodingPattern(index);
    cv::Mat gray;
    cv::cvtColor(cv::repeat(pattern, (size.height + pattern.rows - 1) / pattern.rows,
        (size.width + pattern.cols - 1) / pattern.cols)(cv::Rect({}, size)), gray, cv::COLOR_RGB2GRAY);
    gray.convertTo(gray, CV_8U, gain);
    frames.push_back(std::move(gray));
  }
  return frames;
}

void testCompleteSequenceInput() {
  const cv::Size size(127, 21);
  for (const auto &codec : Codecs) {
    auto encoder = EncoderFactory::NewEncoder(codec.type, size.width, size.height);
    auto decoder = DecoderFactory::NewDecoder(codec.type, size.width, size.height);
    const auto frames = render(*encoder, size);
    std::vector<cv::Mat> original;
    for (const auto &frame : frames) original.push_back(frame.clone());
    cv::Mat up, vp, mask, shading;
    decoder->decodeFrames(frames, up, vp, mask, shading);
    const auto expectedUp = up.clone(), expectedMask = mask.clone();
    rejects([&] { decoder->decodeFrames({}, up, vp, mask, shading); });
    {
      auto invalid = frames;
      invalid.pop_back();
      rejects([&] { decoder->decodeFrames(invalid, up, vp, mask, shading); });
      invalid = frames;
      invalid.push_back(frames.front());
      rejects([&] { decoder->decodeFrames(invalid, up, vp, mask, shading); });
      invalid = frames;
      invalid[0].release();
      rejects([&] { decoder->decodeFrames(invalid, up, vp, mask, shading); });
      invalid[0] = cv::Mat1f(size, 0.0f);
      rejects([&] { decoder->decodeFrames(invalid, up, vp, mask, shading); });
      invalid[0] = cv::Mat1b(size.height + 1, size.width, std::uint8_t(0));
      rejects([&] { decoder->decodeFrames(invalid, up, vp, mask, shading); });
    }
    decoder->decodeFrames(frames, up, vp, mask, shading);
    require(cv::norm(up, expectedUp, cv::NORM_INF) == 0 &&
            cv::norm(mask, expectedMask, cv::NORM_INF) == 0,
            "Rejected input changed the next complete-sequence decode");
    up.release();
    vp.release();
    mask.release();
    shading.release();
    for (std::size_t index = 0; index < frames.size(); ++index) {
      require(cv::norm(frames[index], original[index], cv::NORM_INF) == 0,
              "Decoder modified input frames");
      require(frames[index].u->refcount == 1, "Decoder retained an input image after decoding");
    }
  }
}

void testPhaseShift() {
  const cv::Size size(127, 93);
  for (const auto parameters : {PhaseShiftSettings{3, 16}, {3, 1}, {4, 1}, {5, 4},
                                {9, 24}, {16, 16}, {256, 1}}) {
    for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
      CodecAlgorithmSettings settings;
      settings.phaseShift = parameters;
      auto encoder = EncoderFactory::NewEncoder(CodecType::PhaseShift, size.width, size.height, direction, settings);
      auto decoder = DecoderFactory::NewDecoder(CodecType::PhaseShift, size.width, size.height, direction, true, settings);
      const auto expected = static_cast<unsigned int>(
          (parameters.steps + (parameters.periods > 1 ? 3 : 0)) * (direction == CodecDirBoth ? 2 : 1));
      require(encoder->getNPatterns() == expected &&
          codecPatternCount(CodecType::PhaseShift, direction, settings) == expected,
          "Configurable phase-shift pattern count differs from the displayed total");
      auto frames = render(*encoder, size);
      cv::Mat up, vp, mask, shading;
      decoder->decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == size.area(), "Ideal phase-shift frames lost valid samples");
      require(up.empty() == !(direction & CodecDirHorizontal) && vp.empty() == !(direction & CodecDirVertical),
              "Phase shifting returned the wrong coordinate axes");
      for (int row = 2; row < size.height - 2; ++row)
        for (int col = 2; col < size.width - 2; ++col) {
          if (!up.empty())
            require(std::abs(up.at<float>(row, col) - col) < 0.15,
                    "Horizontal configurable phase-shift round trip is inaccurate");
          if (!vp.empty())
            require(std::abs(vp.at<float>(row, col) - row) < 0.15,
                    "Vertical configurable phase-shift round trip is inaccurate");
        }
      frames = render(*encoder, size, 0.1);
      decoder->decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == 0, "Phase-shift modulation threshold ignored weak frames");
      decoder->setMinimumModulation(0);
      decoder->decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == size.area(), "Low threshold failed to recover valid weak frames");
    }
  }
  DecoderPhaseShift missing(size.width, size.height, CodecDirBoth, {5, 4});
  cv::Mat up, vp, mask, shading;
  rejects([&] { missing.decodeFrames({}, up, vp, mask, shading); });
  EncoderPhaseShift encoder(size.width, size.height, CodecDirHorizontal);
  rejects([&] { encoder.getEncodingPattern(encoder.getNPatterns()); });
}

void testGrayCode() {
  const cv::Size size(127, 93);
  for (const auto parameters : {GrayCodeSettings{1, 1}, {4, 3}, {7, 7}, {31, 31}}) {
    for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
      CodecAlgorithmSettings settings;
      settings.grayCode = parameters;
      EncoderGrayCode encoder(size.width, size.height, direction, parameters);
      DecoderGrayCode decoder(size.width, size.height, direction, parameters);
      require(encoder.getNPatterns() == codecPatternCount(CodecType::GrayCode, direction, settings),
              "Gray-code bit counts differ from the displayed total");
      const auto frames = render(encoder, size);
      cv::Mat up, vp, mask, shading;
      decoder.decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == size.area(), "Configurable Gray code lost ideal samples");
      for (const auto axis : {CodecDirHorizontal, CodecDirVertical}) {
        if (!(direction & axis)) continue;
        const int length = axis == CodecDirHorizontal ? size.width : size.height;
        const int count = axis == CodecDirHorizontal ? parameters.horizontalBits : parameters.verticalBits;
        unsigned int bits = 0;
        for (auto last = length - 1; last; last >>= 1) ++bits;
        const int step = 1u << (bits - std::min(bits, static_cast<unsigned int>(count)));
        const cv::Mat &coordinates = axis == CodecDirHorizontal ? up : vp;
        for (int row = 0; row < size.height; ++row)
          for (int col = 0; col < size.width; ++col) {
            const int position = axis == CodecDirHorizontal ? col : row;
            const int start = position / step * step;
            const float expected = (start + std::min(start + step, length) - 1) * 0.5f;
            require(coordinates.at<float>(row, col) == expected, "Gray-code prefix bin centers are incorrect");
          }
      }
      const auto transitions = decoder.decodeTransitions(frames, up, vp, mask, shading);
      require(!transitions.empty(), "Configurable Gray code did not extract transitions");
      for (const auto &transition : transitions)
        require(std::abs((transition.direction == CodecDirHorizontal ? transition.camera.x : transition.camera.y) -
            transition.coordinate) < 1e-5, "Gray-code bit counts corrupted projector boundary constraints");
      auto factoryDecoder = DecoderFactory::NewDecoder(
          CodecType::GrayCode, size.width, size.height, direction, true, settings);
      require(factoryDecoder->getNPatterns() == decoder.getNPatterns(), "Factory ignored Gray-code settings");
    }
  }
}

void testSpecializedPhaseShift() {
  const cv::Size size(127, 21);
  for (const auto type : {CodecType::PhaseShiftHeterodyne, CodecType::PhaseShiftMicro,
                          CodecType::PhaseShiftModulated, CodecType::PhaseShiftSpatialUnwrap}) {
    for (const int steps : {3, 4, 5, 8}) {
      CodecAlgorithmSettings settings;
      settings.heterodyne = {steps, steps == 3 ? 60.0 : 42.5};
      settings.micro.steps = steps;
      settings.spatialUnwrap = {steps, steps == 3 ? 4 : 7};
      if (steps != 3) {
        settings.modulated = {steps, 5, 4, 8, 4};
      }
      auto encoder = EncoderFactory::NewEncoder(type, size.width, size.height, CodecDirHorizontal, settings);
      auto decoder = DecoderFactory::NewDecoder(type, size.width, size.height, CodecDirHorizontal, true, settings);
      require(encoder->getNPatterns() == codecPatternCount(type, CodecDirHorizontal, settings),
              "Specialized codec total does not match its settings");
      cv::Mat up, vp, mask, shading;
      rejects([&] { decoder->decodeFrames({}, up, vp, mask, shading); });
      rejects([&] { encoder->getEncodingPattern(encoder->getNPatterns()); });
      const auto frames = render(*encoder, size);
      decoder->decodeFrames(frames, up, vp, mask, shading);
      if (up.size() != size || !vp.empty() || cv::countNonZero(mask) <= size.area() / 4)
        throw std::runtime_error(std::string(codecName(type)) + " failed to decode " +
            std::to_string(steps) + "-step patterns: " + std::to_string(cv::countNonZero(mask)) + " valid");
      for (int row = 2; row < size.height - 2; ++row)
        for (int col = 3; col < size.width - 2; ++col) {
          if (!mask.at<std::uint8_t>(row, col)) continue;
          const float coordinate = up.at<float>(row, col);
          require(std::isfinite(coordinate), "Specialized codec returned nonfinite valid coordinates");
          if (type == CodecType::PhaseShiftSpatialUnwrap)
            require(std::abs(coordinate - up.at<float>(row, col - 1) - 1) < 0.15,
                    "Spatial unwrapping ignored phase steps or fringe periods");
          else if (std::abs(coordinate - col) >= 0.2)
            throw std::runtime_error(std::string(codecName(type)) + " inaccurate at " +
                std::to_string(col) + " with " + std::to_string(steps) + " steps: " +
                std::to_string(coordinate));
        }
      if (type == CodecType::PhaseShiftMicro) {
        for (std::size_t frequency = 1; frequency < MicroPhasePitches.size(); ++frequency) {
          const float phase = frequency % 2 ? static_cast<float>(CV_PI) : 0;
          const auto expected = pstools::computePhaseVector(size.width, phase, MicroPhasePitches[frequency]).t();
          require(cv::norm(encoder->getEncodingPattern(steps + frequency - 1), expected, cv::NORM_INF) == 0,
                  "Micro frequency code or its phase ordering changed");
        }
      }
      decoder->setMinimumModulation(255);
      decoder->decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == 0, "Specialized codec ignored the active modulation threshold");
    }
  }
  for (const auto direction : {CodecDirVertical, CodecDirBoth}) {
    CodecAlgorithmSettings settings;
    settings.heterodyne = {5, 30};
    auto encoder = EncoderFactory::NewEncoder(CodecType::PhaseShiftHeterodyne, 127, 93, direction, settings);
    auto decoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftHeterodyne, 127, 93, direction, true, settings);
    require(encoder->getNPatterns() == codecPatternCount(CodecType::PhaseShiftHeterodyne, direction, settings),
            "Heterodyne axis-dependent pattern count is incorrect");
    const auto frames = render(*encoder, {127, 93});
    cv::Mat up, vp, mask, shading;
    decoder->decodeFrames(frames, up, vp, mask, shading);
    require(!vp.empty() && up.empty() == (direction == CodecDirVertical),
            "Heterodyne settings returned the wrong axes");
    for (int row = 2; row < 91; ++row)
      for (int col = 2; col < 125; ++col) {
        require(std::abs(vp.at<float>(row, col) - row) < 0.15,
                "Configured vertical heterodyne reconstruction is inaccurate");
        if (!up.empty())
          require(std::abs(up.at<float>(row, col) - col) < 0.15,
                  "Configured horizontal heterodyne reconstruction is inaccurate");
      }
  }
  for (const auto size : {cv::Size(7, 5), cv::Size(127, 9)}) {
    CodecAlgorithmSettings settings;
    settings.spatialUnwrap = {5, 2};
    auto encoder = EncoderFactory::NewEncoder(CodecType::PhaseShiftSpatialUnwrap, size.width, size.height,
                                             CodecDirHorizontal, settings);
    auto decoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftSpatialUnwrap, size.width, size.height,
                                             CodecDirHorizontal, true, settings, 0);
    const auto frames = render(*encoder, size);
    cv::Mat up, vp, mask, shading;
    decoder->decodeFrames(frames, up, vp, mask, shading);
    require(cv::countNonZero(mask) == size.area(), "Spatial unwrapping lost valid small-image support");
    for (int row = 0; row < size.height; ++row)
      for (int col = 1; col < size.width; ++col)
        require(std::abs(up.at<float>(row, col) - up.at<float>(row, col - 1) - 1) < 0.15,
                "Spatial unwrapping corrupted small-image phase differences");
  }
}

void testCenterFringeOrder() {
  const cv::Size size(127, 21);
  cv::Mat1f wrapped(size);
  for (int row = 0; row < size.height; ++row)
    for (int col = 0; col < size.width; ++col)
      wrapped(row, col) = std::fmod(0.21f + 0.22f * col + 0.04f * row, 2 * static_cast<float>(CV_PI));
  cv::Mat phaseZero = wrapped.clone(), phaseThree = wrapped.clone();
  cv::Mat maskZero = cv::Mat1b(size, std::uint8_t(255)), maskThree = maskZero.clone();
  spatialUnwrapping(phaseZero, maskZero, 0);
  spatialUnwrapping(phaseThree, maskThree, 3);
  require(cv::countNonZero(maskZero) == size.area() &&
          cv::norm(maskZero, maskThree, cv::NORM_INF) == 0,
          "Changing center fringe order changed spatial-unwrapping support");
  require(cv::norm(phaseThree - phaseZero, cv::Mat1f(size, 6 * static_cast<float>(CV_PI)),
                   cv::NORM_INF) < 1e-5,
          "Algorithm did not apply the requested fringe order in radians");
  const auto original = wrapped.clone();
  cv::Mat phase = wrapped.clone(), mask = cv::Mat1b(size, std::uint8_t(255));
  rejects([&] { spatialUnwrapping(phase, mask, -1); });
  require(cv::norm(phase, original, cv::NORM_INF) == 0,
          "Invalid center fringe order modified the phase image");
  cv::Mat wrongMask = cv::Mat1b(2, 3, std::uint8_t(255));
  rejects([&] { spatialUnwrapping(phase, wrongMask, 0); });

  CodecAlgorithmSettings coding;
  coding.spatialUnwrap = {3, 4};
  auto encoder = EncoderFactory::NewEncoder(CodecType::PhaseShiftSpatialUnwrap, size.width, size.height,
                                           CodecDirHorizontal, coding);
  const auto frames = render(*encoder, size);
  DecoderPhaseShiftSpatialUnwrap defaultDecoder(size.width, size.height, CodecDirHorizontal);
  cv::Mat defaultUp, vp, defaultMask, shading;
  defaultDecoder.decodeFrames(frames, defaultUp, vp, defaultMask, shading);
  cv::Mat zeroUp;
  for (const int order : {0, 1, 3}) {
    auto decoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftSpatialUnwrap, size.width, size.height,
                                             CodecDirHorizontal, true, coding, order);
    cv::Mat up, valid, magnitude;
    decoder->decodeFrames(frames, up, vp, valid, magnitude);
    require(cv::norm(valid, defaultMask, cv::NORM_INF) == 0,
            "Decoder fringe-order parameter changed validity");
    if (order == 0) zeroUp = up.clone();
    require(cv::norm(up - zeroUp, cv::Mat1f(size, order * size.width / 4.0f), cv::NORM_INF) < 1e-4,
            "Decoder fringe order did not shift coordinates by the expected projector pitch");
    if (order == 3)
      require(cv::norm(up, defaultUp, cv::NORM_INF) == 0,
              "Default center fringe order no longer preserves the original three-fringe offset");
    GeometricCalibrationData calibration;
    calibration.frameWidth = calibration.screenResX = size.width;
    calibration.frameHeight = calibration.screenResY = size.height;
    ReconstructionSettings settings;
    settings.coding.codec = CodecType::PhaseShiftSpatialUnwrap;
    settings.coding.algorithmSettings = coding;
    settings.centerFringeOrder = order;
    cv::Mat1f decodedUp, decodedVp;
    cv::Mat1b decodedMask, decodedShading;
    DecodeStage(calibration, settings).decodeMaps(frames, decodedUp, decodedVp, decodedMask, decodedShading);
    require(cv::norm(decodedUp, up, cv::NORM_INF) == 0 &&
            cv::norm(decodedMask, valid, cv::NORM_INF) == 0,
            "Reconstruction did not pass the active center fringe order to the decoder");
    auto changed = settings;
    changed.centerFringeOrder = (order + 1) % 4;
    require(!(changed == settings), "Pipeline equality ignored center fringe order");
  }
  for (const int order : {-1, 4}) {
    rejects([&] { DecoderPhaseShiftSpatialUnwrap invalid(size.width, size.height, CodecDirHorizontal,
                                                        {3, 4}, order); });
    rejects([&] { DecoderFactory::NewDecoder(CodecType::PhaseShiftSpatialUnwrap, size.width, size.height,
                                            CodecDirHorizontal, true, coding, order); });
    ReconstructionSettings settings;
    settings.coding.codec = CodecType::PhaseShiftSpatialUnwrap;
    settings.centerFringeOrder = order;
    rejects([&] { validateReconstructionSettings(settings); });
  }
}

void testValidation() {
  for (const auto parameters : {PhaseShiftSettings{2, 16}, {257, 16}, {3, 0}, {3, 256}}) {
    CodecAlgorithmSettings settings;
    settings.phaseShift = parameters;
    rejects([&] { EncoderFactory::NewEncoder(CodecType::PhaseShift, 127, 93, CodecDirBoth, settings); });
    rejects([&] { DecoderFactory::NewDecoder(CodecType::PhaseShift, 127, 93, CodecDirBoth, true, settings); });
    rejects([&] { codecPatternCount(CodecType::PhaseShift, CodecDirBoth, settings); });
  }

  for (const auto parameters : {GrayCodeSettings{0, 6}, {10, -1}, {32, 6}, {10, 32}}) {
    CodecAlgorithmSettings settings;
    settings.grayCode = parameters;
    rejects([&] { EncoderGrayCode invalid(127, 93, CodecDirBoth, parameters); });
    rejects([&] { DecoderGrayCode invalid(127, 93, CodecDirBoth, parameters); });
    rejects([&] { codecPatternCount(CodecType::GrayCode, CodecDirBoth, settings); });
  }
  require(Codecs.size() == 7, "Codec catalog has unexpected entries");
  for (int field = 0; field < 11; ++field) {
    CodecAlgorithmSettings settings;
    switch (field) {
    case 0: settings.heterodyne.steps = 2; break;
    case 1: settings.heterodyne.primaryPitch = std::numeric_limits<double>::quiet_NaN(); break;
    case 2: settings.heterodyne.primaryPitch = 1; break;
    case 3: settings.spatialUnwrap.steps = 257; break;
    case 4: settings.spatialUnwrap.periods = 0; break;
    case 5: settings.micro.steps = 2; break;
    case 6: settings.modulated.signalSteps = 2; break;
    case 7: settings.modulated.carrierSteps = 2; break;
    case 8: settings.modulated.cueSteps = 257; break;
    case 9: settings.modulated.signalPeriods = 0; break;
    case 10: settings.modulated.carrierPeriods = 256; break;
    }
    rejects([&] { validateCodecSettings(settings); });
    rejects([&] { codecPatternCount(CodecType::PhaseShiftHeterodyne, CodecDirBoth, settings); });
  }
  CodecAlgorithmSettings maximum;
  maximum.heterodyne.steps = maximum.micro.steps = maximum.spatialUnwrap.steps = 256;
  maximum.modulated = {256, 256, 256, 255, 255};
  require(codecPatternCount(CodecType::PhaseShiftHeterodyne, CodecDirBoth, maximum) == 1024 &&
          codecPatternCount(CodecType::PhaseShiftMicro, CodecDirHorizontal, maximum) == 265 &&
          codecPatternCount(CodecType::PhaseShiftSpatialUnwrap, CodecDirHorizontal, maximum) == 256 &&
          codecPatternCount(CodecType::PhaseShiftModulated, CodecDirHorizontal, maximum) == 65792,
          "Maximum supported step counts overflowed or produced incorrect totals");
}

void testReconstruction() {
    GeometricCalibrationData calibration;
    calibration.frameWidth = 127;
    calibration.frameHeight = 93;
    calibration.screenResX = 256;
    calibration.screenResY = 192;
    calibration.Kc = {80, 0, 64, 0, 80, 46, 0, 0, 1};
    calibration.Kp = {80, 0, 128, 0, 80, 96, 0, 0, 1};
    calibration.Tp = {-0.3f, -0.2f, 0};
    ReconstructionSettings settings;
    settings.coding.codec = CodecType::PhaseShift;
    settings.coding.direction = CodecDirBoth;
    settings.coding.algorithmSettings.phaseShift = {5, 4};
    settings.points.maxDepth = 10;
    auto encoder = EncoderFactory::NewEncoder(settings.coding.codec, calibration.screenResX,
        calibration.screenResY, settings.coding.direction, settings.coding.algorithmSettings);
    auto frames = render(*encoder, {calibration.screenResX, calibration.screenResY});
    for (auto &frame : frames)
      frame = frame(cv::Rect(40, 34, calibration.frameWidth, calibration.frameHeight)).clone();
    DecodeStage decode(calibration, settings);
    TriangulationStage triangulate(calibration, settings);
    require(decode.patternCount() == frames.size(), "Reconstruction ignored custom codec parameters");
    cv::Mat1f up, vp;
    cv::Mat1b mask, shading;
    decode.decodeMaps(frames, up, vp, mask, shading);
    const auto expected = triangulate.triangulateDense(up, vp, mask, shading);
    require(expected->points.size() > calibration.frameWidth * calibration.frameHeight * 0.9,
            "Custom phase-shift reconstruction lost valid geometry");
    for (const auto &point : expected->points)
      require(std::abs(point.z - 1) < 0.005, "Custom codec parameters produced incorrect plane depth");
    TriangulatorWorker triangulator(calibration, settings);
    DecoderWorker decoder(calibration, settings, [&](std::uint64_t sequence, cv::Mat1f u, cv::Mat1f v,
                                                   cv::Mat1b valid, cv::Mat1b shade) {
      require(triangulator.trySubmitDense(sequence, std::move(u), std::move(v),
          std::move(valid), std::move(shade)), "Triangulator refused decoded data");
    });
    require(decoder.trySubmit(frames), "Decoder refused a custom codec sequence");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    std::optional<ReconstructionResult> result;
    while (!result) {
      decoder.checkError();
      result = triangulator.tryTake();
      if (std::chrono::steady_clock::now() > deadline) throw std::runtime_error("Custom codec pipeline timed out");
      std::this_thread::yield();
    }
    require(result->cloud->points.size() == expected->points.size(), "Custom codec pipeline lost points");
    for (std::size_t index = 0; index < expected->points.size(); ++index) {
      const auto &a = expected->points[index], &b = result->cloud->points[index];
      require(a.x == b.x && a.y == b.y && a.z == b.z && a.r == b.r,
              "Custom codec pipeline differs from synchronous reconstruction");
    }
    auto changed = settings;
    changed.coding.algorithmSettings.phaseShift.periods += 1;
    require(!(changed == settings), "Pipeline settings equality ignores codec settings");
    frames.pop_back();
    rejects([&] { decode.decodeMaps(frames, up, vp, mask, shading); });
  }

void testCodecCatalog() {
  struct ExpectedCodec {
    const char *name;
    CodecType type;
    std::array<unsigned int, 3> patternCounts;
  };
  const std::array<ExpectedCodec, 7> expected{{
      {"PhaseShift", CodecType::PhaseShift, {6, 6, 12}},
      {"GrayCode", CodecType::GrayCode, {12, 8, 18}},
      {"PhaseShiftMotionCompensated", CodecType::PhaseShiftMotionCompensated, {3, 3, 3}},
      {"PhaseShiftSpatialUnwrap", CodecType::PhaseShiftSpatialUnwrap, {3, 3, 3}},
      {"PhaseShiftMicro", CodecType::PhaseShiftMicro, {12, 12, 12}},
      {"PhaseShiftModulated", CodecType::PhaseShiftModulated, {12, 12, 12}},
      {"PhaseShiftHeterodyne", CodecType::PhaseShiftHeterodyne, {6, 6, 12}}}};
  require(Codecs.size() == expected.size(), "Selectable codec count changed");
  require(static_cast<int>(CodecType::PhaseShiftHeterodyne) == 7, "Remaining codec identifiers changed");
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const auto &codec = expected[index];
    require(Codecs[index].type == codec.type && std::string_view(Codecs[index].name) == codec.name,
            "Codec catalog has unexpected names or ordering");
    require(std::string_view(codecName(codec.type)) == codec.name &&
        codecTypeFromName(codec.name) == codec.type, "Codec name/type round trip failed");
    for (std::size_t previous = 0; previous < index; ++previous)
      require(Codecs[previous].type != codec.type && std::string_view(Codecs[previous].name) != codec.name,
              "Codec catalog contains duplicate names or types");
    for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
      auto encoder = EncoderFactory::NewEncoder(codec.type, 1024, 768, direction);
      auto decoder = DecoderFactory::NewDecoder(codec.type, 1024, 768, direction);
      const auto count = codec.patternCounts[static_cast<int>(direction) - 1];
      require(encoder->getNPatterns() == count && decoder->getNPatterns() == count,
              "Factory changed a codec's pattern count");
      for (unsigned int pattern = 0; pattern < count; ++pattern) {
        const auto image = encoder->getEncodingPattern(pattern);
        require(!image.empty() && image.type() == CV_8UC3, "Factory returned an invalid encoding pattern");
      }
    }
    rejects([&] { EncoderFactory::NewEncoder(codec.type, 0, 768); });
    rejects([&] { DecoderFactory::NewDecoder(codec.type, 1024, 0); });
  }
  for (const auto *name : {"", "Unknown", "FastRatio", "phaseshift", "PhaseShift ",
                          "PhaseShift2x3", "PhaseShift3", "PhaseShift4", "PhaseShiftNStep"})
    rejects([&] { codecTypeFromName(name); });
  for (const auto type : {static_cast<CodecType>(-1), static_cast<CodecType>(6), static_cast<CodecType>(10)}) {
    rejects([&] { codecName(type); });
    rejects([&] { EncoderFactory::NewEncoder(type, 1024, 768); });
    rejects([&] { DecoderFactory::NewDecoder(type, 1024, 768); });
    rejects([&] { codecPatternCount(type, CodecDirHorizontal); });
  }
}

void testMotionCompensation() {
  const cv::Size size(128, 96);
  cv::Mat1f texture(size);
  cv::RNG random(9317);
  random.fill(texture, cv::RNG::UNIFORM, 60.0, 100.0);
  cv::GaussianBlur(texture, texture, {}, 1.4);
  std::vector<cv::Mat> reference(3);
  cv::Mat1b first(size), second(size);
  for (int row = 0; row < size.height; ++row)
    for (int col = 0; col < size.width; ++col) {
      const float phase = 0.6f + 0.025f * col + 0.004f * row;
      first(row, col) = cv::saturate_cast<std::uint8_t>(texture(row, col) + 24 * std::cos(phase));
      second(row, col) = cv::saturate_cast<std::uint8_t>(texture(row, col) + 24 * std::sin(phase));
    }
  reference[0] = first;
  reference[1] = second;
  texture.convertTo(reference[2], CV_8U);
  auto translate = [](const cv::Mat &image, double x, double y) {
    const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1, 0, x, 0, 1, y);
    cv::Mat shifted;
    cv::warpAffine(image, shifted, transform, image.size(), cv::INTER_LINEAR, cv::BORDER_WRAP);
    return shifted;
  };
  // Match the existing decoder's fractional-motion and half-pixel sampling convention.
  const std::vector<cv::Mat> moving{
      translate(first, 0.5 + 0.333 * 6, 0.5),
      translate(second, 0.5 - 0.333 * 6, 0.5), reference[2]};
  auto previous = reference;
  previous[2] = translate(reference[2], -6, 0);
  std::vector<cv::Mat> originalMoving;
  for (const auto &frame : moving) originalMoving.push_back(frame.clone());
  auto decode = [](Decoder &decoder, const std::vector<cv::Mat> &frames) {
    cv::Mat up, vp, mask, shading;
    decoder.decodeFrames(frames, up, vp, mask, shading);
    return up;
  };
  auto makeDecoder = [&](bool enabled) {
    return DecoderFactory::NewDecoder(CodecType::PhaseShiftMotionCompensated, size.width, size.height,
                                      CodecDirHorizontal, enabled);
  };
  auto idealDecoder = makeDecoder(false);
  const auto ideal = decode(*idealDecoder, reference);
  auto disabled = makeDecoder(false);
  decode(*disabled, previous);
  const auto uncorrected = decode(*disabled, moving);
  auto enabled = makeDecoder(true);
  decode(*enabled, previous);
  const auto corrected = decode(*enabled, moving);
  const cv::Rect interior(16, 16, 96, 64);
  const double correctedError = cv::norm(corrected(interior), ideal(interior), cv::NORM_L1) / interior.area();
  const double uncorrectedError = cv::norm(uncorrected(interior), ideal(interior), cv::NORM_L1) / interior.area();
  if (!(correctedError < 0.5 && correctedError < uncorrectedError * 0.5))
    throw std::runtime_error("Motion compensation did not reduce synthetic phase-coordinate error: " +
        std::to_string(correctedError) + " corrected vs " + std::to_string(uncorrectedError) + " uncorrected");
  auto defaultDecoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftMotionCompensated, size.width, size.height);
  decode(*defaultDecoder, previous);
  require(cv::norm(decode(*defaultDecoder, moving), corrected, cv::NORM_INF) < 1e-5,
          "PhaseShiftMotionCompensated motion compensation is no longer enabled by default");
  auto freshDisabled = makeDecoder(false);
  require(cv::norm(decode(*freshDisabled, moving), uncorrected, cv::NORM_INF) == 0,
          "Disabled motion compensation still depends on previous frames");

  DecoderPhaseShiftMotionCompensated toggled(size.width, size.height, CodecDirHorizontal);
  decode(toggled, previous);
  toggled.setMotionCompensation(false);
  require(cv::norm(decode(toggled, moving), uncorrected, cv::NORM_INF) == 0,
          "Codec motion-compensation setter did not disable alignment");
  toggled.setMotionCompensation(true);
  auto freshEnabled = makeDecoder(true);
  require(cv::norm(decode(toggled, moving), decode(*freshEnabled, moving), cv::NORM_INF) < 1e-5,
          "Re-enabling motion compensation retained stale motion history");

  GeometricCalibrationData calibration;
  calibration.frameWidth = size.width;
  calibration.frameHeight = size.height;
  calibration.screenResX = size.width;
  calibration.screenResY = size.height;
  ReconstructionSettings settings;
  settings.coding.codec = CodecType::PhaseShiftMotionCompensated;
  settings.minimumModulation = 0;
  require(settings.motionCompensation, "Reconstruction must enable motion compensation by default");
  DecodeStage enabledStage(calibration, settings);
  cv::Mat1f up, vp;
  cv::Mat1b mask, shading;
  enabledStage.decodeMaps(previous, up, vp, mask, shading);
  enabledStage.decodeMaps(moving, up, vp, mask, shading);
  require(cv::norm(up, corrected, cv::NORM_INF) < 1e-5,
          "Decode stage did not enable codec motion compensation");
  const auto enabledSettings = settings;
  settings.motionCompensation = false;
  require(!(settings == enabledSettings), "Motion compensation did not participate in pipeline settings equality");
  DecodeStage disabledStage(calibration, settings);
  disabledStage.decodeMaps(previous, up, vp, mask, shading);
  disabledStage.decodeMaps(moving, up, vp, mask, shading);
  require(cv::norm(up, uncorrected, cv::NORM_INF) == 0,
          "Decode stage did not disable codec motion compensation");
  for (std::size_t index = 0; index < moving.size(); ++index)
    require(cv::norm(moving[index], originalMoving[index], cv::NORM_INF) == 0,
            "Motion compensation modified input frames");
  std::vector<cv::Mat> floatMoving, originalFloatMoving;
  for (const auto &frame : moving) {
    cv::Mat converted;
    frame.convertTo(converted, CV_32F);
    floatMoving.push_back(converted);
    originalFloatMoving.push_back(converted.clone());
  }
  auto floatDecoder = makeDecoder(true);
  decode(*floatDecoder, floatMoving);
  for (std::size_t index = 0; index < floatMoving.size(); ++index)
    require(cv::norm(floatMoving[index], originalFloatMoving[index], cv::NORM_INF) == 0,
            "Motion compensation modified floating-point input frames");
}

void testHeterodyne() {
  for (const cv::Size size : {cv::Size(360, 240), cv::Size(1216, 684)}) {
    for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
      const auto encoder = EncoderFactory::NewEncoder(CodecType::PhaseShiftHeterodyne, size.width, size.height, direction);
      const auto decoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftHeterodyne, size.width, size.height, direction);
      require(encoder->getNPatterns() == (direction == CodecDirBoth ? 12u : 6u) &&
          decoder->getNPatterns() == encoder->getNPatterns(), "Heterodyne pattern count is incorrect");
      std::vector<cv::Mat> frames(decoder->getNPatterns());
      for (unsigned int index = 0; index < encoder->getNPatterns(); ++index) {
        const auto pattern = encoder->getEncodingPattern(index);
        cv::Mat gray, frame;
        cv::cvtColor(pattern, gray, cv::COLOR_BGR2GRAY);
        cv::repeat(gray, size.height / gray.rows, size.width / gray.cols, frame);
        frames[index] = frame;
      }
      cv::Mat up, vp, mask, shading;
      decoder->decodeFrames(frames, up, vp, mask, shading);
      require(up.empty() == !(direction & CodecDirHorizontal) &&
          vp.empty() == !(direction & CodecDirVertical), "Heterodyne direction was ignored");
      std::size_t checked = 0;
      for (int row = 2; row < size.height - 2; ++row)
        for (int col = 2; col < size.width - 2; ++col) {
          require(mask.at<std::uint8_t>(row, col) != 0, "Heterodyne rejected an illuminated interior pixel");
          if (!up.empty()) require(std::abs(up.at<float>(row, col) - col) < 0.15f, "Horizontal heterodyne error exceeds 0.15 pixels");
          if (!vp.empty()) require(std::abs(vp.at<float>(row, col) - row) < 0.15f, "Vertical heterodyne error exceeds 0.15 pixels");
          ++checked;
        }
      require(checked > 10000, "Insufficient heterodyne round-trip coverage");
      for (unsigned int index = 0; index < decoder->getNPatterns(); ++index)
        frames[index] = cv::Mat1b(size, std::uint8_t(50));
      decoder->decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == 0, "Heterodyne accepted zero modulation");
    }
  }
  auto decoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftHeterodyne, 32, 24);
  rejects([&] { cv::Mat up, vp, mask, shading; decoder->decodeFrames({}, up, vp, mask, shading); });

  decoder = DecoderFactory::NewDecoder(CodecType::PhaseShiftHeterodyne, 1216, 684, CodecDirBoth);
  std::mt19937 random(42);
  cv::Mat1f expectedUp(48, 64), expectedVp(48, 64);
  for (int row = 0; row < 48; ++row)
    for (int col = 0; col < 64; ++col) {
      expectedUp(row, col) = 20.375f + col * 18.25f;
      expectedVp(row, col) = 20.625f + row * 13.25f;
    }
  std::vector<cv::Mat> frames(12);
  for (unsigned int index = 0; index < 12; ++index) {
    cv::Mat1b frame(48, 64);
    const double length = index < 6 ? 1216 : 684;
    const double cycles = length / 60.0 + ((index % 6) >= 3 ? 1 : 0);
    for (int row = 0; row < frame.rows; ++row)
      for (int col = 0; col < frame.cols; ++col) {
        const double position = index < 6 ? expectedUp(row, col) : expectedVp(row, col);
        const double phase = 2 * CV_PI * (cycles * position / length - (index % 3) / 3.0);
        const double illumination = 0.6 + 0.3 * col / frame.cols;
        const int noise = static_cast<int>(random() % 3) - 1;
        frame(row, col) = cv::saturate_cast<std::uint8_t>(15 + illumination * (127 + 110 * std::cos(phase)) + noise);
      }
    frames[index] = frame;
  }
  cv::Mat up, vp, mask, shading;
  decoder->decodeFrames(frames, up, vp, mask, shading);
  require(cv::countNonZero(mask) == static_cast<int>(mask.total()) &&
      cv::norm(up, expectedUp, cv::NORM_INF) < 0.3 && cv::norm(vp, expectedVp, cv::NORM_INF) < 0.3,
      "Heterodyne subpixel/noisy decoding error exceeds 0.3 pixels");
}
}

void testPhaseAndMagnitude() {
  cv::RNG random(12345);
  cv::Mat1b first(19, 31), second(first.size()), third(first.size());
  random.fill(first, cv::RNG::UNIFORM, 0, 256);
  random.fill(second, cv::RNG::UNIFORM, 0, 256);
  random.fill(third, cv::RNG::UNIFORM, 0, 256);
  cv::Mat phase, magnitude;
  pstools::getPhaseAndMagnitude(first, second, third, phase, magnitude);
  require(cv::norm(phase, pstools::getPhase(first, second, third), cv::NORM_INF) < 1e-6,
      "Combined phase decoding changed the result");
  require(cv::norm(magnitude, pstools::getMagnitude(first, second, third), cv::NORM_INF) == 0,
      "Combined magnitude decoding changed the result");
}

int main() {
  try {
    testPhaseAndMagnitude();
    testCodecCatalog();
    testMotionCompensation();
    testHeterodyne();
    testCompleteSequenceInput();
    testPhaseShift();
    testGrayCode();
    testSpecializedPhaseShift();
    testCenterFringeOrder();
    testValidation();
    testReconstruction();
    std::cout << "Codec catalog, round trips, motion compensation, parameters and input contracts passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
