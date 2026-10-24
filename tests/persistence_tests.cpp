#include "FrameSequence.h"
#include "DecoderWorker.h"
#include "ApplicationSettings.h"
#include <cstdio>

#include <array>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using slstudio::gui::ApplicationSettings;
using slstudio::gui::RenderSettings;

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void rejects(Function action, const std::string &message = "Invalid persistence input was accepted") {
  bool rejected = false;
  try { action(); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, message.c_str());
}

struct TestDirectory {
  std::filesystem::path path = std::filesystem::current_path() /
      ("slstudio-persistence-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));

  TestDirectory() {
    require(std::filesystem::create_directory(path), "Cannot create test folder");
  }
  ~TestDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

CodecAlgorithmSettings customSettings() {
  CodecAlgorithmSettings result;
  result.phaseShift.steps = 7;
  result.phaseShift.periods = 23;
  result.grayCode.horizontalBits = 9;
  result.grayCode.verticalBits = 8;
  result.heterodyne = {5, 42.5};
  result.spatialUnwrap = {5, 7};
  result.micro.steps = 5;
  result.modulated = {5, 4, 6, 12, 7};
  return result;
}

constexpr std::array<const char *, 4> parameterNames{
    "phase_steps", "fringe_periods", "gray_horizontal_bits", "gray_vertical_bits"};
constexpr std::array<int, 4> parameterDefaults{3, 16, 10, 6};

void writeMetadata(const std::filesystem::path &directory, int version, const char *changed = nullptr,
                   const std::string &replacement = {}, bool omit = false, int frameCount = 0,
                   const char *codec = nullptr) {
  cv::FileStorage storage((directory / "sequence.xml").string(),
                          cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
  const auto *selectedCodec = codec ? codec : "PhaseShift";
  storage << "version" << version << "codec" << selectedCodec
          << "direction" << "Both"
          << "frame_count" << (frameCount ? frameCount : 12);
  for (std::size_t index = 0; index < parameterNames.size(); ++index) {
    const auto *name = parameterNames[index];
    const bool relevant = (std::string_view(selectedCodec) == "PhaseShift" && index < 2) ||
        (std::string_view(selectedCodec) == "GrayCode" && index >= 2);
    if (!relevant && !(changed && std::string(name) == changed)) continue;
    if (changed && std::string(name) == changed) {
      if (!omit) {
        storage << name;
        if (replacement == "real") storage << 3.5;
        else if (replacement == "string") storage << "3";
        else if (replacement == "sequence") storage << "[" << 3 << "]";
        else storage << std::stoi(replacement);
      }
    } else storage << name << parameterDefaults[index];
  }
}

void testSequenceRoundTrips() {
  for (const auto &codec : Codecs) {
    TestDirectory directory;
    FrameSequence sequence;
    sequence.coding.codec = codec.type;
    sequence.coding.algorithmSettings = customSettings();
    sequence.coding.direction = CodecDirBoth;
    const auto count = codecPatternCount(sequence.coding.codec, sequence.coding.direction,
        sequence.coding.algorithmSettings);
    for (unsigned int index = 0; index < count; ++index)
      sequence.frames.emplace_back(cv::Mat1b(2, 3, std::uint8_t(71 + index)));
    saveSequence(directory.path, sequence);
    cv::FileStorage storage((directory.path / "sequence.xml").string(), cv::FileStorage::READ);
    std::vector<std::string> fields;
    auto expected = sequence.coding;
    expected.algorithmSettings = {};
    switch (codec.type) {
    case CodecType::PhaseShift:
      fields = {"phase_steps", "fringe_periods"};
      expected.algorithmSettings.phaseShift = sequence.coding.algorithmSettings.phaseShift;
      break;
    case CodecType::GrayCode:
      fields = {"gray_horizontal_bits", "gray_vertical_bits"};
      expected.algorithmSettings.grayCode = sequence.coding.algorithmSettings.grayCode;
      break;
    case CodecType::PhaseShiftHeterodyne:
      fields = {"phase_steps", "primary_pitch"};
      expected.algorithmSettings.heterodyne = sequence.coding.algorithmSettings.heterodyne;
      break;
    case CodecType::PhaseShiftSpatialUnwrap:
      fields = {"phase_steps", "fringe_periods"};
      expected.algorithmSettings.spatialUnwrap = sequence.coding.algorithmSettings.spatialUnwrap;
      break;
    case CodecType::PhaseShiftMicro:
      fields = {"phase_steps"};
      expected.algorithmSettings.micro = sequence.coding.algorithmSettings.micro;
      break;
    case CodecType::PhaseShiftModulated:
      fields = {"signal_steps", "carrier_steps", "cue_steps", "signal_periods", "carrier_periods"};
      expected.algorithmSettings.modulated = sequence.coding.algorithmSettings.modulated;
      break;
    default: break;
    }
    require(static_cast<int>(storage["version"]) == 6 &&
            storage.root().size() == 4 + fields.size(),
            "Sequence metadata contains unrelated settings");
    require(storage["centerFringeOrder"].isNone(), "Sequence persisted a reconstruction fringe-order assumption");
    for (const auto &field : fields)
      require(!storage[field].isNone(), "Sequence omitted its codec's parameter");
    for (std::size_t index = 0; index < parameterNames.size(); ++index) {
      const bool relevant = std::find(fields.begin(), fields.end(), parameterNames[index]) != fields.end();
      require(relevant ? storage[parameterNames[index]].isInt() : storage[parameterNames[index]].isNone(),
              "Sequence metadata must contain only its codec's parameters");
    }
    const auto restored = loadSequence(directory.path);
    require(restored.coding == expected && restored.frames.size() == count &&
            cv::norm(restored.frames.front(), sequence.frames.front(), cv::NORM_INF) == 0,
            "Custom codec sequence did not round trip");
    for (std::size_t index = 0; index < sequence.frames.size(); ++index)
      require(cv::norm(restored.frames[index], sequence.frames[index], cv::NORM_INF) == 0,
          "Sequence round trip changed image pixels or ordering");
    rejects([&] { saveSequence(directory.path, sequence); });
    require(loadSequence(directory.path).coding == expected,
            "Refusing an occupied destination damaged the original sequence");
  }
}

void testMetadataFreeSequence() {
  TestDirectory directory;
  for (int index = 0; index < 3; ++index)
    require(cv::imwrite((directory.path / ("frameSeq_" + std::to_string(index) + ".png")).string(),
                       cv::Mat1b(2, 3, std::uint8_t(71))), "Cannot write test frame");
  ReconstructionSettings fallback;
  fallback.coding.algorithmSettings = customSettings();
  fallback.motionCompensation = false;
  fallback.centerFringeOrder = 1;
  fallback.minimumModulation = 77;
  fallback.lisPhaseFilter = true;
  fallback.correctProjectorDistortion = false;
  fallback.minimumComponentPixels = 20;
  fallback.rejectDepthSpikes = true;
  fallback.maximumDepthDeviation = 0.125f;
  fallback.points.minDepth = 0.25f;
  fallback.points.maxDepth = 20;
  require(loadSequence(directory.path, fallback.coding).coding == fallback.coding,
          "Metadata-free sequence lost custom fallback settings");
  fallback.coding.algorithmSettings.phaseShift.steps = 0;
  rejects([&] { loadSequence(directory.path, fallback.coding); });
}

void testSequenceValidation() {
  TestDirectory directory;
  for (int index = 0; index < 12; ++index)
    require(cv::imwrite((directory.path / ("frameSeq_" + std::to_string(index) + ".png")).string(),
                       cv::Mat1b(2, 3, std::uint8_t(71))), "Cannot write test frame");
  writeMetadata(directory.path, 6);
  require(loadSequence(directory.path).frames.size() == 12, "Valid current-schema control fixture failed");
  ReconstructionSettings current;
  current.coding.direction = CodecDirBoth;
  current.minimumModulation = 93;
  current.motionCompensation = false;
  current.centerFringeOrder = 2;
  current.lisPhaseFilter = false;
  current.correctProjectorDistortion = false;
  current.minimumComponentPixels = 10;
  current.points = {0.5f, 20};
  const auto loaded = loadSequence(directory.path, current.coding);
  require(withCodecSettings(current, loaded.coding) == current,
          "Sequence metadata replaced active reconstruction settings");
  for (const auto *codec : {"FastRatio", "PhaseShift2x3", "PhaseShift3", "PhaseShift4", "PhaseShiftNStep",
                            "PhaseShift2x3Het", "PhaseShift3Unwrap", "PhaseShift3FastWrap", "PhaseShift2p1",
                            "PhaseShiftDescatter", "PhaseShiftFastWrap", "PhaseShiftShading", "PhaseShiftUnwrap"}) {
    writeMetadata(directory.path, 6, nullptr, {}, false, 12, codec);
    bool rejected = false;
    try { loadSequence(directory.path); }
    catch (const std::invalid_argument &error) {
      rejected = std::string(error.what()) == std::string("Unknown codec: ") + codec;
    }
    require(rejected, "Removed codec name was not explicitly rejected in sequence metadata");
  }
  writeMetadata(directory.path, 6, nullptr, {}, false, 11);
  rejects([&] { loadSequence(directory.path); });
  for (const auto *codec : {"PhaseShift", "GrayCode"}) {
    const bool phaseShift = std::string_view(codec) == "PhaseShift";
    const int count = phaseShift ? 12 : 18;
    TestDirectory parameters;
    for (int index = 0; index < count; ++index)
      require(cv::imwrite((parameters.path / ("frameSeq_" + std::to_string(index) + ".png")).string(),
                         cv::Mat1b(2, 3, std::uint8_t(71))), "Cannot write test frame");
    for (std::size_t index = 0; index < parameterNames.size(); ++index) {
      const bool relevant = phaseShift ? index < 2 : index >= 2;
      const auto *name = parameterNames[index];
      writeMetadata(parameters.path, 6, name, {}, true, count, codec);
      if (relevant) rejects([&] { loadSequence(parameters.path); });
      else require(loadSequence(parameters.path).frames.size() == static_cast<std::size_t>(count),
                   "Sequence required another codec's parameters");
      for (const auto *value : {"real", "string", "sequence", "0", "1000"}) {
        writeMetadata(parameters.path, 6, name, value, false, count, codec);
        if (relevant) rejects([&] { loadSequence(parameters.path); });
        else require(loadSequence(parameters.path).frames.size() == static_cast<std::size_t>(count),
                     "Sequence validated another codec's parameters");
      }
    }
  }
  for (const int version : {0, 1, 2, 3, 4, 5, 7}) {
    writeMetadata(directory.path, version);
    rejects([&] { loadSequence(directory.path); });
  }
  TestDirectory destination;
  FrameSequence sequence;
  sequence.frames.emplace_back(cv::Mat1b(2, 3, std::uint8_t(71)));
  rejects([&] { saveSequence(destination.path, sequence); });
  require(std::filesystem::is_empty(destination.path), "Mismatched frame count left partial sequence files");
  for (std::size_t index = 0; index < parameterNames.size(); ++index) {
    sequence.coding.algorithmSettings = customSettings();
    std::array<int *, 4> values{&sequence.coding.algorithmSettings.phaseShift.steps,
                               &sequence.coding.algorithmSettings.phaseShift.periods,
                               &sequence.coding.algorithmSettings.grayCode.horizontalBits,
                               &sequence.coding.algorithmSettings.grayCode.verticalBits};
    *values[index] = 0;
    rejects([&] { saveSequence(destination.path, sequence); });
    require(std::filesystem::is_empty(destination.path), "Invalid save left partial sequence files");
  }
}

void testSpecializedSequenceParameters() {
  struct Specification {
    CodecType type;
    std::vector<std::pair<const char *, double>> fields;
  };
  const std::vector<Specification> specifications{
      {CodecType::PhaseShiftHeterodyne, {{"phase_steps", 3}, {"primary_pitch", 60}}},
      {CodecType::PhaseShiftSpatialUnwrap, {{"phase_steps", 3}, {"fringe_periods", 4}}},
      {CodecType::PhaseShiftMicro, {{"phase_steps", 3}}},
      {CodecType::PhaseShiftModulated, {{"signal_steps", 3}, {"carrier_steps", 3}, {"cue_steps", 3},
                                       {"signal_periods", 24}, {"carrier_periods", 16}}}};
  for (const auto &specification : specifications) {
    TestDirectory directory;
    const auto count = codecPatternCount(specification.type, CodecDirBoth);
    for (unsigned int index = 0; index < count; ++index)
      require(cv::imwrite((directory.path / ("frameSeq_" + std::to_string(index) + ".png")).string(),
                         cv::Mat1b(2, 3, std::uint8_t(71))), "Cannot write test frame");
    for (std::size_t changed = 0; changed < specification.fields.size(); ++changed) {
      for (const auto *invalid : {"omit", "string", "sequence", "zero", "nan"}) {
        cv::FileStorage storage((directory.path / "sequence.xml").string(), cv::FileStorage::WRITE);
        storage << "version" << 6 << "codec" << codecName(specification.type)
                << "direction" << "Both" << "frame_count" << static_cast<int>(count);
        for (std::size_t index = 0; index < specification.fields.size(); ++index) {
          const auto &[name, value] = specification.fields[index];
          if (index == changed) {
            const std::string_view kind = invalid;
            if (kind == "omit") continue;
            storage << name;
            if (kind == "string") storage << "3";
            else if (kind == "sequence") storage << "[" << 3 << 4 << "]";
            else if (kind == "zero") storage << 0;
            else storage << std::numeric_limits<double>::quiet_NaN();
          } else {
            storage << name;
            if (std::string_view(name).find("pitch") != std::string_view::npos) storage << value;
            else storage << static_cast<int>(value);
          }
        }
        storage.release();
        rejects([&] { loadSequence(directory.path); }, std::string(codecName(specification.type)) +
            " accepted invalid " + specification.fields[changed].first + ": " + invalid);
      }
    }
  }
}

ApplicationSettings parse(const ApplicationSettings &baseline, const std::string &fields) {
  cv::FileStorage storage("%YAML:1.0\n" + fields + "\n", cv::FileStorage::READ | cv::FileStorage::MEMORY);
  return baseline.read(storage);
}

void testSettingsDefaults() {
  ApplicationSettings baseline;
  baseline.reconstruction.coding.algorithmSettings = customSettings();
  baseline.reconstruction.motionCompensation = false;
  baseline.previewFrameCount = 3;
  for (const auto *value : {"-1", "0", "999"})
    rejects([&] { parse(baseline, std::string("codec: ") + value); });
  rejects([&] { parse(baseline, "codec: FastRatio"); });
  const auto restored = parse(baseline, "exposure: 2");
  require(restored.reconstruction.coding.algorithmSettings == CodecAlgorithmSettings{} &&
      restored.reconstruction.coding.codec == baseline.reconstruction.coding.codec && !restored.reconstruction.motionCompensation &&
      restored.previewFrameCount == 3,
      "Missing settings codec parameters inherited custom values or changed unrelated settings");
}

void testSettingsRoundTrips() {
  for (const auto &codec : Codecs) {
    ApplicationSettings original;
    original.reconstruction.coding.codec = codec.type;
    original.reconstruction.coding.algorithmSettings = customSettings();
    original.reconstruction.centerFringeOrder = 2;
    cv::FileStorage writer("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
    original.write(writer);
    const auto yaml = writer.releaseAndGetString();
    cv::FileStorage reader(yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY);
    const auto restored = ApplicationSettings{}.read(reader);
    require(reader["codec"].isString() && static_cast<std::string>(reader["codec"]) == codec.name &&
            restored.reconstruction == original.reconstruction, "Named codec settings did not round trip");
    const auto named = parse(original, std::string("codec: ") + codec.name);
    require(named.reconstruction.coding.codec == codec.type &&
            named.reconstruction.coding.algorithmSettings == CodecAlgorithmSettings{}, "Codec name was not authoritative");
  }
}

void testSettingsValidation() {
  ApplicationSettings baseline;
  baseline.reconstruction.coding.algorithmSettings = customSettings();
  for (const auto *value : {"3.5", "\"3\"", "[3]", "-1"})
    rejects([&] { parse(baseline, std::string("centerFringeOrder: ") + value); });
  require(parse(baseline, "centerFringeOrder: 0").reconstruction.centerFringeOrder == 0,
          "Zero-based center fringe order was not loaded from settings");
  rejects([&] { parse(baseline, "codec: PhaseShiftSpatialUnwrap\ncenterFringeOrder: 4"); });
  auto invalidOrder = baseline;
  invalidOrder.reconstruction.centerFringeOrder = -1;
  cv::FileStorage invalidWriter("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  rejects([&] { invalidOrder.write(invalidWriter); });
  for (const auto *field : {"codec: unknown", "codec: 3", "codec: [GrayCode]",
                            "codec: PhaseShift2x3", "codec: PhaseShift3",
                            "codec: PhaseShift4", "codec: PhaseShiftNStep",
                            "codec: PhaseShift2x3Het", "codec: PhaseShift3Unwrap",
                            "codec: PhaseShift3FastWrap", "codec: PhaseShift2p1",
                            "codec: PhaseShiftDescatter", "codec: PhaseShiftFastWrap",
                            "codec: PhaseShiftShading", "codec: PhaseShiftUnwrap"})
    rejects([&] { parse(baseline, field); });
  for (std::size_t index = 0; index < parameterNames.size(); ++index) {
    const auto *name = parameterNames[index];
    for (const auto *value : {"3.5", "\"3\"", "[3]", "0", "1000"})
      rejects([&] { parse(baseline, std::string(name) + ": " + value); });
    const auto partial = parse(baseline, std::string(name) + ": " + std::to_string(parameterDefaults[index] + 1));
    auto expected = CodecAlgorithmSettings{};
    std::array<int *, 4> values{&expected.phaseShift.steps, &expected.phaseShift.periods,
                               &expected.grayCode.horizontalBits, &expected.grayCode.verticalBits};
    *values[index] += 1;
    require(partial.reconstruction.coding.algorithmSettings == expected,
            "Partially populated settings parameters did not default independently");
    auto invalid = baseline;
    std::array<int *, 4> invalidValues{&invalid.reconstruction.coding.algorithmSettings.phaseShift.steps,
                                      &invalid.reconstruction.coding.algorithmSettings.phaseShift.periods,
                                      &invalid.reconstruction.coding.algorithmSettings.grayCode.horizontalBits,
                                      &invalid.reconstruction.coding.algorithmSettings.grayCode.verticalBits};
    *invalidValues[index] = 0;
    cv::FileStorage writer("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
    rejects([&] { invalid.write(writer); });
  }
  for (const auto *field : {"heterodyne_steps", "spatial_unwrap_steps", "spatial_unwrap_periods",
                            "micro_steps", "modulated_signal_steps", "modulated_carrier_steps",
                            "modulated_cue_steps", "modulated_signal_periods", "modulated_carrier_periods"})
    for (const auto *value : {"3.5", "\"3\"", "[3]", "0", "1000"})
      rejects([&] { parse(baseline, std::string(field) + ": " + value); },
              std::string("Accepted settings ") + field + ": " + value);
  for (const auto *field : {"heterodyne_primary_pitch"})
    for (const auto *value : {"\"3\"", "[3]", "0", ".Nan", ".Inf"})
      rejects([&] { parse(baseline, std::string(field) + ": " + value); },
              std::string("Accepted settings ") + field + ": " + value);
  require(baseline.reconstruction.coding.algorithmSettings == customSettings(), "Rejected settings read mutated its baseline");
}

ApplicationSettings parseApplicationSettings(const ApplicationSettings &baseline, const std::string &yaml) {
  cv::FileStorage storage(yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY);
  return baseline.read(storage);
}

void testApplicationSettings() {
  ApplicationSettings settings;
  settings.reconstruction.motionCompensation = false;
  settings.geometricCalibrationSettings.interpolation = GeometricCalibrationInterpolation::RBF;
  settings.rendering.colorMode = RenderSettings::ColorMode::Solid;
  settings.rendering.solidColor = {1, 0.25f, 0.5f};
  settings.previewFrameCount = 3;
  settings.intensityAverages = 8;
  settings.lc4500Settings.testPattern = 9;
  settings.intensityLut.emplace();
  settings.intensityCompensationEnabled = true;
  std::snprintf(settings.geometricCalibrationPath.data(), settings.geometricCalibrationPath.size(), "/tmp/calibration with spaces.xml");
  cv::FileStorage writer("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  settings.write(writer);
  const auto yaml = writer.releaseAndGetString();
  const auto restored = parseApplicationSettings(ApplicationSettings{}, yaml);
  require(restored.reconstruction == settings.reconstruction && restored.rendering.colorMode == RenderSettings::ColorMode::Solid &&
      restored.rendering.solidColor == settings.rendering.solidColor && restored.previewFrameCount == 3 &&
      restored.geometricCalibrationSettings.interpolation == GeometricCalibrationInterpolation::RBF &&
      restored.intensityAverages == 8 && restored.intensityCompensationEnabled && restored.intensityLut &&
      restored.intensityLut->inverseLut == settings.intensityLut->inverseLut &&
      restored.lc4500Settings.testPattern == 9 && restored.geometricCalibrationPath == settings.geometricCalibrationPath,
      "Settings did not round trip independently of runtime objects");
  const auto partial = parseApplicationSettings(settings, "%YAML:1.0\ncolorMode: 1\n");
  require(partial.rendering.colorMode == RenderSettings::ColorMode::Depth && !partial.reconstruction.motionCompensation &&
      partial.previewFrameCount == 3 &&
      partial.geometricCalibrationSettings.interpolation == GeometricCalibrationInterpolation::RBF,
      "Partial settings changed unrelated fields");
  for (const auto *invalid : {"colorMode: 3", "solidColor: [1, .Nan, 0]", "previewFrameCount: 0",
       "geometricCalibrationInterpolation: unknown", "minimumComponentPixels: -1", "lc4500: {testPattern: 11}"}) {
    rejects([&] { parseApplicationSettings(settings, std::string("%YAML:1.0\nexposure: 200\n") + invalid + "\n"); });
    require(settings.exposure == -6 && settings.rendering.colorMode == RenderSettings::ColorMode::Solid && settings.previewFrameCount == 3,
            "Failed settings parsing changed the baseline");
  }
  rejects([&] { parseApplicationSettings(settings, "%YAML:1.0\ngeometricCalibrationPath: \"" + std::string(1024, 'a') + "\"\n"); });
}

void testProjectorSettings() {
  ApplicationSettings settings;
  auto &lc3000 = settings.lc3000Settings;
  auto &lc4500 = settings.lc4500Settings;
  require(lc3000.ledCurrent == std::array<int, 3>{255, 255, 255} &&
      lc3000.flipShortAxis && lc3000.frameRate == 60 && lc3000.triggerEnabled &&
      lc4500.ledCurrent == std::array<int, 3>{90, 0, 0} && !lc4500.flipShortAxis &&
      lc4500.exposure == std::chrono::microseconds(8333),
      "Model-specific projector defaults changed");
  lc3000.standby = true;
  lc3000.ledCurrent = {1, 2, 3};
  lc3000.flipLongAxis = true;
  lc3000.flipShortAxis = false;
  lc3000.rotate = true;
  lc3000.frameRate = 45;
  lc3000.triggerEnabled = false;
  lc3000.triggerInverted = true;
  lc3000.triggerDelayUs = 55;
  lc3000.triggerPulseUs = 150;
  lc4500.standby = true;
  lc4500.ledCurrent = {13, 27, 45};
  lc4500.ledSelection = 5;
  lc4500.flipLongAxis = true;
  lc4500.flipShortAxis = true;
  lc4500.triggerInverted = true;
  lc4500.displayMode = slstudio::LightCrafter4500Mode::FlashPatterns;
  lc4500.testPattern = 9;
  lc4500.exposure = std::chrono::microseconds(12000);
  lc4500.framePeriod = std::chrono::microseconds(16000);

  cv::FileStorage writer("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
  settings.write(writer);
  const auto yaml = writer.releaseAndGetString();
  cv::FileStorage reader(yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY);
  require(reader["lc3000"].size() == 12 && reader["lc4500"].size() == 12,
      "Projector settings contain missing or unrelated fields");
  for (const auto *field : {"ledSelection", "displayMode", "testPattern", "exposureUs", "periodUs", "repeat"})
    require(reader["lc3000"][field].isNone(), "LC3000 persisted an LC4500-only setting");
  for (const auto *field : {"rotate", "frameRate", "triggerEnabled", "triggerDelayUs", "triggerPulseUs"})
    require(reader["lc4500"][field].isNone(), "LC4500 persisted an LC3000-only setting");

  const auto restored = parseApplicationSettings(ApplicationSettings{}, yaml);
  const auto &restored3000 = restored.lc3000Settings;
  const auto &restored4500 = restored.lc4500Settings;
  require(restored3000.standby == lc3000.standby && restored3000.ledCurrent == lc3000.ledCurrent &&
      restored3000.flipLongAxis == lc3000.flipLongAxis && restored3000.flipShortAxis == lc3000.flipShortAxis &&
      restored3000.rotate == lc3000.rotate && restored3000.frameRate == lc3000.frameRate &&
      restored3000.triggerEnabled == lc3000.triggerEnabled && restored3000.triggerInverted == lc3000.triggerInverted &&
      restored3000.triggerDelayUs == lc3000.triggerDelayUs && restored3000.triggerPulseUs == lc3000.triggerPulseUs,
      "LC3000 settings did not round trip");
  require(restored4500.standby == lc4500.standby && restored4500.ledCurrent == lc4500.ledCurrent &&
      restored4500.ledSelection == lc4500.ledSelection && restored4500.flipLongAxis == lc4500.flipLongAxis &&
      restored4500.flipShortAxis == lc4500.flipShortAxis && restored4500.triggerInverted == lc4500.triggerInverted &&
      restored4500.displayMode == lc4500.displayMode && restored4500.testPattern == lc4500.testPattern &&
      restored4500.exposure == lc4500.exposure && restored4500.framePeriod == lc4500.framePeriod,
      "LC4500 settings did not round trip");
}
}

void testSequenceFiles() {
  TestDirectory directory;
  FrameSequence sequence;
  sequence.coding.codec = CodecType::PhaseShiftMotionCompensated;
  for (int index = 0; index < 3; ++index)
    sequence.frames.emplace_back(cv::Mat1b(4, 5, std::uint8_t(71 + index)));
  const auto unrelated = directory.path / "keep.txt";
  std::ofstream(unrelated) << "unrelated";
  saveSequence(directory.path, sequence);
  rejects([&] { saveSequence(directory.path, sequence); });
  require(std::filesystem::exists(unrelated), "Saving a sequence removed an unrelated file");
  std::ofstream(directory.path / "sequence.xml", std::ios::trunc) << "<broken>";
  rejects([&] { loadSequence(directory.path); });
  {
    cv::FileStorage metadata((directory.path / "sequence.xml").string(), cv::FileStorage::WRITE);
    metadata << "version" << 6;
  }
  rejects([&] { loadSequence(directory.path); });
  writeMetadata(directory.path, 6, nullptr, {}, false, 3, "PhaseShiftMotionCompensated");
  require(loadSequence(directory.path).frames.size() == 3, "Valid sequence file fixture failed");
  const auto frame = directory.path / "frameSeq_01.png";
  cv::imwrite(frame.string(), cv::Mat1b(2, 2, std::uint8_t(0)));
  rejects([&] { loadSequence(directory.path); });
  std::ofstream(frame, std::ios::trunc) << "not an image";
  rejects([&] { loadSequence(directory.path); });
  std::filesystem::remove(frame);
  rejects([&] { loadSequence(directory.path); });

  TestDirectory destination;
  rejects([&] { saveSequence(destination.path, {}); });
  auto invalid = sequence;
  invalid.frames[1] = cv::Mat1b(2, 2, std::uint8_t(0));
  rejects([&] { saveSequence(destination.path, invalid); });
  require(std::filesystem::is_empty(destination.path), "Invalid sequence left partial output");
}

void testBundledSequences() {
  const std::filesystem::path source(SLSTUDIO_SOURCE_DIR);
  GeometricCalibrationData calibration;
  require(calibration.load(source / "example_data/calibration/calibration.xml"), "Missing example calibration");
  for (const char *folder : {"skull_2x3", "skull_3ps", "skull_gray", "skull_microps", "skull_modulatedps",
       "calibration/calseq1", "calibration/calseq2", "calibration/calseq3", "calibration/calseq4",
       "calibration/calseq5", "calibration/calseq6"}) {
    const auto sequence = loadSequence(source / "example_data" / folder);
    DecodeStage decoder(calibration, withCodecSettings({}, sequence.coding));
    require(sequence.frames.size() == decoder.patternCount(), "Example metadata has the wrong codec/direction");
    require(sequence.frames.front().size() == cv::Size(calibration.frameWidth, calibration.frameHeight),
        "Example sequence has unexpected dimensions");
  }
}

int main() {
  try {
    testSequenceFiles();
    testBundledSequences();
    testApplicationSettings();
    testProjectorSettings();
    testSequenceRoundTrips();
    testMetadataFreeSequence();
    testSequenceValidation();
    testSpecializedSequenceParameters();
    testSettingsDefaults();
    testSettingsRoundTrips();
    testSettingsValidation();
    std::cout << "Sequence files, codec metadata, application and projector settings persistence passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
