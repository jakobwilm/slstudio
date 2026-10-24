#include "FrameSequence.h"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
const char *validatedCodecName(const CodecSettings &settings) {
  codecPatternCount(settings.codec, settings.direction, settings.algorithmSettings);
  return codecName(settings.codec);
}

void validateFrames(const std::vector<cv::Mat> &frames) {
  if (frames.empty()) throw std::invalid_argument("No complete frame sequence to save or load");
  for (const auto &frame : frames)
    if (frame.empty() || frame.dims != 2 || frame.type() != CV_8UC1 || frame.size() != frames.front().size())
      throw std::invalid_argument("Sequence images must be matching grayscale frames");
}

const char *directionName(CodecDir direction) {
  switch (direction) {
  case CodecDirNone: break;
  case CodecDirHorizontal: return "Horizontal";
  case CodecDirVertical: return "Vertical";
  case CodecDirBoth: return "Both";
  }
  throw std::invalid_argument("Invalid sequence coding direction");
}
}

FrameSequence loadSequence(const std::filesystem::path &directory, const CodecSettings &fallback) {
  FrameSequence sequence{{}, fallback};
  const auto metadata = directory / "sequence.xml";
  int expectedFrames = 0;
  if (std::filesystem::exists(metadata)) {
    cv::FileStorage file(metadata.string(), cv::FileStorage::READ);
    if (!file.isOpened() || !file["version"].isInt() ||
        static_cast<int>(file["version"]) != 6 ||
        !file["codec"].isString() || !file["direction"].isString() ||
        !file["frame_count"].isInt())
      throw std::runtime_error("Invalid or unsupported sequence.xml");
    sequence.coding.algorithmSettings = CodecAlgorithmSettings{};
    sequence.coding.codec = codecTypeFromName(static_cast<std::string>(file["codec"]));
    const auto direction = static_cast<std::string>(file["direction"]);
    if (direction == "Horizontal") sequence.coding.direction = CodecDirHorizontal;
    else if (direction == "Vertical") sequence.coding.direction = CodecDirVertical;
    else if (direction == "Both") sequence.coding.direction = CodecDirBoth;
    else throw std::runtime_error("Unknown direction in sequence.xml");
    expectedFrames = static_cast<int>(file["frame_count"]);
    if (expectedFrames <= 0) throw std::runtime_error("Invalid frame count in sequence.xml");
    auto integer = [&](const char *name) {
      if (!file[name].isInt()) throw std::runtime_error(std::string("Invalid sequence field: ") + name);
      return static_cast<int>(file[name]);
    };
    auto real = [&](const char *name) {
      if (!file[name].isReal() && !file[name].isInt())
        throw std::runtime_error(std::string("Invalid sequence field: ") + name);
      return static_cast<double>(file[name]);
    };
    auto &settings = sequence.coding.algorithmSettings;
    switch (sequence.coding.codec) {
    case CodecType::PhaseShift:
      settings.phaseShift.steps = integer("phase_steps");
      settings.phaseShift.periods = integer("fringe_periods");
      break;
    case CodecType::GrayCode:
      settings.grayCode.horizontalBits = integer("gray_horizontal_bits");
      settings.grayCode.verticalBits = integer("gray_vertical_bits");
      break;
    case CodecType::PhaseShiftHeterodyne:
      settings.heterodyne.steps = integer("phase_steps");
      settings.heterodyne.primaryPitch = real("primary_pitch");
      break;
    case CodecType::PhaseShiftSpatialUnwrap:
      settings.spatialUnwrap.steps = integer("phase_steps");
      settings.spatialUnwrap.periods = integer("fringe_periods");
      break;
    case CodecType::PhaseShiftMicro:
      settings.micro.steps = integer("phase_steps");
      break;
    case CodecType::PhaseShiftModulated:
      settings.modulated.signalSteps = integer("signal_steps");
      settings.modulated.carrierSteps = integer("carrier_steps");
      settings.modulated.cueSteps = integer("cue_steps");
      settings.modulated.signalPeriods = integer("signal_periods");
      settings.modulated.carrierPeriods = integer("carrier_periods");
      break;
    default: break;
    }
    if (static_cast<unsigned int>(expectedFrames) != codecPatternCount(sequence.coding.codec,
        sequence.coding.direction, sequence.coding.algorithmSettings))
      throw std::runtime_error("Frame count does not match codec parameters in sequence.xml");
  }
  validatedCodecName(sequence.coding);
  sequence.frames = loadFrameSequence(directory);
  validateFrames(sequence.frames);
  if (expectedFrames && sequence.frames.size() != static_cast<std::size_t>(expectedFrames))
    throw std::runtime_error("Frame count does not match sequence.xml");
  return sequence;
}

void saveSequence(const std::filesystem::path &directory, const FrameSequence &sequence) {
  validateFrames(sequence.frames);
  const auto codec = validatedCodecName(sequence.coding);
  if (sequence.frames.size() != codecPatternCount(sequence.coding.codec,
      sequence.coding.direction, sequence.coding.algorithmSettings))
    throw std::invalid_argument("Frame count does not match the sequence codec parameters");
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    const auto filename = entry.path().filename().string();
    if (filename == "sequence.xml" || filename.rfind("frameSeq_", 0) == 0)
      throw std::runtime_error("Destination already contains a sequence; choose an empty folder");
  }
  std::vector<std::filesystem::path> created;
  try {
    for (std::size_t index = 0; index < sequence.frames.size(); ++index) {
      std::ostringstream filename;
      filename << "frameSeq_" << std::setfill('0') << std::setw(2) << index << ".png";
      created.push_back(directory / filename.str());
      if (!cv::imwrite(created.back().string(), sequence.frames[index]))
        throw std::runtime_error("Cannot save frame: " + created.back().string());
    }
    created.push_back(directory / "sequence.xml");
    cv::FileStorage file(created.back().string(), cv::FileStorage::WRITE);
    if (!file.isOpened()) throw std::runtime_error("Cannot save sequence.xml");
    file << "version" << 6 << "codec" << codec << "direction" << directionName(sequence.coding.direction)
         << "frame_count" << static_cast<int>(sequence.frames.size());
    if (sequence.coding.codec == CodecType::PhaseShift) {
      file << "phase_steps" << sequence.coding.algorithmSettings.phaseShift.steps
           << "fringe_periods" << sequence.coding.algorithmSettings.phaseShift.periods;
    } else if (sequence.coding.codec == CodecType::GrayCode) {
      file << "gray_horizontal_bits" << sequence.coding.algorithmSettings.grayCode.horizontalBits
           << "gray_vertical_bits" << sequence.coding.algorithmSettings.grayCode.verticalBits;
    } else {
      const auto &settings = sequence.coding.algorithmSettings;
      switch (sequence.coding.codec) {
      case CodecType::PhaseShiftHeterodyne:
        file << "phase_steps" << settings.heterodyne.steps << "primary_pitch" << settings.heterodyne.primaryPitch;
        break;
      case CodecType::PhaseShiftSpatialUnwrap:
        file << "phase_steps" << settings.spatialUnwrap.steps << "fringe_periods" << settings.spatialUnwrap.periods;
        break;
      case CodecType::PhaseShiftMicro:
        file << "phase_steps" << settings.micro.steps;
        break;
      case CodecType::PhaseShiftModulated:
        file << "signal_steps" << settings.modulated.signalSteps << "carrier_steps" << settings.modulated.carrierSteps
             << "cue_steps" << settings.modulated.cueSteps << "signal_periods" << settings.modulated.signalPeriods
             << "carrier_periods" << settings.modulated.carrierPeriods;
        break;
      default: break;
      }
    }
    file.release();
  } catch (const std::exception &) {
    for (const auto &path : created) {
      std::error_code error;
      std::filesystem::remove(path, error);
      if (error) std::cerr << "Cannot remove incomplete sequence file " << path << ": " << error.message() << '\n';
    }
    throw;
  }
}

std::vector<cv::Mat> loadFrameSequence(const std::filesystem::path &directory) {
  std::vector<std::filesystem::path> files;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    const auto stem = entry.path().stem().string();
    if (entry.is_regular_file() && stem.rfind("frameSeq_", 0) == 0 && stem.size() > 9 &&
        std::all_of(stem.begin() + 9, stem.end(), [](unsigned char character) {
          return std::isdigit(character);
        }))
      files.push_back(entry.path());
  }
  std::sort(files.begin(), files.end(), [](const auto &left, const auto &right) {
    return std::stoull(left.stem().string().substr(9)) <
           std::stoull(right.stem().string().substr(9));
  });
  if (files.empty())
    throw std::runtime_error("No frameSeq_<number> images found");
  std::vector<cv::Mat> frames;
  for (const auto &file : files) {
    auto frame = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);
    if (frame.empty())
      throw std::runtime_error("Cannot read camera frame: " + file.string());
    frames.push_back(frame);
  }
  return frames;
}