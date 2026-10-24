#include "CodecGrayCode.h"
#include <algorithm>
#include <cmath>

namespace {
struct BitLayout {
  unsigned int dimensionBits = 0, encodedBits, step;
  BitLayout(unsigned int length, unsigned int patterns) {
    for (auto last = length - 1; last; last >>= 1) ++dimensionBits;
    encodedBits = std::min(dimensionBits, patterns);
    step = 1u << (dimensionBits - encodedBits);
  }
};

unsigned int binaryToGray(unsigned int value) { return value ^ (value >> 1); }

unsigned int grayToBinary(unsigned int value) {
  for (unsigned int shift = 1; shift < 32; shift <<= 1) value ^= value >> shift;
  return value;
}

unsigned int patternCount(CodecDir direction, const GrayCodeSettings &settings) {
  CodecAlgorithmSettings requestedSettings;
  requestedSettings.grayCode = settings;
  validateCodecSettings(requestedSettings);
  return 2 + (direction & CodecDirHorizontal ? settings.horizontalBits : 0) +
      (direction & CodecDirVertical ? settings.verticalBits : 0);
}

}

EncoderGrayCode::EncoderGrayCode(unsigned int columns, unsigned int rows, CodecDir direction,
                                 const GrayCodeSettings &settings)
    : Encoder(columns, rows, direction) {
  N = patternCount(dir, settings);
  patterns.emplace_back(1, 1, CV_8UC3, cv::Scalar::all(255));
  patterns.emplace_back(1, 1, CV_8UC3, cv::Scalar::all(0));
  const auto append = [&](unsigned int length, unsigned int count, bool horizontal) {
    const BitLayout layout(length, count);
    for (unsigned int bit = 0; bit < count; ++bit) {
      cv::Mat pattern(horizontal ? 1 : length, horizontal ? length : 1, CV_8UC3, cv::Scalar::all(0));
      if (bit < layout.encodedBits) {
        for (unsigned int position = 0; position < length; ++position) {
          const auto gray = binaryToGray(position);
          const auto value = ((gray >> (layout.dimensionBits - bit - 1)) & 1) ? 255 : 0;
          pattern.at<cv::Vec3b>(horizontal ? 0 : position, horizontal ? position : 0) = cv::Vec3b::all(value);
        }
      }
      patterns.push_back(std::move(pattern));
    }
  };
  if (dir & CodecDirHorizontal) append(screenCols, settings.horizontalBits, true);
  if (dir & CodecDirVertical) append(screenRows, settings.verticalBits, false);
}

cv::Mat EncoderGrayCode::getEncodingPattern(unsigned int depth) const { return patterns.at(depth); }

DecoderGrayCode::DecoderGrayCode(unsigned int columns, unsigned int rows, CodecDir direction,
                                 const GrayCodeSettings &requestedSettings)
    : Decoder(columns, rows, direction), settings(requestedSettings) {
  N = patternCount(dir, settings);
}

void DecoderGrayCode::decodeFrames(const std::vector<cv::Mat> &frames, cv::Mat &up, cv::Mat &vp,
                                   cv::Mat &mask, cv::Mat &shading) {
  validateCodecFrames(frames, N);
  up.release();
  vp.release();
  shading = frames[0] - frames[1];
  mask = shading > modulationThreshold(20);
  const auto decode = [&](cv::Mat &coordinates, unsigned int length, unsigned int count, unsigned int offset) {
    const BitLayout layout(length, count);
    coordinates.create(frames[0].size(), CV_32FC1);
    for (int row = 0; row < coordinates.rows; ++row) {
      auto *values = coordinates.ptr<float>(row);
      auto *valid = mask.ptr<std::uint8_t>(row);
      const auto *white = frames[0].ptr<std::uint8_t>(row);
      const auto *black = frames[1].ptr<std::uint8_t>(row);
      for (int column = 0; column < coordinates.cols; ++column) {
        unsigned int word = 0;
        for (unsigned int bit = 0; bit < layout.encodedBits; ++bit) {
          const int intensity = frames[offset + bit].at<std::uint8_t>(row, column);
          word = (word << 1) | (2 * intensity > white[column] + black[column]);
        }
        const auto start = grayToBinary(word) * layout.step;
        if (start >= length) {
          valid[column] = 0;
          values[column] = NAN;
        } else {
          values[column] = 0.5f * (start + std::min(start + layout.step, length) - 1);
        }
      }
    }
  };
  if (dir & CodecDirHorizontal) decode(up, screenCols, settings.horizontalBits, 2);
  if (dir & CodecDirVertical)
    decode(vp, screenRows, settings.verticalBits, 2 + (dir & CodecDirHorizontal ? settings.horizontalBits : 0));
}

std::vector<ProjectorTransition> DecoderGrayCode::decodeTransitions(
    const std::vector<cv::Mat> &frames, const cv::Mat1f &up, const cv::Mat1f &vp,
    const cv::Mat1b &mask, const cv::Mat1b &shading) const {
  validateCodecFrames(frames, N);
  const auto size = frames[0].size();
  if (mask.size() != size || shading.size() != size ||
      ((dir & CodecDirHorizontal) && up.size() != size) ||
      ((dir & CodecDirVertical) && vp.size() != size))
    throw std::invalid_argument("Gray-code transitions require matching decoded images");
  std::vector<ProjectorTransition> result;
  const auto extract = [&](const cv::Mat1f &coordinates, unsigned int length,
                           unsigned int count, unsigned int offset, CodecDir axis) {
    const BitLayout layout(length, count);
    cv::Mat1b emittedEndpoints(size, std::uint8_t(0));
    const auto crossing = [&](cv::Point first, cv::Point second) {
      if (!mask(first) || !mask(second)) return;
      const float a = coordinates(first), b = coordinates(second);
      if (!std::isfinite(a) || !std::isfinite(b)) return;
      const int firstLabel = static_cast<int>((a + 0.5f) / layout.step);
      const int secondLabel = static_cast<int>((b + 0.5f) / layout.step);
      if (std::abs(firstLabel - secondLabel) != 1) return;
      auto changed = binaryToGray(firstLabel) ^ binaryToGray(secondLabel);
      unsigned int bit = 0;
      while (changed > 1) { changed >>= 1; ++bit; }
      if (bit >= layout.encodedBits) return;
      const auto &pattern = frames[offset + layout.encodedBits - bit - 1];
      const auto normalized = [&](cv::Point pixel) {
        const double black = frames[1].at<std::uint8_t>(pixel);
        const double contrast = frames[0].at<std::uint8_t>(pixel) - black;
        return contrast > 0 ? (pattern.at<std::uint8_t>(pixel) - black) / contrast : NAN;
      };
      const double firstValue = normalized(first), secondValue = normalized(second);
      if (!std::isfinite(firstValue) || !std::isfinite(secondValue) || firstValue == secondValue) return;
      const int contrast = std::min(
          frames[0].at<std::uint8_t>(first) - frames[1].at<std::uint8_t>(first),
          frames[0].at<std::uint8_t>(second) - frames[1].at<std::uint8_t>(second));
      if (std::abs(secondValue - firstValue) * contrast <= modulationThreshold(20)) return;
      const double fraction = (0.5 - firstValue) / (secondValue - firstValue);
      if (fraction < 0 || fraction > 1) return;
      if (fraction == 0 || fraction == 1) {
        const auto endpoint = fraction == 0 ? first : second;
        if (emittedEndpoints(endpoint)) return;
        emittedEndpoints(endpoint) = 255;
      }
      const cv::Point2f camera = cv::Point2f(first) + static_cast<float>(fraction) * cv::Point2f(second - first);
      const float boundary = std::max(firstLabel, secondLabel) * layout.step - 0.5f;
      const auto brightness = cv::saturate_cast<std::uint8_t>(
          (1 - fraction) * shading(first) + fraction * shading(second));
      result.push_back({camera, axis, boundary, brightness});
    };
    for (int row = 0; row < size.height; ++row)
      for (int column = 0; column < size.width; ++column) {
        const cv::Point first(column, row);
        if (column + 1 < size.width) crossing(first, {column + 1, row});
        if (row + 1 < size.height) crossing(first, {column, row + 1});
      }
  };
  if (dir & CodecDirHorizontal) extract(up, screenCols, settings.horizontalBits, 2, CodecDirHorizontal);
  if (dir & CodecDirVertical)
    extract(vp, screenRows, settings.verticalBits,
            2 + (dir & CodecDirHorizontal ? settings.horizontalBits : 0), CodecDirVertical);
  return result;
}
