#include "gray_code_test_data.h"
#include "DecoderWorker.h"
#include "TriangulatorWorker.h"
#include "FrameSequence.h"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function> void rejects(Function action) {
  bool rejected = false;
  try { action(); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "Invalid Gray-code input was accepted");
}

ReconstructionSettings graySettings(CodecDir direction = CodecDirBoth) {
  ReconstructionSettings settings;
  settings.coding.codec = CodecType::GrayCode;
  settings.coding.direction = direction;
  settings.points.maxDepth = 10;
  return settings;
}

void testIntegerDecoding() {
  for (const auto size : {cv::Size(1, 1), cv::Size(7, 5), cv::Size(127, 93),
                          cv::Size(1024, 64), cv::Size(1500, 67)}) {
    for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
      EncoderGrayCode encoder(size.width, size.height, direction);
      DecoderGrayCode decoder(size.width, size.height, direction);
      const auto count = 2u + (direction & CodecDirHorizontal ? 10 : 0) + (direction & CodecDirVertical ? 6 : 0);
      require(encoder.getNPatterns() == count && decoder.getNPatterns() == count,
              "Legacy Gray-code capture lengths changed");
      unsigned int offset = 2;
      for (const auto axis : {CodecDirHorizontal, CodecDirVertical}) {
        if (!(direction & axis)) continue;
        const int length = axis == CodecDirHorizontal ? size.width : size.height;
        const int patterns = axis == CodecDirHorizontal ? 10 : 6;
        const int bits = static_cast<int>(std::ceil(std::log2(length)));
        for (int bit = 0; bit < patterns; ++bit) {
          const auto pattern = encoder.getEncodingPattern(offset++);
          for (int pixel = 0; pixel < length; ++pixel) {
            const auto gray = static_cast<unsigned int>(pixel ^ (pixel >> 1));
            const int expected = bit < bits ? ((gray >> (bits - bit - 1)) & 1) * 255 : 0;
            require(pattern.at<cv::Vec3b>(axis == CodecDirHorizontal ? 0 : pixel,
                axis == CodecDirHorizontal ? pixel : 0) == cv::Vec3b::all(expected),
                "Gray-code pattern pixels or bit ordering changed");
          }
        }
      }
      std::vector<cv::Mat> frames;
      for (unsigned int index = 0; index < count; ++index) {
        const auto pattern = encoder.getEncodingPattern(index);
        cv::Mat gray;
        cv::cvtColor(cv::repeat(pattern, size.height / pattern.rows, size.width / pattern.cols),
                     gray, cv::COLOR_RGB2GRAY);
        frames.push_back(gray);
      }
      cv::Mat up, vp, mask, shading;
      decoder.decodeFrames(frames, up, vp, mask, shading);
      require(cv::countNonZero(mask) == size.area() && cv::countNonZero(shading != 255) == 0,
              "Gray-code ideal frames lost valid pixels or shading");
      for (int row = 0; row < size.height; ++row)
        for (int column = 0; column < size.width; ++column) {
          const auto expected = [](int pixel, int length, int patterns) {
            const int bits = static_cast<int>(std::ceil(std::log2(length)));
            const int step = 1 << std::max(0, bits - patterns);
            const int start = pixel / step * step;
            return 0.5f * (start + std::min(start + step, length) - 1);
          };
          if (direction & CodecDirHorizontal)
            require(up.at<float>(row, column) == expected(column, size.width, 10), "Horizontal Gray-code word decoded incorrectly");
          if (direction & CodecDirVertical)
            require(vp.at<float>(row, column) == expected(row, size.height, 6), "Truncated vertical Gray-code word decoded incorrectly");
        }
      require(up.empty() == !(direction & CodecDirHorizontal) && vp.empty() == !(direction & CodecDirVertical),
              "Gray decoder retained an unrequested coordinate map");
      rejects([&] { encoder.getEncodingPattern(count); });
    }
  }
  DecoderGrayCode missing(7, 5, CodecDirBoth);
  cv::Mat up, vp, mask, shading;
  rejects([&] { missing.decodeFrames({}, up, vp, mask, shading); });
  std::vector<cv::Mat> frames(missing.getNPatterns(), cv::Mat1b(5, 7, std::uint8_t(20)));
  frames[2] = cv::Mat(5, 7, CV_32FC1, cv::Scalar(0));
  rejects([&] { missing.decodeFrames(frames, up, vp, mask, shading); });
  frames[2] = cv::Mat1b(4, 7, std::uint8_t(20));
  rejects([&] { missing.decodeFrames(frames, up, vp, mask, shading); });
}

void checkTransitionAccuracy(const std::vector<ProjectorTransition> &decoded, const GeometricCalibrationData &calibration,
                             bool rotated = false, double tolerance = 0.02) {
  require(!decoded.empty(),
          "Gray code did not produce explicit sparse samples");
  std::set<std::tuple<float, float, int, float>> unique;
  bool fractional = false;
  for (const auto &sample : decoded) {
    const int axis = sample.direction == CodecDirHorizontal ? 0 : 1;
    const cv::Point2f camera = rotated ?
        cv::Point2f(sample.camera.y, calibration.frameHeight - 1 - sample.camera.x) : sample.camera;
    const double pixel = axis == 0 ? camera.x : camera.y;
    const double projector = pixel - calibration.Kc(axis, 2) + calibration.Kp(axis, 2) +
        calibration.Kp(axis, axis) * calibration.Tp[axis];
    require(std::abs(projector - sample.coordinate) < tolerance, "Subpixel transition position exceeded the error bound");
    require(std::abs(sample.coordinate + 0.5f - std::round(sample.coordinate + 0.5f)) < 1e-6,
            "Gray-code transition used pixel centers instead of projector boundaries");
    require(unique.emplace(sample.camera.x, sample.camera.y, axis, sample.coordinate).second,
            "Transition extraction emitted duplicate samples");
    fractional |= std::abs(sample.camera.x - std::round(sample.camera.x)) > 0.01 ||
        std::abs(sample.camera.y - std::round(sample.camera.y)) > 0.01;
  }
  require(fractional, "Transition extraction snapped camera positions to the integer grid");
}

void testTransitions() {
  const auto calibration = grayCodeCalibration();
  for (const auto direction : {CodecDirHorizontal, CodecDirVertical, CodecDirBoth}) {
    for (const bool reflectance : {false, true}) {
      auto frames = grayCodeFrames(calibration, direction, 1, reflectance);
      DecodeStage decoder(calibration, graySettings(direction));
      const auto decoded = decoder.decodeTransitions(frames);
      checkTransitionAccuracy(decoded, calibration);
      for (const auto &sample : decoded)
        require(sample.shading >= 120 && sample.shading <= 200, "Transition shading lost black/white normalization");
    }
  }
  auto frames = grayCodeFrames(calibration, CodecDirBoth);
  auto rotatedCalibration = calibration;
  std::swap(rotatedCalibration.frameWidth, rotatedCalibration.frameHeight);
  std::vector<cv::Mat> rotated;
  for (const auto &frame : frames) {
    rotated.emplace_back();
    cv::rotate(frame, rotated.back(), cv::ROTATE_90_CLOCKWISE);
  }
  checkTransitionAccuracy(DecodeStage(rotatedCalibration, graySettings()).decodeTransitions(rotated), calibration, true);
  cv::RNG random(1729);
  std::vector<cv::Mat> noisy;
  for (const auto &source : frames) {
    auto frame = source.clone();
    cv::Mat noise(frame.size(), CV_16SC1), value;
    random.fill(noise, cv::RNG::NORMAL, 0, 0.6);
    frame.convertTo(value, CV_16SC1);
    value += noise;
    value.convertTo(frame, CV_8UC1);
    noisy.push_back(std::move(frame));
  }
  checkTransitionAccuracy(DecodeStage(calibration, graySettings()).decodeTransitions(noisy), calibration, false, 0.06);
  for (const double sigma : {0.4, 0.7}) {
    std::vector<cv::Mat> blurred;
    for (const auto &frame : frames) {
      blurred.emplace_back();
      cv::GaussianBlur(frame, blurred.back(), cv::Size(), sigma, sigma);
    }
    checkTransitionAccuracy(DecodeStage(calibration, graySettings()).decodeTransitions(blurred), calibration, false, 0.12);
  }
  auto shadowed = grayCodeFrames(calibration, CodecDirBoth);
  for (auto &frame : shadowed) frame.colRange(20, 26).setTo(20);
  const auto shadow = DecodeStage(calibration, graySettings()).decodeTransitions(shadowed);
  require(!shadow.empty(), "A local shadow discarded all transitions");
  for (const auto &sample : shadow)
    require(sample.camera.x < 20 || sample.camera.x > 25, "Transition extraction bridged an invalid region");
  auto discontinuous = grayCodeFrames(calibration, CodecDirBoth);
  const auto distant = grayCodeFrames(calibration, CodecDirBoth, 2);
  for (std::size_t index = 0; index < discontinuous.size(); ++index)
    distant[index].colRange(32, 64).copyTo(discontinuous[index].colRange(32, 64));
  const auto step = DecodeStage(calibration, graySettings()).decodeTransitions(discontinuous);
  for (const auto &sample : step)
    require(sample.camera.x <= 31 || sample.camera.x >= 32, "Transition extraction bridged a multi-label depth jump");
  auto weak = graySettings();
  weak.minimumModulation = 201;
  require(DecodeStage(calibration, weak).decodeTransitions(frames).empty(), "Weakly illuminated transitions were accepted");
  weak = graySettings();
  weak.minimumComponentPixels = calibration.frameWidth * calibration.frameHeight + 1;
  require(DecodeStage(calibration, weak).decodeTransitions(frames).empty(), "Image support filtering was not applied to transitions");
  auto weakBits = grayCodeFrames(calibration, CodecDirBoth);
  for (std::size_t index = 2; index < weakBits.size(); ++index)
    weakBits[index].convertTo(weakBits[index], CV_8UC1, 0.02, 117.6);
  require(DecodeStage(calibration, graySettings()).decodeTransitions(weakBits).empty(),
          "Strong white/black references hid an unreliable low-contrast bit transition");
  auto flat = grayCodeFrames(calibration, CodecDirBoth);
  for (std::size_t index = 2; index < flat.size(); ++index) flat[index].setTo(20);
  const auto noEdges = TriangulationStage(calibration, graySettings()).triangulateSparse(
      DecodeStage(calibration, graySettings()).decodeTransitions(flat));
  require(noEdges->points.empty() && noEdges->pixelIndices.empty(), "Stripe interiors fabricated sparse measurements");
  auto tiesCalibration = calibration;
  tiesCalibration.Tp[0] = -0.15625f;
  const auto ties = DecodeStage(tiesCalibration, graySettings(CodecDirHorizontal)).decodeTransitions(
      grayCodeFrames(tiesCalibration, CodecDirHorizontal));
  std::set<std::pair<float, float>> endpoints;
  for (const auto &sample : ties)
    require(endpoints.emplace(sample.camera.x, sample.camera.y).second, "Integer-position crossings were duplicated");
}

void testTriangulation() {
  auto calibration = grayCodeCalibration();
  calibration.kc = {0.35f, -0.05f, 0.015f, -0.01f, 0};
  calibration.kp = {0.2f, -0.02f, 0.01f, -0.01f, 0};
  cv::Rodrigues(cv::Vec3f(0.02f, 0.03f, -0.01f), calibration.Rp);
  std::vector<cv::Point3f> points;
  for (int index = 0; index < 12; ++index) {
    const float depth = 0.7f + 0.04f * index;
    points.emplace_back((-0.32f + 0.19f * (index % 4)) * depth,
                        (-0.20f + 0.19f * (index / 4)) * depth, depth);
  }
  std::vector<cv::Point2f> camera, projector;
  cv::Vec3d rotation;
  cv::Rodrigues(cv::Matx33d(calibration.Rp), rotation);
  cv::projectPoints(points, cv::Vec3d::all(0), cv::Vec3d::all(0), calibration.Kc, calibration.kc, camera);
  cv::projectPoints(points, rotation, calibration.Tp, calibration.Kp, calibration.kp, projector);
  std::vector<ProjectorTransition> samples;
  for (std::size_t index = 0; index < points.size(); ++index)
    for (const auto axis : {CodecDirHorizontal, CodecDirVertical})
      samples.push_back({camera[index], axis, axis == CodecDirHorizontal ? projector[index].x : projector[index].y, 173});
  const auto corrected = TriangulatorUnorganized(calibration, true).triangulate(samples);
  const auto baseline = TriangulatorUnorganized(calibration).triangulate(samples);
  require(corrected->pixelIndices.empty() && corrected->points.size() == samples.size() &&
      baseline->points.size() == samples.size(), "Sparse triangulation lost valid samples or fabricated grid indices");
  double correctedError = 0, baselineError = 0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto &point = corrected->points[index];
    const auto &original = baseline->points[index];
    const auto expected = points[index / 2];
    const double error = cv::norm(cv::Point3f(point.x, point.y, point.z) - expected);
    require(error < 2e-5 && point.r == 173 && point.g == 173 && point.b == 173,
            "Sparse distortion-corrected triangulation exceeded its 3D error bound or lost shading");
    correctedError += error;
    baselineError += cv::norm(cv::Point3f(original.x, original.y, original.z) - expected);
  }
  require(baselineError > correctedError * 10, "Sparse projector distortion option did not improve geometry");
  const auto bounded = TriangulatorUnorganized(calibration, true).triangulate(samples, {0, 10});
  require(bounded->points.size() == samples.size(), "Sparse depth bounds dropped valid transitions");
  const auto filtered = TriangulatorUnorganized(calibration, true).triangulate(samples, {2, 3});
  require(filtered->points.empty() && filtered->pixelIndices.empty(), "Sparse depth bounds retained excluded points");
  require(TriangulatorUnorganized(calibration).triangulate({})->points.empty(), "Empty sparse input fabricated points");
  auto invalid = samples;
  invalid.front().direction = CodecDirBoth;
  rejects([&] { TriangulatorUnorganized(calibration).triangulate(invalid); });
  invalid = samples;
  invalid.front().camera.x = NAN;
  rejects([&] { TriangulatorUnorganized(calibration).triangulate(invalid); });
  invalid = samples;
  invalid.front().coordinate = -2;
  rejects([&] { TriangulatorUnorganized(calibration).triangulate(invalid); });
  rejects([&] { TriangulatorUnorganized(calibration).triangulate(samples, {3, 2}); });
  auto degenerate = grayCodeCalibration();
  degenerate.Tp = cv::Vec3f::all(0);
  require(TriangulatorUnorganized(degenerate).triangulate(
      {{{20.25f, 15.75f}, CodecDirHorizontal, 52.25f, 200}})->points.empty(),
      "Parallel camera/projector constraint fabricated a depth");
  auto badCalibration = calibration;
  badCalibration.Kc(0, 0) = 0;
  rejects([&] { TriangulatorUnorganized invalidTriangulator(badCalibration); });
}

void testReconstructionAndPersistence() {
  const auto calibration = grayCodeCalibration();
  const auto settings = graySettings();
  const auto frames = grayCodeFrames(calibration, settings.coding.direction);
  DecodeStage decode(calibration, settings);
  TriangulationStage triangulate(calibration, settings);
  const auto decoded = decode.decodeTransitions(frames);
  const auto expected = triangulate.triangulateSparse(decoded);
  require(!expected->points.empty() && expected->points.size() == decoded.size() && expected->pixelIndices.empty(),
          "Gray-code reconstruction did not preserve sparse transition samples");
  for (const auto &point : expected->points)
    require(std::abs(point.z - 1) < 0.002, "Gray-code plane reconstruction has excessive depth bias");
  cv::Mat1f up, vp;
  cv::Mat1b mask, shading;
  rejects([&] { decode.decodeMaps(frames, up, vp, mask, shading); });
  rejects([&] { DecodeStage(calibration).decodeTransitions(frames); });
  rejects([&] { triangulate.triangulateDense(up, vp, mask, shading); });
  rejects([&] { TriangulationStage(calibration).triangulateSparse(decoded); });
  for (const bool lis : {false, true}) {
    auto unsupported = settings;
    unsupported.lisPhaseFilter = lis;
    unsupported.rejectDepthSpikes = !lis;
    rejects([&] { DecodeStage invalid(calibration, unsupported); });
    rejects([&] { TriangulationStage invalid(calibration, unsupported); });
  }
  TriangulatorWorker triangulator(calibration, settings);
  rejects([&] { DecoderWorker invalid(calibration, settings, {}); });
  rejects([&] {
    DecoderWorker invalid(calibration, {}, {}, [](std::uint64_t, std::vector<ProjectorTransition>) {});
  });
  DecoderWorker decoder(calibration, settings, {}, [&](std::uint64_t sequence, std::vector<ProjectorTransition> coordinates) {
    require(triangulator.trySubmitSparse(sequence, std::move(coordinates)), "Triangulator refused sparse data");
  });
  require(decoder.trySubmit(frames), "Sparse decoder rejected a sequence");
  std::optional<ReconstructionResult> result;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!result) {
    decoder.checkError();
    result = triangulator.tryTake();
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Sparse reconstruction pipeline timed out");
    std::this_thread::yield();
  }
  require(result->cloud->pixelIndices.empty() && result->cloud->points.size() == expected->points.size(),
          "Asynchronous Gray-code reconstruction changed sample count or organization");
  for (std::size_t index = 0; index < expected->points.size(); ++index) {
    const auto &a = expected->points[index], &b = result->cloud->points[index];
    require(a.x == b.x && a.y == b.y && a.z == b.z && a.r == b.r,
            "Asynchronous Gray-code geometry differs from synchronous output");
  }
  auto flat = frames;
  for (std::size_t index = 2; index < flat.size(); ++index)
    flat[index] = cv::Mat1b(frames.front().size(), std::uint8_t(20));
  require(decoder.trySubmit(flat), "Sparse worker rejected a sequence with no transitions");
  result.reset();
  while (!result) {
    decoder.checkError();
    result = triangulator.tryTake();
    if (std::chrono::steady_clock::now() >= deadline)
      throw std::runtime_error("Empty sparse coordinate batch was not delivered");
    std::this_thread::yield();
  }
  require(result->sequence == 2 && result->cloud->points.empty() && result->cloud->pixelIndices.empty(),
          "Empty sparse coordinates were mistaken for missing input or produced geometry");
  decoder.cancel();
  triangulator.cancel();
  require(!decoder.trySubmit(frames), "Cancelled sparse reconstruction accepted new work");
  DecoderWorker failing(calibration, settings, {}, [](std::uint64_t, std::vector<ProjectorTransition>) {
    throw std::runtime_error("Invalid frames reached the result handler");
  });
  auto malformed = frames;
  malformed.pop_back();
  require(failing.trySubmit(malformed), "Sparse pipeline did not accept an input for validation");
  bool failed = false;
  while (!failed) {
    try { failing.checkError(); }
    catch (const std::invalid_argument &error) {
      failed = std::string(error.what()) == "Frame count does not match the selected codec and direction";
      if (!failed) throw;
    }
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Sparse pipeline failure was not propagated");
    std::this_thread::yield();
  }
  require(!failing.canSubmit(), "Failed sparse pipeline accepted further work");
  const auto directory = std::filesystem::temp_directory_path() /
      ("slstudio-gray-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  require(std::filesystem::create_directory(directory), "Cannot create Gray-code sequence test directory");
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::error_code error; std::filesystem::remove_all(path, error); }
  } cleanup{directory};
  saveSequence(directory, {frames, settings.coding});
  const auto restored = loadSequence(directory);
  require(restored.coding == settings.coding && restored.frames.size() == frames.size(),
          "Gray-code sequence settings or fixed pattern count did not survive persistence");
  const auto restoredSettings = withCodecSettings(settings, restored.coding);
  const auto restoredCloud = TriangulationStage(calibration, restoredSettings).triangulateSparse(
      DecodeStage(calibration, restoredSettings).decodeTransitions(restored.frames));
  require(restoredCloud->points.size() == expected->points.size() && restoredCloud->pixelIndices.empty(),
          "Persisted Gray-code sequence did not reconstruct as an unorganized cloud");
  auto current = settings;
  current.minimumModulation = 255;
  auto active = withCodecSettings(current, restored.coding);
  require(TriangulationStage(calibration, active).triangulateSparse(
      DecodeStage(calibration, active).decodeTransitions(restored.frames))->points.empty(),
      "Loaded Gray-code sequence ignored the current modulation threshold");
  current = settings;
  current.points.minDepth = 1.5f;
  active = withCodecSettings(current, restored.coding);
  require(TriangulationStage(calibration, active).triangulateSparse(
      DecodeStage(calibration, active).decodeTransitions(restored.frames))->points.empty(),
      "Loaded Gray-code sequence ignored the current depth limits");
  require(savePointCloudPLY((directory / "sparse.ply").string(), *expected) &&
      savePointCloudPCD((directory / "sparse.pcd").string(), *expected), "Sparse point cloud export failed");
  std::filesystem::remove(directory / "sequence.xml");
  require(loadSequence(directory, settings.coding).coding == settings.coding,
          "Gray-code image folders lost fallback codec information");
}
}

int main() {
  try {
    cv::setNumThreads(1);
    testIntegerDecoding();
    testTransitions();
    testTriangulation();
    testReconstructionAndPersistence();
    std::cout << "Gray-code bits, subpixel transitions, unorganized geometry, pipeline and persistence passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
