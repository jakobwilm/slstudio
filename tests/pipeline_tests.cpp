#include "DecoderWorker.h"
#include "TriangulatorWorker.h"
#include "CaptureWorker.h"
#include "FrameSequence.h"
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

static void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

int main() {
  try {
    cv::setNumThreads(1);
    const std::filesystem::path source(SLSTUDIO_SOURCE_DIR);
    GeometricCalibrationData calibration;
    require(calibration.load(source / "example_data/calibration/calibration.xml"), "Pipeline calibration missing");
    const auto frames = loadFrameSequence(source / "example_data/skull_2x3");
    DecodeStage decode(calibration);
    TriangulationStage triangulate(calibration);
    cv::Mat1f up, vp;
    cv::Mat1b mask, shading;
    decode.decodeMaps(frames, up, vp, mask, shading);
    const auto originalUp = up.clone();
    const auto originalMask = mask.clone(), originalShading = shading.clone();
    const auto expected = triangulate.triangulateDense(up, vp, mask, shading);
    require(expected->points.size() > 1000 && cv::norm(up, originalUp, cv::NORM_INF) == 0 &&
        cv::norm(mask, originalMask, cv::NORM_INF) == 0 && cv::norm(shading, originalShading, cv::NORM_INF) == 0,
        "Bundled reconstruction failed or triangulation changed its input images");
    auto await = [](auto action) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
      while (!action()) {
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Worker timed out");
        std::this_thread::yield();
      }
    };
    auto rejects = [](auto action) {
      bool rejected = false;
      try { action(); }
      catch (const std::exception &) { rejected = true; }
      require(rejected, "Invalid capture settings or input was accepted");
    };
    rejects([] { CaptureWorker invalid(0, [] { return CaptureFrame{}; }); });
    rejects([] { CaptureWorker invalid(3, CaptureWorker::StreamReader{}); });
    {
      cv::Mat borrowed(4, 5, CV_8UC1, cv::Scalar(80));
      const auto caller = std::this_thread::get_id();
      bool initialized = false;
      CaptureWorker capture([&] {
        require(std::this_thread::get_id() != caller, "Capture ran on the caller thread");
        require(initialized, "Capture ran before device initialization");
        return borrowed;
      }, [&] {
        require(std::this_thread::get_id() != caller, "Camera initialized on the caller thread");
        initialized = true;
      });
      require(capture.requestFrame(), "Capture request rejected");
      std::optional<cv::Mat> captured;
      await([&] { captured = capture.tryTake(); return captured.has_value(); });
      require(capture.ready(), "Initialized capture worker is not ready");
      borrowed.setTo(0);
      require(captured->at<std::uint8_t>(0, 0) == 80, "Borrowed camera buffer was not copied");
      capture.cancel();
      require(!capture.requestFrame() && !capture.tryTake(), "Cancelled capture accepted work");
    }
    {
      int reads = 0;
      CaptureWorker capture([&] {
        return ++reads < 3 ? cv::Mat{} : cv::Mat(2, 3, CV_8UC1, cv::Scalar(42));
      }, {}, true);
      require(capture.requestFrame(), "Capture request rejected");
      std::optional<cv::Mat> captured;
      await([&] { captured = capture.tryTake(); return captured.has_value(); });
      require(reads == 3 && captured->at<std::uint8_t>(0, 0) == 42,
          "Camera timeout interrupted a pending frame request");
    }
    {
      CaptureWorker capture([]() -> cv::Mat { throw std::runtime_error("capture test failure"); });
      capture.requestFrame();
      bool failed = false;
      await([&] {
        try { capture.tryTake(); }
        catch (const std::runtime_error &error) { failed = std::string(error.what()) == "capture test failure"; }
        return failed;
      });
    }
    {
      std::promise<void> release;
      auto gate = release.get_future().share();
      int reads = 0;
      CaptureWorker capture(3, [&] {
        if (++reads > 1 && gate.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
          throw std::runtime_error("Sequence live-frame test timed out");
        return CaptureFrame{cv::Mat(2, 3, CV_8UC1, cv::Scalar(42))};
      });
      await([&] { return capture.ready(); });
      require(capture.start(), "Sequence capture did not start");
      std::optional<cv::Mat> live;
      await([&] { live = capture.tryTake(); return live.has_value(); });
      require(live->at<std::uint8_t>(0, 0) == 42 && !capture.tryTakeSequence() && !capture.tryTake(),
          "Live camera feed waited for a complete sequence or repeated an old frame");
      capture.cancel();
      release.set_value();
    }
    {
      std::promise<void> release;
      auto gate = release.get_future().share();
      cv::Mat1b borrowed(2, 3, std::uint8_t(0));
      int reads = 0;
      CaptureWorker capture(3, [&] {
        if (reads == 3 && gate.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
          throw std::runtime_error("Sequence ownership test timed out");
        borrowed.setTo(++reads);
        return CaptureFrame{borrowed, reads == 1, static_cast<std::uint64_t>(reads)};
      });
      await([&] { return capture.ready(); });
      require(reads == 0 && !capture.requestFrame(),
              "Sequence worker read before start or accepted an individual-frame request");
      require(capture.start() && !capture.start(), "Sequence worker started twice");
      std::optional<CapturedSequence> result;
      await([&] { result = capture.tryTakeSequence(); return result.has_value(); });
      auto live = capture.tryTake();
      require(result->sequence == 1 && result->frames.size() == 3 && live &&
          live->data == result->frames.back().data,
          "Sequence and preview did not share the single owned camera-buffer copy");
      borrowed.setTo(99);
      for (int index = 0; index < 3; ++index)
        require(result->frames[index].at<std::uint8_t>(0, 0) == index + 1,
                "Sequence worker retained borrowed camera storage");
      require(!capture.tryTakeSequence() && !capture.tryTake(), "Worker repeated consumed results");
      capture.cancel();
      require(!capture.start() && !capture.requestFrame() && !capture.tryTakeSequence() && !capture.tryTake(),
              "Cancelled sequence worker retained work");
      release.set_value();
    }
    for (const auto patternCount : {1u, 3u}) {
      for (const bool markers : {false, true}) {
        cv::Mat1b borrowed(2, 3, static_cast<unsigned char>(0));
        std::size_t reads = 0;
        CaptureWorker capture(patternCount, [&] {
          const auto position = reads++ % patternCount;
          borrowed.setTo(static_cast<unsigned char>((reads - 1) % 90));
          return CaptureFrame{borrowed, markers && position == 0};
        });
        await([&] { return capture.ready(); });
        require(capture.start(), "Continuous sequence capture did not start");
        await([&] { return capture.statistics().completed >= 20; });
        auto result = capture.tryTakeSequence();
        require(result && result->sequence >= 20 && result->frames.size() == patternCount,
                "Sequence capture retained old pending sequences");
        const auto first = result->frames[0].at<unsigned char>(0, 0);
        for (unsigned int index = 1; index < patternCount; ++index)
          require(result->frames[index].at<unsigned char>(0, 0) == first + index,
                  "Sequence capture mixed patterns or reused borrowed storage");
        require(capture.statistics().dropped >= 19, "Sequence capture failed to overwrite stale results");
        capture.cancel();
        require(!capture.start() && !capture.tryTakeSequence() && !capture.tryTake(),
                "Cancelled sequence capture retained data");
      }
    }
    {
      const std::vector<CaptureFrame> input{
          {cv::Mat1b(1, 1, static_cast<unsigned char>(99)), false},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(1)), true},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(2)), false},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(4)), true},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(5)), false},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(6)), false}};
      std::size_t position = 0;
      CaptureWorker capture(3, [&] {
        if (position < input.size()) return input[position++];
        return CaptureFrame{};
      });
      await([&] { return capture.ready(); });
      capture.start();
      std::optional<CapturedSequence> result;
      await([&] { result = capture.tryTakeSequence(); return result.has_value(); });
      require(result->frames[0].at<unsigned char>(0, 0) == 4 &&
        result->frames[1].at<unsigned char>(0, 0) == 5 &&
        result->frames[2].at<unsigned char>(0, 0) == 6 && capture.statistics().dropped == 2,
              "Sequence capture failed to discard an incomplete sequence and resynchronize");
    }
    {
      const std::vector<CaptureFrame> input{
          {cv::Mat1b(1, 1, static_cast<unsigned char>(1)), true, 1},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(3)), false, 3},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(4)), false, 4},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(7)), true, 7},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(8)), false, 8},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(9)), false, 9}};
      std::size_t position = 0;
      CaptureWorker capture(3, [&] {
        if (position < input.size()) return input[position++];
        return CaptureFrame{};
      });
      await([&] { return capture.ready(); });
      capture.start();
      std::optional<CapturedSequence> result;
      await([&] { result = capture.tryTakeSequence(); return result.has_value(); });
      require(result->frames[0].at<unsigned char>(0, 0) == 7 &&
              result->frames[2].at<unsigned char>(0, 0) == 9 && capture.statistics().dropped == 2,
              "Camera frame loss silently shifted the pattern sequence");
    }
    {
      const std::vector<CaptureFrame> input{
          {cv::Mat1b(1, 1, std::uint8_t(99)), true, 10},
          {},
          {cv::Mat1b(1, 1, std::uint8_t(4)), true, 12},
          {cv::Mat1b(1, 1, std::uint8_t(5)), false, 13},
          {cv::Mat1b(1, 1, std::uint8_t(6)), false, 14}};
      std::size_t position = 0;
      CaptureWorker capture(3, [&] {
        return position < input.size() ? input[position++] : CaptureFrame{};
      });
      await([&] { return capture.ready(); });
      capture.start();
      std::optional<CapturedSequence> result;
      await([&] { result = capture.tryTakeSequence(); return result.has_value(); });
      require(result->frames.front().at<std::uint8_t>(0, 0) == 4 &&
          result->frames.back().at<std::uint8_t>(0, 0) == 6 && capture.statistics().dropped == 1,
          "An empty camera frame did not discard the partial sequence");
    }
    for (const auto maximum : {std::uint64_t(UINT32_MAX), std::uint64_t(UINT64_MAX)}) {
      const std::vector<CaptureFrame> input{
          {cv::Mat1b(1, 1, static_cast<unsigned char>(1)), true, maximum - 1, maximum},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(2)), false, maximum, maximum},
          {cv::Mat1b(1, 1, static_cast<unsigned char>(3)), false, 0, maximum}};
      std::size_t position = 0;
      CaptureWorker capture(3, [&] {
        if (position < input.size()) return input[position++];
        return CaptureFrame{};
      });
      await([&] { return capture.ready(); });
      capture.start();
      std::optional<CapturedSequence> result;
      await([&] { result = capture.tryTakeSequence(); return result.has_value(); });
      require(result->frames[2].at<unsigned char>(0, 0) == 3 && capture.statistics().dropped == 0,
              "Camera counter rollover discarded a complete sequence");
    }
    for (const auto &invalid : {
        CaptureFrame{cv::Mat1b(2, 3, std::uint8_t(7)), false, 8, 7},
        CaptureFrame{cv::Mat(2, 3, CV_32FC1, cv::Scalar(7))},
        CaptureFrame{cv::Mat(2, 3, CV_8UC3, cv::Scalar::all(7))},
        CaptureFrame{cv::Mat1b(1, 1, std::uint8_t(7))}}) {
      for (const bool sequenceConsumer : {false, true}) {
        int reads = 0;
        CaptureWorker capture(3, [&] {
          if (++reads <= 4) return CaptureFrame{cv::Mat1b(2, 3, std::uint8_t(7))};
          return invalid;
        });
        await([&] { return capture.ready(); });
        capture.start();
        await([&] { return !capture.ready(); });
        rejects([&] {
          if (sequenceConsumer) capture.tryTakeSequence();
          else capture.tryTake();
        });
        require(capture.statistics().completed == 1 && !capture.tryTakeSequence() &&
            !capture.tryTake() && !capture.start() && !capture.requestFrame(),
            "Failed sequence worker retained pending results or continued capturing");
      }
    }
    {
      CaptureWorker capture(3, [] { return CaptureFrame{}; },
                            [] { throw std::runtime_error("Sequence initialization failure"); });
      bool failed = false;
      await([&] {
        try { capture.tryTakeSequence(); }
        catch (const std::runtime_error &error) {
          failed = std::string(error.what()) == "Sequence initialization failure";
        }
        return failed;
      });
      require(!capture.ready() && !capture.start() && !capture.tryTake(),
              "Failed sequence initialization allowed capture");
    }
    {
      TriangulatorWorker triangulator(calibration);
      DecoderWorker decoder(calibration, {}, [&](std::uint64_t sequence, cv::Mat1f u, cv::Mat1f v,
                                                cv::Mat1b valid, cv::Mat1b shade) {
        require(triangulator.trySubmitDense(sequence, std::move(u), std::move(v),
            std::move(valid), std::move(shade)), "Triangulator refused decoded data");
      });
      std::vector<cv::Mat> borrowed;
      for (const auto &frame : frames) borrowed.push_back(frame.clone());
      require(decoder.trySubmit(borrowed), "Initial decoder submission failed");
      for (auto &frame : borrowed) frame.setTo(0);
      std::optional<ReconstructionResult> result;
      await([&] { decoder.checkError(); result = triangulator.tryTake(); return result.has_value(); });
      require(result->sequence == 1, "Initial sequence ID differs");
      auto verify = [&] {
        require(result->cloud->points.size() == expected->points.size(), "Pipeline output count differs");
        for (std::size_t index = 0; index < expected->points.size(); ++index) {
          const auto &actual = result->cloud->points[index];
          const auto &reference = expected->points[index];
          require(actual.x == reference.x && actual.y == reference.y && actual.z == reference.z &&
                  actual.r == reference.r && actual.g == reference.g && actual.b == reference.b,
                  "Pipeline output or ownership differs from synchronous reconstruction");
        }
      };
      verify();
      await([&] { return !decoder.busy() && !triangulator.busy(); });
      constexpr std::size_t submissions = 16;
      for (std::size_t sequence = 0; sequence < submissions; ++sequence)
        require(decoder.trySubmit(frames) && decoder.canSubmit(), "Latest-only decoder applied backpressure");
      std::uint64_t lastSequence = 1;
      await([&] {
        decoder.checkError();
        result = triangulator.tryTake();
        if (!result) return false;
        require(result->sequence > lastSequence, "Pipeline results moved backwards");
        lastSequence = result->sequence;
        return result->sequence == submissions + 1;
      });
      verify();
      await([&] { return !decoder.busy() && !triangulator.busy(); });
      require(!triangulator.tryTake(), "Workers repeated a consumed result");
      const auto produced = triangulator.statistics().reconstructed;
      require(decoder.trySubmit(frames), "Cannot submit stalled-consumer test sequence");
      await([&] { decoder.checkError(); return triangulator.statistics().reconstructed == produced + 1; });
      require(decoder.trySubmit(frames), "Unread result blocked a newer submission");
      await([&] { decoder.checkError(); return triangulator.statistics().reconstructed == produced + 2; });
      result = triangulator.tryTake();
      await([&] { return !decoder.busy() && !triangulator.busy(); });
      require(result && result->sequence == submissions + 3 && !triangulator.tryTake() &&
        triangulator.statistics().dropped > 0,
        "Stalled consumer received an old result rather than the newest reconstruction");
      for (int sequence = 0; sequence < 4; ++sequence) decoder.trySubmit(frames);
      decoder.cancel();
      triangulator.cancel();
      require(!decoder.busy() && !triangulator.busy() && !decoder.canSubmit() &&
          !triangulator.canSubmit() && !decoder.trySubmit(frames) && !triangulator.tryTake(),
              "Cancelled pipeline retained work");
    }
    {
      DecoderWorker decoder(calibration, {}, [](std::uint64_t, cv::Mat1f, cv::Mat1f, cv::Mat1b, cv::Mat1b) {
        throw std::runtime_error("Invalid frames reached the result handler");
      });
      require(decoder.trySubmit({}), "Invalid sequence submission failed unexpectedly");
      bool failed = false;
      await([&] {
        try { decoder.checkError(); }
        catch (const std::invalid_argument &) { failed = true; }
        return failed;
      });
      require(!decoder.busy() && !decoder.canSubmit(), "Failed decoder did not stop");
    }
    {
      std::promise<void> release;
      auto gate = release.get_future().share();
      const auto caller = std::this_thread::get_id();
      TriangulatorWorker triangulator(calibration);
      DecoderWorker decoder(calibration, {}, [&](std::uint64_t sequence, cv::Mat1f u, cv::Mat1f v,
                                                cv::Mat1b valid, cv::Mat1b shade) {
        require(std::this_thread::get_id() != caller, "Decoding ran on the caller thread");
        require(triangulator.trySubmitDense(sequence, std::move(u), std::move(v),
            std::move(valid), std::move(shade)), "Triangulator rejected decoded data");
        if (gate.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          throw std::runtime_error("Decode/triangulation overlap test timed out");
      });
      require(decoder.trySubmit(frames), "Overlap test submission failed");
      std::optional<ReconstructionResult> result;
      await([&] { decoder.checkError(); result = triangulator.tryTake(); return result.has_value(); });
      require(decoder.busy() && result->cloud->points.size() == expected->points.size(),
              "Triangulation did not complete independently of the decoder thread");
      release.set_value();
      await([&] { decoder.checkError(); return !decoder.busy(); });
    }
    {
      rejects([&] { DecoderWorker invalid(calibration, {}, {}); });
      DecoderWorker failing(calibration, {}, [](std::uint64_t, cv::Mat1f, cv::Mat1f, cv::Mat1b, cv::Mat1b) {
        throw std::runtime_error("Decoded result delivery failed");
      });
      require(failing.trySubmit(frames), "Cannot submit result-handler failure test");
      bool failed = false;
      await([&] {
        try { failing.checkError(); }
        catch (const std::runtime_error &error) {
          failed = std::string(error.what()) == "Decoded result delivery failed";
          if (!failed) throw;
        }
        return failed;
      });
      require(!failing.canSubmit() && !failing.busy(), "Failed result handler left decoding active");
    }
    {
      TriangulatorWorker failing(calibration);
      require(failing.trySubmitDense(1, {}, {}, {}, {}), "Cannot submit invalid triangulation input");
      bool failed = false;
      await([&] {
        try { failing.tryTake(); }
        catch (const std::invalid_argument &) { failed = true; }
        return failed;
      });
      require(!failing.canSubmit() && !failing.busy() && !failing.tryTake(),
              "Failed triangulation worker retained work or stale results");
    }
    for (const bool sparseSettings : {false, true}) {
      ReconstructionSettings settings;
      if (sparseSettings) settings.coding.codec = CodecType::GrayCode;
      TriangulatorWorker failing(calibration, settings);
      const bool accepted = sparseSettings ?
          failing.trySubmitDense(1, up, vp, mask, shading) :
          failing.trySubmitSparse(1, {});
      require(accepted, "Worker refused an input before asynchronous validation");
      bool failed = false;
      await([&] {
        try { failing.tryTake(); }
        catch (const std::invalid_argument &) { failed = true; }
        return failed;
      });
      require(!failing.canSubmit() && !failing.busy() && !failing.tryTake(),
              "Wrong triangulation input form failed silently or retained stale data");
    }
    std::cout << "Worker ownership, latest-only overload, ordering, overlap, errors and cancellation passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}