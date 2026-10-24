#include "CaptureWorker.h"
#include "ScanSession.h"
#include "ProjectorPresentationController.h"
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {
using slstudio::gui::ScanSession;
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function> void rejects(Function action) {
  bool rejected = false;
  try { action(); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "Invalid acquisition operation was accepted");
}

template <typename Function> void await(Function ready) {
  const auto deadline = Clock::now() + 5s;
  while (!ready()) {
    if (Clock::now() >= deadline) throw std::runtime_error("Acquisition test timed out");
    std::this_thread::yield();
  }
}

struct Trace {
  struct Event { std::string operation; std::thread::id thread; };
  std::mutex mutex;
  std::vector<Event> events;
  std::atomic<int> pattern{0}, reads{0};
  std::atomic<bool> running{false};
  Clock::time_point presented{};
  std::vector<int> uploaded;
  bool hardware = false, failPresentation = false;
  std::shared_future<void> uploadGate;
  std::promise<void> uploading;
  std::chrono::milliseconds settling{};
  void event(const char *operation) {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back({operation, std::this_thread::get_id()});
  }
};

class Camera final : public CaptureWorker::Camera {
public:
  explicit Camera(CaptureWorker::StreamReader source, std::function<void()> closing = {})
      : reader(std::move(source)), close(std::move(closing)) {}
  ~Camera() override { if (close) close(); }
  CaptureFrame read() override { return reader(); }
private:
  CaptureWorker::StreamReader reader;
  std::function<void()> close;
};

class Projector final : public slstudio::Projector {
public:
  explicit Projector(std::shared_ptr<Trace> log) : trace(std::move(log)) { trace->event("projector open"); }
  ~Projector() override { trace->event("projector close"); }
  cv::Size resolution() const override { return {3, 2}; }
  slstudio::ProjectorCapabilities capabilities() const override {
    return {trace->hardware, trace->hardware, true};
  }
  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    trace->event("upload");
    if (trace->uploadGate.valid()) {
      trace->uploading.set_value();
      require(trace->uploadGate.wait_for(3s) == std::future_status::ready, "Upload was not released");
    }
    trace->uploaded.clear();
    for (const auto &pattern : patterns) trace->uploaded.push_back(pattern.at<std::uint8_t>(0, 0));
  }
  void present(std::size_t index) override {
    trace->event("present");
    if (trace->failPresentation) throw std::runtime_error("Presentation failed");
    require(index < trace->uploaded.size(), "Presented an invalid pattern");
    trace->pattern = static_cast<int>(index);
    std::lock_guard<std::mutex> lock(trace->mutex);
    trace->presented = Clock::now();
  }
  void blank(bool = false) override { trace->event("blank"); }
  void configureSequence(const slstudio::ProjectorSequenceSettings &timing) override {
    trace->event(timing.repeat ? "configure repeat" : "configure single");
  }
  void startSequence() override { trace->event("start"); trace->running = true; }
  void stopSequence() override { trace->event("stop"); trace->running = false; }
  void programPatterns(const std::vector<cv::Mat> &, const slstudio::ProjectorProgrammingProgress &progress) override {
    trace->event("program");
    if (progress) progress(1, "Done");
  }
private:
  std::shared_ptr<Trace> trace;
};

std::vector<cv::Mat> patterns() {
  return {cv::Mat1b(2, 3, std::uint8_t(0)), cv::Mat1b(2, 3, std::uint8_t(1)),
          cv::Mat1b(2, 3, std::uint8_t(2))};
}

CaptureWorker::CameraFactory cameraFactory(const std::shared_ptr<Trace> &trace) {
  return [trace] {
    trace->event("camera open");
    return std::make_unique<Camera>([trace] {
      trace->event("read");
      ++trace->reads;
      if (trace->settling.count()) {
        std::lock_guard<std::mutex> lock(trace->mutex);
        require(Clock::now() - trace->presented >= trace->settling, "Captured before presentation settled");
      }
      return CaptureFrame{cv::Mat1b(2, 3, static_cast<std::uint8_t>(10 + trace->pattern.load()))};
    }, [trace] { trace->event("camera close"); });
  };
}

void testSoftwareOwnership() {
  const auto caller = std::this_thread::get_id();
  auto trace = std::make_shared<Trace>();
  trace->settling = 15ms;
  {
    CaptureWorker capture;
    capture.openCamera(cameraFactory(trace)).get();
    capture.openProjector([trace] { return std::make_unique<Projector>(trace); }).get();
    require(capture.ready() && trace->reads == 0 && capture.projector()->resolution() == cv::Size(3, 2),
            "Opening acquisition started unsolicited reads or lost projector metadata");
    auto input = patterns();
    auto started = capture.captureSequence(input, trace->settling);
    input.front().setTo(99);
    started.get();
    await([&] { return capture.statistics().completed == 1; });
    capture.stopSequence().get();
    auto sequence = capture.tryTakeSequence();
    auto preview = capture.tryTake();
    require(sequence && sequence->sequence == 1 && sequence->frames.size() == 3 && preview &&
        preview->data == sequence->frames.back().data && trace->uploaded == std::vector<int>{0, 1, 2},
        "Software acquisition lost order, pattern ownership, or the shared preview copy");
    for (int index = 0; index < 3; ++index)
      require(sequence->frames[index].at<std::uint8_t>(0, 0) == 10 + index, "Captured the wrong projected pattern");
    trace->settling = 0ms;
    require(capture.ready() && capture.requestFrame(), "Stopping a sequence stopped the camera");
    std::optional<cv::Mat> frame;
    await([&] { frame = capture.tryTake(); return frame.has_value(); });
    capture.programPatterns(patterns(), [](float fraction, const char *) {
      require(fraction == 1, "Programming progress was lost");
    }).get();
    capture.programPatterns({cv::Mat3b(2, 3, cv::Vec3b(1, 2, 3))}).get();
    capture.closeCamera().get();
    require(!capture.ready() && capture.projector().has_value(), "Closing a camera closed its projector");
    capture.closeProjector().get();
    require(!capture.projector(), "Closing a projector retained stale metadata");
  }
  std::lock_guard<std::mutex> lock(trace->mutex);
  require(!trace->events.empty(), "No device calls were recorded");
  const auto owner = trace->events.front().thread;
  for (const auto &event : trace->events)
    require(event.thread == owner && event.thread != caller, "Devices were used or destroyed on different threads");
}

void testHardwareStopDuringRead() {
  auto trace = std::make_shared<Trace>();
  trace->hardware = true;
  CaptureWorker capture;
  std::promise<void> entered, release;
  auto gate = release.get_future().share();
  auto waiting = entered.get_future();
  capture.openCamera([trace, gate, &entered] {
    trace->event("camera open");
    return std::make_unique<Camera>([trace, gate, &entered] {
      require(trace->running, "Read started before the hardware projector");
      const auto read = ++trace->reads;
      if (read == 4) {
        entered.set_value();
        require(gate.wait_for(3s) == std::future_status::ready, "SDK read was not released");
      }
      return CaptureFrame{cv::Mat1b(2, 3, static_cast<std::uint8_t>(read)), read == 1,
                          static_cast<std::uint64_t>(read)};
    });
  }).get();
  capture.openProjector([trace] { return std::make_unique<Projector>(trace); }).get();
  capture.captureSequence(patterns(), 0ms, true, true).get();
  require(waiting.wait_for(3s) == std::future_status::ready, "Hardware acquisition did not run independently of GUI polling");
  const auto before = Clock::now();
  auto stopping = capture.stopSequence();
  require(Clock::now() - before < 100ms && stopping.wait_for(0ms) != std::future_status::ready,
          "Stopping acquisition blocked the caller or interrupted an SDK call");
  release.set_value();
  stopping.get();
  auto sequence = capture.tryTakeSequence();
  auto preview = capture.tryTake();
  require(sequence && sequence->frames.size() == 3 && preview &&
      preview->at<std::uint8_t>(0, 0) == 3 && sequence->frames.back().data == preview->data &&
      capture.statistics().completed == 1 && !trace->running && capture.ready(),
      "Stop published an in-flight frame, discarded a complete sequence, or closed the camera");
}

void testCancellationDuringSetupAndSettling() {
  for (const bool replacement : {false, true}) {
    auto trace = std::make_shared<Trace>();
    std::promise<void> release;
    trace->uploadGate = release.get_future().share();
    auto uploading = trace->uploading.get_future();
    CaptureWorker capture;
    capture.openCamera(cameraFactory(trace)).get();
    capture.openProjector([trace] { return std::make_unique<Projector>(trace); }).get();
    auto starting = capture.captureSequence(patterns(), 0ms, true);
    require(uploading.wait_for(3s) == std::future_status::ready, "Setup did not enter projector upload");
    auto cancelled = replacement ? capture.openCamera(cameraFactory(trace)) : capture.stopSequence();
    release.set_value();
    starting.get();
    cancelled.get();
    require(trace->reads == 0 && capture.ready() && capture.requestFrame(),
            "Cancelling queued setup left acquisition running or lost its camera");
    std::optional<cv::Mat> frame;
    await([&] { frame = capture.tryTake(); return frame.has_value(); });
  }
  auto trace = std::make_shared<Trace>();
  CaptureWorker capture;
  capture.openCamera(cameraFactory(trace)).get();
  capture.openProjector([trace] { return std::make_unique<Projector>(trace); }).get();
  capture.captureSequence(patterns(), 2s).get();
  await([&] {
    std::lock_guard<std::mutex> lock(trace->mutex);
    return trace->presented != Clock::time_point{};
  });
  const auto before = Clock::now();
  capture.stopSequence().get();
  require(Clock::now() - before < 500ms && trace->reads == 0 && !capture.tryTakeSequence() && capture.ready(),
          "Stop did not interrupt settling or emitted a partial sequence");
  require(capture.requestFrame(), "Stopped acquisition did not resume individual captures");
  std::optional<cv::Mat> preview;
  await([&] { preview = capture.tryTake(); return preview.has_value(); });
  require(Clock::now() - before < 500ms, "Preview inherited a cancelled scan's settling deadline");
}

void testSoftwareRetryAndContinuousCapture() {
  auto trace = std::make_shared<Trace>();
  CaptureWorker capture;
  capture.openCamera([trace] {
    return std::make_unique<Camera>([trace] {
      const int read = ++trace->reads;
      if (read == 2) return CaptureFrame{};
      return CaptureFrame{cv::Mat1b(2, 3, static_cast<std::uint8_t>(trace->pattern.load())),
                          false, static_cast<std::uint64_t>(read * 2)};
    });
  }).get();
  capture.openProjector([trace] { return std::make_unique<Projector>(trace); }).get();
  capture.captureSequence(patterns(), 0ms, true).get();
  rejects([&] { capture.blankProjector().get(); });
  await([&] { return capture.statistics().completed >= 3; });
  capture.stopSequence().get();
  auto sequence = capture.tryTakeSequence();
  require(sequence && sequence->sequence >= 3 && capture.statistics().dropped >= 2,
          "Continuous capture queued stale sequences or lost replacement statistics");
  for (int index = 0; index < 3; ++index)
    require(sequence->frames[index].at<std::uint8_t>(0, 0) == index,
            "Software retry advanced the pattern or treated discarded camera buffers as missing patterns");
}

void testHandoverAndErrors() {
  CaptureWorker capture;
  auto trace = std::make_shared<Trace>();
  std::promise<void> entered, release;
  auto gate = release.get_future().share();
  auto reading = entered.get_future();
  std::atomic<bool> closed{false};
  const auto caller = std::this_thread::get_id();
  std::thread::id oldOwner, newOwner, releaseOwner;
  capture.openCamera([&] {
    oldOwner = std::this_thread::get_id();
    return std::make_unique<Camera>([&] {
      entered.set_value();
      require(gate.wait_for(3s) == std::future_status::ready, "Old camera SDK call did not finish");
      return CaptureFrame{cv::Mat1b(2, 3, std::uint8_t(1))};
    }, [&] { releaseOwner = std::this_thread::get_id(); closed = true; });
  }).get();
  capture.requestFrame();
  require(reading.wait_for(3s) == std::future_status::ready, "Old camera never read");
  auto opening = capture.openCamera([&] {
    newOwner = std::this_thread::get_id();
    require(closed, "Replacement camera opened before the old camera was released");
    return std::make_unique<Camera>([] { return CaptureFrame{cv::Mat1b(2, 3, std::uint8_t(2))}; });
  });
  require(!capture.ready() && opening.wait_for(0ms) != std::future_status::ready, "Camera replacement blocked or reported early readiness");
  release.set_value();
  opening.get();
  require(oldOwner == releaseOwner && oldOwner == newOwner && newOwner != caller && !capture.tryTake(),
          "Camera handover changed ownership thread or retained an old frame");
  require(capture.requestFrame(), "Replacement camera did not accept a frame request");
  std::optional<cv::Mat> frame;
  await([&] { frame = capture.tryTake(); return frame.has_value(); });
  require(frame->at<std::uint8_t>(0, 0) == 2, "Replacement camera returned an old frame");
  rejects([&] { capture.openCamera([]() -> std::unique_ptr<CaptureWorker::Camera> {
    throw std::runtime_error("Open failed");
  }).get(); });
  require(!capture.ready(), "Failed camera initialization reported readiness");
  capture.openCamera(cameraFactory(trace)).get();
  capture.openProjector([trace] { return std::make_unique<Projector>(trace); }).get();
  rejects([&] { capture.captureSequence({}, 0ms); });
  rejects([&] { capture.captureSequence(patterns(), -1ms); });
  rejects([&] { capture.captureSequence(patterns(), 0ms, false, true).get(); });
  trace->failPresentation = true;
  capture.captureSequence(patterns(), 0ms).get();
  bool failed = false;
  await([&] {
    try { capture.tryTakeSequence(); }
    catch (const std::runtime_error &error) { failed = std::string(error.what()) == "Presentation failed"; }
    return failed;
  });
  require(!capture.ready() && !capture.requestFrame(), "Failed acquisition kept capturing");
}

void testDesktopAcknowledgement() {
  auto trace = std::make_shared<Trace>();
  trace->settling = 10ms;
  const auto gui = std::this_thread::get_id();
  slstudio::gui::ProjectorPresentationController desktop(std::make_unique<Projector>(trace));
  CaptureWorker capture;
  capture.openCamera(cameraFactory(trace)).get();
  capture.openProjector(desktop.factory()).get();
  auto started = capture.captureSequence(patterns(), trace->settling);
  std::this_thread::sleep_for(20ms);
  require(trace->reads == 0 && started.wait_for(0ms) != std::future_status::ready,
          "Capture ignored the GUI upload acknowledgement");
  std::optional<CapturedSequence> sequence;
  await([&] {
    desktop.process();
    sequence = capture.tryTakeSequence();
    return sequence.has_value();
  });
  started.get();
  for (int index = 0; index < 3; ++index)
    require(sequence->frames[index].at<std::uint8_t>(0, 0) == 10 + index, "Desktop capture skipped presentation acknowledgement");
  auto closed = capture.closeProjector();
  await([&] { desktop.process(); return closed.wait_for(0ms) == std::future_status::ready; });
  closed.get();
  capture.closeCamera().get();
  std::lock_guard<std::mutex> lock(trace->mutex);
  for (const auto &event : trace->events) {
    const bool camera = event.operation == "read" || event.operation == "camera open" || event.operation == "camera close";
    require(camera ? event.thread != gui : event.thread == gui, "Desktop rendering or camera I/O used the wrong thread");
  }
}

void testDesktopCancellation() {
  auto trace = std::make_shared<Trace>();
  slstudio::gui::ProjectorPresentationController desktop(std::make_unique<Projector>(trace));
  CaptureWorker capture;
  capture.openCamera(cameraFactory(trace)).get();
  capture.openProjector(desktop.factory()).get();
  auto started = capture.captureSequence(patterns(), 0ms);
  desktop.cancel();
  rejects([&] { started.get(); });
  rejects([&] { capture.closeProjector().get(); });
}

void testScanSession() {
  ScanSession scan;
  CaptureWorker capture([] { return cv::Mat1b(2, 3, std::uint8_t(7)); });
  rejects([&] { scan.begin(0, false, false, capture); });
  rejects([&] { scan.begin(3, true, true, capture); });
  scan.begin(3, true, false, capture);
  require(scan.active() && scan.capture() == &capture && scan.index() == 0 &&
      scan.count() == 3 && scan.geometricCalibration(),
      "GUI scan did not retain its capture mode and worker binding");
  rejects([&] { scan.begin(3, false, false, capture); });
  scan.stop();
  scan.stop();
  require(!scan.active() && !scan.continuous() && !scan.capture() && scan.index() == -1 &&
      scan.geometricCalibration() && scan.count() == 3,
      "Scan stop did not clear transient state while preserving settings");
  scan.begin(2, false, true, capture);
  require(scan.continuous() && !scan.geometricCalibration(), "Scan mode did not change");
  scan.stop();
  require(!scan.continuous(), "Stopping a continuous scan left restart enabled");
}
}

int main() {
  try {
    testScanSession();
    testSoftwareOwnership();
    testHardwareStopDuringRead();
    testCancellationDuringSetupAndSettling();
    testSoftwareRetryAndContinuousCapture();
    testHandoverAndErrors();
    testDesktopAcknowledgement();
    testDesktopCancellation();
    std::cout << "Scan lifecycle, acquisition ownership, ordering, cancellation and desktop acknowledgements passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
