#include "CaptureWorker.h"

#include <condition_variable>
#include <cstdio>
#include <deque>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace {
class ReaderCamera final : public CaptureWorker::Camera {
public:
  explicit ReaderCamera(CaptureWorker::StreamReader source) : reader(std::move(source)) {}
  CaptureFrame read() override { return reader(); }
private:
  CaptureWorker::StreamReader reader;
};

std::vector<cv::Mat> copyPatterns(const std::vector<cv::Mat> &patterns) {
  if (patterns.empty()) throw std::invalid_argument("Capture requires at least one pattern");
  std::vector<cv::Mat> result;
  for (const auto &pattern : patterns) {
    if (pattern.empty() || pattern.dims != 2 || (pattern.type() != CV_8UC1 && pattern.type() != CV_8UC3) ||
        pattern.size() != patterns.front().size())
      throw std::invalid_argument("Projector patterns must be byte images with matching dimensions");
    result.push_back(pattern.clone());
  }
  return result;
}
}

struct CaptureWorker::Impl {
  using Clock = std::chrono::steady_clock;
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::deque<std::packaged_task<void()>> commands;
  std::unique_ptr<Camera> camera;
  std::unique_ptr<slstudio::Projector> output;
  std::optional<ProjectorInfo> outputInfo;
  std::optional<cv::Mat> completed;
  std::optional<CapturedSequence> completedSequence;
  SequenceCaptureStatistics counts;
  std::exception_ptr error;
  bool stopped = false, requested = false, working = false, initialized = false, failed = false;
  bool capturing = false;
  std::uint64_t generation = 0;
  std::size_t index = 0;
  std::size_t legacyCount = 0;
  bool retryEmptyFrames = true;
  std::thread thread;

  // Only the acquisition thread accesses devices, patterns and partial sequences.
  std::vector<cv::Mat> frames;
  std::optional<std::uint64_t> previousFrame;
  std::size_t patternCount = 0;
  bool continuous = false, hardware = false, legacy = false, presented = false;
  bool projectorCleaned = true;
  std::chrono::milliseconds settling{};
  Clock::time_point after{};

  Impl() : thread([this] { run(); }) {}
  ~Impl() {
    cancel();
    thread.join();
  }

  std::future<void> enqueue(std::function<void()> action) {
    std::lock_guard<std::mutex> lock(mutex);
    if (stopped) throw std::logic_error("Capture worker stopped");
    std::packaged_task<void()> command(std::move(action));
    auto result = command.get_future();
    commands.push_back(std::move(command));
    changed.notify_one();
    return result;
  }

  void interrupt(bool cameraChange = false) {
    std::lock_guard<std::mutex> lock(mutex);
    ++generation;
    capturing = requested = false;
    if (cameraChange) {
      initialized = false;
      completed.reset();
    }
    changed.notify_all();
  }

  void cancel() {
    std::lock_guard<std::mutex> lock(mutex);
    stopped = true;
    capturing = requested = false;
    completed.reset();
    completedSequence.reset();
    changed.notify_all();
  }

  void stopProjector() {
    if (!output) return;
    projectorCleaned = true;
    std::string failure;
    try { output->stopSequence(); }
    catch (const std::exception &e) { failure = e.what(); }
    try { output->blank(); }
    catch (const std::exception &e) {
      if (!failure.empty()) failure += "; ";
      failure += e.what();
    }
    if (!failure.empty()) throw std::runtime_error(failure);
  }

  void resetSequence() {
    frames.clear();
    previousFrame.reset();
    patternCount = 0;
    presented = false;
    after = {};
    std::lock_guard<std::mutex> lock(mutex);
    index = 0;
  }

  void requireIdleProjector() const {
    std::lock_guard<std::mutex> lock(mutex);
    if (capturing || (patternCount && !projectorCleaned))
      throw std::logic_error("Stop sequence capture before changing the projector");
  }

  void run() {
    for (;;) {
      std::packaged_task<void()> command;
      std::uint64_t current;
      bool sequence;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [&] { return stopped || !commands.empty() || capturing || requested; });
        if (stopped) break;
        if (!commands.empty()) {
          command = std::move(commands.front());
          commands.pop_front();
        }
        current = generation;
        sequence = capturing;
        if (!command.valid()) {
          requested = false;
          working = true;
        }
      }
      if (command.valid()) {
        command();
        continue;
      }
      try {
        if (!camera) throw std::logic_error("Open a camera before capturing");
        if (sequence && !legacy && !presented) {
          if (hardware) {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopped || generation != current || !commands.empty()) {
              working = false;
              continue;
            }
          }
          projectorCleaned = false;
          if (hardware) output->startSequence();
          else output->present(frames.size());
          presented = true;
          after = Clock::now() + (hardware ? std::chrono::milliseconds(0) : settling);
        }
        {
          std::unique_lock<std::mutex> lock(mutex);
          changed.wait_until(lock, after, [&] {
            return stopped || generation != current || !commands.empty();
          });
          if (stopped || generation != current || !commands.empty()) {
            if (!sequence && generation == current) requested = true;
            working = false;
            continue;
          }
        }
        auto frame = camera->read();
        if (frame.timedOut) {
          std::lock_guard<std::mutex> lock(mutex);
          working = false;
          if (!stopped && generation == current && !sequence) requested = true;
          continue;
        }
        if (sequence) {
          if (frame.frameNumber && *frame.frameNumber > frame.frameNumberMaximum)
            throw std::runtime_error("Camera frame counter exceeds its declared range");
          const bool gap = frame.frameNumber && previousFrame &&
              *frame.frameNumber != (*previousFrame == frame.frameNumberMaximum ?
                                    0 : *previousFrame + std::uint64_t(1));
          previousFrame = frame.frameNumber;
          const bool reset = (legacy || hardware) && (frame.sequenceStart || frame.image.empty() || gap);
          const bool droppedPartial = reset && !frames.empty();
          if (reset) frames.clear();
          if (!frame.image.empty()) {
            if (frame.image.dims != 2 || frame.image.type() != CV_8UC1)
              throw std::runtime_error("Sequence camera must return two-dimensional grayscale byte images");
            if (!frames.empty() && frame.image.size() != frames.front().size())
              throw std::runtime_error("Sequence camera frame dimensions changed");
            frame.image = frame.image.clone();
            frames.push_back(frame.image);
          }
          const bool complete = frames.size() == patternCount;
          if (!legacy && complete && (!continuous || !hardware)) stopProjector();
          if (!legacy && complete && !continuous) camera->setSequenceMode(false);
          std::lock_guard<std::mutex> lock(mutex);
          working = false;
          if (stopped || generation != current) continue;
          if (droppedPartial) ++counts.dropped;
          const bool received = !frame.image.empty();
          if (received) completed = std::move(frame.image);
          index = frames.size();
          if (complete) {
            if (completedSequence) ++counts.dropped;
            completedSequence = CapturedSequence{++counts.completed, std::move(frames)};
            frames.clear();
            index = 0;
            if (!continuous) capturing = false;
          }
          if (!legacy && !hardware && received) presented = false;
          continue;
        }
        if (frame.image.empty() && !retryEmptyFrames)
          throw std::runtime_error("Camera returned an empty frame");
        if (!frame.image.empty()) frame.image = frame.image.clone();
        std::lock_guard<std::mutex> lock(mutex);
        working = false;
        if (stopped || generation != current) continue;
        if (frame.image.empty()) requested = true;
        else completed = std::move(frame.image);
      } catch (...) {
        auto failure = std::current_exception();
        {
          std::lock_guard<std::mutex> lock(mutex);
          working = false;
          if (stopped || generation != current) continue;
          capturing = requested = false;
          completed.reset();
          completedSequence.reset();
        }
        try { stopProjector(); }
        catch (const std::exception &cleanup) {
          std::fprintf(stderr, "SLStudio acquisition cleanup: %s\n", cleanup.what());
          try { std::rethrow_exception(failure); }
          catch (const std::exception &primary) {
            failure = std::make_exception_ptr(std::runtime_error(
                std::string(primary.what()) + "; projector cleanup: " + cleanup.what()));
          }
          output.reset();
          std::lock_guard<std::mutex> lock(mutex);
          outputInfo.reset();
        }
        resetSequence();
        std::lock_guard<std::mutex> lock(mutex);
        error = failure;
        initialized = false;
        failed = true;
      }
    }
    try { stopProjector(); }
    catch (const std::exception &e) { std::fprintf(stderr, "SLStudio projector shutdown: %s\n", e.what()); }
    output.reset();
    camera.reset();
  }
};

CaptureWorker::CaptureWorker() : impl(std::make_unique<Impl>()) {}

CaptureWorker::CaptureWorker(Reader reader, Initializer initialize, bool retryEmptyFrames)
    : CaptureWorker() {
  if (!reader) throw std::invalid_argument("Capture reader is required");
  impl->retryEmptyFrames = retryEmptyFrames;
  auto initialized = openCamera([read = std::move(reader), initialize = std::move(initialize)] {
    if (initialize) initialize();
    return std::make_unique<ReaderCamera>([read] { return CaptureFrame{read()}; });
  });
  impl->enqueue([state = impl.get(), initialized = std::make_shared<std::future<void>>(std::move(initialized))] {
    try { initialized->get(); }
    catch (...) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->error = std::current_exception();
      state->failed = true;
    }
  });
}

CaptureWorker::CaptureWorker(std::size_t count, StreamReader reader, Initializer initialize)
    : CaptureWorker() {
  if (!count || !reader) throw std::invalid_argument("Sequence capture requires a positive pattern count and reader");
  impl->legacyCount = count;
  auto initialized = openCamera([read = std::move(reader), initialize = std::move(initialize)] {
    if (initialize) initialize();
    return std::make_unique<ReaderCamera>(read);
  });
  impl->enqueue([state = impl.get(), initialized = std::make_shared<std::future<void>>(std::move(initialized))] {
    try { initialized->get(); }
    catch (...) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->error = std::current_exception();
      state->failed = true;
    }
  });
}
CaptureWorker::~CaptureWorker() = default;

std::future<void> CaptureWorker::openCamera(CameraFactory factory) {
  if (!factory) throw std::invalid_argument("Camera factory is required");
  impl->interrupt(true);
  return impl->enqueue([state = impl.get(), factory = std::move(factory)] {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->capturing = false;
    }
    std::exception_ptr failure;
    try { if (state->patternCount && !state->projectorCleaned) state->stopProjector(); }
    catch (...) { failure = std::current_exception(); }
    state->resetSequence();
    state->camera.reset();
    if (failure) std::rethrow_exception(failure);
    auto camera = factory();
    if (!camera) throw std::runtime_error("Camera factory returned no camera");
    state->camera = std::move(camera);
    std::lock_guard<std::mutex> lock(state->mutex);
    state->initialized = true;
    state->failed = false;
  });
}

std::future<void> CaptureWorker::closeCamera() {
  impl->interrupt(true);
  return impl->enqueue([state = impl.get()] {
    std::exception_ptr failure;
    try { if (state->patternCount && !state->projectorCleaned) state->stopProjector(); }
    catch (...) { failure = std::current_exception(); }
    state->resetSequence();
    state->camera.reset();
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->initialized = state->capturing = false;
    }
    if (failure) std::rethrow_exception(failure);
  });
}

std::future<void> CaptureWorker::openProjector(ProjectorFactory factory) {
  if (!factory) throw std::invalid_argument("Projector factory is required");
  impl->interrupt();
  return impl->enqueue([state = impl.get(), factory = std::move(factory)] {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->capturing = false;
    }
    state->resetSequence();
    std::exception_ptr failure;
    try { state->stopProjector(); }
    catch (...) { failure = std::current_exception(); }
    state->output.reset();
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->outputInfo.reset();
    }
    if (failure) std::rethrow_exception(failure);
    if (state->camera) state->camera->setSequenceMode(false);
    auto output = factory();
    if (!output) throw std::runtime_error("Projector factory returned no projector");
    ProjectorInfo info{output->resolution(), output->geometry(), output->capabilities()};
    state->output = std::move(output);
    state->projectorCleaned = false;
    std::lock_guard<std::mutex> lock(state->mutex);
    state->outputInfo = info;
  });
}

std::future<void> CaptureWorker::closeProjector() {
  impl->interrupt();
  return impl->enqueue([state = impl.get()] {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->capturing = false;
    }
    std::exception_ptr failure;
    try { state->stopProjector(); }
    catch (...) { failure = std::current_exception(); }
    state->resetSequence();
    state->output.reset();
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->outputInfo.reset();
    }
    if (failure) std::rethrow_exception(failure);
    if (state->camera) state->camera->setSequenceMode(false);
  });
}

std::optional<CaptureWorker::ProjectorInfo> CaptureWorker::projector() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->outputInfo;
}

std::future<void> CaptureWorker::blankProjector(bool white) {
  return impl->enqueue([state = impl.get(), white] {
    state->requireIdleProjector();
    if (!state->output) throw std::logic_error("Open a projector first");
    state->projectorCleaned = false;
    state->output->blank(white);
  });
}

std::future<void> CaptureWorker::showSolid(std::uint8_t level, std::chrono::milliseconds settling) {
  if (settling.count() < 0) throw std::invalid_argument("Settling time must be nonnegative");
  return impl->enqueue([state = impl.get(), level, settling] {
    state->requireIdleProjector();
    if (!state->output) throw std::logic_error("Open a projector first");
    state->projectorCleaned = false;
    state->output->stopSequence();
    state->output->showSolid(level);
    state->after = Impl::Clock::now() + settling;
  });
}

std::future<void> CaptureWorker::programPatterns(std::vector<cv::Mat> patterns,
    slstudio::ProjectorProgrammingProgress progress) {
  return impl->enqueue([state = impl.get(), patterns = copyPatterns(patterns), progress = std::move(progress)] {
    state->requireIdleProjector();
    if (!state->output) throw std::logic_error("Open a projector first");
    state->output->programPatterns(patterns, progress);
  });
}

std::future<void> CaptureWorker::captureSequence(std::vector<cv::Mat> patterns,
    std::chrono::milliseconds settling, bool continuous, bool hardware, slstudio::ProjectorSequenceSettings timing) {
  auto copies = copyPatterns(patterns);
  if (settling.count() < 0) throw std::invalid_argument("Settling time must be nonnegative");
  if (hardware) slstudio::validateProjectorSequence(timing);
  impl->interrupt();
  return impl->enqueue([state = impl.get(), patterns = std::move(copies), settling, continuous, hardware, timing]() mutable {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->capturing = false;
    }
    if (state->patternCount && !state->projectorCleaned) state->stopProjector();
    state->resetSequence();
    if (!state->camera || !state->output) throw std::logic_error("Open a camera and projector before capturing");
    if (hardware && (!state->output->capabilities().hardwareSequence || !state->output->capabilities().triggerOutput))
      throw std::invalid_argument("Projector cannot trigger a hardware sequence");
    state->projectorCleaned = false;
    state->output->setPatterns(patterns);
    if (hardware) {
      auto settings = timing;
      settings.repeat = continuous;
      state->output->configureSequence(settings);
    }
    state->camera->setSequenceMode(true);
    state->patternCount = patterns.size();
    patterns.clear();
    state->settling = hardware ? std::chrono::milliseconds(0) : settling;
    state->continuous = continuous;
    state->hardware = hardware;
    state->legacy = false;
    state->presented = false;
    state->after = {};
    std::lock_guard<std::mutex> lock(state->mutex);
    state->counts = {};
    state->completed.reset();
    state->completedSequence.reset();
    state->capturing = true;
  });
}

std::future<void> CaptureWorker::stopSequence() {
  impl->interrupt();
  return impl->enqueue([state = impl.get()] {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->capturing = false;
    }
    state->resetSequence();
    try { if (!state->projectorCleaned) state->stopProjector(); }
    catch (...) {
      state->output.reset();
      std::lock_guard<std::mutex> lock(state->mutex);
      state->outputInfo.reset();
      throw;
    }
    if (state->camera) state->camera->setSequenceMode(false);
  });
}

std::size_t CaptureWorker::sequenceIndex() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->index;
}

bool CaptureWorker::framePending() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->requested || impl->working || impl->completed.has_value();
}

bool CaptureWorker::requestFrame() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->legacyCount || impl->stopped || impl->failed || impl->error || impl->capturing ||
      impl->requested || impl->working || impl->completed) return false;
  impl->requested = true;
  impl->changed.notify_one();
  return true;
}

bool CaptureWorker::start() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (!impl->legacyCount || !impl->initialized || impl->stopped || impl->failed || impl->capturing) return false;
  impl->patternCount = impl->legacyCount;
  impl->legacy = impl->continuous = impl->capturing = true;
  impl->changed.notify_one();
  return true;
}

std::optional<cv::Mat> CaptureWorker::tryTake() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->error) std::rethrow_exception(std::exchange(impl->error, std::exception_ptr{}));
  return std::exchange(impl->completed, std::nullopt);
}

std::optional<CapturedSequence> CaptureWorker::tryTakeSequence() {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->error) std::rethrow_exception(std::exchange(impl->error, std::exception_ptr{}));
  return std::exchange(impl->completedSequence, std::nullopt);
}

SequenceCaptureStatistics CaptureWorker::statistics() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->counts;
}

void CaptureWorker::cancel() { impl->cancel(); }
bool CaptureWorker::ready() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  return impl->initialized && !impl->stopped && !impl->error;
}
