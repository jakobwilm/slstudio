#include "ProjectorPresentationController.h"
#include <condition_variable>
#include <mutex>
#include <stdexcept>

namespace slstudio::gui {
struct ProjectorPresentationController::State {
  std::mutex mutex;
  std::condition_variable changed;
  std::function<void(slstudio::Projector &)> pending;
  std::exception_ptr error;
  bool stopped = false, done = false;
  CaptureWorker::ProjectorInfo info;

  void dispatch(std::function<void(slstudio::Projector &)> action) {
    std::unique_lock<std::mutex> lock(mutex);
    if (stopped) throw std::runtime_error("Desktop projector presentation stopped");
    pending = std::move(action);
    done = false;
    error = {};
    changed.wait(lock, [&] { return stopped || done; });
    if (error) std::rethrow_exception(error);
    if (stopped) throw std::runtime_error("Desktop projector presentation stopped");
  }
};

class ProjectorPresentationController::Proxy final : public slstudio::Projector {
public:
  explicit Proxy(std::shared_ptr<State> shared) : state(std::move(shared)) {}
  cv::Size resolution() const override { return state->info.size; }
  slstudio::ProjectorGeometry geometry() const override { return state->info.outputGeometry; }
  slstudio::ProjectorCapabilities capabilities() const override { return state->info.features; }
  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    state->dispatch([patterns](auto &output) { output.setPatterns(patterns); });
  }
  void present(std::size_t index) override {
    state->dispatch([index](auto &output) { output.present(index); });
  }
  void blank(bool white) override {
    state->dispatch([white](auto &output) { output.blank(white); });
  }
  void showSolid(std::uint8_t level) override {
    state->dispatch([level](auto &output) { output.showSolid(level); });
  }
private:
  std::shared_ptr<State> state;
};

ProjectorPresentationController::ProjectorPresentationController(std::unique_ptr<slstudio::Projector> device)
    : state(std::make_shared<State>()), output(std::move(device)) {
  if (!output) throw std::invalid_argument("Desktop presentation requires a projector");
  state->info = {output->resolution(), output->geometry(), output->capabilities()};
}

ProjectorPresentationController::~ProjectorPresentationController() { cancel(); }

CaptureWorker::ProjectorFactory ProjectorPresentationController::factory() const {
  return [shared = state] { return std::make_unique<Proxy>(shared); };
}

void ProjectorPresentationController::process() {
  std::function<void(slstudio::Projector &)> action;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->stopped || !state->pending) return;
    action = std::move(state->pending);
    state->pending = {};
  }
  std::exception_ptr failure;
  try { action(*output); }
  catch (...) { failure = std::current_exception(); }
  std::lock_guard<std::mutex> lock(state->mutex);
  state->error = failure;
  state->done = true;
  state->changed.notify_one();
}

void ProjectorPresentationController::cancel() {
  std::lock_guard<std::mutex> lock(state->mutex);
  state->stopped = true;
  state->pending = {};
  state->changed.notify_all();
}
}
