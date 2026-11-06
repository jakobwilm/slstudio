#include "ProjectorPresentationController.h"
#include <deque>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace slstudio::gui {
struct ProjectorPresentationController::State {
  std::mutex mutex;
  std::deque<std::packaged_task<void()>> pending;
  const std::thread::id mainThread = std::this_thread::get_id();
  bool stopped = false;
};

ProjectorPresentationController::ProjectorPresentationController()
    : state(std::make_shared<State>()) {}

ProjectorPresentationController::~ProjectorPresentationController() { cancel(); }

std::function<void(std::function<void()>)> ProjectorPresentationController::dispatcher() const {
  return [shared = state](std::function<void()> action) {
    std::unique_lock<std::mutex> lock(shared->mutex);
    if (shared->stopped) throw std::runtime_error("Main-thread projector dispatch stopped");
    if (std::this_thread::get_id() == shared->mainThread) {
      lock.unlock();
      action();
      return;
    }
    std::packaged_task<void()> task(std::move(action));
    auto completed = task.get_future();
    shared->pending.push_back(std::move(task));
    lock.unlock();
    completed.get();
  };
}

void ProjectorPresentationController::process() {
  if (std::this_thread::get_id() != state->mainThread)
    throw std::logic_error("Projector window dispatch requires the main thread");
  std::deque<std::packaged_task<void()>> pending;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    pending.swap(state->pending);
  }
  for (auto &task : pending) task();
}

void ProjectorPresentationController::cancel() {
  std::lock_guard<std::mutex> lock(state->mutex);
  state->stopped = true;
  state->pending.clear();
}
}
