#pragma once

#include <functional>
#include <memory>

namespace slstudio::gui {
// Executes brief window lifecycle requests; projector ownership and rendering stay with acquisition.
class ProjectorPresentationController {
public:
  ProjectorPresentationController();
  ~ProjectorPresentationController();
  ProjectorPresentationController(const ProjectorPresentationController &) = delete;
  ProjectorPresentationController &operator=(const ProjectorPresentationController &) = delete;
  std::function<void(std::function<void()>)> dispatcher() const;
  void process();
  void cancel();

private:
  struct State;
  std::shared_ptr<State> state;
};
}
