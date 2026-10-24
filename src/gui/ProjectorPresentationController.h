#pragma once

#include "CaptureWorker.h"

namespace slstudio::gui {
// Owns desktop rendering on the GUI thread; the acquisition thread waits for each acknowledgement.
class ProjectorPresentationController {
public:
  explicit ProjectorPresentationController(std::unique_ptr<slstudio::Projector> output);
  ~ProjectorPresentationController();
  CaptureWorker::ProjectorFactory factory() const;
  void process();
  void cancel();

private:
  struct State;
  class Proxy;
  std::shared_ptr<State> state;
  std::unique_ptr<slstudio::Projector> output;
};
}
