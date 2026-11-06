#pragma once

#include "ProjectorFactory.h"

namespace slstudio {

std::vector<ProjectorOutputInfo> gnomeLeaseCandidates();

class GnomeDisplayReservation {
public:
  GnomeDisplayReservation(const std::string &connector, std::chrono::milliseconds timeout);
  ~GnomeDisplayReservation();

private:
  struct State;
  std::unique_ptr<State> state;
};

}