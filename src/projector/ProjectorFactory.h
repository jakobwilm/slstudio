#pragma once

#include "Projector.h"
#include <memory>
#include <string>

namespace slstudio {

enum class OutputBackend {
  Desktop, WaylandLease, XRandRLease, LightCrafter3000, LightCrafter4500, DisplayCore, CoreGraphics
};

struct ProjectorOpenSettings {
  bool allowDesktop = false;
  std::chrono::milliseconds timeout{5000};
  std::string flashBackupDirectory;
  std::string networkAddress = "192.168.1.100";
};

struct ProjectorOutputInfo {
  std::string id;
  std::string name;
  OutputBackend backend = OutputBackend::Desktop;
  bool exclusive = false;

  std::string selectionId() const {
    return std::to_string(static_cast<int>(backend)) + ":" + id;
  }
};

bool lightCrafterAvailable(OutputBackend backend);
std::vector<ProjectorOutputInfo> projectorOutputs(bool allowDesktop = false);
std::unique_ptr<Projector> openProjector(const ProjectorOutputInfo &output,
                                       const ProjectorOpenSettings &settings = {});

}
