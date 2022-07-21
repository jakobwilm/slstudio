#include "ProjectorFactory.h"

#include "ProjectorLC3000.h"
#include "ProjectorLC4500.h"
#include "ProjectorOpenGL.h"
#include "ProjectorVirtual.h"

std::unique_ptr<Projector> ProjectorFactory::NewProjector(const int screenNum) {

  if (screenNum >= 0) {
    return std::make_unique<ProjectorOpenGL>(screenNum);
  } else if (screenNum == -1) {
    return std::make_unique<ProjectorVirtual>(screenNum);
  } else if (screenNum == -2) {
    return std::make_unique<ProjectorLC3000>(0);
  } else if (screenNum == -3) {
    return std::make_unique<ProjectorLC4500>(0);
  }

  return nullptr;
}
