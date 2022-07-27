#pragma once

#include "Projector.h"
#include <iostream>
#include <memory>
#include <vector>

class ProjectorFactory {
public:
  static std::unique_ptr<Projector> NewProjector(const int screenNum);
};
