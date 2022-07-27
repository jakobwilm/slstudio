#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include "Camera.h"
#include <iostream>
#include <memory>
#include <vector>

// Camera factory methods and abstract base class for camera implementations
class CameraFactory {

public:
  // Static "camera factory" methods
  static std::vector<std::vector<CameraInfo>> GetInterfaceCameraList();
  static std::unique_ptr<Camera> NewCamera(int interfaceNum,
                                           int camNum,
                                           CameraTriggerMode triggerMode);
};

#endif
