#pragma once

#include "CameraPreviewBuffer.h"
#include <GL/glew.h>

namespace slstudio::gui {
class CameraPreviewTexture {
public:
  GLuint texture() const { return texture_; }
  void update(CameraPreviewBuffer &preview);
  void release();

private:
  GLuint texture_ = 0;
};
}
