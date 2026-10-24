#pragma once

#include <GL/glew.h>
#include <opencv2/core.hpp>

namespace slstudio::gui {
class CameraPreviewTexture {
public:
  GLuint texture() const { return texture_; }
  void update(const cv::Mat &preview, bool &dirty);
  void release();

private:
  GLuint texture_ = 0;
};
}
