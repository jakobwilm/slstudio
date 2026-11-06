#include "CameraPreviewTexture.h"
#include <opencv2/imgproc.hpp>

namespace slstudio::gui {
void CameraPreviewTexture::update(CameraPreviewBuffer &preview) {
  if (preview.empty() || !preview.dirty()) return;
  if (!texture_) glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  cv::Mat rgb;
  cv::cvtColor(preview.image(), rgb, cv::COLOR_GRAY2RGB);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, rgb.cols, rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);
  glBindTexture(GL_TEXTURE_2D, 0);
  preview.markUploaded();
}

void CameraPreviewTexture::release() {
  if (texture_) glDeleteTextures(1, &texture_);
  texture_ = 0;
}
}
