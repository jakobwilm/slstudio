#pragma once

#include "ApplicationSettings.h"
#include "CloudRenderer.h"
#include "FrameSequence.h"
#include "GeometricCalibration.h"
#include "Projector.h"
#include "CaptureWorker.h"
#include "ScanSession.h"
#include <GL/glew.h>
#include <array>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace slstudio::gui {
class FrameSequenceWindow {
public:
  bool open = false;

  void draw(const FrameSequence *sequence,
      const std::function<void(const std::function<void()> &)> &attempt, const std::string &title);
  void markDirty() { texturesDirty_ = true; }
  void markClean() { texturesDirty_ = false; }
  bool texturesDirty() const { return texturesDirty_; }
  const std::vector<GLuint> &textures() const { return textures_; }
  void updateTextures(const FrameSequence &sequence);
  void release();

private:
  std::vector<GLuint> textures_;
  bool texturesDirty_ = false;
};

}
