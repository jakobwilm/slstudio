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
enum class FilePickerMode { GeometricCalibration, LoadSequence, SaveSequence };

struct FilePickerState {
  FilePickerMode mode = FilePickerMode::GeometricCalibration;
  bool requested = false, open = false;
  std::filesystem::path directory;
  std::optional<std::filesystem::path> selectedDirectory;
  std::optional<std::filesystem::path> overwritePath;
  std::vector<std::filesystem::path> directories, files;
  std::array<char, 1024> path{}, filename{};
  std::string error;
};

class FilePickerDialog {
public:
  FilePickerState state;

  void draw(const FrameSequence *lastSequence, bool processing, bool scanActive, std::string &status,
      const std::function<void(const std::filesystem::path &)> &browse,
      const std::function<void(const std::filesystem::path &)> &loadCalibration,
      const std::function<void(const std::filesystem::path &)> &loadSequence,
      const std::function<void(const std::filesystem::path &, const FrameSequence &)> &saveSequence);
};

}
