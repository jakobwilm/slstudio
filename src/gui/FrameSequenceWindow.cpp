#include "FrameSequenceWindow.h"
#include "GuiLayout.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>
#include <stdexcept>

namespace slstudio::gui {
void FrameSequenceWindow::draw(const FrameSequence *sequence,
    const std::function<void(const std::function<void()> &)> &attempt, const std::string &title) {
  if (!open) return;
  const auto *viewport = ImGui::GetMainViewport();
  const auto frameSize = sequence ? sequence->frames.front().size() : cv::Size(640, 240);
  const auto frameCount = sequence ? sequence->frames.size() : 1;
  struct SizeConstraint {
    float aspect, titleHeight, minimumWidth, maximumWidth;
  } constraint;
  constraint.aspect = static_cast<float>(frameSize.width) * frameCount / frameSize.height;
  constraint.titleHeight = ImGui::GetFrameHeight();
  constraint.maximumWidth = std::max(1.0f, std::min(viewport->WorkSize.x,
      (viewport->WorkSize.y - constraint.titleHeight) * constraint.aspect));
  constraint.minimumWidth = std::min(uiPixels(240), constraint.maximumWidth);
  const float width = std::min(uiPixels(400), constraint.maximumWidth);
  ImGui::SetNextWindowSize(ImVec2(width, width / constraint.aspect + constraint.titleHeight), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(
      ImVec2(constraint.minimumWidth, constraint.minimumWidth / constraint.aspect + constraint.titleHeight),
      ImVec2(constraint.maximumWidth, constraint.maximumWidth / constraint.aspect + constraint.titleHeight),
      [](ImGuiSizeCallbackData *data) {
        const auto &size = *static_cast<const SizeConstraint *>(data->UserData);
        const float horizontalChange = std::abs(data->DesiredSize.x - data->CurrentSize.x);
        const float verticalChange = std::abs(data->DesiredSize.y - data->CurrentSize.y) * size.aspect;
        const auto cursor = ImGui::GetMouseCursor();
        const bool corner = cursor == ImGuiMouseCursor_ResizeNWSE || cursor == ImGuiMouseCursor_ResizeNESW;
        // Corner targets are absolute; choosing an axis from the previous constrained size oscillates.
        const float desiredWidth = corner ?
            (size.aspect * size.aspect * data->DesiredSize.x +
                size.aspect * (data->DesiredSize.y - size.titleHeight)) / (size.aspect * size.aspect + 1) :
            horizontalChange >= verticalChange ? data->DesiredSize.x :
                (data->DesiredSize.y - size.titleHeight) * size.aspect;
        data->DesiredSize.x = std::clamp(desiredWidth, size.minimumWidth, size.maximumWidth);
        data->DesiredSize.y = data->DesiredSize.x / size.aspect + size.titleHeight;
      }, &constraint);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
  const bool visible = ImGui::Begin(title.c_str(), &open,
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleVar(2);
  if (!visible) { ImGui::End(); return; }
  ImGui::SetCursorPos(ImVec2(0, constraint.titleHeight));
  if (sequence) {
    attempt([&] { updateTextures(*sequence); });
    const auto available = ImGui::GetContentRegionAvail();
    const float scale = std::max(0.0f, std::min(available.x / (frameSize.width * static_cast<float>(frameCount)),
        available.y / frameSize.height));
    if (!texturesDirty_)
      for (std::size_t index = 0; index < textures_.size(); ++index) {
        if (index) ImGui::SameLine(0, 0);
        ImGui::Image(static_cast<ImTextureID>(textures_[index]), ImVec2(frameSize.width * scale, frameSize.height * scale));
      }
  } else {
    ImGui::TextUnformatted("No frame sequence captured");
  }
  ImGui::End();
}

void FrameSequenceWindow::updateTextures(const FrameSequence &sequence) {
  if (!texturesDirty_) return;
  const auto &frames = sequence.frames;
  if (frames.empty()) return;
  GLint maximumSize = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximumSize);
  if (frames.front().cols > maximumSize || frames.front().rows > maximumSize)
    throw std::runtime_error("Frame sequence images exceed the supported texture size");
  while (textures_.size() > frames.size()) {
    glDeleteTextures(1, &textures_.back());
    textures_.pop_back();
  }
  while (textures_.size() < frames.size()) {
    GLuint texture = 0;
    glGenTextures(1, &texture);
    textures_.push_back(texture);
  }
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const auto image = frames[index].isContinuous() ? frames[index] : frames[index].clone();
    glBindTexture(GL_TEXTURE_2D, textures_[index]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    const GLint channels[] = {GL_RED, GL_RED, GL_RED, GL_ONE};
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, channels);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, image.cols, image.rows, 0, GL_RED, GL_UNSIGNED_BYTE, image.data);
  }
  glBindTexture(GL_TEXTURE_2D, 0);
  texturesDirty_ = false;
}

void FrameSequenceWindow::release() {
  for (const auto texture : textures_) glDeleteTextures(1, &texture);
  textures_.clear();
}
}
