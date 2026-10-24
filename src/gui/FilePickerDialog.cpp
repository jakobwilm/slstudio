#include "FilePickerDialog.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

namespace slstudio::gui {
void FilePickerDialog::draw(const FrameSequence *lastSequence, bool processing, bool scanActive,
    std::string &status, const std::function<void(const std::filesystem::path &)> &browseDirectory,
    const std::function<void(const std::filesystem::path &)> &loadCalibration,
    const std::function<void(const std::filesystem::path &)> &loadSequence,
    const std::function<void(const std::filesystem::path &, const FrameSequence &)> &saveSequence) {
  const bool folder = state.mode != FilePickerMode::GeometricCalibration;
  const bool saving = state.mode == FilePickerMode::SaveSequence;
  const char *title = saving ? "Save Sequence" : folder ? "Load Sequence" : "Load Geometric Calibration";
  if (state.requested) {
    state.requested = false;
    state.open = true;
    ImGui::OpenPopup(title);
  }
  const auto *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
      viewport->WorkPos.y + viewport->WorkSize.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(std::min(700.0f, viewport->WorkSize.x),
      std::min(480.0f, viewport->WorkSize.y)), ImGuiCond_FirstUseEver);
  if (!ImGui::BeginPopupModal(title, &state.open)) return;
  auto browse = [&](const std::filesystem::path &directory) {
    try { browseDirectory(directory); }
    catch (const std::exception &error) { state.error = status = error.what(); }
  };
  ImGui::SetNextItemWidth(-100);
  const bool enteredDirectory = ImGui::InputText("##Directory", state.path.data(),
      state.path.size(), ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::SameLine();
  if (ImGui::Button("Go") || enteredDirectory) browse(state.path.data());
  ImGui::SameLine();
  ImGui::BeginDisabled(state.directory.empty() || state.directory == state.directory.root_path());
  if (ImGui::Button("Up")) browse(state.directory.parent_path());
  ImGui::EndDisabled();
  std::optional<std::filesystem::path> navigate;
  bool load = false;
  ImGui::BeginChild("Entries", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() * 4), ImGuiChildFlags_Borders);
  for (const auto &directory : state.directories) {
    const auto label = "[Folder] " + directory.filename().string();
    if (ImGui::Selectable(label.c_str(), folder && state.selectedDirectory == directory,
        ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_DontClosePopups)) {
      if (folder) state.selectedDirectory = directory;
      if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) navigate = directory;
    }
  }
  for (const auto &file : state.files) {
    const auto filename = file.filename().string();
    if (ImGui::Selectable(filename.c_str(), filename == state.filename.data(),
        ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_DontClosePopups)) {
      if (filename.size() >= state.filename.size())
        state.error = status = "Geometric Calibration filename is too long";
      else {
        std::snprintf(state.filename.data(), state.filename.size(), "%s", filename.c_str());
        load = ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
      }
    }
  }
  ImGui::EndChild();
  if (navigate) browse(*navigate);
  if (folder) {
    ImGui::TextWrapped("Folder: %s", state.selectedDirectory.value_or(state.directory).filename().string().c_str());
  } else {
    ImGui::SetNextItemWidth(-80);
    load |= ImGui::InputText("File", state.filename.data(), state.filename.size(),
        ImGuiInputTextFlags_EnterReturnsTrue);
  }
  ImGui::TextWrapped("%s",
      saving ? "Choose a destination folder without existing sequence files." :
       folder ? "Select a folder containing frameSeq_<number> images and optional sequence.xml." :
       "Geometric Calibration XML (*.xml)");
  ImGui::BeginDisabled((!folder && !state.filename[0]) ||
      (saving ? !lastSequence : processing || scanActive));
  load |= ImGui::Button(saving ? "Save" : "Load");
  ImGui::SetItemDefaultFocus();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    state.open = false;
    ImGui::CloseCurrentPopup();
  } else if (load) {
    try {
      const auto directory = state.selectedDirectory.value_or(state.directory);
      if (saving) {
        if (!lastSequence) throw std::logic_error("No complete frame sequence to save");
        saveSequence(directory, *lastSequence);
        status = "Saved sequence: " + directory.string();
      } else if (folder) loadSequence(directory);
      else loadCalibration(state.directory / state.filename.data());
      state.open = false;
      ImGui::CloseCurrentPopup();
    } catch (const std::exception &error) {
      state.error = status = error.what();
    }
  }
  ImGui::EndPopup();
}
}
