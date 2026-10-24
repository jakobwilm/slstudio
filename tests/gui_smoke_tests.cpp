#include "gui_smoke_tests.h"
#include "ApplicationController.h"
#include "GuiRuntime.h"
#include "gray_code_test_data.h"
#include <imgui.h>
#include <imgui_internal.h>
#include <GLFW/glfw3.h>
#include <atomic>
#include <iostream>
#include <limits>
#include <thread>

namespace slstudio::gui {
int runGuiSmokeTest(GuiSmokeTest mode, const char *screenshot) {
  return runGui(true, [&](ApplicationController &application, const GuiContext &gui) {
    if (mode == GuiSmokeTest::Package) {
      for (int frame = 0; frame < 3; ++frame) {
        glfwPollEvents();
        gui.prepareFrame();
        ImGui::NewFrame();
        application.draw();
        ImGui::Render();
        gui.renderDrawData();
        glfwSwapBuffers(gui.window);
      }
      if (glGetError() != GL_NO_ERROR) throw std::runtime_error("OpenGL error in packaged application");
      std::cout << "Packaged GUI smoke test passed\n";
      return 0;
    }
    const bool geometricCalibrationSmoke = mode == GuiSmokeTest::GeometricCalibration;
    const bool intensityCalibrationSmoke = mode == GuiSmokeTest::IntensityCalibration;
    const bool projectorSmoke = mode == GuiSmokeTest::Projector;
    auto *window = gui.window;
    auto drawFrameSequenceWindow = [&] {
      application.frameSequenceWindow.draw(application.lastSequence ? &*application.lastSequence : nullptr,
          [&](const std::function<void()> &action) { application.attempt(action); },
          application.frameSequenceWindowTitle());
    };
    auto replaceProjector = [&](std::unique_ptr<Projector> output) {
      std::future<void> operation;
      if (output) {
        auto owned = std::make_shared<std::unique_ptr<Projector>>(std::move(output));
        operation = application.acquisition.openProjector([owned] { return std::move(*owned); });
      } else {
        operation = application.acquisition.closeProjector();
      }
      const auto deadline = Clock::now() + std::chrono::seconds(5);
      while (operation.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        if (application.projectorPresentationController) application.projectorPresentationController->process();
        if (Clock::now() >= deadline) throw std::runtime_error("Projector ownership handover timed out");
        std::this_thread::yield();
      }
      operation.get();
      application.projector = application.acquisition.projector();
    };
    auto awaitAcquisition = [&] {
      const auto deadline = Clock::now() + std::chrono::seconds(5);
      while (application.cameraJob.valid() || application.acquisitionJob.valid() ||
             application.projectorJob.valid() || !application.pendingAcquisitionJobs.empty() || application.recoveringAcquisition) {
        application.attempt([&] { application.tick(); }, ApplicationController::FailurePolicy::StopAcquisition);
        if (Clock::now() >= deadline) throw std::runtime_error("Acquisition commands timed out: " + application.status);
        std::this_thread::yield();
      }
    };
    if ((application.cameraRequested || application.cameraWorker || application.intensityCalibration.worker))
      throw std::runtime_error("Application startup opened the camera");
    if (application.ros2Output) throw std::runtime_error("Application startup enabled ROS 2 output");
    if (!geometricCalibrationSmoke && !intensityCalibrationSmoke && !projectorSmoke) {
      int monitorCount = 0;
      auto **monitors = glfwGetMonitors(&monitorCount);
      int expectedWindows = 0;
      for (int index = 0; index < monitorCount; ++index) {
        if (monitors[index] == glfwGetPrimaryMonitor()) continue;
        ++expectedWindows;
        const auto found = std::find_if(application.outputs.begin(), application.outputs.end(), [&](const auto &output) {
          return output.backend == OutputBackend::Desktop && output.id == std::to_string(index);
        });
        if (found == application.outputs.end() || found->name.find("(Fullscreen window)") == std::string::npos)
          throw std::runtime_error("Secondary-screen output was not listed automatically");
      }
      const auto actualWindows = std::count_if(application.outputs.begin(), application.outputs.end(),
          [](const auto &output) { return output.backend == OutputBackend::Desktop; });
      if (actualWindows != expectedWindows) throw std::runtime_error("Projector list included the primary screen");

      const int originalInterface = application.settings.cameraInterface;
      const float originalExposure = application.settings.exposure;
      const int originalDiscard = application.settings.discardFrames;
      application.settings.cameraInterface = static_cast<int>(application.cameraBackends.size());
      ImVec2 exposurePosition, discardPosition, unfocusPosition;
      auto controlsFrame = [&](const std::function<void(ImGuiIO &)> &input = {}) {
        gui.prepareFrame();
        if (input) input(ImGui::GetIO());
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(20, 20));
        ImGui::SetNextWindowSize(ImVec2(600, 450));
        ImGui::Begin("Camera input commit test", nullptr, ImGuiWindowFlags_NoSavedSettings);
        ImGui::PushItemWidth(350);
        const bool committed = application.cameraSettingsControls();
        const auto last = ImGui::GetItemRectMin();
        exposurePosition = ImVec2(last.x + 40, last.y - 3 * ImGui::GetFrameHeightWithSpacing() + ImGui::GetFrameHeight() / 2);
        discardPosition = ImVec2(last.x + 280, last.y + ImGui::GetFrameHeight() / 2);
        ImGui::Button("Lose focus");
        const auto button = ImGui::GetItemRectMin();
        unfocusPosition = ImVec2(button.x + 20, button.y + ImGui::GetFrameHeight() / 2);
        ImGui::PopItemWidth();
        ImGui::End();
        ImGui::Render();
        return committed;
      };
      const int originalCameraIndex = application.settings.cameraIndex;
      auto awaitDiscovery = [&] {
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        while (application.cameraDiscoveryJob.valid()) {
          application.tick();
          if (Clock::now() >= deadline) throw std::runtime_error("Camera discovery timed out");
          std::this_thread::yield();
        }
      };
      std::promise<void> releaseDiscovery;
      auto discoveryGate = releaseDiscovery.get_future().share();
      const auto uiThread = std::this_thread::get_id();
      application.refreshCameras([discoveryGate, uiThread] {
        if (std::this_thread::get_id() == uiThread)
          throw std::runtime_error("Camera discovery ran on the UI thread");
        if (discoveryGate.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
          throw std::runtime_error("UI blocked while camera discovery was pending");
        CameraDiscoveryResult result;
        result.index = 6;
        result.triggerSources = {"TestLine"};
        return result;
      });
      application.tick();
      controlsFrame();
      if (!application.cameraDiscoveryJob.valid() ||
          application.cameraDiscoveryJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready ||
          ImGui::GetDrawData()->TotalVtxCount == 0)
        throw std::runtime_error("Camera discovery did not leave the UI responsive");
      application.frameSequenceWindow.open = true;
      application.startCamera();
      if (!application.cameraRequested || application.cameraWorker)
        throw std::runtime_error("Preview did not wait for camera discovery");
      application.frameSequenceWindow.open = false;
      application.stopCamera();
      releaseDiscovery.set_value();
      awaitDiscovery();
      if (application.settings.cameraIndex != 6 || application.frameTriggerSources != std::vector<std::string>{"TestLine"} ||
          application.cameraWorker)
        throw std::runtime_error("Camera discovery results or deferred preview cancellation were lost");
      application.refreshCameras([]() -> CameraDiscoveryResult {
        throw std::runtime_error("Camera discovery test failure");
      });
      awaitDiscovery();
      if (application.status != "Camera discovery test failure" ||
          application.triggerSourcesError != application.status)
        throw std::runtime_error("Camera discovery failure was not reported");
      application.refreshCameras([] {
        CameraDiscoveryResult result;
        result.index = 77;
        return result;
      });
      application.settings.cameraIndex = 8;
      awaitDiscovery();
      if (application.settings.cameraIndex != 8)
        throw std::runtime_error("Stale camera discovery changed the selected camera");
      application.settings.cameraIndex = originalCameraIndex;
      {
        const auto originalSources = application.frameTriggerSources;
        const auto originalSource = application.settings.frameTriggerSource;
        const bool originalHardwareTrigger = application.settings.hardwareTrigger;
        const auto originalDevices = application.cameraList;
        const auto originalSourcesCamera = application.triggerSourcesCamera;
        const auto originalSourcesError = application.triggerSourcesError;
        const bool addedBackend = application.cameraBackends.empty();
        if (addedBackend) application.cameraBackends.push_back(CameraBackend::Aravis);
        application.settings.cameraInterface = 0;
        application.settings.hardwareTrigger = false;
        const auto backend = application.cameraBackends.front();
        application.refreshCameras([backend] {
          CameraDiscoveryResult result;
          result.devices = {{backend, "trigger-test", "Test", "Camera", "123", "Test"}};
          result.index = 0;
          result.triggersInspected = true;
          result.triggerSources = {"Line1", "Line2"};
          return result;
        });
        awaitDiscovery();
        if (application.cameraRequested || application.cameraWorker || !application.triggerSourcesCamera ||
            application.frameTriggerSources != std::vector<std::string>{"Line1", "Line2"})
          throw std::runtime_error("Camera discovery did not expose trigger inputs with preview stopped");
        ImVec2 sourcePosition;
        auto triggerFrame = [&](const std::function<void(ImGuiIO &)> &input = {}) {
          gui.prepareFrame();
          if (input) input(ImGui::GetIO());
          ImGui::NewFrame();
          ImGui::SetNextWindowPos(ImVec2(20, 20));
          ImGui::SetNextWindowSize(ImVec2(600, 450));
          ImGui::Begin("Camera trigger test", nullptr, ImGuiWindowFlags_NoSavedSettings);
          const auto start = ImGui::GetCursorScreenPos();
          sourcePosition = ImVec2(start.x + 40, start.y + ImGui::GetFrameHeight() / 2);
          const bool changed = application.cameraTriggerControls();
          ImGui::End();
          ImGui::Render();
          return changed;
        };
        auto clickTrigger = [&](ImVec2 position) {
          const bool pressed = triggerFrame([&](auto &io) {
            io.AddMousePosEvent(position.x, position.y);
            io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
          });
          const bool released = triggerFrame([](auto &io) { io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); });
          return pressed || released;
        };
        auto selectSource = [&](int index) {
          clickTrigger(sourcePosition);
          triggerFrame();
          const auto *popup = ImGui::FindWindowByName("##Combo_00");
          if (!popup || !popup->Active) throw std::runtime_error("Camera trigger source did not open");
          return clickTrigger(ImVec2(popup->DC.CursorStartPos.x + 20,
              popup->DC.CursorStartPos.y + index * ImGui::GetTextLineHeightWithSpacing() + ImGui::GetTextLineHeight() / 2));
        };
        triggerFrame();
        triggerFrame();
        if (!selectSource(2) || !application.settings.hardwareTrigger ||
            std::string(application.settings.frameTriggerSource.data()) != "Line2")
          throw std::runtime_error("Camera input selection did not enable hardware triggering");
        application.frameTriggerSources.clear();
        if (!selectSource(0) || application.settings.hardwareTrigger ||
            std::string(application.settings.frameTriggerSource.data()) != "Line2")
          throw std::runtime_error("Software trigger selection lost the saved input or required an available input");
        if (selectSource(0))
          throw std::runtime_error("Selecting the current trigger source restarted capture");
        application.frameTriggerSources = originalSources;
        application.settings.frameTriggerSource = originalSource;
        application.settings.hardwareTrigger = originalHardwareTrigger;
        application.cameraList = originalDevices;
        application.settings.cameraIndex = originalCameraIndex;
        application.triggerSourcesCamera = originalSourcesCamera;
        application.triggerSourcesError = originalSourcesError;
        if (addedBackend) application.cameraBackends.pop_back();
        application.settings.cameraInterface = static_cast<int>(application.cameraBackends.size());
      }
      auto clickExposure = [&] {
        if (controlsFrame([&](auto &io) {
          io.AddMousePosEvent(exposurePosition.x, exposurePosition.y);
          io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
        }) || controlsFrame([](auto &io) { io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); }))
          throw std::runtime_error("Focusing a camera field applied settings");
      };
      auto typeExposure = [&](const char *value) {
        if (controlsFrame([](auto &io) {
          io.AddKeyEvent(ImGuiMod_Ctrl, true);
          io.AddKeyEvent(ImGuiKey_A, true);
        }) || controlsFrame([](auto &io) {
          io.AddKeyEvent(ImGuiKey_A, false);
          io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }) || controlsFrame([&](auto &io) { io.AddInputCharactersUTF8(value); }))
          throw std::runtime_error("Camera settings applied while typing");
      };
      controlsFrame();
      controlsFrame();
      clickExposure();
      typeExposure("12.5");
      if (application.settings.exposure != 12.5f ||
          !controlsFrame([](auto &io) { io.AddKeyEvent(ImGuiKey_Enter, true); }) ||
          controlsFrame([](auto &io) { io.AddKeyEvent(ImGuiKey_Enter, false); }))
        throw std::runtime_error("Enter did not commit the camera edit exactly once");
      clickExposure();
      typeExposure("8.25");
      if (application.settings.exposure != 8.25f || !controlsFrame([&](auto &io) {
        io.AddMousePosEvent(unfocusPosition.x, unfocusPosition.y);
        io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
      }) || controlsFrame([](auto &io) { io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); }))
        throw std::runtime_error("Losing focus did not commit the camera edit exactly once");
      if (controlsFrame([&](auto &io) {
        io.AddMousePosEvent(discardPosition.x, discardPosition.y);
        io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
      }) || !controlsFrame([](auto &io) { io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); }))
        throw std::runtime_error("Camera slider did not wait for release before applying");
      application.settings.cameraInterface = originalInterface;
      application.settings.exposure = originalExposure;
      application.settings.discardFrames = originalDiscard;

      struct ScanFeedProjector : Projector {
        cv::Size size;
        std::shared_ptr<std::atomic<std::size_t>> index = std::make_shared<std::atomic<std::size_t>>(0);
        std::size_t count = 0;
        cv::Size resolution() const override { return size; }
        void setPatterns(const std::vector<cv::Mat> &patterns) override { count = patterns.size(); }
        void present(std::size_t next) override {
          if (next >= count) throw std::runtime_error("Invalid scan pattern");
          index->store(next);
        }
        void blank(bool = false) override { index->store(0); }
      };
      GeometricCalibrationData scanCalibration;
      if (!scanCalibration.load(application.settings.geometricCalibrationPath.data()))
        throw std::runtime_error("Camera stream test calibration missing");
      auto scanOutput = std::make_unique<ScanFeedProjector>();
      scanOutput->size = {scanCalibration.screenResX, scanCalibration.screenResY};
      const auto patternIndex = scanOutput->index;
      replaceProjector(std::move(scanOutput));
      {
        const auto original = application.settings;
        CodecAlgorithmSettings settings;
        settings.phaseShift = {5, 8};
        settings.grayCode = {8, 5};
        application.settings.directionIndex = 2;
        application.settings.reconstruction.coding.direction = CodecDirBoth;
        application.configureCodec(CodecType::PhaseShift, settings);
        const auto phasePatterns = application.projectorPatterns(false);
        if (phasePatterns.size() != 16 ||
            application.settings.reconstruction.coding.algorithmSettings.phaseShift.steps != 5)
          throw std::runtime_error("GUI pattern generation ignored configurable phase steps");
        settings.phaseShift.periods = 4;
        application.configureCodec(CodecType::PhaseShift, settings);
        if (cv::norm(phasePatterns.front(), application.projectorPatterns(false).front(), cv::NORM_INF) == 0)
          throw std::runtime_error("GUI pattern generation ignored configurable fringe periods");
        settings.phaseShift.periods = 1;
        application.configureCodec(CodecType::PhaseShift, settings);
        if (application.projectorPatterns(false).size() != 10)
          throw std::runtime_error("GUI single-frequency patterns retained cue images");
        application.settings.reconstruction.lisPhaseFilter = true;
        application.configureCodec(CodecType::GrayCode, settings);
        if (application.projectorPatterns(false).size() != 15 || application.settings.reconstruction.lisPhaseFilter)
          throw std::runtime_error("GUI Gray-code settings ignored bits or retained incompatible filters");
        settings.phaseShift.steps = 2;
        const auto saved = application.settings.reconstruction;
        bool rejected = false;
        try { application.configureCodec(CodecType::PhaseShift, settings); }
        catch (const std::invalid_argument &) { rejected = true; }
        if (!rejected || !(application.settings.reconstruction == saved))
          throw std::runtime_error("GUI accepted invalid codec settings or changed them on failure");
        application.sequencePending = true;
        rejected = false;
        try { application.configureCodec(CodecType::PhaseShift, {}); }
        catch (const std::logic_error &) { rejected = true; }
        application.sequencePending = false;
        if (!rejected || !(application.settings.reconstruction == saved))
          throw std::runtime_error("GUI changed codec settings while processing");
        application.applySettings(original);
      }
      const auto fullProjector = application.projector->resolution();
      application.settings.projectorWidth = fullProjector.width / 2;
      application.settings.projectorHeight = fullProjector.height / 2;
      const auto croppedPatterns = application.projectorPatterns(false);
      for (const auto &pattern : croppedPatterns) {
        if (pattern.size() != fullProjector ||
            cv::norm(pattern(cv::Rect(application.settings.projectorWidth, 0,
                fullProjector.width - application.settings.projectorWidth, fullProjector.height)), cv::NORM_INF) != 0 ||
            cv::norm(pattern(cv::Rect(0, application.settings.projectorHeight,
                fullProjector.width, fullProjector.height - application.settings.projectorHeight)), cv::NORM_INF) != 0)
          throw std::runtime_error("Projector crop did not preserve full output geometry with black borders");
      }
      application.settings.projectorWidth = 0;
      if (application.projectorSize() != cv::Size(fullProjector.width, fullProjector.height / 2))
        throw std::runtime_error("Resetting projector width changed projector height");
      application.settings.projectorHeight = 0;
      if (application.projectorSize() != fullProjector)
        throw std::runtime_error("Resetting projector height did not restore full projector dimensions");
      application.settings.projectorWidth = fullProjector.width + 1;
      bool oversizedProjectorRejected = false;
      try { application.projectorPatterns(false); }
      catch (const std::invalid_argument &) { oversizedProjectorRejected = true; }
      if (!oversizedProjectorRejected) throw std::runtime_error("Oversized projector crop was accepted");
      application.settings.projectorWidth = 0;
      const cv::Size originalCameraSize(application.settings.cameraWidth, application.settings.cameraHeight);
      application.settings.cameraWidth = scanCalibration.frameWidth;
      application.settings.cameraHeight = scanCalibration.frameHeight;
      application.frameSequenceWindow.open = true;
      const auto samples = loadFrameSequence(application.settings.sequencePath.data());
      auto cameraReads = std::make_shared<std::atomic<int>>(0);
      application.settings.cameraWidth = application.settings.cameraHeight = 0;
      application.openCalibrationDialog(false, [samples, patternIndex, cameraReads, uiThread] {
        if (std::this_thread::get_id() == uiThread) throw std::runtime_error("Camera read on UI thread");
        ++*cameraReads;
        return samples.at(patternIndex->load());
      });
      auto awaitCamera = [&](auto ready) {
        const auto deadline = Clock::now() + std::chrono::seconds(10);
        while (!ready()) {
          application.tick();
          if (Clock::now() >= deadline) throw std::runtime_error("Shared camera stream timed out");
          std::this_thread::yield();
        }
      };
      awaitCamera([&] { return !application.cameraPreview.empty(); });
      if (!application.geometricCalibrationDialog.open || application.settings.cameraWidth != scanCalibration.frameWidth ||
          application.settings.cameraHeight != scanCalibration.frameHeight)
        throw std::runtime_error("Opening calibration did not start the camera and resolve full sensor dimensions");
      application.geometricCalibrationDialog.open = false;
      const auto *sharedCamera = application.cameraWorker;
      const auto previewBeforeFailure = application.cameraPreview.clone();
      const auto settingsBeforeFailure = application.settings;
      application.attempt([&] { application.saveCloud(); });
      if (application.status != "No point cloud to save" || application.cameraWorker != sharedCamera ||
          !application.cameraRequested ||
          cv::norm(previewBeforeFailure, application.cameraPreview, cv::NORM_INF) != 0)
        throw std::runtime_error("Export failure interrupted camera preview");
      application.attempt([&] {
        cv::FileStorage invalid("%YAML:1.0\nexposure: 200\npreviewFrameCount: 2147483647\n",
            cv::FileStorage::READ | cv::FileStorage::MEMORY);
        application.serializeSettings(invalid, true);
      });
      if (application.settings.exposure != settingsBeforeFailure.exposure ||
          application.settings.previewFrameCount != settingsBeforeFailure.previewFrameCount ||
          application.cameraPreviewFrameCount != settingsBeforeFailure.previewFrameCount ||
          application.cameraWorker != sharedCamera || !application.cameraRequested ||
          cv::norm(previewBeforeFailure, application.cameraPreview, cv::NORM_INF) != 0)
        throw std::runtime_error("Rejected settings partially changed settings or interrupted preview");
      application.openCalibrationDialog(true);
      if (!application.intensityCalibration.open || application.cameraWorker != sharedCamera)
        throw std::runtime_error("Opening intensity calibration did not reuse the active camera");
      application.intensityCalibration.open = false;
      application.startScan();
      if (!application.frameSequenceWindow.open || application.cameraWorker != sharedCamera)
        throw std::runtime_error("Starting a scan closed Frame sequence or reopened the camera");
      awaitCamera([&] { return application.acquisition.sequenceIndex() >= 2; });
      const auto *activePipeline = application.decoder.get();
      application.attempt([&] { application.loadGeometricCalibrationFile("missing-calibration.xml"); });
      if (!application.scan.active() ||
          application.cameraWorker != sharedCamera || application.decoder.get() != activePipeline)
        throw std::runtime_error("Rejected file action stopped an active scan");
      if (application.cameraPreview.empty())
        throw std::runtime_error("Live preview stopped during scanning");
      if (application.lastSequence) throw std::runtime_error("Partial capture replaced the completed frame sequence");
      awaitCamera([&] { return application.scan.index() < 0; });
      if (!application.lastSequence || application.lastSequence->frames.size() != samples.size() ||
          !application.frameSequenceWindow.texturesDirty())
        throw std::runtime_error("Completed software capture did not update Frame sequence");
      for (std::size_t index = 0; index < samples.size(); ++index)
        if (cv::norm(application.lastSequence->frames[index], samples[index], cv::NORM_INF) != 0)
          throw std::runtime_error("Completed capture changed frame sequence order or pixels");
      application.stopCapture();
      if (!application.frameSequenceWindow.open || application.cameraWorker != sharedCamera || application.scanActive())
        throw std::runtime_error("Stopping a scan interrupted the shared camera stream");
      application.frameSequenceWindow.open = false;
      drawFrameSequenceWindow();
      const auto readsBeforeHiding = cameraReads->load();
      awaitCamera([&] { return cameraReads->load() > readsBeforeHiding; });
      if (application.cameraWorker != sharedCamera)
        throw std::runtime_error("Hiding Frame sequence replaced the camera worker");
      application.stopCamera();
      struct TriggeredScanProjector : ScanFeedProjector {
        std::shared_ptr<std::atomic<bool>> triggered = std::make_shared<std::atomic<bool>>(false);
        ProjectorCapabilities capabilities() const override { return {true, true, false}; }
        void configureSequence(const ProjectorSequenceSettings &) override {}
        void startSequence() override { triggered->store(true); }
        void stopSequence() override { triggered->store(false); }
      };
      const bool addedBackend = application.cameraBackends.empty();
      if (addedBackend) application.cameraBackends.push_back(CameraBackend::Aravis);
      application.settings.cameraInterface = 0;
      application.settings.hardwareTrigger = true;
      application.settings.exposure = 1;
      auto triggeredOutput = std::make_unique<TriggeredScanProjector>();
      triggeredOutput->size = {scanCalibration.screenResX, scanCalibration.screenResY};
      const auto triggered = triggeredOutput->triggered;
      replaceProjector(std::move(triggeredOutput));
      auto waitingForTrigger = std::make_shared<std::atomic<bool>>(false);
      application.startCamera([samples, triggered, waitingForTrigger] {
        waitingForTrigger->store(true);
        if (!triggered->load()) return cv::Mat{};
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return samples.front();
      });
      awaitCamera([&] { return waitingForTrigger->load(); });
      awaitAcquisition();
      const auto *armedCamera = application.cameraWorker;
      application.startScan();
      awaitCamera([&] { return application.acquisition.sequenceIndex() >= 2; });
      if (application.cameraWorker != armedCamera || !triggered->load() ||
          cv::norm(application.cameraPreview, samples.front(), cv::NORM_INF) != 0)
        throw std::runtime_error("Hardware scanning did not reuse the armed camera and display its frames");
      application.stopCapture();
      application.stopCamera();
      application.settings.hardwareTrigger = false;
      application.settings.cameraInterface = originalInterface;
      application.settings.exposure = originalExposure;
      if (addedBackend) application.cameraBackends.pop_back();
      awaitAcquisition();
      replaceProjector({});
      application.settings.cameraWidth = originalCameraSize.width;
      application.settings.cameraHeight = originalCameraSize.height;
      const auto preservedFrame = application.lastSequence->frames.front().clone();
      application.startCamera([]() -> cv::Mat { throw std::runtime_error("Synthetic shared camera failure"); });
      const auto failureDeadline = Clock::now() + std::chrono::seconds(5);
      while (application.cameraWorker) {
        application.attempt([&] { application.tick(); }, ApplicationController::FailurePolicy::StopAcquisition);
        if (Clock::now() >= failureDeadline) throw std::runtime_error("Camera failure cleanup timed out");
        std::this_thread::yield();
      }
      if (application.status != "Synthetic shared camera failure" || application.cameraRequested ||
          !application.cameraPreview.empty() ||
          cv::norm(application.lastSequence->frames.front(), preservedFrame, cv::NORM_INF) != 0)
        throw std::runtime_error("Camera failure cleanup lost the completed sequence or retained a live session");
      struct CleanupFailureProjector : ScanFeedProjector {
        std::shared_ptr<std::array<int, 2>> calls = std::make_shared<std::array<int, 2>>();
        void stopSequence() override {
          ++(*calls)[0];
          throw std::runtime_error("Synthetic projector stop failure");
        }
        void blank(bool = false) override {
          ++(*calls)[1];
          throw std::runtime_error("Synthetic projector blank failure");
        }
      };
      auto failingOutput = std::make_unique<CleanupFailureProjector>();
      const auto cleanupCalls = failingOutput->calls;
      replaceProjector(std::move(failingOutput));
      application.projectorJob = std::async(std::launch::async, [] {
        throw std::runtime_error("Synthetic programming cleanup failure");
      });
      application.attempt([] { throw std::runtime_error("Primary operation failure"); },
                          ApplicationController::FailurePolicy::StopAcquisition);
      awaitAcquisition();
      if (application.projector || *cleanupCalls != std::array<int, 2>{1, 1} ||
          application.status.find("Primary operation failure") != 0 ||
          application.status.find("Synthetic programming cleanup failure") == std::string::npos ||
          application.status.find("Synthetic projector stop failure") == std::string::npos ||
          application.status.find("Synthetic projector blank failure") == std::string::npos)
        throw std::runtime_error("Recovery hid secondary failures or skipped projector blanking");
    }
    PointCloudConstPtr smokeReference;
    bool reconstructionReviewed = geometricCalibrationSmoke || intensityCalibrationSmoke || projectorSmoke;
    struct IntensitySmokeProjector : Projector {
      std::shared_ptr<std::atomic<int>> level = std::make_shared<std::atomic<int>>(0);
      cv::Mat pattern;
      int blanks = 0;
      cv::Size resolution() const override { return {320, 240}; }
      void setPatterns(const std::vector<cv::Mat> &patterns) override {
        if (patterns.size() != 1 || patterns.front().type() != CV_8UC1 || patterns.front().size() != resolution())
          throw std::runtime_error("Invalid solid-field pattern");
        pattern = patterns.front();
      }
      void present(std::size_t index) override {
        if (index != 0 || pattern.empty()) throw std::runtime_error("Invalid solid-field index");
        level->store(pattern.at<std::uint8_t>(0, 0));
      }
      void blank(bool = false) override { level->store(0); ++blanks; }
    };
    IntensitySmokeProjector *intensityOutput = nullptr;
    CaptureWorker::Reader intensityReader;
    cv::Mat intensityCalibrationSmokePreview;
    IntensityCalibrationLut originalCorrection;
    std::optional<IntensityCalibrationLut> verifiedIntensity;
    std::vector<IntensitySample> verificationSamples;
    int intensityStage = 0;
    int intensityLiveFrames = 0;
    ImVec2 intensityImageMin, intensityImageMax;
    const auto smokeDeadline = Clock::now() + std::chrono::seconds(geometricCalibrationSmoke ? 180 : 45);
    const std::string initialCalibrationPath = application.settings.geometricCalibrationPath.data();
    struct TemporaryCalibration {
      std::filesystem::path path;
      ~TemporaryCalibration() {
        if (!path.empty()) {
          std::error_code error;
          std::filesystem::remove(path, error);
        }
      }
    } temporaryCalibration;
    TemporaryCalibration temporaryInvalidCalibration;
    TemporaryCalibration temporaryIntensityYaml, temporaryIntensityCsv;
    struct TemporarySequence {
      std::filesystem::path path;
      ~TemporarySequence() {
        if (!path.empty()) {
          std::error_code error;
          std::filesystem::remove_all(path, error);
        }
      }
    } temporarySequence, temporarySettings;
    {
      temporarySettings.path = std::filesystem::temp_directory_path() /
          ("slstudio-settings-" + std::to_string(Clock::now().time_since_epoch().count()));
      application.settingsDirectory = temporarySettings.path;
    }
    if (!geometricCalibrationSmoke && !intensityCalibrationSmoke && !projectorSmoke) {
      std::vector<cv::Mat> sequence;
      for (int index = 0; index < 3; ++index)
        sequence.emplace_back(240, 320, CV_8UC1, cv::Scalar(20 + index));
      application.setLastSequence(FrameSequence{sequence, application.settings.reconstruction.coding});
      application.frameSequenceWindow.open = true;
      struct WindowBounds { ImVec2 position, size; };
      auto resizeFrame = [&](const std::function<void(ImGuiIO &)> &input = {}) {
        gui.prepareFrame();
        if (input) input(ImGui::GetIO());
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(100, 100), ImGuiCond_Always);
        drawFrameSequenceWindow();
        ImGui::Begin(application.frameSequenceWindowTitle().c_str());
        const WindowBounds bounds{ImGui::GetWindowPos(), ImGui::GetWindowSize()};
        ImGui::End();
        ImGui::Render();
        if (std::abs(bounds.size.y - ImGui::GetFrameHeight() - bounds.size.x / 4) > 1.1f)
          throw std::runtime_error("Frame sequence lost its aspect ratio during mouse resizing");
        return bounds;
      };
      resizeFrame();
      auto bounds = resizeFrame();
      auto drag = [&](ImVec2 start, ImVec2 delta) {
        resizeFrame([&](auto &io) { io.AddMousePosEvent(start.x, start.y); });
        resizeFrame([&](auto &io) { io.AddMouseButtonEvent(ImGuiMouseButton_Left, true); });
        const ImVec2 target(start.x + delta.x, start.y + delta.y);
        auto resized = resizeFrame([&](auto &io) { io.AddMousePosEvent(target.x, target.y); });
        for (int index = 0; index < 8; ++index) {
          const auto held = resizeFrame([&](auto &io) { io.AddMousePosEvent(target.x, target.y); });
          if (std::abs(held.size.x - resized.size.x) > 1 || std::abs(held.size.y - resized.size.y) > 1)
            throw std::runtime_error("Frame sequence flickered while the resize pointer was stationary");
          resized = held;
        }
        resizeFrame([](auto &io) { io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); });
        return resized;
      };
      auto resized = drag(ImVec2(bounds.position.x + bounds.size.x - 3, bounds.position.y + bounds.size.y - 3),
          ImVec2(120, 60));
      if (resized.size.x <= bounds.size.x + 40)
        throw std::runtime_error("Corner resize regression test did not grow Frame sequence");
      bounds = resized;
      resized = drag(ImVec2(bounds.position.x + bounds.size.x - 3, bounds.position.y + bounds.size.y - 3),
          ImVec2(-80, -40));
      if (resized.size.x >= bounds.size.x - 20)
        throw std::runtime_error("Corner resize regression test did not shrink Frame sequence");
      bounds = resized;
      resized = drag(ImVec2(bounds.position.x + bounds.size.x - 1, bounds.position.y + bounds.size.y / 2),
          ImVec2(80, 0));
      if (resized.size.x <= bounds.size.x + 40)
        throw std::runtime_error("Horizontal edge resize did not grow Frame sequence");
      bounds = resized;
      resized = drag(ImVec2(bounds.position.x + bounds.size.x / 2, bounds.position.y + bounds.size.y - 1),
          ImVec2(0, 20));
      if (resized.size.x <= bounds.size.x + 40)
        throw std::runtime_error("Vertical edge resize did not grow Frame sequence");
      application.frameSequenceWindow.open = false;
      application.lastSequence.reset();
      application.frameSequenceWindow.markClean();
      ImGui::SetWindowSize("###FrameSequence", ImVec2(400, 150 + ImGui::GetFrameHeight()));
    }
    {
      application.setCameraPreviewFrameCount(3);
      std::array<int, 3> slots{};
      cv::Mat frame(4, 6, CV_8UC1);
      for (int index = 1; index <= 8; ++index) {
        frame.setTo(index);
        application.appendCameraPreview(frame);
        frame.setTo(99);
        slots[(index - 1) % 3] = index;
        if (application.cameraPreview.size() != cv::Size(18, 4) ||
            application.cameraPreviewFrameSize != cv::Size(6, 4) || application.cameraNextPreviewFrame != index % 3 ||
            !application.cameraPreviewDirty)
          throw std::runtime_error("Round-robin preview has incorrect dimensions or next slot");
        for (int slot = 0; slot < 3; ++slot)
          if (cv::countNonZero(application.cameraPreview(cv::Rect(slot * 6, 0, 6, 4)) != slots[slot]))
            throw std::runtime_error("Round-robin preview overwrote the wrong slot or retained camera-owned pixels");
      }
      application.appendCameraPreview(cv::Mat(3, 8, CV_8UC1, cv::Scalar(17)));
      if (application.cameraPreview.size() != cv::Size(24, 3) || application.cameraNextPreviewFrame != 1 ||
          cv::countNonZero(application.cameraPreview(cv::Rect(0, 0, 8, 3)) != 17) ||
          cv::countNonZero(application.cameraPreview(cv::Rect(8, 0, 16, 3))))
        throw std::runtime_error("Preview resolution change retained old slots");
      for (const int count : {0, -1, std::numeric_limits<int>::max()}) {
        bool rejected = false;
        try { application.setCameraPreviewFrameCount(count); }
        catch (const std::invalid_argument &) { rejected = true; }
        if (!rejected || application.cameraPreviewFrameCount != 3 || application.cameraNextPreviewFrame != 1)
          throw std::runtime_error("Invalid preview frame count changed live state");
      }
      bool invalidFrameRejected = false;
      try { application.appendCameraPreview(cv::Mat(3, 8, CV_8UC3)); }
      catch (const std::invalid_argument &) { invalidFrameRejected = true; }
      if (!invalidFrameRejected || application.cameraNextPreviewFrame != 1)
        throw std::runtime_error("Invalid preview frame changed the round-robin buffer");
      cv::FileStorage previewSettings("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
      application.serializeSettings(previewSettings, false);
      application.setCameraPreviewFrameCount(1);
      application.appendCameraPreview(cv::Mat(3, 8, CV_8UC1, cv::Scalar(42)));
      if (application.cameraPreview.size() != cv::Size(8, 3) ||
          cv::countNonZero(application.cameraPreview != 42) || application.cameraNextPreviewFrame != 0)
        throw std::runtime_error("Single-frame preview did not replace the concatenated buffer");
      cv::FileStorage restoredPreviewSettings(previewSettings.releaseAndGetString(),
          cv::FileStorage::READ | cv::FileStorage::MEMORY);
      application.serializeSettings(restoredPreviewSettings, true);
      if (application.cameraPreviewFrameCount != 3 ||
          !application.cameraPreview.empty())
        throw std::runtime_error("Preview frame count did not round-trip through saved settings");
      application.setCameraPreviewFrameCount(1, application.maximumPreviewSize);
      application.appendCameraPreview(cv::Mat(1, application.maximumPreviewSize, CV_8UC1, cv::Scalar(7)));
      if (application.cameraPreview.cols != application.maximumPreviewSize)
        throw std::runtime_error("Preview rejected the exact texture-width limit");
      application.stopCamera();
      const int maximumPreviewFrameCount = application.maximumPreviewSize;
      application.setCameraPreviewFrameCount(maximumPreviewFrameCount, 1);
      application.appendCameraPreview(cv::Mat(1, 1, CV_8UC1, cv::Scalar(8)));
      if (application.cameraPreview.cols != maximumPreviewFrameCount)
        throw std::runtime_error("Preview rejected the maximum frame count");
      application.stopCamera();
      application.setCameraPreviewFrameCount(3, 4);
      if (application.cameraPreviewFrameCount != 3 || !application.cameraPreviewFrameSize.empty() ||
          application.cameraNextPreviewFrame != 0)
        throw std::runtime_error("Stopping preview did not clear slots while preserving the configured count");
      bool oversizedPreviewRejected = false;
      try {
        application.appendCameraPreview(cv::Mat(application.maximumPreviewSize + 1, 1, CV_8UC1));
      } catch (const std::invalid_argument &) {
        oversizedPreviewRejected = true;
      }
      if (!oversizedPreviewRejected || !application.cameraPreview.empty() || application.cameraPreviewDirty)
        throw std::runtime_error("Preview accepted dimensions beyond the texture limit");
      const auto previewStart = Clock::time_point{};
      for (int index = 1; index <= 5; ++index)
        application.appendCameraPreview(cv::Mat(2, 4, CV_8UC1, cv::Scalar(index)),
            previewStart + std::chrono::milliseconds(index * 100));
      if (application.cameraPreviewSampleCount() != 5 ||
          std::abs(application.cameraPreviewFps(previewStart + std::chrono::milliseconds(600)) - 10) > 1e-9 ||
          application.cameraPreviewFps(previewStart + std::chrono::milliseconds(1500)) != 0)
        throw std::runtime_error("Preview FPS window or stale-frame behavior changed");
      application.stopCamera();
      if (application.cameraPreviewSampleCount() != 0 || application.cameraPreviewDirty)
        throw std::runtime_error("Stopping preview did not clear timing and upload state");
      application.setCameraPreviewFrameCount(1);
      GeometricCalibrationData calibration;
      if (!calibration.load(application.settings.geometricCalibrationPath.data())) throw std::runtime_error("Smoke test calibration missing");
      DecodeStage decoder(calibration);
      TriangulationStage triangulator(calibration);
      cv::Mat1f up, vp;
      cv::Mat1b mask, shading;
      decoder.decodeMaps(loadFrameSequence(application.settings.sequencePath.data()), up, vp, mask, shading);
      application.cloud = triangulator.triangulateDense(up, vp, mask, shading);
      application.renderer.setCloud(application.cloud);
      application.status = std::to_string(application.cloud->points.size()) + " points";
      for (const auto size : {cv::Size(900, 600), cv::Size(400, 600)}) {
        application.renderer.render(size.width, size.height, application.settings.rendering);
        if (!application.renderer.hasVisiblePixels()) throw std::runtime_error("Blank point-cloud framebuffer");
      }
      application.renderer.yaw = 0.3f;
      application.renderer.render(900, 600, application.settings.rendering);
      if (!application.renderer.hasVisiblePixels()) throw std::runtime_error("Rotated point cloud is blank");
      application.renderer.resetView();
      auto renderedImage = [&] {
        cv::Mat image(600, 900, CV_8UC3);
        glBindTexture(GL_TEXTURE_2D, application.renderer.render(900, 600, application.settings.rendering));
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);
        return image;
      };
      const auto originalView = renderedImage();
      {
        const auto grayCalibration = grayCodeCalibration();
        ReconstructionSettings graySettings;
        graySettings.coding.codec = CodecType::GrayCode;
        graySettings.coding.direction = CodecDirBoth;
        const auto sparse = TriangulationStage(grayCalibration, graySettings).triangulateSparse(
            DecodeStage(grayCalibration, graySettings).decodeTransitions(grayCodeFrames(grayCalibration, CodecDirBoth)));
        if (sparse->points.empty() || !sparse->pixelIndices.empty())
          throw std::runtime_error("Gray-code reconstruction did not produce an unorganized cloud");
        application.renderer.setCloud(sparse);
        const bool shading = application.settings.rendering.surfaceShading;
        application.settings.rendering.surfaceShading = false;
        const auto unshaded = renderedImage();
        if (!application.renderer.hasVisiblePixels())
          throw std::runtime_error("Sparse Gray-code point cloud is blank");
        application.settings.rendering.surfaceShading = true;
        if (cv::norm(unshaded, renderedImage(), cv::NORM_INF) != 0)
          throw std::runtime_error("Sparse Gray-code rendering invented surface connectivity");
        application.settings.rendering.surfaceShading = shading;
        application.renderer.setCloud(application.cloud);
        if (cv::norm(originalView, renderedImage(), cv::NORM_INF) != 0)
          throw std::runtime_error("Switching between sparse and organized clouds retained stale GPU state");
      }
      application.renderer.pan(40, 30, 600);
      const auto pannedView = renderedImage();
      if (!application.renderer.hasVisiblePixels() || cv::norm(originalView, pannedView, cv::NORM_INF) == 0)
        throw std::runtime_error("Point-cloud panning failed to change the visible image");
      application.renderer.resetView();
      if (cv::norm(originalView, renderedImage(), cv::NORM_INF) != 0)
        throw std::runtime_error("Reset view failed to clear point-cloud panning");
      cv::Point selectedPixel(-1, -1);
      for (int row = 0; row < originalView.rows && selectedPixel.x < 0; ++row)
        for (int column = 0; column < originalView.cols; ++column) {
          const auto pixel = originalView.at<cv::Vec3b>(row, column);
          if (pixel[0] > 35 || pixel[1] > 35 || pixel[2] > 35) { selectedPixel = {column, row}; break; }
        }
      if (application.renderer.setRotationCenter(-0.1f, 0.5f) || application.renderer.setRotationCenter(0, 0))
        throw std::runtime_error("Background picking changed the point-cloud rotation center");
      if (selectedPixel.x < 0 || !application.renderer.setRotationCenter((selectedPixel.x + 0.5f) / 900,
          1.0f - (selectedPixel.y + 0.5f) / 600))
        throw std::runtime_error("Point-cloud rotation center could not pick a visible point");
      const auto centeredView = renderedImage();
      if (centeredView.at<cv::Vec3b>(300, 450) == originalView.at<cv::Vec3b>(0, 0) ||
          cv::norm(originalView, centeredView, cv::NORM_INF) == 0)
        throw std::runtime_error("Picking a point did not center it in the view");
      application.renderer.yaw = 0.2f;
      application.renderer.render(900, 600, application.settings.rendering);
      if (!application.renderer.hasVisiblePixels()) throw std::runtime_error("Rotation around the selected point is blank");
      application.renderer.resetView();
      if (cv::norm(originalView, renderedImage(), cv::NORM_INF) != 0)
        throw std::runtime_error("Reset view failed to restore the original rotation center");
      auto pivotCloud = std::make_shared<PointCloud>();
      pivotCloud->points = {{-10, 0, 1000, 255, 0, 0}, {0, 0, 1000, 0, 255, 0}, {10, 0, 1000, 0, 0, 255}};
      application.renderer.setCloud(pivotCloud);
      application.renderer.pan(40, 30, 600);
      const auto pivotView = renderedImage();
      cv::Point pivotPixel(-1, -1);
      auto red = [](cv::Vec3b pixel) { return pixel[0] > 200 && pixel[1] < 10 && pixel[2] < 10; };
      for (int row = 0; row < pivotView.rows && pivotPixel.x < 0; ++row)
        for (int column = 0; column < pivotView.cols; ++column)
          if (red(pivotView.at<cv::Vec3b>(row, column))) { pivotPixel = {column, row}; break; }
      if (pivotPixel.x < 0 || !application.renderer.setRotationCenter((pivotPixel.x + 0.5f) / 900,
          1.0f - (pivotPixel.y + 0.5f) / 600) || !red(renderedImage().at<cv::Vec3b>(300, 450)))
        throw std::runtime_error("Picked point was not centered after panning");
      application.renderer.yaw = 0.4f;
      application.renderer.pitch = 0.2f;
      if (!red(renderedImage().at<cv::Vec3b>(300, 450)))
        throw std::runtime_error("Picked point did not remain centered during rotation");
      application.renderer.zoom(1000);
      if (std::abs(application.renderer.distance - 0.01f) > 1e-6f ||
          !red(renderedImage().at<cv::Vec3b>(300, 450)) || !application.renderer.setRotationCenter(0.5f, 0.5f))
        throw std::runtime_error("Close zoom clipped the selected point or broke picking");
      application.renderer.zoom(-1000);
      if (application.renderer.distance != 30.0f) throw std::runtime_error("Zoom-out limit changed");
      application.renderer.resetView();
      application.renderer.setCloud(application.cloud);
      application.settings.rendering.surfaceShading = true;
      const auto shadedView = renderedImage();
      if (!application.renderer.hasVisiblePixels() || cv::norm(originalView, shadedView, cv::NORM_INF) == 0)
        throw std::runtime_error("Surface shading is blank or did not change the point-cloud image");
      const auto background = originalView.at<cv::Vec3b>(0, 0);
      for (int row = 0; row < originalView.rows; ++row)
        for (int column = 0; column < originalView.cols; ++column)
          if (originalView.at<cv::Vec3b>(row, column) == background && shadedView.at<cv::Vec3b>(row, column) != background)
            throw std::runtime_error("Surface shading created pixels outside measured points");
      if (!application.renderer.setRotationCenter((selectedPixel.x + 0.5f) / 900,
          1.0f - (selectedPixel.y + 0.5f) / 600))
        throw std::runtime_error("Surface shading broke point-cloud picking");
      application.renderer.resetView();
      application.settings.rendering.surfaceShading = false;
      if (cv::norm(originalView, renderedImage(), cv::NORM_INF) != 0)
        throw std::runtime_error("Surface shading modified the original point cloud");
      application.settings.rendering.colorMode = PointCloudRenderer::ColorMode::Depth;
      if (!application.renderer.hasVisiblePixels() || cv::norm(originalView, renderedImage(), cv::NORM_INF) == 0)
        throw std::runtime_error("Depth coloring did not change the measured grayscale image");
      application.settings.rendering.colorMode = PointCloudRenderer::ColorMode::Modulation;
      if (cv::norm(originalView, renderedImage(), cv::NORM_INF) != 0)
        throw std::runtime_error("Changing color mode modified measured point colors");
      cv::Mat3f normalGrid(3, 3);
      const cv::Mat1b normalShading(3, 3, std::uint8_t(240));
      for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
          normalGrid(row, column) = cv::Vec3f((column - 1) * 10, (row - 1) * 10, 1000);
      auto centerIsShaded = [&] {
        application.renderer.setCloud(makePointCloud(normalGrid, normalShading));
        application.settings.rendering.surfaceShading = false;
        const auto unlit = renderedImage().at<cv::Vec3b>(300, 450);
        application.settings.rendering.surfaceShading = true;
        const auto lit = renderedImage().at<cv::Vec3b>(300, 450);
        if (unlit != cv::Vec3b(240, 240, 240))
          throw std::runtime_error("Synthetic normal test did not render its measured center point");
        return lit != unlit;
      };
      if (!centerIsShaded()) throw std::runtime_error("Measured grid neighbors did not produce a surface normal");
      normalGrid(1, 0)[2] = normalGrid(1, 2)[2] = std::numeric_limits<float>::quiet_NaN();
      if (centerIsShaded()) throw std::runtime_error("Surface normal crossed missing grid neighbors");
      normalGrid(1, 0)[2] = normalGrid(1, 2)[2] = 1100;
      if (centerIsShaded()) throw std::runtime_error("Surface normal crossed a depth discontinuity");
      normalGrid(1, 0)[2] = normalGrid(1, 2)[2] = 1000;
      application.renderer.setCloud(makePointCloud(normalGrid, normalShading));
      application.settings.rendering.surfaceShading = false;
      const auto measuredGrid = renderedImage();
      const auto defaultSolidColor = application.settings.rendering.solidColor;
      application.settings.rendering.colorMode = PointCloudRenderer::ColorMode::Solid;
      application.settings.rendering.solidColor = {1, 0.25f, 0.5f};
      const auto solidGrid = renderedImage();
      for (int row = 0; row < solidGrid.rows; ++row)
        for (int column = 0; column < solidGrid.cols; ++column) {
          const auto measured = measuredGrid.at<cv::Vec3b>(row, column);
          const auto solid = solidGrid.at<cv::Vec3b>(row, column);
          if (measured == background) {
            if (solid != background) throw std::runtime_error("Solid coloring changed point-cloud coverage");
          } else if (std::abs(static_cast<int>(solid[0]) - 255) > 1 ||
                     std::abs(static_cast<int>(solid[1]) - 64) > 1 ||
                     std::abs(static_cast<int>(solid[2]) - 128) > 1)
            throw std::runtime_error("Solid coloring did not use the selected RGB color for every point");
        }
      application.settings.rendering.surfaceShading = true;
      if (renderedImage().at<cv::Vec3b>(300, 450) == solidGrid.at<cv::Vec3b>(300, 450))
        throw std::runtime_error("Surface shading did not apply to solid coloring");
      cv::FileStorage renderSettings("settings.yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
      application.serializeSettings(renderSettings, false);
      const auto serializedRenderSettings = renderSettings.releaseAndGetString();
      auto readRenderSettings = [&](const std::string &serialized) {
        cv::FileStorage storage(serialized, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        application.serializeSettings(storage, true);
      };
      application.settings.rendering.colorMode = PointCloudRenderer::ColorMode::Modulation;
      application.settings.rendering.solidColor = defaultSolidColor;
      readRenderSettings(serializedRenderSettings);
      if (application.settings.rendering.colorMode != PointCloudRenderer::ColorMode::Solid ||
          application.settings.rendering.solidColor != cv::Vec3f(1, 0.25f, 0.5f))
        throw std::runtime_error("Color mode and solid color did not survive settings serialization");
      readRenderSettings("%YAML:1.0\ncolorMode: 1\n");
      if (application.settings.rendering.colorMode != PointCloudRenderer::ColorMode::Depth)
        throw std::runtime_error("Depth coloring settings were lost");
      readRenderSettings("%YAML:1.0\ncolorMode: 0\n");
      if (application.settings.rendering.colorMode != PointCloudRenderer::ColorMode::Modulation)
        throw std::runtime_error("Modulation coloring settings were lost");
      readRenderSettings("%YAML:1.0\ncolorMode: 2\n");
      if (application.settings.rendering.colorMode != PointCloudRenderer::ColorMode::Solid)
        throw std::runtime_error("Solid coloring settings were lost");
      for (const char *invalid : {"colorMode: 3", "solidColor: [1, -1, 0]", "solidColor: [1, 2, 0]",
           "solidColor: [1, .Nan, 0]", "solidColor: [1, 0]"}) {
        bool rejected = false;
        try { readRenderSettings(std::string("%YAML:1.0\n") + invalid + "\n"); }
        catch (const std::runtime_error &) { rejected = true; }
        if (!rejected) throw std::runtime_error("Invalid saved color settings were accepted");
      }
      readRenderSettings("%YAML:1.0\ngeometricCalibrationInterpolation: rbf\n");
      if (application.settings.geometricCalibrationSettings.interpolation != GeometricCalibrationInterpolation::RBF)
        throw std::runtime_error("Saved RBF calibration method was not loaded");
      cv::FileStorage geometricCalibrationSettings("settings.yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
      application.serializeSettings(geometricCalibrationSettings, false);
      const auto serializedGeometricCalibrationSettings = geometricCalibrationSettings.releaseAndGetString();
      readRenderSettings("%YAML:1.0\ngeometricCalibrationInterpolation: homography\n");
      if (application.settings.geometricCalibrationSettings.interpolation != GeometricCalibrationInterpolation::LocalHomography)
        throw std::runtime_error("Saved homography calibration method was not loaded");
      readRenderSettings("%YAML:1.0\n");
      if (application.settings.geometricCalibrationSettings.interpolation != GeometricCalibrationInterpolation::LocalHomography)
        throw std::runtime_error("Older settings changed the default calibration method");
      readRenderSettings(serializedGeometricCalibrationSettings);
      if (application.settings.geometricCalibrationSettings.interpolation != GeometricCalibrationInterpolation::RBF)
        throw std::runtime_error("RBF calibration method did not survive settings serialization");
      bool rejectedInterpolation = false;
      try { readRenderSettings("%YAML:1.0\ngeometricCalibrationInterpolation: unknown\n"); }
      catch (const std::runtime_error &) { rejectedInterpolation = true; }
      if (!rejectedInterpolation) throw std::runtime_error("Unknown saved calibration method was accepted");
      application.settings.geometricCalibrationSettings.interpolation = GeometricCalibrationInterpolation::LocalHomography;
      application.settings.rendering.colorMode = PointCloudRenderer::ColorMode::Modulation;
      application.settings.rendering.solidColor = defaultSolidColor;
      application.renderer.setCloud(application.cloud);
      application.settings.rendering.surfaceShading = true;
      application.geometricCalibrationDialog.open = geometricCalibrationSmoke;
      if (projectorSmoke) {
        struct ProgrammingSmokeProjector : IntensitySmokeProjector {
          std::shared_future<void> release;
          std::promise<void> uploading;
          bool fail = false;
          ProjectorCapabilities capabilities() const override { return {false, false, true}; }
          void programPatterns(const std::vector<cv::Mat> &, const ProjectorProgrammingProgress &progress) override {
            progress(0.5f, "Uploading pattern data");
            uploading.set_value();
            release.wait();
            if (fail) throw std::runtime_error("Simulated projector upload failure");
            progress(1, "Programming complete");
          }
        };
        const int originalWidth = application.settings.projectorWidth, originalHeight = application.settings.projectorHeight;
        application.settings.projectorWidth = application.settings.projectorHeight = 0;
        for (bool fail : {false, true}) {
          std::promise<void> release;
          auto output = std::make_unique<ProgrammingSmokeProjector>();
          output->release = release.get_future().share();
          output->fail = fail;
          auto uploading = output->uploading.get_future();
          replaceProjector(std::move(output));
          application.startProjectorProgramming(true);
          if (uploading.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
            throw std::runtime_error("Simulated projector upload did not start");
          {
            std::lock_guard<std::mutex> lock(application.projectorPatternUploadDialog.progress->mutex);
            if (application.projectorPatternUploadDialog.progress->fraction != 0.5f ||
                application.projectorPatternUploadDialog.progress->stage != "Uploading pattern data")
              throw std::runtime_error("Projector upload progress did not reach the UI");
          }
          bool clicked = false;
          auto progressFrame = [&](bool press = false, bool escape = false) {
            gui.prepareFrame();
            auto &io = ImGui::GetIO();
            io.AddMousePosEvent(50, 65);
            io.AddMouseButtonEvent(ImGuiMouseButton_Left, press);
            io.AddKeyEvent(ImGuiKey_Escape, escape);
            ImGui::NewFrame();
            ImGui::SetNextWindowPos(ImVec2(20, 20));
            ImGui::SetNextWindowSize(ImVec2(200, 120));
            ImGui::Begin("Upload blocking test", nullptr, ImGuiWindowFlags_NoSavedSettings);
            clicked |= ImGui::Button("Underlying action", ImVec2(150, 45));
            ImGui::End();
            application.projectorPatternUploadDialog.draw(application.projectorJob.valid());
            const bool open = ImGui::IsPopupOpen("Uploading projector patterns");
            ImGui::Render();
            return open;
          };
          progressFrame();
          progressFrame();
          progressFrame(true, true);
          if (!progressFrame() || clicked)
            throw std::runtime_error("Projector upload modal allowed dismissal or interaction with underlying controls");
          release.set_value();
          application.projectorJob.wait();
          application.attempt([&] { application.tick(); }, ApplicationController::FailurePolicy::StopAcquisition);
          awaitAcquisition();
          if (progressFrame() || application.projectorPatternUploadDialog.progress)
            throw std::runtime_error("Projector upload modal did not close after completion");
          if (application.status != (fail ? "Simulated projector upload failure" : "Projector patterns programmed"))
            throw std::runtime_error("Projector upload did not report completion or failure in the status bar");
          replaceProjector({});
        }
        application.settings.projectorWidth = originalWidth;
        application.settings.projectorHeight = originalHeight;
        {
          LightCrafter4500Window timingWindow;
          timingWindow.open = true;
          ApplicationSettings timingSettings;
          int applied = 0;
          int refreshed = 0;
          bool connected = false, busy = false, owned = false;
          std::optional<slstudio::LightCrafter4500Status> playback;
          std::string playbackError;
          std::vector<slstudio::LightCrafter4500Action> actions;
          ImVec2 pointer(-100, -100);
          bool mouseDown = false;
          auto timingFrame = [&] {
            gui.prepareFrame();
            auto &io = ImGui::GetIO();
            io.AddMousePosEvent(pointer.x, pointer.y);
            io.AddMouseButtonEvent(ImGuiMouseButton_Left, mouseDown);
            ImGui::NewFrame();
            timingWindow.draw(timingSettings, true, connected, {}, false, "", busy, owned,
                [&] { ++applied; }, [] {}, playback, playbackError,
                [&](auto action) { actions.push_back(action); }, [&] { ++refreshed; });
            ImGui::Render();
          };
          auto scrollTiming = [&](bool bottom) {
            auto *window = ImGui::FindWindowByName("LightCrafter 4500 Settings");
            if (!window) throw std::runtime_error("LC4500 settings window was not opened");
            ImGui::SetScrollY(window, bottom ? window->ScrollMax.y : 0);
            timingFrame();
            timingFrame();
          };
          auto clickApply = [&] {
            scrollTiming(true);
            const auto *window = ImGui::FindWindowByName("LightCrafter 4500 Settings");
            if (!window) throw std::runtime_error("LC4500 timing validation window was not opened");
            const float width = ImGui::CalcTextSize("Apply settings").x + ImGui::GetStyle().FramePadding.x * 2;
            pointer = ImVec2(window->DC.CursorPosPrevLine.x - width * 0.5f,
                window->DC.CursorPosPrevLine.y + ImGui::GetFrameHeight() * 0.5f);
            timingFrame();
            mouseDown = true;
            timingFrame();
            mouseDown = false;
            timingFrame();
            pointer = ImVec2(-100, -100);
          };
          for (const int darkTime : {230, 231}) {
            timingSettings.lc4500Settings.exposure = std::chrono::microseconds(8333);
            timingSettings.lc4500Settings.framePeriod = std::chrono::microseconds(8333 + darkTime);
            timingFrame();
            timingFrame();
            clickApply();
            if (applied != (darkTime == 231 ? 1 : 0))
              throw std::runtime_error("LC4500 Apply settings did not enforce dark time " +
                  std::to_string(darkTime) + " us; applied " + std::to_string(applied) + " times");
          }
          timingSettings.lc4500Settings.exposure = timingSettings.lc4500Settings.framePeriod =
              std::chrono::microseconds(8332);
          timingFrame();
          timingFrame();
          clickApply();
          if (applied != 1)
            throw std::runtime_error("LC4500 Apply settings accepted a sub-minimum eight-bit exposure");
          auto &hdmiSettings = timingSettings.lc4500Settings;
          hdmiSettings.displayMode = slstudio::LightCrafter4500Mode::FlashPatterns;
          hdmiSettings.exposure = std::chrono::microseconds(12000);
          hdmiSettings.framePeriod = std::chrono::microseconds(16000);
          hdmiSettings.ledSelection = 4;
          hdmiSettings.ledCurrent = {13, 27, 45};
          hdmiSettings.flipLongAxis = hdmiSettings.flipShortAxis = true;
          timingFrame();
          timingFrame();
          scrollTiming(false);
          auto *window = ImGui::FindWindowByName("LightCrafter 4500 Settings");
          ImGui::ActivateItemByID(window->GetID("Display mode"));
          timingFrame();
          timingFrame();
          if (GImGui->OpenPopupStack.empty() || !GImGui->OpenPopupStack.back().Window)
            throw std::runtime_error("LC4500 display mode selector did not open");
          const auto *popup = GImGui->OpenPopupStack.back().Window;
          pointer = ImVec2(popup->DC.CursorStartPos.x + 20,
              popup->DC.CursorStartPos.y + ImGui::GetTextLineHeight() * 0.5f);
          timingFrame();
          mouseDown = true;
          timingFrame();
          mouseDown = false;
          timingFrame();
          pointer = ImVec2(-100, -100);
          if (hdmiSettings.displayMode != slstudio::LightCrafter4500Mode::HdmiPatterns ||
              hdmiSettings.exposure != std::chrono::microseconds(8333) ||
              hdmiSettings.framePeriod != std::chrono::microseconds(8333) ||
              hdmiSettings.ledSelection != 4 || hdmiSettings.ledCurrent != std::array<int, 3>{13, 27, 45} ||
              !hdmiSettings.flipLongAxis || !hdmiSettings.flipShortAxis || applied != 1)
            throw std::runtime_error("Selecting HDMI patterns must restore timing defaults without changing LEDs or applying settings");
          hdmiSettings.exposure = std::chrono::microseconds(12000);
          hdmiSettings.framePeriod = std::chrono::microseconds(16000);
          timingFrame();
          cv::FileStorage writer("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
          timingSettings.write(writer);
          cv::FileStorage reader(writer.releaseAndGetString(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
          const auto restored = ApplicationSettings{}.read(reader).lc4500Settings;
          if (restored.exposure != std::chrono::microseconds(12000) ||
              restored.framePeriod != std::chrono::microseconds(16000) ||
              restored.ledSelection != 4 || restored.ledCurrent != hdmiSettings.ledCurrent)
            throw std::runtime_error("Drawing HDMI settings or saving them reset customized timing or LEDs");
          auto clickPlayback = [&](const char *button) {
            const auto beforeApply = applied;
            scrollTiming(true);
            const auto *window = ImGui::FindWindowByName("LightCrafter 4500 Settings");
            const auto &style = ImGui::GetStyle();
            float x = window->DC.CursorStartPos.x;
            for (const auto *label : {"Start", "Pause", "Stop", "Blank", "Refresh status"}) {
              const float width = ImGui::CalcTextSize(label).x + style.FramePadding.x * 2;
              if (std::string(label) == button) {
                pointer = ImVec2(x + width * 0.5f, window->DC.CursorPosPrevLine.y -
                    ImGui::GetFrameHeight() * 0.5f - style.ItemSpacing.y);
                break;
              }
              x += width + style.ItemSpacing.x;
            }
            timingFrame();
            mouseDown = true;
            timingFrame();
            mouseDown = false;
            timingFrame();
            pointer = ImVec2(-100, -100);
            if (applied != beforeApply)
              throw std::runtime_error(std::string("Playback click unexpectedly applied settings: ") + button +
                  ", busy=" + std::to_string(busy) + ", owned=" + std::to_string(owned) +
                  ", connected=" + std::to_string(connected));
          };
          connected = true;
          playback = slstudio::LightCrafter4500Status{};
          playback->patternMode = true;
          playback->ledsDisabled = true;
          playback->validationReady = true;
          timingFrame();
          clickPlayback("Start");
          clickPlayback("Pause");
          clickPlayback("Stop");
          if (actions != std::vector{slstudio::LightCrafter4500Action::Start} || applied != 1)
            throw std::runtime_error("Stopped playback must allow Start without applying edited settings");
          playback->playback = slstudio::LightCrafter4500Playback::Running;
          playback->ledsDisabled = false;
          clickPlayback("Start");
          clickPlayback("Pause");
          clickPlayback("Stop");
          clickPlayback("Blank");
          clickPlayback("Refresh status");
          if (actions != std::vector{slstudio::LightCrafter4500Action::Start,
                  slstudio::LightCrafter4500Action::Pause, slstudio::LightCrafter4500Action::Stop,
                  slstudio::LightCrafter4500Action::Blank} || refreshed != 1 || applied != 1)
            throw std::runtime_error("Running playback buttons or independent status refresh failed");
          playback->playback = slstudio::LightCrafter4500Playback::Paused;
          clickPlayback("Start");
          if (actions.size() != 5 || actions.back() != slstudio::LightCrafter4500Action::Start)
            throw std::runtime_error("Paused playback did not allow resume");
          for (int disabled = 0; disabled < 5; ++disabled) {
            busy = disabled == 0;
            owned = disabled == 1;
            connected = disabled != 2;
            playback->standby = disabled == 3;
            playback->validationFlags = disabled == 4 ? 1u : 0u;
            clickPlayback("Start");
            clickPlayback("Pause");
            if (disabled < 3) {
              clickPlayback("Stop");
              clickPlayback("Blank");
              clickPlayback("Refresh status");
            }
          }
          if (actions.size() != 5 || refreshed != 1)
            throw std::runtime_error("Disabled LC4500 playback controls accepted mouse input");
          playback->validationFlags = 0;
          playback->patternMode = false;
          clickPlayback("Start");
          clickPlayback("Pause");
          clickPlayback("Stop");
          clickPlayback("Blank");
          if (actions.size() != 6 || actions.back() != slstudio::LightCrafter4500Action::Blank)
            throw std::runtime_error("Video mode must allow Blank but not pattern playback controls");
          playback.reset();
          playbackError = "Simulated LC4500 status query failure";
          clickPlayback("Start");
          clickPlayback("Refresh status");
          if (actions.size() != 6 || refreshed != 2 || applied != 1)
            throw std::runtime_error("Unknown playback state must allow status retry without starting or applying: actions=" +
                std::to_string(actions.size()) + ", refreshed=" + std::to_string(refreshed) +
                ", applied=" + std::to_string(applied));
        }
        application.settings.lc4500Settings.displayMode = slstudio::LightCrafter4500Mode::TestPattern;
        application.settings.lc4500Settings.testPattern = 9;
        cv::FileStorage settings("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
        application.serializeSettings(settings, false);
        application.settings.lc4500Settings.displayMode = slstudio::LightCrafter4500Mode::HdmiPatterns;
        application.settings.lc4500Settings.testPattern = 7;
        cv::FileStorage restored(settings.releaseAndGetString(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
        application.serializeSettings(restored, true);
        if (application.settings.lc4500Settings.displayMode != slstudio::LightCrafter4500Mode::TestPattern ||
            application.settings.lc4500Settings.testPattern != 9)
          throw std::runtime_error("LC4500 display mode and test pattern were not restored from settings");
        application.settings.lc4500Settings.displayMode = slstudio::LightCrafter4500Mode::HdmiPatterns;
        application.settings.lc4500Settings.testPattern = 7;
        application.lc3000Window.open = true;
        slstudio::validateLightCrafterSettings(application.settings.lc3000Settings);
        slstudio::validateLightCrafterSettings(application.settings.lc4500Settings);
        replaceProjector(std::make_unique<IntensitySmokeProjector>());
        for (const auto backend : {OutputBackend::LightCrafter3000, OutputBackend::LightCrafter4500}) {
          bool rejected = false;
          try { application.applyLightCrafterSettings(backend); }
          catch (const std::logic_error &) { rejected = true; }
          if (!rejected || application.projectorJob.valid())
            throw std::runtime_error("LightCrafter settings accepted an open projector output");
        }
        for (const auto action : {slstudio::LightCrafter4500Action::Start, slstudio::LightCrafter4500Action::Pause,
            slstudio::LightCrafter4500Action::Stop, slstudio::LightCrafter4500Action::Blank}) {
          bool rejected = false;
          try { application.controlLightCrafter4500(action); }
          catch (const std::logic_error &) { rejected = true; }
          if (!rejected || application.lc4500PlaybackJob.valid())
            throw std::runtime_error("LC4500 manual playback accepted an acquisition-owned projector");
        }
        replaceProjector({});
        awaitAcquisition();
        std::promise<slstudio::LightCrafter4500Status> playbackResult;
        application.lc4500PlaybackJob = playbackResult.get_future();
        if (!application.processing())
          throw std::runtime_error("LC4500 status queries must participate in USB job exclusion");
        bool queryBlockedOpen = false;
        try { application.openOutput(); }
        catch (const std::logic_error &) { queryBlockedOpen = true; }
        if (!queryBlockedOpen)
          throw std::runtime_error("Opening a projector must wait for in-flight LC4500 queries");
        for (const bool applySettings : {false, true}) {
          bool rejected = false;
          try {
            if (applySettings) application.applyLightCrafterSettings(OutputBackend::LightCrafter4500);
            else application.controlLightCrafter4500(slstudio::LightCrafter4500Action::Blank);
          } catch (const std::logic_error &) { rejected = true; }
          if (!rejected) throw std::runtime_error("LC4500 commands must not overlap status queries");
        }
        slstudio::LightCrafter4500Status actualPlayback;
        actualPlayback.patternMode = true;
        actualPlayback.playback = slstudio::LightCrafter4500Playback::Paused;
        playbackResult.set_value(actualPlayback);
        application.tick();
        if (application.lc4500PlaybackJob.valid() || !application.lc4500Playback ||
            application.lc4500Playback->playback != slstudio::LightCrafter4500Playback::Paused)
          throw std::runtime_error("LC4500 async status result was not cached");
        std::promise<slstudio::LightCrafter4500Status> failedPlayback;
        application.lc4500PlaybackJob = failedPlayback.get_future();
        failedPlayback.set_exception(std::make_exception_ptr(std::runtime_error("Simulated playback USB failure")));
        application.tick();
        if (application.lc4500PlaybackJob.valid() || application.lc4500Playback ||
            application.lc4500PlaybackError != "Simulated playback USB failure" ||
            application.status != application.lc4500PlaybackError)
          throw std::runtime_error("LC4500 async status errors must invalidate cached state and report failure");
        application.lc4500PlaybackError.clear();
        application.status = "LightCrafter settings preview";
      } else if (intensityCalibrationSmoke) {
        auto output = std::make_unique<IntensitySmokeProjector>();
        intensityOutput = output.get();
        intensityReader = [level = output->level] {
          return cv::Mat(240, 320, CV_8UC1, cv::Scalar(std::lround(12 + 210 * std::pow(level->load() / 255.0, 2.2))));
        };
        replaceProjector(std::move(output));
        application.intensityCalibration.open = true;
        application.settings.cameraWidth = 320;
        application.settings.cameraHeight = 240;
        application.settings.autoExposure = false;
        application.settings.settleMilliseconds = 0;
        application.setCameraPreviewFrameCount(3);
        application.startCamera(intensityReader);
        const auto base = std::filesystem::temp_directory_path() /
            ("slstudio-intensity-" + std::to_string(Clock::now().time_since_epoch().count()));
        temporaryIntensityYaml.path = base.string() + ".yml";
        temporaryIntensityCsv.path = base.string() + ".csv";
      } else if (geometricCalibrationSmoke) {
        application.settings.geometricCalibrationSettings.boardCorners = cv::Size(9, 13);
        application.settings.geometricCalibrationSettings.interpolation = GeometricCalibrationInterpolation::RBF;
        const cv::Size configuredCamera(application.settings.cameraWidth, application.settings.cameraHeight);
        application.loadGeometricCalibrationPoses(std::filesystem::path(SLSTUDIO_SOURCE_DIR) / "example_data/calibration");
        if (configuredCamera != cv::Size(application.settings.cameraWidth, application.settings.cameraHeight))
          throw std::runtime_error("Loading calibration poses changed the live camera settings");
        temporaryCalibration.path = application.dataPath("calibration.xml");
        application.startGeometricCalibration();
      } else {
        smokeReference = application.cloud;
        application.frames = loadFrameSequence(application.settings.sequencePath.data());
        if (slstudio::Ros2Output::available())
          application.ros2Output = std::make_unique<slstudio::Ros2Output>(slstudio::Ros2OutputSettings{});
        application.reconstruct();
      }
    }
    int renderedFrames = 0;
    const bool cameraWindowSmoke = !geometricCalibrationSmoke && !intensityCalibrationSmoke && !projectorSmoke;
    int cameraWindowSmokeFrame = 0;
    int interfaceSmokeFrame = 0;
    std::optional<ReconstructionSettings> sequenceLoadSettings;
    bool interfaceReviewed = !cameraWindowSmoke;
    bool ros2OutputReviewed = !cameraWindowSmoke || !slstudio::Ros2Output::available();
    bool ros2FailureSubmitted = false;
    ImVec2 calibrationLoadButton;
    int renderPanelVisibleVertices = 0, renderPanelHiddenVertices = 0;
    bool calibrationReviewed = false;
    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();
      application.attempt([&] { application.tick(); }, ApplicationController::FailurePolicy::StopAcquisition);
      if (!ros2OutputReviewed && application.cloud && application.cloud != smokeReference) {
        if (application.ros2Output && !ros2FailureSubmitted) {
          application.ros2Output->submit(application.cloud,
              std::chrono::system_clock::time_point(std::chrono::seconds(-1)));
          ros2FailureSubmitted = true;
        }
      }
      if (!ros2OutputReviewed && !application.ros2Output) {
        if (application.ros2OutputError.find("PointCloud2 limits") == std::string::npos ||
            !application.cloud || !application.decoder || !application.triangulator)
          throw std::runtime_error("ROS 2 output failure was hidden or stopped reconstruction");
        ros2OutputReviewed = true;
      }
      if (Clock::now() >= smokeDeadline)
        throw std::runtime_error("GUI worker smoke test timed out: " + application.status);
      if (intensityCalibrationSmoke) {
        if (intensityStage == 0 && !application.cameraPreview.empty()) {
          if (application.intensityCalibration.worker || application.processing())
            throw std::runtime_error("Opening intensity view started a separate preview session");
          ++intensityLiveFrames;
          if (intensityLiveFrames >= 5) {
            if (!application.cameraPreviewTexture.texture())
              throw std::runtime_error("Intensity Calibration view did not automatically display the shared camera feed");
            const std::array<int, 4> expectedRoi{64, 48, 129, 97};
            // ImGui quantizes pointer coordinates to whole display pixels.
            for (std::size_t index = 0; index < expectedRoi.size(); ++index)
              if (std::abs(application.intensityCalibration.roi[index] - expectedRoi[index]) > 1)
                throw std::runtime_error("Dragging the intensity ROI did not select the corresponding camera pixels");
            application.startIntensityCalibration(intensityReader);
            if (!application.intensityCalibration.measureRequested || !application.intensityCalibration.worker)
              throw std::runtime_error("Measure did not prepare asynchronous camera acquisition");
            intensityStage = 1;
          }
        } else if (intensityStage == 2 && !application.intensityCalibration.preview.empty() && !application.intensityCalibration.frameRequested) {
          if (!application.processing()) throw std::runtime_error("Intensity Calibration preview does not guard capture resources");
          bool rejected = false;
          try { application.prepareIntensityCalibration(intensityReader); } catch (const std::runtime_error &) { rejected = true; }
          if (!rejected) throw std::runtime_error("Concurrent intensity preview accepted");
          application.startIntensityCalibration();
          if (intensityStage == 2) application.intensityCalibration.open = false;
          ++intensityStage;
        } else if (intensityStage == 1 && !application.intensityCalibration.worker && !application.processing()) {
          if (!application.intensityCalibration.result || application.intensityCalibration.result->linearityRmsError < 10 ||
              application.intensityCalibration.samples.size() != 17 || intensityOutput->level->load() != 0 || intensityOutput->blanks != 1)
            throw std::runtime_error("Intensity Calibration acquisition/fit/blanking failed: " + application.status);
          saveIntensityLut(temporaryIntensityYaml.path, application.intensityCalibration.samples, *application.intensityCalibration.result);
          saveIntensityCsv(temporaryIntensityCsv.path, application.intensityCalibration.samples);
          if (application.settings.intensityLut) throw std::runtime_error("Intensity Calibration measurement activated compensation implicitly");
          originalCorrection = *application.intensityCalibration.result;
          const auto loaded = loadIntensitySamples(temporaryIntensityYaml.path);
          if (analyzeIntensity(loaded).inverseLut != originalCorrection.inverseLut)
            throw std::runtime_error("Intensity Calibration reload differs from measurement");
          const auto originalPatterns = application.projectorPatterns(false);
          application.activateMeasuredLut();
          if (!application.settings.intensityCompensationEnabled || !application.settings.intensityLut ||
              application.settings.intensityLut->inverseLut != originalCorrection.inverseLut)
            throw std::runtime_error("Measured intensity activation failed");
          if (analyzeIntensity(loadIntensitySamples(application.dataPath("intensity-lut.yml"))).inverseLut != originalCorrection.inverseLut)
            throw std::runtime_error("Accepted intensity was not persisted beside settings");
          application.intensityCalibration.open = true;
          application.settings.intensityCompensationEnabled = false;
          const auto disabledPatterns = application.projectorPatterns(false);
          for (std::size_t index = 0; index < originalPatterns.size(); ++index)
            if (cv::norm(disabledPatterns[index], originalPatterns[index], cv::NORM_INF) != 0)
              throw std::runtime_error("Disabled intensity compensation still corrects scan patterns");
          application.showProjectorLevel(128);
          awaitAcquisition();
          if (intensityOutput->level->load() != 128)
            throw std::runtime_error("Disabled intensity compensation still corrects solid fields");
          bool replacementRejected = false;
          try { application.activateMeasuredLut(); }
          catch (const std::logic_error &) { replacementRejected = true; }
          if (!replacementRejected || application.settings.intensityCompensationEnabled ||
              application.settings.intensityLut->inverseLut != originalCorrection.inverseLut)
            throw std::runtime_error("Stored projector/camera intensity curve was replaced without confirmation");
          application.activateMeasuredLut(true);
          if (!application.settings.intensityCompensationEnabled) throw std::runtime_error("Confirmed intensity curve replacement failed");
          application.intensityCalibration.open = true;
          if (loadIntensityLut(temporaryIntensityYaml.path).inverseLut != originalCorrection.inverseLut)
            throw std::runtime_error("Stored projector/camera intensity curve differs from the accepted curve");
          const auto correctedPatterns = application.projectorPatterns(false);
          double difference = 0;
          for (std::size_t index = 0; index < originalPatterns.size(); ++index) {
            if (cv::norm(correctedPatterns[index], compensateIntensity(originalPatterns[index], originalCorrection), cv::NORM_INF) != 0)
              throw std::runtime_error("Scan patterns do not use intensity compensation");
            difference += cv::norm(correctedPatterns[index], originalPatterns[index], cv::NORM_INF);
          }
          if (difference == 0) throw std::runtime_error("Intensity compensation did not change nonlinear scan patterns");
          cv::FileStorage stored(temporaryIntensityYaml.path.string(), cv::FileStorage::READ);
          cv::Mat lut;
          stored["inverse_lut"] >> lut;
          if (lut.type() != CV_8UC1 || lut.total() != 256 || stored["samples"].size() != 17 ||
              std::filesystem::file_size(temporaryIntensityCsv.path) < 100)
            throw std::runtime_error("Intensity Calibration export roundtrip failed");
          intensityCalibrationSmokePreview = application.intensityCalibration.preview;
          application.prepareIntensityCalibration(intensityReader);
          intensityStage = 6;
        } else if (intensityStage == 6 && !application.intensityCalibration.preview.empty() && !application.intensityCalibration.frameRequested) {
          application.startIntensityCalibration();
          intensityStage = 7;
        } else if (intensityStage == 7 && !application.intensityCalibration.worker && !application.processing()) {
          if (!application.intensityCalibration.result || application.intensityCalibration.result->linearityRmsError > 1.5 ||
              application.intensityCalibration.result->linearityRmsError >= originalCorrection.linearityRmsError * 0.15 ||
              !application.settings.intensityLut || application.settings.intensityLut->inverseLut != originalCorrection.inverseLut ||
              application.settings.intensityLut->linearityRmsError != originalCorrection.linearityRmsError ||
              intensityOutput->level->load() != 0 || intensityOutput->blanks != 2)
            throw std::runtime_error("Compensated verification failed to improve linearity or changed the stored intensity curve");
          verifiedIntensity = application.intensityCalibration.result;
          verificationSamples = application.intensityCalibration.samples;
          application.prepareIntensityCalibration(intensityReader);
          intensityStage = 2;
        } else if (intensityStage == 3 && !application.intensityCalibration.worker && !application.processing()) {
          if (application.intensityCalibration.measuring || intensityOutput->level->load() != 0 || intensityOutput->blanks != 3 ||
            !application.settings.intensityLut || application.settings.intensityLut->inverseLut != originalCorrection.inverseLut)
            throw std::runtime_error("Closing intensity measurement failed to cancel and blank");
          application.intensityCalibration.open = true;
          application.startIntensityCalibration([]() -> cv::Mat { throw std::runtime_error("Synthetic intensity capture failure"); });
          intensityStage = 4;
        } else if (intensityStage == 4 && !application.intensityCalibration.worker && !application.processing()) {
          if (intensityOutput->level->load() != 0 || intensityOutput->blanks != 4 ||
              application.status != "Synthetic intensity capture failure" || application.intensityCalibration.measureRequested)
            throw std::runtime_error("Intensity Calibration error cleanup failed");
          application.intensityCalibration.preview = intensityCalibrationSmokePreview;
          application.appendCameraPreview(intensityCalibrationSmokePreview);
          application.intensityCalibration.samples = verificationSamples;
          application.intensityCalibration.result = verifiedIntensity;
          application.intensityCalibration.compensated = true;
          application.status = "Intensity compensation verified; stored projector/camera intensity curve preserved";
          intensityStage = 5;
          renderedFrames = 0;
        }
      }
      if (!geometricCalibrationSmoke && !intensityCalibrationSmoke && !projectorSmoke && !application.processing() && !reconstructionReviewed) {
        if (!application.decoder || !application.triangulator || application.cloud == smokeReference ||
            application.cloud->points.size() != smokeReference->points.size())
          throw std::runtime_error("GUI reconstruction pipeline failed: " + application.status);
        for (std::size_t index = 0; index < smokeReference->points.size(); ++index) {
          const auto &expected = smokeReference->points[index];
          const auto &actual = application.cloud->points[index];
          if (actual.x != expected.x || actual.y != expected.y || actual.z != expected.z ||
              actual.r != expected.r || actual.g != expected.g || actual.b != expected.b)
            throw std::runtime_error("GUI pipeline output differs from synchronous reconstruction");
        }
        reconstructionReviewed = true;
      }
      if (geometricCalibrationSmoke && !application.processing() && !calibrationReviewed) {
        if (!application.pendingGeometricCalibration || application.geometricCalibrationDiagnostics.size() != 6)
          throw std::runtime_error("Geometric Calibration smoke test failed: " + application.status);
        if (application.settings.geometricCalibrationPath.data() != initialCalibrationPath || std::filesystem::exists(temporaryCalibration.path))
          throw std::runtime_error("Geometric Calibration was activated or saved before review");
        const auto accepted = std::find_if(application.geometricCalibrationDiagnostics.begin(), application.geometricCalibrationDiagnostics.end(),
                                           [](const auto &pose) { return pose.accepted; });
        if (accepted == application.geometricCalibrationDiagnostics.end()) throw std::runtime_error("No accepted calibration pose");
        application.geometricCalibrationDialog.selectedPose =
            static_cast<int>(accepted - application.geometricCalibrationDiagnostics.begin());
        application.geometricCalibrationDialog.invalidatePreview();
        const auto liveFrame = application.geometricCalibrationPoses[
            application.geometricCalibrationDialog.selectedPose].front();
        application.setCameraPreviewFrameCount(3);
        application.startCamera([liveFrame] { return liveFrame; });
        application.status = "Geometric Calibration ready for review";
        calibrationReviewed = true;
        renderedFrames = 0;
      }
      gui.prepareFrame();
      if (intensityCalibrationSmoke && intensityStage == 0) {
        if (intensityLiveFrames >= 2 && intensityLiveFrames <= 3) {
          const float scale = (intensityImageMax.x - intensityImageMin.x) / application.settings.cameraWidth;
          if (scale <= 0) throw std::runtime_error("Intensity Calibration camera image was not rendered");
          auto &io = ImGui::GetIO();
          io.AddMousePosEvent(intensityImageMin.x + (intensityLiveFrames == 2 ? 64.5f : 192.5f) * scale,
              intensityImageMin.y + (intensityLiveFrames == 2 ? 48.5f : 144.5f) * scale);
          if (intensityLiveFrames == 2) io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
        } else if (intensityLiveFrames == 4) {
          ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, false);
        }
      }
      const bool testCameraThisFrame = cameraWindowSmoke && reconstructionReviewed && interfaceReviewed;
      if (cameraWindowSmoke && reconstructionReviewed && !interfaceReviewed && interfaceSmokeFrame == 0)
        application.openFilePicker(FilePickerMode::GeometricCalibration);
      if (cameraWindowSmoke && !interfaceReviewed &&
          (interfaceSmokeFrame == 5 || interfaceSmokeFrame == 6 || interfaceSmokeFrame == 9 ||
           interfaceSmokeFrame == 10 || interfaceSmokeFrame == 13 || interfaceSmokeFrame == 14)) {
        ImGui::GetIO().AddMousePosEvent(calibrationLoadButton.x, calibrationLoadButton.y);
        ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left,
            interfaceSmokeFrame == 5 || interfaceSmokeFrame == 9 || interfaceSmokeFrame == 13);
      }
      if (testCameraThisFrame && cameraWindowSmokeFrame < 5) {
        auto &io = ImGui::GetIO();
        application.frameSequenceWindow.open = true;
        if (cameraWindowSmokeFrame == 0) {
          application.lastSequence.reset();
        } else if (cameraWindowSmokeFrame == 1) {
          if (application.cameraWorker || application.cameraRequested)
            throw std::runtime_error("Opening the empty Frame sequence window started the camera");
          std::vector<cv::Mat> sequence;
          for (int index = 0; index < 3; ++index)
            sequence.emplace_back(240, 320, CV_8UC1, cv::Scalar(17 + index * 16));
          application.setLastSequence(FrameSequence{sequence, application.settings.reconstruction.coding});
          ImGui::SetWindowPos("###FrameSequence", ImVec2(420, 80));
          io.AddMousePosEvent(1200, 700);
          io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
        } else if (cameraWindowSmokeFrame == 2) {
          io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
        } else if (cameraWindowSmokeFrame >= 3) {
          io.AddMousePosEvent(500, 100);
        }
      }
      ImGui::NewFrame();
      if (geometricCalibrationSmoke && renderedFrames == 9) {
        ImGui::SetWindowSize("Geometric Calibration", ImVec2(560, 550));
        ImGui::SetWindowPos("Geometric Calibration", ImVec2(170, 25));
      }
      if (intensityCalibrationSmoke && intensityStage == 5 && renderedFrames == 5) {
        ImGui::SetWindowSize("Intensity Calibration", ImVec2(560, 560));
        ImGui::SetWindowPos("Intensity Calibration", ImVec2(170, 20));
      }
      if (intensityCalibrationSmoke && intensityStage == 5 && renderedFrames == 6) {
        application.intensityCurveWindow.open = true;
        application.settings.intensityCompensationEnabled = false;
      }
      if (projectorSmoke && renderedFrames >= 12 && renderedFrames <= 15) {
        const slstudio::LightCrafter4500Mode modes[] = {slstudio::LightCrafter4500Mode::FlashPatterns,
            slstudio::LightCrafter4500Mode::HdmiVideo, slstudio::LightCrafter4500Mode::TestPattern,
            slstudio::LightCrafter4500Mode::HdmiPatterns};
        application.settings.lc4500Settings.displayMode = modes[renderedFrames - 12];
      }
      application.draw();
      if (intensityCalibrationSmoke && intensityStage == 5 && renderedFrames == 6) {
        ImGui::Begin("Projector/camera intensity curve");
        const auto size = ImGui::GetWindowSize();
        const float overflow = ImGui::GetScrollMaxY();
        ImGui::End();
        if (size.x != 560 || size.y != 280 || overflow > 1 ||
            application.settings.intensityLut->inverseLut != originalCorrection.inverseLut ||
            application.settings.intensityCompensationEnabled || application.intensityCalibration.worker)
          throw std::runtime_error("Stored intensity curve viewer changed compensation or did not retain its compact read-only layout");
        application.intensityCurveWindow.open = false;
        application.settings.intensityCompensationEnabled = true;
      }
      if (intensityCalibrationSmoke && intensityStage == 5 && renderedFrames >= 6) {
        const auto *calibrationWindow = ImGui::FindWindowByName("Intensity Calibration");
        if (!calibrationWindow) throw std::runtime_error("Intensity Calibration modal was not opened");
        const auto size = calibrationWindow->Size;
        const float overflow = calibrationWindow->ScrollMax.y;
        if (size.x != 560 || size.y != 560 || overflow > 1 || application.intensityCalibration.worker ||
            application.intensityCalibration.measureRequested)
          throw std::runtime_error("Intensity Calibration review did not retain its compact layout and completed state");
      }
      if (geometricCalibrationSmoke && calibrationReviewed && application.geometricCalibrationDialog.open && renderedFrames >= 10) {
        const auto *calibrationWindow = ImGui::FindWindowByName("Geometric Calibration");
        if (!calibrationWindow) throw std::runtime_error("Geometric Calibration modal was not opened");
        const float overflow = calibrationWindow->ScrollMax.y;
        const auto size = calibrationWindow->Size;
        if (size.x != 560 || size.y != 550)
          throw std::runtime_error("Geometric Calibration smoke test did not exercise the compact dialog size");
        if (overflow > 1) throw std::runtime_error("Single-page calibration overflowed by " +
            std::to_string(overflow) + " pixels at frame " + std::to_string(renderedFrames));
      }
      if (cameraWindowSmoke && application.settings.renderSettingsOpen) {
        ImGui::Begin("Render Settings");
        const auto size = ImGui::GetWindowSize();
        const auto work = ImGui::GetMainViewport()->WorkSize;
        ImGui::End();
        const float maximumHeight = application.settings.rendering.colorMode == PointCloudRenderer::ColorMode::Solid ? 160 : 135;
        if (size.x != std::min(300.0f, work.x) || size.y > maximumHeight)
          throw std::runtime_error("Render Settings did not keep its fixed width and compact content height");
        if (cameraWindowSmokeFrame == 5)
          ImGui::SetWindowSize("Render Settings", ImVec2(800, 600));
      }
      if (cameraWindowSmoke && application.settings.settingsWindowOpen) {
        ImGui::Begin("Settings");
        const float width = ImGui::GetWindowWidth();
        ImGui::End();
        if (width != std::min(420.0f, ImGui::GetMainViewport()->WorkSize.x))
          throw std::runtime_error("Settings did not retain its fixed width");
        ImGui::SetWindowSize("Settings", ImVec2(800, 600));
      }
      if (testCameraThisFrame && cameraWindowSmokeFrame == 0) {
        ImGui::Begin(application.frameSequenceWindowTitle().c_str());
        const float width = ImGui::GetWindowWidth();
        ImGui::End();
        if (width != 400)
          throw std::runtime_error("Frame sequence did not open at its compact default width");
      }
      if (projectorSmoke) {
        for (const auto &dialog : {std::pair{"LightCrafter 3000 Settings", application.lc3000Window.open},
                                  std::pair{"LightCrafter 4500 Settings", application.lc4500Window.open}}) {
          if (!dialog.second) continue;
          ImGui::Begin(dialog.first);
          const auto size = ImGui::GetWindowSize();
          const auto work = ImGui::GetMainViewport()->WorkSize;
          if (size.x != std::min(440.0f, work.x) || size.y > work.y)
            throw std::runtime_error("LightCrafter dialog did not keep its fixed width and viewport-limited height");
          if ((renderedFrames == 3 || renderedFrames == 6 || renderedFrames == 9) &&
              size.y - ImGui::GetCursorPosY() - ImGui::GetStyle().WindowPadding.y > ImGui::GetFrameHeight())
            throw std::runtime_error("LightCrafter dialog retained unused space below its controls");
          ImGui::End();
          ImGui::SetWindowSize(dialog.first, ImVec2(800, 750));
        }
      }
      if (cameraWindowSmoke && reconstructionReviewed && !interfaceReviewed) {
        if (interfaceSmokeFrame == 0) {
          if (!application.filePickerDialog.state.open ||
              std::find(application.filePickerDialog.state.files.begin(), application.filePickerDialog.state.files.end(),
                  std::filesystem::path(initialCalibrationPath)) == application.filePickerDialog.state.files.end())
            throw std::runtime_error("Geometric Calibration picker did not list the current calibration XML");
        } else if (interfaceSmokeFrame == 1) {
          const auto directory = application.filePickerDialog.state.directory;
          application.browseDirectory(directory.parent_path());
          application.browseDirectory(directory);
          for (const auto &file : application.filePickerDialog.state.files)
            if (file.extension() != ".xml") throw std::runtime_error("Geometric Calibration picker did not filter XML files");
          const auto *oldPipeline = application.decoder.get();
          temporaryInvalidCalibration.path = std::filesystem::temp_directory_path() /
              ("slstudio-invalid-calibration-" + std::to_string(Clock::now().time_since_epoch().count()) + ".xml");
          cv::FileStorage invalid(temporaryInvalidCalibration.path.string(), cv::FileStorage::WRITE);
          if (!invalid.isOpened()) throw std::runtime_error("Cannot create invalid calibration fixture");
          invalid << "not_a_calibration" << 1;
          invalid.release();
          bool rejected = false;
          try { application.loadGeometricCalibrationFile(temporaryInvalidCalibration.path); }
          catch (const std::runtime_error &) { rejected = true; }
          if (!rejected || application.settings.geometricCalibrationPath.data() != initialCalibrationPath || application.decoder.get() != oldPipeline)
            throw std::runtime_error("Invalid calibration selection changed the active settings");
          ImGui::GetIO().AddKeyEvent(ImGuiKey_Escape, true);
        } else if (interfaceSmokeFrame == 2) {
          if (application.filePickerDialog.state.open || application.settings.geometricCalibrationPath.data() != initialCalibrationPath)
            throw std::runtime_error("Cancelling the calibration picker changed the active calibration");
          ImGui::GetIO().AddKeyEvent(ImGuiKey_Escape, false);
          application.openFilePicker(FilePickerMode::GeometricCalibration);
        } else if (interfaceSmokeFrame == 4) {
          GeometricCalibrationData selected;
          temporaryCalibration.path = std::filesystem::temp_directory_path() /
              ("slstudio selected calibration " + std::to_string(Clock::now().time_since_epoch().count()) + ".xml");
          if (!selected.load(initialCalibrationPath) || !selected.save(temporaryCalibration.path))
            throw std::runtime_error("Cannot create calibration picker fixture");
          std::snprintf(application.filePickerDialog.state.filename.data(), application.filePickerDialog.state.filename.size(),
              "%s", temporaryCalibration.path.string().c_str());
          ImGui::Begin("Load Geometric Calibration");
          const auto position = ImGui::GetWindowPos();
          const auto size = ImGui::GetWindowSize();
          calibrationLoadButton = ImVec2(position.x + ImGui::GetStyle().WindowPadding.x + 15,
              position.y + size.y - ImGui::GetFrameHeightWithSpacing() * 2);
          ImGui::End();
        } else if (interfaceSmokeFrame == 6) {
          if (application.filePickerDialog.state.open) throw std::runtime_error("Geometric Calibration picker did not load the selected XML");
          cv::FileStorage saved("settings.yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
          application.serializeSettings(saved, false);
          cv::FileStorage settings(saved.releaseAndGetString(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
          std::string selectedPath;
          settings["geometricCalibrationPath"] >> selectedPath;
          if (selectedPath != temporaryCalibration.path.string() || application.geometricCalibrationState != "Loaded" ||
              application.decoder || application.triangulator)
            throw std::runtime_error("Loading a calibration did not update main settings or invalidate the old pipeline");
          application.scan.begin(6, false, false, application.acquisition);
          bool rejected = false;
          try { application.startScan(); } catch (const std::logic_error &) { rejected = true; }
          if (!rejected || !application.scanActive()) throw std::runtime_error("Start Scan accepted an active capture");
          application.stopCapture();
          if (application.scanActive() || application.scan.continuous()) throw std::runtime_error("Stop Scan did not stop capture");
          if (!application.lastSequence || application.lastSequence->frames.size() != 6)
            throw std::runtime_error("Stopping capture discarded the last complete sequence");
          application.settings.continuousCapture = true;
          application.stopCapture();
          if (!application.settings.continuousCapture) throw std::runtime_error("Stopping capture changed the selected capture mode");
          awaitAcquisition();
          application.loadGeometricCalibrationFile(initialCalibrationPath);
          application.settings.reconstruction.lisPhaseFilter = true;
          application.ensurePipeline();
          if (!application.pipelineSettings->lisPhaseFilter) throw std::runtime_error("Pipeline ignored LIS phase filter setting changes");
          const auto originalSettings = application.settings.reconstruction;
          application.settings.reconstruction.correctProjectorDistortion = true;
          application.settings.reconstruction.motionCompensation = false;
          application.settings.reconstruction.minimumComponentPixels = 15;
          application.settings.reconstruction.rejectDepthSpikes = true;
          application.settings.reconstruction.maximumDepthDeviation = 0.75f;
          application.ensurePipeline();
          if (!(*application.pipelineSettings == application.settings.reconstruction))
            throw std::runtime_error("Pipeline ignored distortion or cleanup option changes");
          const auto improvedSettings = application.settings.reconstruction;
          cv::FileStorage settingsSnapshot("settings.yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
          application.serializeSettings(settingsSnapshot, false);
          cv::FileStorage persisted(settingsSnapshot.releaseAndGetString(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
          if (static_cast<int>(persisted["lis_phase_filter"]) != 1 || static_cast<int>(persisted["continuous"]) != 1 ||
              !persisted["motionCompensation"].isInt() || static_cast<int>(persisted["motionCompensation"]) != 0 ||
              !application.settings.settingsWindowOpen)
            throw std::runtime_error("Settings persistence or default visibility is incorrect");
          application.settings.reconstruction = originalSettings;
          application.serializeSettings(persisted, true);
          if (!(application.settings.reconstruction == improvedSettings))
            throw std::runtime_error("Distortion and cleanup settings did not survive serialization");
          application.settings.reconstruction = originalSettings;
          application.ensurePipeline();
          if (!(*application.pipelineSettings == originalSettings))
            throw std::runtime_error("Disabling reconstruction improvements did not replace the pipeline");
          auto algorithmSettings = originalSettings.coding.algorithmSettings;
          algorithmSettings.heterodyne = {5, 42.5};
          algorithmSettings.spatialUnwrap = {5, 7};
          algorithmSettings.micro.steps = 5;
          algorithmSettings.modulated = {5, 4, 6, 12, 7};
          for (const auto codec : {CodecType::PhaseShiftHeterodyne, CodecType::PhaseShiftSpatialUnwrap,
                                   CodecType::PhaseShiftMicro, CodecType::PhaseShiftModulated}) {
            application.configureCodec(codec, algorithmSettings);
            application.ensurePipeline();
            if (application.pipelineSettings->coding.codec != codec ||
                !(application.pipelineSettings->coding.algorithmSettings == algorithmSettings))
              throw std::runtime_error("GUI pipeline ignored specialized codec settings");
            if (codec == CodecType::PhaseShiftSpatialUnwrap) {
              application.settings.reconstruction.centerFringeOrder = 1;
              application.ensurePipeline();
              if (application.pipelineSettings->centerFringeOrder != 1)
                throw std::runtime_error("GUI pipeline ignored the center fringe order");
              application.settings.reconstruction.centerFringeOrder = 2;
              application.ensurePipeline();
              if (application.pipelineSettings->centerFringeOrder != 2)
                throw std::runtime_error("Changing center fringe order did not replace the GUI pipeline");
              auto incompatible = algorithmSettings;
              incompatible.spatialUnwrap.periods = 2;
              bool rejected = false;
              try { application.configureCodec(codec, incompatible); }
              catch (const std::invalid_argument &) { rejected = true; }
              if (!rejected || !(application.settings.reconstruction.coding.algorithmSettings == algorithmSettings) ||
                  application.settings.reconstruction.centerFringeOrder != 2)
                throw std::runtime_error("GUI accepted an incompatible fringe count or changed settings on failure");
            }
          }
          application.settings.reconstruction = originalSettings;
          application.ensurePipeline();
          application.settings.reconstruction.lisPhaseFilter = false;
          application.settings.reconstruction.minimumModulation = 73;
          application.settings.reconstruction.centerFringeOrder = 2;
          application.settings.reconstruction.motionCompensation = false;
          application.settings.reconstruction.correctProjectorDistortion = true;
          application.settings.reconstruction.minimumComponentPixels = 15;
          application.settings.reconstruction.rejectDepthSpikes = true;
          application.settings.reconstruction.maximumDepthDeviation = 0.75f;
          application.settings.reconstruction.points.maxDepth = 9000;
          sequenceLoadSettings = application.settings.reconstruction;
          application.openFilePicker(FilePickerMode::LoadSequence);
        } else if (interfaceSmokeFrame == 7) {
          if (!application.filePickerDialog.state.open || application.filePickerDialog.state.mode != FilePickerMode::LoadSequence)
            throw std::runtime_error("Load Sequence did not open its folder picker");
          application.browseDirectory(std::filesystem::path(SLSTUDIO_SOURCE_DIR) / "example_data/skull_3ps");
        } else if (interfaceSmokeFrame == 8 || interfaceSmokeFrame == 12) {
          ImGui::Begin(interfaceSmokeFrame == 8 ? "Load Sequence" : "Save Sequence");
          const auto position = ImGui::GetWindowPos();
          const auto size = ImGui::GetWindowSize();
          calibrationLoadButton = ImVec2(position.x + ImGui::GetStyle().WindowPadding.x + 15,
              position.y + size.y - ImGui::GetFrameHeightWithSpacing() * 2);
          ImGui::End();
        } else if (interfaceSmokeFrame == 10) {
          if (application.filePickerDialog.state.open || !application.lastSequence || application.lastSequence->frames.size() != 3 ||
              application.settings.reconstruction.coding.codec != CodecType::PhaseShift ||
              application.settings.reconstruction.coding.algorithmSettings.phaseShift.steps != 3 ||
              application.settings.reconstruction.coding.algorithmSettings.phaseShift.periods != 1)
            throw std::runtime_error("Folder picker did not load sequence metadata and retain its frames");
          auto expected = *sequenceLoadSettings;
          expected.coding.codec = CodecType::PhaseShift;
          expected.coding.direction = CodecDirHorizontal;
          expected.coding.algorithmSettings.phaseShift = {3, 1};
          if (!(application.settings.reconstruction == expected) ||
              !application.pipelineSettings || !(*application.pipelineSettings == expected))
            throw std::runtime_error("Loading a sequence replaced the active reconstruction settings");
          application.settings.reconstruction.coding.codec = CodecType::GrayCode;
          temporarySequence.path = std::filesystem::temp_directory_path() /
              ("slstudio saved sequence " + std::to_string(Clock::now().time_since_epoch().count()));
          if (!std::filesystem::create_directory(temporarySequence.path))
            throw std::runtime_error("Cannot create sequence picker fixture");
          application.openFilePicker(FilePickerMode::SaveSequence);
        } else if (interfaceSmokeFrame == 11) {
          if (!application.filePickerDialog.state.open || application.filePickerDialog.state.mode != FilePickerMode::SaveSequence)
            throw std::runtime_error("Save Sequence did not open its folder picker");
          application.browseDirectory(temporarySequence.path);
        } else if (interfaceSmokeFrame == 14) {
          if (application.filePickerDialog.state.open) throw std::runtime_error("Sequence picker did not save its selected folder");
          const auto saved = loadSequence(temporarySequence.path, application.settings.reconstruction.coding);
          if (saved.coding.codec != CodecType::PhaseShift || saved.frames.size() != 3 ||
              saved.coding.algorithmSettings.phaseShift.periods != 1 ||
              application.settings.reconstruction.coding.codec != CodecType::GrayCode)
            throw std::runtime_error("Sequence save used current settings instead of its original coding settings");
          for (std::size_t index = 0; index < saved.frames.size(); ++index)
            if (cv::norm(saved.frames[index], application.lastSequence->frames[index], cv::NORM_INF) != 0)
              throw std::runtime_error("GUI sequence save changed camera pixels");
          cv::FileStorage metadata((temporarySequence.path / "sequence.xml").string(), cv::FileStorage::READ);
          if (metadata.root().size() != 6 || !metadata["gray_horizontal_bits"].isNone() ||
              !metadata["gray_vertical_bits"].isNone() || !metadata["minimum_modulation"].isNone() ||
              !metadata["motion_compensation"].isNone() || !metadata["max_depth"].isNone())
            throw std::runtime_error("GUI sequence save included reconstruction settings");
        } else if (interfaceSmokeFrame >= 15 && !application.processing()) {
          application.settings.reconstruction.coding.codec = CodecType::PhaseShift;
          application.settings.reconstruction.coding.algorithmSettings = CodecAlgorithmSettings{};
          application.ensurePipeline();
          if (application.pipelineSettings->lisPhaseFilter) throw std::runtime_error("Pipeline kept a disabled LIS phase filter");
          interfaceReviewed = true;
        }
        ++interfaceSmokeFrame;
      }
      auto checkSequenceAspect = [&] {
        ImGui::Begin(application.frameSequenceWindowTitle().c_str());
        const auto size = ImGui::GetWindowSize();
        const float imageHeight = size.y - ImGui::GetFrameHeight();
        ImGui::End();
        const auto &sequence = application.lastSequence->frames;
        if (std::abs(imageHeight - size.x * sequence.front().rows /
            (sequence.front().cols * static_cast<float>(sequence.size()))) > 1.1f)
          throw std::runtime_error("Frame sequence window does not match the concatenated aspect ratio");
      };
      auto checkSequenceTextures = [&] {
        const auto &sequence = application.lastSequence->frames;
        const auto &textures = application.frameSequenceWindow.textures();
        if (application.frameSequenceWindow.texturesDirty() || textures.size() != sequence.size())
          throw std::runtime_error("Frame sequence textures did not update to the completed sequence");
        for (std::size_t index = 0; index < sequence.size(); ++index) {
          GLint width = 0, height = 0;
          glBindTexture(GL_TEXTURE_2D, textures[index]);
          glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
          glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
          if (cv::Size(width, height) != sequence[index].size())
            throw std::runtime_error("Frame sequence texture dimensions do not match the captured frames");
          cv::Mat displayed(height, width, CV_8UC1);
          glPixelStorei(GL_PACK_ALIGNMENT, 1);
          glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, displayed.data);
          if (cv::norm(displayed, sequence[index], cv::NORM_INF) != 0)
            throw std::runtime_error("Frame sequence texture changed frame order or captured pixels");
        }
        glBindTexture(GL_TEXTURE_2D, 0);
      };
      const bool checkSequenceLayoutThisFrame = cameraWindowSmoke && cameraWindowSmokeFrame == 5;
      if (testCameraThisFrame && cameraWindowSmokeFrame < 5 && ++cameraWindowSmokeFrame == 5) {
        ImGui::Begin(application.frameSequenceWindowTitle().c_str());
        const bool accessible = ImGui::IsWindowHovered();
        ImGui::End();
        if (!accessible) throw std::runtime_error("Frame sequence became inaccessible after clicking the workspace");
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 5) {
        checkSequenceTextures();
        if (application.cameraWorker || application.cameraRequested ||
            application.frameSequenceWindowTitle().find("3 frames | 320 x 240") == std::string::npos)
          throw std::runtime_error("Frame sequence opened a camera or showed an incorrect title");
        application.appendCameraPreview(cv::Mat(240, 320, CV_8UC1, cv::Scalar(99)));
        if (application.frameSequenceWindow.texturesDirty() ||
            application.lastSequence->frames.front().at<std::uint8_t>(0, 0) != 17)
          throw std::runtime_error("An individual live frame changed the completed sequence");
        checkSequenceAspect();
        ImGui::SetWindowSize("###FrameSequence", ImVec2(700, 230));
        ++cameraWindowSmokeFrame;
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 6) {
        checkSequenceAspect();
        std::vector<cv::Mat> sequence;
        for (int index = 0; index < 2; ++index)
          sequence.emplace_back(270, 480, CV_8UC1, cv::Scalar(71 + index * 12));
        application.setLastSequence(FrameSequence{sequence, application.settings.reconstruction.coding});
        ImGui::SetWindowSize("###FrameSequence", ImVec2(300, 650));
        ++cameraWindowSmokeFrame;
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 7) {
        checkSequenceAspect();
        checkSequenceTextures();
        GLint maximumWidth = 0;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximumWidth);
        std::vector<cv::Mat> sequence;
        for (int index = 0; index <= maximumWidth / 320; ++index)
          sequence.emplace_back(240, 320, CV_8UC1, cv::Scalar(index % 255));
        application.setLastSequence(FrameSequence{sequence, application.settings.reconstruction.coding});
        ++cameraWindowSmokeFrame;
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 8) {
        checkSequenceTextures();
        application.frameSequenceWindow.open = false;
        application.setLastSequence(FrameSequence{{cv::Mat(240, 320, CV_8UC1, cv::Scalar(123))},
                                                   application.settings.reconstruction.coding});
        application.renderer.resetView();
        ++cameraWindowSmokeFrame;
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 9) {
        if (!application.frameSequenceWindow.texturesDirty() ||
            application.frameSequenceWindow.textures().size() == 1)
          throw std::runtime_error("A hidden Frame sequence window uploaded display textures");
        application.frameSequenceWindow.open = true;
        ++cameraWindowSmokeFrame;
      }
      ImGui::Render();
      if (checkSequenceLayoutThisFrame) {
        const auto &textures = application.frameSequenceWindow.textures();
        const auto count = textures.size();
        std::vector<std::array<float, 4>> bounds(count, {std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()});
        const auto *data = ImGui::GetDrawData();
        for (int index = 0; index < data->CmdListsCount; ++index) {
          const auto *list = data->CmdLists[index];
          for (const auto &command : list->CmdBuffer) {
            const auto texture = std::find(textures.begin(), textures.end(), command.GetTexID());
            if (texture == textures.end()) continue;
            auto &rectangle = bounds[texture - textures.begin()];
            for (unsigned int offset = 0; offset < command.ElemCount; ++offset) {
              const auto &position = list->VtxBuffer[command.VtxOffset + list->IdxBuffer[command.IdxOffset + offset]].pos;
              rectangle[0] = std::min(rectangle[0], position.x);
              rectangle[1] = std::min(rectangle[1], position.y);
              rectangle[2] = std::max(rectangle[2], position.x);
              rectangle[3] = std::max(rectangle[3], position.y);
            }
          }
        }
        for (std::size_t index = 0; index < count; ++index) {
          const auto &rectangle = bounds[index];
          if (rectangle[2] <= rectangle[0] || rectangle[3] <= rectangle[1])
            throw std::runtime_error("Frame sequence did not render every captured frame");
          if (index && (std::abs(rectangle[0] - bounds[index - 1][2]) > 0.01f ||
              std::abs(rectangle[1] - bounds[0][1]) > 0.01f || std::abs(rectangle[3] - bounds[0][3]) > 0.01f))
            throw std::runtime_error("Frame sequence did not concatenate frames horizontally without gaps");
        }
      }
      if (intensityCalibrationSmoke && intensityStage == 0 && application.cameraPreviewTexture.texture()) {
        intensityImageMin = ImVec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        intensityImageMax = ImVec2(0, 0);
        const auto *data = ImGui::GetDrawData();
        for (int index = 0; index < data->CmdListsCount; ++index) {
          const auto *list = data->CmdLists[index];
          for (const auto &command : list->CmdBuffer) {
            if (command.GetTexID() != static_cast<ImTextureID>(application.cameraPreviewTexture.texture())) continue;
            for (unsigned int offset = 0; offset < command.ElemCount; ++offset) {
              const auto &vertex = list->VtxBuffer[command.VtxOffset + list->IdxBuffer[command.IdxOffset + offset]];
              intensityImageMin.x = std::min(intensityImageMin.x, vertex.pos.x);
              intensityImageMin.y = std::min(intensityImageMin.y, vertex.pos.y);
              intensityImageMax.x = std::max(intensityImageMax.x, vertex.pos.x);
              intensityImageMax.y = std::max(intensityImageMax.y, vertex.pos.y);
            }
          }
        }
      }
      if (cameraWindowSmoke && cameraWindowSmokeFrame == 10) {
        if (application.frameSequenceWindow.open && !application.frameSequenceWindow.texturesDirty())
          checkSequenceTextures();
        renderPanelVisibleVertices = ImGui::GetDrawData()->TotalVtxCount;
        application.settings.renderSettingsOpen = false;
        ++cameraWindowSmokeFrame;
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 11) {
        checkSequenceTextures();
        renderPanelHiddenVertices = ImGui::GetDrawData()->TotalVtxCount;
        if (renderPanelHiddenVertices >= renderPanelVisibleVertices)
          throw std::runtime_error("Hiding render settings did not remove the panel");
        application.settings.renderSettingsOpen = true;
        ++cameraWindowSmokeFrame;
      } else if (cameraWindowSmoke && cameraWindowSmokeFrame == 12) {
        if (ImGui::GetDrawData()->TotalVtxCount <= renderPanelHiddenVertices)
          throw std::runtime_error("Showing render settings did not restore the panel");
        ++cameraWindowSmokeFrame;
      }
      gui.renderDrawData();
      if (cameraWindowSmoke && cameraWindowSmokeFrame == 9 && screenshot) {
        const std::filesystem::path path(screenshot);
        gui.saveScreenshot(path.parent_path() / (path.stem().string() + "-latest-sequence" + path.extension().string()));
      }
      if (cameraWindowSmoke && cameraWindowSmokeFrame == 6 && screenshot) {
        const std::filesystem::path path(screenshot);
        gui.saveScreenshot(path.parent_path() / (path.stem().string() + "-frame-sequence" + path.extension().string()));
      }
      if (projectorSmoke && renderedFrames >= 12 && renderedFrames <= 14 && screenshot) {
        const std::filesystem::path path(screenshot);
        const char *modes[] = {"-flash-patterns", "-hdmi-video", "-test-pattern"};
        gui.saveScreenshot(path.parent_path() / (path.stem().string() + modes[renderedFrames - 12] + path.extension().string()));
      }
      if (cameraWindowSmoke && interfaceSmokeFrame == 2 && screenshot) {
        const std::filesystem::path path(screenshot);
        gui.saveScreenshot(path.parent_path() / (path.stem().string() + "-calibration-picker" + path.extension().string()));
      }
      if (cameraWindowSmoke && cameraWindowSmokeFrame == 12 && screenshot) {
        const std::filesystem::path path(screenshot);
        gui.saveScreenshot(path.parent_path() / (path.stem().string() + "-render-settings-hidden" + path.extension().string()));
      }
      if (reconstructionReviewed && (!geometricCalibrationSmoke || calibrationReviewed) &&
        (!intensityCalibrationSmoke || intensityStage == 5) && ++renderedFrames >= 3 &&
        (!cameraWindowSmoke || cameraWindowSmokeFrame >= 13)) {
        if (projectorSmoke && renderedFrames < 15) {
          if (renderedFrames == 3 || renderedFrames == 6 || renderedFrames == 9 || renderedFrames == 12) {
            if (screenshot) {
              const std::filesystem::path path(screenshot);
              const std::string model = renderedFrames == 3 || renderedFrames == 9 ? "-lc3000" : "-lc4500";
              gui.saveScreenshot(path.parent_path() / (path.stem().string() + model +
                  (renderedFrames <= 6 ? "-wide" : "-compact") + path.extension().string()));
            }
            application.lc3000Window.open = renderedFrames == 6;
            application.lc4500Window.open = renderedFrames != 6;
            if (renderedFrames == 6) glfwSetWindowSize(window, 900, 600);
          }
        } else if (intensityCalibrationSmoke && renderedFrames < 8) {
          if (renderedFrames == 3) {
            if (screenshot) {
              const std::filesystem::path path(screenshot);
              gui.saveScreenshot(path.parent_path() / (path.stem().string() + "-wide" + path.extension().string()));
            }
            glfwSetWindowSize(window, 900, 600);
          }
        } else if (geometricCalibrationSmoke && renderedFrames < 12) {
          if (renderedFrames == 3 || renderedFrames == 6 || renderedFrames == 10) {
            if (screenshot) {
              const std::filesystem::path path(screenshot);
              gui.saveScreenshot(path.parent_path() / (path.stem().string() +
                  (renderedFrames == 3 ? "-poses" : renderedFrames == 6 ? "-live" : "-compact-poses") + path.extension().string()));
            }
            if (renderedFrames == 3) {
              application.geometricCalibrationDialog.selectedPose = -1;
              application.geometricCalibrationDialog.invalidatePreview();
              application.geometricCalibrationDialog.refreshPreview(application.settings,
                  application.geometricCalibrationDiagnostics, application.geometricCalibrationPoses);
              if (!application.geometricCalibrationDialog.previewSize().empty() || !application.pendingGeometricCalibration)
                throw std::runtime_error("Switching to the shared live feed lost calibration review state");
            }
            if (renderedFrames == 6) {
              if (!application.cameraPreviewTexture.texture() || application.cameraPreview.empty() ||
                  application.cameraPreviewDirty || application.frameSequenceWindow.open)
                throw std::runtime_error("Geometric Calibration did not render the shared camera feed independently");
              application.geometricCalibrationDialog.selectedPose = static_cast<int>(
                  std::find_if(application.geometricCalibrationDiagnostics.begin(), application.geometricCalibrationDiagnostics.end(),
                      [](const auto &pose) { return pose.accepted; }) - application.geometricCalibrationDiagnostics.begin());
              application.geometricCalibrationDialog.invalidatePreview();
              glfwSetWindowSize(window, 900, 600);
              application.geometricCalibrationDialog.open = false;
            }
          }
          if (renderedFrames == 7) application.geometricCalibrationDialog.open = true;
        } else if (projectorSmoke && (application.lc4500FirmwareJob.valid() || application.lc4500PlaybackJob.valid())) {
          // Wait for the read-only version query before reviewing the connected-device UI.
        } else {
          if (screenshot) gui.saveScreenshot(screenshot);
          if (geometricCalibrationSmoke) {
            if (application.geometricCalibrationDialog.previewSize().empty()) throw std::runtime_error("Geometric Calibration preview is empty");
            cv::Mat preview(application.geometricCalibrationDialog.previewSize(), CV_8UC3);
            glBindTexture(GL_TEXTURE_2D, application.geometricCalibrationDialog.previewTexture());
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, preview.data);
            glBindTexture(GL_TEXTURE_2D, 0);
            if (cv::countNonZero(preview.reshape(1)) < 100) throw std::runtime_error("Geometric Calibration preview texture is blank");
            application.saveAndActivateGeometricCalibration();
            GeometricCalibrationData saved;
            if (application.geometricCalibrationState != "Active" ||
                application.settings.geometricCalibrationPath.data() != temporaryCalibration.path.string() ||
                !saved.load(temporaryCalibration.path) || saved.stereo_error >= 1.0)
              throw std::runtime_error("Geometric Calibration save/activation smoke test failed");
            if (saved.cam_pve.empty() || saved.proj_pve.empty() ||
                saved.cam_stdint.empty() || saved.proj_stdint.empty())
              throw std::runtime_error("Accepted calibration lost per-view errors or uncertainties");
            application.invalidateGeometricCalibration();
            bool rejected = false;
            try { application.saveAndActivateGeometricCalibration(); }
            catch (const std::runtime_error &) { rejected = true; }
            if (!rejected) throw std::runtime_error("Invalidated calibration could be activated");
          }
          if (geometricCalibrationSmoke || intensityCalibrationSmoke) {
            application.settingsEnabled = true;
            application.saveSettings();
            ApplicationController restored(true);
            restored.settingsDirectory = application.settingsDirectory;
            restored.loadSettings();
            if (restored.cameraRequested || restored.cameraWorker || !restored.geometricCalibrationPoses.empty())
              throw std::runtime_error("Restoring settings started a camera or persisted calibration poses");
            if (geometricCalibrationSmoke && restored.settings.geometricCalibrationPath != application.settings.geometricCalibrationPath)
              throw std::runtime_error("Accepted calibration did not survive settings reload");
            if (geometricCalibrationSmoke && restored.settings.geometricCalibrationSettings.interpolation != GeometricCalibrationInterpolation::RBF)
              throw std::runtime_error("Geometric Calibration interpolation method did not survive settings reload");
            if (intensityCalibrationSmoke && (!restored.intensityCalibration.result || !restored.settings.intensityLut || !restored.settings.intensityCompensationEnabled ||
                restored.intensityCalibration.result->inverseLut != originalCorrection.inverseLut ||
                restored.settings.intensityLut->inverseLut != originalCorrection.inverseLut))
              throw std::runtime_error("Accepted projector/camera intensity curve and compensation did not survive settings reload");
            application.settingsEnabled = false;
          }
          if (glGetError() != GL_NO_ERROR) throw std::runtime_error("OpenGL error during GUI smoke test");
          if (intensityCalibrationSmoke) {
            cv::Mat preview(application.cameraPreview.size(), CV_8UC3);
            glBindTexture(GL_TEXTURE_2D, application.cameraPreviewTexture.texture());
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, preview.data);
            glBindTexture(GL_TEXTURE_2D, 0);
            if (cv::countNonZero(preview.reshape(1)) < 100) throw std::runtime_error("Intensity Calibration preview texture is blank");
          }
          if (!ros2OutputReviewed) throw std::runtime_error("Reconstructed clouds did not reach ROS 2 output");
          std::cout << "GUI smoke test passed: " << application.cloud->points.size() << " points";
          if (geometricCalibrationSmoke) std::cout << "; single-page calibration, shared live feed, compact layout, review, save and activation passed";
          if (intensityCalibrationSmoke) std::cout << "; projector/camera intensity measurement, shared live feed, intensity compensation, read-only stored curve viewer, verification, persistence and cleanup passed";
          if (projectorSmoke) {
            if (application.lc4500Connected.value_or(false) && !application.lc4500Firmware)
              throw std::runtime_error("Connected LC4500 firmware query failed: " + application.lc4500FirmwareError);
            std::cout << "; LC3000/LC4500 dialogs, display modes, persistence, firmware display, playback controls/status and ownership guards passed";
          }
          if (cameraWindowSmoke) std::cout << "; calibration/sequence pickers, settings/LIS phase filter activation, complete-sequence capture, ordered horizontal frame display, sequence replacement, hidden-window updates, GPU-width-independent strips, aspect-locked resizing, floating render settings and modulation/depth/solid coloring passed";
          std::cout << '\n';
          break;
        }
      }
      glfwSwapBuffers(window);
    }
    return 0;
  });
}
}
