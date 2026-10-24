#include "ApplicationController.h"
#include <imgui.h>
#include "CodecCalibration.h"
#include <opencv2/videoio.hpp>

namespace slstudio::gui {
namespace {
struct CaptureSettings {
  int deviceIndex, width, height, discard;
  float exposure, gain;
  bool autoExposure;
  std::optional<slstudio::CameraBackend> backend;
  slstudio::CameraSettings nativeSettings;
  bool requireManual = false;
};

struct CaptureDevice final : CaptureWorker::Camera {
  CaptureSettings captureSettings{};
  cv::VideoCapture camera;
  std::unique_ptr<slstudio::Camera> nativeCamera;
  CaptureWorker::Reader syntheticReader;

  void open(CaptureSettings settings) {
    captureSettings = settings;
    if (settings.backend) {
      if (settings.deviceIndex < 0) throw std::invalid_argument("Camera index must be nonnegative");
      nativeCamera = slstudio::openCamera(*settings.backend, static_cast<std::size_t>(settings.deviceIndex), settings.nativeSettings);
      if ((settings.width > 0 && nativeCamera->resolution().width != settings.width) ||
          (settings.height > 0 && nativeCamera->resolution().height != settings.height))
        throw std::runtime_error("Camera resolution does not match calibration");
      nativeCamera->start();
    } else {
      if (!camera.open(settings.deviceIndex)) throw std::runtime_error("Cannot open camera device");
      const bool widthSet = camera.set(cv::CAP_PROP_FRAME_WIDTH, settings.width > 0 ? settings.width : 16384);
      const bool heightSet = camera.set(cv::CAP_PROP_FRAME_HEIGHT, settings.height > 0 ? settings.height : 16384);
      if ((settings.width == 0 && !widthSet) || (settings.height == 0 && !heightSet))
        throw std::runtime_error("Camera does not support resetting capture dimensions");
      const bool automaticSet = camera.set(cv::CAP_PROP_AUTO_EXPOSURE, settings.autoExposure ? 0.75 : 0.25);
      const bool exposureSet = settings.autoExposure || camera.set(cv::CAP_PROP_EXPOSURE, settings.exposure);
      const bool gainSet = camera.set(cv::CAP_PROP_GAIN, settings.gain);
        if (settings.requireManual && (!automaticSet || !exposureSet || !gainSet ||
          std::abs(camera.get(cv::CAP_PROP_AUTO_EXPOSURE) - 0.25) > 0.01 ||
          std::abs(camera.get(cv::CAP_PROP_EXPOSURE) - settings.exposure) > 0.01 ||
          std::abs(camera.get(cv::CAP_PROP_GAIN) - settings.gain) > 0.01))
        throw std::runtime_error("Camera cannot lock automatic exposure, exposure and gain for intensity measurement");
    }
  }

  ~CaptureDevice() {
    try {
      if (nativeCamera && nativeCamera->isCapturing()) nativeCamera->stop();
    } catch (const std::exception &error) {
      std::fprintf(stderr, "SLStudio camera shutdown: %s\n", error.what());
    }
  }

  CaptureFrame read() override {
    if (syntheticReader) return {syntheticReader()};
    if (nativeCamera) {
      const bool software = captureSettings.nativeSettings.trigger == slstudio::CameraTrigger::Software;
      slstudio::CameraReadResult frame;
      for (int index = 0; index <= (software ? captureSettings.discard : 0); ++index) {
        if (software) nativeCamera->softwareTrigger();
        frame = nativeCamera->read(std::chrono::milliseconds(1000));
        if (!software && frame.status != slstudio::CameraReadStatus::Complete) return {};
        if (frame.status != slstudio::CameraReadStatus::Complete || frame.frame.image.empty())
          throw std::runtime_error("Camera capture timed out or returned an incomplete frame");
      }
      return {std::move(frame.frame.image), frame.frame.metadata.sequenceStart.value_or(false),
          frame.frame.metadata.frameId, frame.frame.metadata.frameIdMaximum};
    }
    cv::Mat frame;
    for (int index = 0; index <= captureSettings.discard; ++index)
      if (!camera.read(frame)) throw std::runtime_error("Camera capture failed");
    if (frame.channels() != 1) cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    return {std::move(frame)};
  }
};

CaptureSettings makeCaptureSettings(const ApplicationController &application, cv::Size size) {
  CaptureSettings requestedSettings{application.settings.cameraIndex, size.width, size.height, application.settings.discardFrames,
      application.settings.exposure, application.settings.gain, application.settings.autoExposure, {}, {}};
  if (!application.nativeCameraSelected()) return requestedSettings;
  if (!std::isfinite(application.settings.exposure) || application.settings.exposure <= 0 || application.settings.exposure > 60000)
    throw std::invalid_argument("Camera exposure must be in (0, 60000] ms");
  requestedSettings.backend = application.cameraBackends.at(application.settings.cameraInterface);
  auto &settings = requestedSettings.nativeSettings;
  settings.region = {0, 0, size.width, size.height};
  settings.exposure = std::chrono::microseconds(static_cast<std::int64_t>(static_cast<double>(application.settings.exposure) * 1000));
  settings.gain = application.settings.gain;
  settings.trigger = application.settings.hardwareTrigger ? slstudio::CameraTrigger::External : slstudio::CameraTrigger::Software;
  settings.frameTriggerSource = application.settings.frameTriggerSource.data();
  settings.risingEdge = application.settings.triggerRisingEdge;
  slstudio::validateCameraSettings(settings);
  return requestedSettings;
}

CaptureWorker::CameraFactory cameraFactory(const CaptureSettings &requestedSettings,
    CaptureWorker::Reader reader = {}) {
  return [requestedSettings, reader = std::move(reader)] {
    auto device = std::make_unique<CaptureDevice>();
    if (reader) device->syntheticReader = reader;
    else device->open(requestedSettings);
    return device;
  };
}

}

bool ApplicationController::cameraReady() const {
  return cameraWorker && cameraWorker->ready();
}

std::future<void> ApplicationController::releaseCameraWorker() {
  auto *previous = std::exchange(cameraWorker, nullptr);
  cameraResolution = {};
  return previous ? previous->closeCamera() : std::future<void>{};
}

bool ApplicationController::requestCameraFrame() {
  if (!cameraWorker) throw std::logic_error("Start the camera before requesting a frame");
  return cameraWorker->requestFrame();
}

bool ApplicationController::cameraFramePending() const {
  return cameraWorker && cameraWorker->framePending();
}

std::optional<cv::Mat> ApplicationController::takeCameraFrame() {
  if (!cameraWorker) throw std::logic_error("Start the camera before taking a frame");
  auto frame = cameraWorker->tryTake();
  if (frame && (!cameraResolution.width || !cameraResolution.height))
    cameraResolution = frame->size();
  return frame;
}

void ApplicationController::stopCamera() {
  cameraRequested = false;
  releaseCameraWorker();
  cameraPreview.release();
  cameraPreviewFrameSize = {};
  cameraNextPreviewFrame = 0;
  cameraPreviewDirty = false;
  cameraPreviewTimes.clear();
}

void ApplicationController::setCameraPreviewFrameCount(int count) {
  setCameraPreviewFrameCount(count, settings.cameraWidth);
}

void ApplicationController::setCameraPreviewFrameCount(int count, int configuredWidth) {
  const int frameWidth = cameraPreviewFrameSize.empty() ?
      std::max(1, configuredWidth) : cameraPreviewFrameSize.width;
  if (count < 1 || count > maximumPreviewSize / frameWidth)
    throw std::invalid_argument("Preview frame count exceeds the supported image width");
  if (cameraPreviewFrameCount != count) {
    cameraPreviewFrameCount = count;
    cameraPreview.release();
    cameraNextPreviewFrame = 0;
    cameraPreviewDirty = false;
  }
  settings.previewFrameCount = count;
}

void ApplicationController::appendCameraPreview(const cv::Mat &frame, Clock::time_point now) {
  if (frame.empty() || frame.dims != 2 || frame.type() != CV_8UC1)
    throw std::invalid_argument("Camera preview requires a nonempty Mono8 frame");
  if (cameraPreview.empty() || frame.size() != cameraPreviewFrameSize) {
    if (cameraPreviewFrameCount > maximumPreviewSize / frame.cols || frame.rows > maximumPreviewSize)
      throw std::invalid_argument("Camera preview dimensions exceed the supported texture size; reduce the frame count or camera resolution");
    cameraPreview = cv::Mat::zeros(frame.rows, frame.cols * cameraPreviewFrameCount, CV_8UC1);
    cameraPreviewFrameSize = frame.size();
    cameraNextPreviewFrame = 0;
  }
  frame.copyTo(cameraPreview(cv::Rect(cameraNextPreviewFrame * frame.cols, 0, frame.cols, frame.rows)));
  cameraNextPreviewFrame = (cameraNextPreviewFrame + 1) % cameraPreviewFrameCount;
  cameraPreviewDirty = true;
  cameraPreviewTimes.push_back(now);
  if (cameraPreviewTimes.size() > 31) cameraPreviewTimes.pop_front();
}

double ApplicationController::cameraPreviewFps(Clock::time_point now) const {
  if (cameraPreviewTimes.size() < 2 || now - cameraPreviewTimes.back() >= std::chrono::seconds(1)) return 0;
  const double elapsed = std::chrono::duration<double>(cameraPreviewTimes.back() - cameraPreviewTimes.front()).count();
  return elapsed > 0 ? (cameraPreviewTimes.size() - 1) / elapsed : 0;
}

void ApplicationController::ensurePipeline() {
  const auto modified = std::filesystem::last_write_time(settings.geometricCalibrationPath.data());
  const bool matching = pipelineSettings && *pipelineSettings == settings.reconstruction;
  if (decoder && triangulator && matching && pipelineGeometricCalibrationPath == settings.geometricCalibrationPath.data() &&
      pipelineGeometricCalibrationTime == modified) return;
  if (reconstructionBusy()) throw std::runtime_error("Stop processing before changing reconstruction settings");
  GeometricCalibrationData calibration;
  if (!calibration.load(settings.geometricCalibrationPath.data()))
    throw std::runtime_error("Cannot load calibration XML");
  geometricCalibrationState = "Loaded";
  auto nextTriangulator = std::make_unique<TriangulatorWorker>(calibration, settings.reconstruction);
  auto nextDecoder = std::make_unique<DecoderWorker>(calibration, settings.reconstruction,
      [target = nextTriangulator.get()](std::uint64_t sequence, cv::Mat1f up, cv::Mat1f vp,
                                       cv::Mat1b mask, cv::Mat1b shading) {
        if (!target->trySubmitDense(sequence, std::move(up), std::move(vp), std::move(mask), std::move(shading)))
          throw std::runtime_error("Triangulator worker stopped");
      },
      [target = nextTriangulator.get()](std::uint64_t sequence, std::vector<ProjectorTransition> coordinates) {
        if (!target->trySubmitSparse(sequence, std::move(coordinates)))
          throw std::runtime_error("Triangulator worker stopped");
      });
  stopReconstruction();
  triangulator = std::move(nextTriangulator);
  decoder = std::move(nextDecoder);
  pipelineSettings = settings.reconstruction;
  pipelineGeometricCalibrationPath = settings.geometricCalibrationPath.data();
  pipelineGeometricCalibrationTime = modified;
}

void ApplicationController::reconstruct() {
  if (geometricCalibrationJob.valid() || sequencePending) throw std::runtime_error("Processing is already running");
  ensurePipeline();
  setLastSequence(FrameSequence{frames, settings.reconstruction.coding});
  sequencePending = true;
  status = "Sequence ready for decoding";
}

void ApplicationController::refreshOutputs() {
  if (projector) throw std::runtime_error("Close the projector before refreshing outputs");
  outputs = slstudio::projectorOutputs(true);
  settings.outputIndex = 0;
  status = outputs.empty() ? "No eligible projectors" : "Projectors refreshed";
}

void ApplicationController::openOutput() {
  if (lc4500FirmwareJob.valid() || lc4500PlaybackJob.valid())
    throw std::logic_error("Wait for LC4500 device queries to finish before opening a projector");
  const auto &output = outputs.at(settings.outputIndex);
  slstudio::ProjectorOpenSettings requestedSettings;
  requestedSettings.allowDesktop = output.backend == slstudio::OutputBackend::Desktop;
  requestedSettings.flashBackupDirectory = (settingsPath().parent_path() / "projector-backups").string();
  requestedSettings.networkAddress = settings.projectorAddress.data();
  if (projector || projectorJob.valid()) throw std::logic_error("Close the current projector before opening another");
  CaptureWorker::ProjectorFactory factory;
  if (output.backend == slstudio::OutputBackend::Desktop) {
    projectorPresentationController = std::make_unique<ProjectorPresentationController>(slstudio::openProjector(output, requestedSettings));
    factory = projectorPresentationController->factory();
  } else {
    factory = [output, requestedSettings] { return slstudio::openProjector(output, requestedSettings); };
  }
  projectorJobMessage = output.backend == slstudio::OutputBackend::Desktop
      ? "Desktop fullscreen projector active (not exclusive)" : "Projector opened";
  projectorJob = acquisition.openProjector(std::move(factory));
  status = "Opening projector";
}

void ApplicationController::closeOutput() {
  if (scan.active() || intensityCalibration.worker || projectorJob.valid())
    throw std::logic_error("Stop acquisition before closing the projector");
  projectorJob = acquisition.closeProjector();
  projector.reset();
  projectorJobMessage = "Projector closed";
}

cv::Size ApplicationController::projectorSize() const {
  const auto full = projector ? projector->resolution() : cv::Size(1024, 768);
  return {settings.projectorWidth ? settings.projectorWidth : full.width, settings.projectorHeight ? settings.projectorHeight : full.height};
}

void ApplicationController::validateProjectorCrop() const {
  const auto size = projectorSize();
  if (size.width <= 0 || size.height <= 0 || size.width > 16384 || size.height > 16384 ||
      (projector && (size.width > projector->resolution().width || size.height > projector->resolution().height)))
    throw std::invalid_argument("Projector crop must fit within the full projector resolution");
}

std::vector<cv::Mat> ApplicationController::projectorPatterns(bool calibration) const {
  if (!projector) throw std::logic_error("Open a projector output first");
  validateProjectorCrop();
  const auto size = projectorSize();
  std::unique_ptr<Encoder> encoder;
  if (calibration)
    encoder = std::make_unique<EncoderCalibration>(size.width, size.height, CodecDirBoth);
  else
    encoder = EncoderFactory::NewEncoder(settings.reconstruction.coding.codec, size.width, size.height,
        settings.reconstruction.coding.direction, settings.reconstruction.coding.algorithmSettings);
  if (!encoder) throw std::runtime_error("Unsupported codec");
  std::vector<cv::Mat> patterns;
  for (unsigned int index = 0; index < encoder->getNPatterns(); ++index) {
    const auto pattern = encoder->getEncodingPattern(index);
    if (pattern.empty() || pattern.dims != 2) throw std::runtime_error("Encoder returned an invalid pattern");
    const auto tiled = cv::repeat(pattern, (size.height + pattern.rows - 1) / pattern.rows,
                                 (size.width + pattern.cols - 1) / pattern.cols);
    const auto fullPattern = tiled(cv::Rect({}, size));
    const auto corrected = settings.intensityCompensationEnabled && settings.intensityLut ? compensateIntensity(fullPattern, *settings.intensityLut) : fullPattern;
    cv::Mat padded = cv::Mat::zeros(projector->resolution(), corrected.type());
    corrected.copyTo(padded(cv::Rect({}, size)));
    patterns.push_back(std::move(padded));
  }
  return patterns;
}

void ApplicationController::configureCodec(CodecType type, const CodecAlgorithmSettings &algorithmSettings) {
  if (processing() || scan.index() >= 0)
    throw std::logic_error("Stop capture and processing before changing codec parameters");
  validateCodecSettings(algorithmSettings);
  const auto name = codecName(type);
  auto candidate = settings.reconstruction;
  candidate.coding.codec = type;
  candidate.coding.algorithmSettings = algorithmSettings;
  validateReconstructionSettings(candidate);
  settings.reconstruction.coding.algorithmSettings = algorithmSettings;
  settings.reconstruction.coding.codec = type;
  status = std::string("Selected ") + name;
  if (type == CodecType::GrayCode) {
    if (settings.reconstruction.lisPhaseFilter || settings.reconstruction.rejectDepthSpikes)
      status += "; unorganized cloud, image-grid filters disabled";
    settings.reconstruction.lisPhaseFilter = false;
    settings.reconstruction.rejectDepthSpikes = false;
  }
}

void ApplicationController::startProjectorProgramming(bool calibration) {
  if (processing() || scan.index() >= 0) throw std::logic_error("Stop capture before programming projector flash");
  if (!projector || !projector->capabilities().persistentPatterns)
    throw std::logic_error("This projector does not support flash programming");
  auto patterns = projectorPatterns(calibration);
  projectorJobMessage = "Projector patterns programmed";
  auto progress = std::make_shared<ProjectorPatternUploadDialog::Progress>();
  projectorJob = acquisition.programPatterns(std::move(patterns), [progress](float fraction, const char *stage) {
      std::lock_guard<std::mutex> lock(progress->mutex);
      progress->fraction = fraction;
      progress->stage = stage;
  });
  projectorPatternUploadDialog.progress = std::move(progress);
  status = "Programming projector patterns; keep power connected";
}

void ApplicationController::applyLightCrafterSettings(slstudio::OutputBackend backend) {
  if (processing() || scan.index() >= 0 || projector)
    throw std::logic_error("Stop capture and close projector outputs before applying LightCrafter settings");
  if (!slstudio::lightCrafterAvailable(backend)) throw std::logic_error("LightCrafter backend unavailable in this build");
  slstudio::ProjectorOpenSettings projectorSettings;
  projectorSettings.networkAddress = settings.projectorAddress.data();
  const std::string model = backend == slstudio::OutputBackend::LightCrafter3000 ? "LC3000" : "LC4500";
  auto apply = [&](const auto &deviceSettings) {
    slstudio::validateLightCrafterSettings(deviceSettings);
    projectorJobMessage = model + (deviceSettings.standby ? " in standby" : " settings applied");
    projectorJob = std::async(std::launch::async, [deviceSettings, projectorSettings] {
      slstudio::configureLightCrafter(deviceSettings, projectorSettings);
    });
  };
  if (backend == slstudio::OutputBackend::LightCrafter3000) apply(settings.lc3000Settings);
  else {
    apply(settings.lc4500Settings);
    lc4500Playback.reset();
    lc4500PlaybackError.clear();
    lc4500PlaybackPollAfter = {};
  }
  status = "Applying " + model + " settings";
}

void ApplicationController::controlLightCrafter4500(slstudio::LightCrafter4500Action action) {
  if (processing() || scan.index() >= 0 || projector)
    throw std::logic_error("Stop capture and close projector outputs before controlling LC4500 playback");
  if (!slstudio::lightCrafterAvailable(slstudio::OutputBackend::LightCrafter4500))
    throw std::logic_error("LC4500 backend unavailable in this build");
  switch (action) {
    case slstudio::LightCrafter4500Action::Start: lc4500PlaybackMessage = "LC4500 playback started"; break;
    case slstudio::LightCrafter4500Action::Pause: lc4500PlaybackMessage = "LC4500 playback paused"; break;
    case slstudio::LightCrafter4500Action::Stop: lc4500PlaybackMessage = "LC4500 playback stopped"; break;
    case slstudio::LightCrafter4500Action::Blank: lc4500PlaybackMessage = "LC4500 blanked"; break;
    default: throw std::invalid_argument("Unknown LC4500 playback action");
  }
  lc4500PlaybackJob = std::async(std::launch::async, [action] {
    return slstudio::controlLightCrafter4500(action);
  });
  lc4500Playback.reset();
  lc4500PlaybackError.clear();
  status = "Controlling LC4500 playback";
}

void ApplicationController::beginCapture(bool geometricCalibration, bool continuous) {
  if (scan.active() || scan.capture()) throw std::logic_error("Stop capture before starting another scan");
  if (intensityCalibration.worker) throw std::logic_error("Stop intensity measurement before scanning");
  if (!projector) throw std::runtime_error("Open a projector output first");
  if (projectorJob.valid()) throw std::logic_error("Wait for projector programming to finish");
  const bool hardwareSequence = settings.hardwareTrigger && projector->capabilities().hardwareSequence;
  if (settings.hardwareTrigger && !nativeCameraSelected())
    throw std::invalid_argument("Hardware triggering requires a native camera driver");
  if (hardwareSequence) {
    if (!projector->capabilities().triggerOutput)
      throw std::invalid_argument("This projector cannot trigger the camera");
    if (!std::isfinite(settings.exposure) || settings.exposure <= 0 ||
        static_cast<double>(settings.exposure) * 1000.0 > settings.projectorExposureUs)
      throw std::invalid_argument("Native camera exposure must be positive and fit within projector exposure");
    slstudio::ProjectorSequenceSettings timing;
    timing.exposure = std::chrono::microseconds(settings.projectorExposureUs);
    timing.framePeriod = std::chrono::microseconds(settings.projectorPeriodUs);
    slstudio::validateProjectorSequence(timing);
  }
  GeometricCalibrationData calibration;
  if (geometricCalibration) {
    calibration.screenResX = projectorSize().width;
    calibration.screenResY = projectorSize().height;
    if (settings.cameraWidth == 0 || settings.cameraHeight == 0)
      throw std::runtime_error("Wait for the full-resolution camera preview before capturing a pose");
    calibration.frameWidth = settings.cameraWidth;
    calibration.frameHeight = settings.cameraHeight;
  } else if (!calibration.load(settings.geometricCalibrationPath.data())) {
    throw std::runtime_error("Cannot load geometric calibration");
  }
  if (cv::Size(calibration.screenResX, calibration.screenResY) != projectorSize())
    throw std::runtime_error("Projector resolution does not match geometric calibration");
  if (!geometricCalibration) ensurePipeline();
  const auto patterns = projectorPatterns(geometricCalibration);
  const auto requestedSettings = makeCaptureSettings(*this, {calibration.frameWidth, calibration.frameHeight});
  setCameraRequested(true);
  if (!cameraWorker || cameraResolution != cv::Size(calibration.frameWidth, calibration.frameHeight)) {
    releaseCameraWorker();
    cameraJob = acquisition.openCamera(cameraFactory(requestedSettings));
    cameraWorker = &acquisition;
    cameraResolution = {calibration.frameWidth, calibration.frameHeight};
  }
  slstudio::ProjectorSequenceSettings timing;
  timing.exposure = std::chrono::microseconds(settings.projectorExposureUs);
  timing.framePeriod = std::chrono::microseconds(settings.projectorPeriodUs);
  scan.begin(static_cast<unsigned int>(patterns.size()), geometricCalibration, continuous,
             acquisition);
  submitAcquisition(acquisition.captureSequence(patterns, std::chrono::milliseconds(settings.settleMilliseconds),
                                               continuous, hardwareSequence, timing));
  status = "Capturing";
}

void ApplicationController::tick() {
  if (projectorPresentationController) projectorPresentationController->process();
  if (recoveringAcquisition) {
    for (auto job = pendingAcquisitionJobs.begin(); job != pendingAcquisitionJobs.end();) {
      if (job->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        ++job;
        continue;
      }
      try { job->get(); }
      catch (const std::exception &error) {
        status += std::string("; acquisition cleanup: ") + error.what();
        std::fprintf(stderr, "SLStudio acquisition cleanup: %s\n", error.what());
      }
      job = pendingAcquisitionJobs.erase(job);
    }
    if (pendingAcquisitionJobs.empty()) {
      projector = acquisition.projector();
      if (!projector) projectorPresentationController.reset();
      recoveringAcquisition = false;
    }
    return;
  }
  for (auto job = pendingAcquisitionJobs.begin(); job != pendingAcquisitionJobs.end();) {
    if (job->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ++job;
      continue;
    }
    auto finished = std::move(*job);
    job = pendingAcquisitionJobs.erase(job);
    finished.get();
  }
  if (cameraJob.valid() && cameraJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    cameraJob.get();
  if (acquisitionJob.valid() && acquisitionJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    acquisitionJob.get();
  if (cameraDiscoveryJob.valid() && cameraDiscoveryJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const bool current = settings.cameraInterface == cameraDiscoveryInterface && settings.cameraIndex == cameraDiscoveryIndex;
    try {
      auto result = cameraDiscoveryJob.get();
      if (current) {
        cameraList = std::move(result.devices);
        settings.cameraIndex = result.index;
        frameTriggerSources = std::move(result.triggerSources);
        triggerSourcesError = std::move(result.triggerError);
        if (result.triggersInspected && triggerSourcesError.empty() && settings.cameraIndex >= 0 &&
            static_cast<std::size_t>(settings.cameraIndex) < cameraList.size())
          triggerSourcesCamera = cameraList[settings.cameraIndex];
        status = !triggerSourcesError.empty() ? triggerSourcesError :
            nativeCameraSelected() && cameraList.empty() ? "No cameras found" : "Camera discovery complete";
        if (cameraRequested && triggerSourcesError.empty() && (!nativeCameraSelected() || !cameraList.empty())) startCamera();
      }
    } catch (const std::exception &error) {
      triggerSourcesError = error.what();
      status = triggerSourcesError;
      setCameraRequested(false);
    }
    if (!current) refreshCameras();
  }
  if (lc4500FirmwareJob.valid() && lc4500FirmwareJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      lc4500Firmware = lc4500FirmwareJob.get();
      lc4500FirmwareError.clear();
    } catch (const std::exception &error) {
      lc4500FirmwareError = error.what();
      status = lc4500FirmwareError;
    }
  }
  if (lc4500PlaybackJob.valid() && lc4500PlaybackJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      lc4500Playback = lc4500PlaybackJob.get();
      lc4500PlaybackError.clear();
      if (!lc4500PlaybackMessage.empty()) status = lc4500PlaybackMessage;
    } catch (const std::exception &error) {
      lc4500Playback.reset();
      lc4500PlaybackError = status = error.what();
    }
    lc4500PlaybackMessage.clear();
    lc4500PlaybackPollAfter = Clock::now() + std::chrono::seconds(1);
  }
  if (cameraWorker && !intensityCalibration.worker) {
    if (auto frame = takeCameraFrame()) {
      if (status == "Starting camera") status = "Camera active";
      if (settings.cameraWidth == 0 || settings.cameraHeight == 0) {
        if (!settings.cameraWidth) settings.cameraWidth = frame->cols;
        if (!settings.cameraHeight) settings.cameraHeight = frame->rows;
      }
      appendCameraPreview(*frame);
    }
    if (!scan.active() && !cameraFramePending() && cameraReady())
      requestCameraFrame();
  }
  if (projectorJob.valid() && projectorJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    projectorJob.get();
    projector = acquisition.projector();
    if (projector) {
      if (!settings.projectorWidth) settings.projectorWidth = projector->resolution().width;
      if (!settings.projectorHeight) settings.projectorHeight = projector->resolution().height;
    } else {
      projectorPresentationController.reset();
    }
    if (!projectorJobMessage.empty()) status = projectorJobMessage;
    if (std::exchange(scanAfterProjectorOpen, false)) beginCapture(false, settings.continuousCapture);
  }
  if (intensityCalibration.worker) { tickIntensity(); return; }
  if (geometricCalibrationJob.valid() && geometricCalibrationJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto result = geometricCalibrationJob.get();
    pendingGeometricCalibration = std::move(result.calibration);
    geometricCalibrationDiagnostics = std::move(result.poses);
    geometricCalibrationDialog.invalidatePreview();
    status = result.error.empty() ? "Geometric Calibration ready for review" : result.error;
  }
  if (ros2Output) {
    try { ros2Output->checkError(); }
    catch (const std::exception &error) {
      ros2Output.reset();
      ros2OutputError = status = error.what();
    }
  }
  if (decoder && triangulator) {
    decoder->checkError();
    if (auto result = triangulator->tryTake()) {
      cloud = std::move(result->cloud);
      renderer.setCloud(cloud);
      cloudTimes.push_back(Clock::now());
      if (cloudTimes.size() > 11) cloudTimes.pop_front();
      status = std::to_string(cloud->points.size()) + " points (sequence " + std::to_string(result->sequence) + ")";
      if (ros2Output) {
        try { ros2Output->submit(cloud, std::chrono::system_clock::now()); }
        catch (const std::exception &error) {
          ros2Output.reset();
          ros2OutputError = status = error.what();
        }
      }
    }
    if (sequencePending && decoder->trySubmit(frames)) {
      sequencePending = false;
      frames.clear();
    }
  }
  if (!scan.active()) return;
  if (auto sequence = acquisition.tryTakeSequence()) {
    auto completed = std::move(sequence->frames);
    if (scan.geometricCalibration()) {
      CodecSettings requestedSettings;
      requestedSettings.direction = CodecDirBoth;
      setLastSequence(FrameSequence{completed, requestedSettings});
      invalidateGeometricCalibration();
      geometricCalibrationPoses.push_back(std::move(completed));
      geometricCalibrationPoseNames.push_back("Capture " + std::to_string(geometricCalibrationPoses.size()));
      geometricCalibrationDialog.selectedPose = static_cast<int>(geometricCalibrationPoses.size()) - 1;
      status = "Captured calibration pose " + std::to_string(geometricCalibrationPoses.size());
    } else {
      setLastSequence(FrameSequence{completed, settings.reconstruction.coding});
      if (!decoder->trySubmit(completed)) throw std::runtime_error("Decoder worker stopped");
      status = "Latest sequence submitted for decoding";
    }
    if (!scan.continuous()) scan.stop();
  }
}

void ApplicationController::activateMeasuredLut(bool replaceExisting) {
  if (processing() || scan.index() >= 0) throw std::logic_error("Stop capture before changing intensity compensation");
  if (!intensityCalibration.result || intensityCalibration.compensated) throw std::logic_error("An uncompensated intensity measurement is required");
  if (settings.intensityLut && !replaceExisting) throw std::logic_error("Confirm replacement of the stored projector/camera intensity curve");
  const auto measured = analyzeIntensity(intensityCalibration.samples);
  const auto path = dataPath("intensity-lut.yml");
  std::filesystem::create_directories(path.parent_path());
  saveIntensityLut(path, intensityCalibration.samples, measured);
  settings.intensityLut = measured;
  settings.intensityCompensationEnabled = true;
  saveSettings();
  status = "Intensity Calibration saved; intensity compensation enabled";
  intensityCalibration.open = false;
}

void ApplicationController::showProjectorLevel(std::uint8_t level) {
  if (!projector) throw std::logic_error("Open a projector output first");
  projectorJobMessage.clear();
  projectorJob = acquisition.showSolid(
      settings.intensityCompensationEnabled && settings.intensityLut ? settings.intensityLut->inverseLut[level] : level,
      std::chrono::milliseconds(settings.settleMilliseconds));
}

void ApplicationController::prepareIntensityCalibration(CaptureWorker::Reader reader) {
  if (processing() || scan.index() >= 0) throw std::runtime_error("Stop capture and processing before intensity measurement");
  if (!projector) throw std::runtime_error("Open a projector output first");
  if (!nativeCameraSelected() && settings.autoExposure) throw std::runtime_error("Disable automatic exposure for intensity measurement");
  if (!std::isfinite(settings.exposure) || !std::isfinite(settings.gain) || settings.settleMilliseconds < 0 || settings.settleMilliseconds > 1000 ||
      settings.discardFrames < 0 || settings.discardFrames > 8)
    throw std::invalid_argument("Invalid camera exposure, gain, settling or discard settings");
  if (settings.cameraWidth <= 0 || settings.cameraHeight <= 0 || settings.cameraWidth > 16384 || settings.cameraHeight > 16384)
    throw std::invalid_argument("Invalid camera dimensions");
  auto requestedSettings = makeCaptureSettings(*this, {settings.cameraWidth, settings.cameraHeight});
  if (!reader) {
    setCameraRequested(true);
    if (!previewCamera) previewCamera = cameraFactory(requestedSettings);
  }
  requestedSettings.autoExposure = false;
  requestedSettings.requireManual = true;
  requestedSettings.nativeSettings.trigger = slstudio::CameraTrigger::Software;
  requestedSettings.nativeSettings.sequenceMarker.policy = slstudio::SequenceMarkerPolicy::Disabled;
  releaseCameraWorker();
  cameraJob = acquisition.openCamera(cameraFactory(requestedSettings, std::move(reader)));
  showProjectorLevel(128);
  intensityCalibration.prepare(acquisition);
  status = "Preparing intensity measurement";
}

void ApplicationController::startIntensityCalibration(CaptureWorker::Reader reader) {
  if (settings.intensityLevels < 5 || settings.intensityLevels > 65 || settings.intensityAverages < 1 || settings.intensityAverages > 32)
    throw std::invalid_argument("Use 5-65 levels and 1-32 frames per level");
  if (!intensityCalibration.worker) {
    prepareIntensityCalibration(std::move(reader));
    intensityCalibration.measureRequested = true;
    return;
  }
  intensityCalibration.start(settings.intensityCompensationEnabled && settings.intensityLut.has_value(),
                 settings.intensityLevels, settings.intensityAverages);
  showProjectorLevel(0);
  status = "Measuring black baseline";
}

void ApplicationController::stopIntensity() {
  intensityCalibration.cancel();
  submitAcquisition(acquisition.stopSequence());
  if (cameraRequested) {
    cameraJob = acquisition.openCamera(previewCamera);
    cameraWorker = &acquisition;
    cameraResolution = {settings.cameraWidth, settings.cameraHeight};
  } else cameraJob = acquisition.closeCamera();
}

void ApplicationController::tickIntensity() {
  auto frame = intensityCalibration.worker->tryTake();
  if (frame) {
    appendCameraPreview(*frame);
    const auto progress = intensityCalibration.consume(std::move(*frame));
    if (progress == IntensityCalibrationDialog::Progress::Complete) {
      stopIntensity();
      try {
        intensityCalibration.result = analyzeIntensity(intensityCalibration.samples);
        status = intensityCalibration.compensated ? "Compensated measurement complete" : "Uncompensated measurement complete";
      } catch (const std::exception &error) { status = error.what(); }
      return;
    }
    if (progress == IntensityCalibrationDialog::Progress::NextLevel) {
      showProjectorLevel(static_cast<std::uint8_t>(intensityCalibration.level()));
      status = "Measuring input " + std::to_string(intensityCalibration.level());
    }
  }
  if (intensityCalibration.measureRequested && !intensityCalibration.preview.empty() && intensityCalibration.worker->ready())
    startIntensityCalibration();
  intensityCalibration.requestIfDue();
}

void ApplicationController::openCalibrationDialog(bool intensityCalibrationDialog, CaptureWorker::Reader reader) {
  if (intensityCalibrationDialog) intensityCalibration.open = true;
  else geometricCalibrationDialog.open = true;
  if (!cameraWorker && !intensityCalibration.worker && !scanActive()) startCamera(std::move(reader));
}

void ApplicationController::startCamera(CaptureWorker::Reader reader) {
  setCameraRequested(true);
  if (cameraDiscoveryJob.valid()) {
    status = "Waiting for camera discovery";
    return;
  }
  if (intensityCalibration.worker || scan.capture() || scan.index() >= 0)
    throw std::logic_error("Stop capture before changing camera settings");
  if (!reader && nativeCameraSelected() && !triggerSourcesCamera && triggerSourcesError.empty()) {
    refreshCameraTriggerSources();
    if (cameraDiscoveryJob.valid()) return;
  }
  if (settings.cameraWidth < 0 || settings.cameraHeight < 0 ||
      settings.cameraWidth > 16384 || settings.cameraHeight > 16384)
    throw std::invalid_argument("Invalid camera dimensions");
  const auto requestedSettings = makeCaptureSettings(*this, {settings.cameraWidth, settings.cameraHeight});
  releaseCameraWorker();
  stopCamera();
  setCameraRequested(true);
  previewCamera = cameraFactory(requestedSettings, std::move(reader));
  cameraJob = acquisition.openCamera(previewCamera);
  cameraWorker = &acquisition;
  cameraResolution = {settings.cameraWidth, settings.cameraHeight};
  status = "Starting camera";
}

void ApplicationController::refreshCameras(std::function<CameraDiscoveryResult()> probe) {
  discoverCameras(true, std::move(probe));
}

void ApplicationController::discoverCameras(bool enumerate, std::function<CameraDiscoveryResult()> probe) {
  if (cameraDiscoveryJob.valid()) throw std::logic_error("Wait for camera discovery to finish");
  if (intensityCalibration.worker || scan.index() >= 0 || scan.capture())
    throw std::logic_error("Stop camera capture before discovering cameras");
  const auto backend = nativeCameraSelected() ?
      std::optional<slstudio::CameraBackend>(cameraBackends.at(settings.cameraInterface)) : std::nullopt;
  cameraDiscoveryInterface = settings.cameraInterface;
  cameraDiscoveryIndex = settings.cameraIndex;
  const bool requested = cameraRequested;
  auto previousCamera = releaseCameraWorker();
  stopCamera();
  setCameraRequested(requested);
  frameTriggerSources.clear();
  triggerSourcesCamera.reset();
  triggerSourcesError.clear();
  auto devices = enumerate ? std::vector<slstudio::CameraDescriptor>{} : cameraList;
  if (enumerate) cameraList.clear();
  cameraDiscoveryJob = std::async(std::launch::async,
      [backend, index = settings.cameraIndex, enumerate, devices = std::move(devices),
       previousCamera = std::move(previousCamera), probe = std::move(probe)]() mutable {
        if (previousCamera.valid()) previousCamera.get();
        if (probe) return probe();
        CameraDiscoveryResult result;
        result.index = index;
        if (!backend) return result;
        result.devices = enumerate ? slstudio::cameraDevices(*backend) : std::move(devices);
        result.index = std::clamp(index, 0, std::max(0, static_cast<int>(result.devices.size()) - 1));
        if (!result.devices.empty()) {
          result.triggersInspected = true;
          try {
            result.triggerSources = slstudio::cameraFrameTriggerSources(*backend, static_cast<std::size_t>(result.index));
          } catch (const std::exception &error) {
            result.triggerError = error.what();
          }
        }
        return result;
      });
  status = enumerate ? "Discovering cameras..." : "Inspecting camera trigger sources...";
}

void ApplicationController::refreshCameraTriggerSources() {
  if (!nativeCameraSelected() || settings.cameraIndex < 0 || static_cast<std::size_t>(settings.cameraIndex) >= cameraList.size()) {
    triggerSourcesCamera.reset();
    frameTriggerSources.clear();
    triggerSourcesError.clear();
    return;
  }
  const auto &device = cameraList[settings.cameraIndex];
  if (triggerSourcesCamera && triggerSourcesCamera->backend == device.backend &&
      triggerSourcesCamera->id == device.id && triggerSourcesCamera->serial == device.serial) return;
  discoverCameras(false);
}

bool ApplicationController::cameraSettingsControls() {
  bool changed = false;
  ImGui::BeginDisabled(cameraDiscoveryJob.valid());
  auto name = [](slstudio::CameraBackend backend) {
    switch (backend) {
    case slstudio::CameraBackend::Aravis: return "Aravis (USB3 Vision / GigE Vision)";
    case slstudio::CameraBackend::Ximea: return "XIMEA";
    case slstudio::CameraBackend::Spinnaker: return "FLIR Spinnaker";
    }
    return "Unknown";
  };
  const char *interfaceName = nativeCameraSelected() ? name(cameraBackends.at(settings.cameraInterface)) : "OpenCV webcam";
  if (ImGui::Button("Refresh cameras")) attempt([&] { refreshCameras(); });
  if (ImGui::BeginCombo("Driver", interfaceName)) {
    for (std::size_t index = 0; index < cameraBackends.size(); ++index)
      if (ImGui::Selectable(name(cameraBackends[index]), settings.cameraInterface == static_cast<int>(index))) {
        settings.cameraInterface = static_cast<int>(index);
        settings.cameraIndex = 0;
        settings.exposure = 16.667f;
        changed = true;
        attempt([&] { refreshCameras(); });
      }
    if (ImGui::Selectable("OpenCV webcam", !nativeCameraSelected())) {
      settings.cameraInterface = static_cast<int>(cameraBackends.size());
      settings.cameraIndex = 0;
      settings.exposure = -6.0f;
      settings.hardwareTrigger = false;
      changed = true;
      attempt([&] { refreshCameras(); });
    }
    ImGui::EndCombo();
  }
  if (nativeCameraSelected()) {
    auto deviceName = [&](std::size_t index) {
      return std::to_string(index) + ": " + cameraList[index].vendor + " " + cameraList[index].model;
    };
    const auto selectedName = settings.cameraIndex >= 0 && static_cast<std::size_t>(settings.cameraIndex) < cameraList.size() ?
        deviceName(static_cast<std::size_t>(settings.cameraIndex)) : "No camera selected";
    if (ImGui::BeginCombo("Device", selectedName.c_str())) {
      for (std::size_t index = 0; index < cameraList.size(); ++index)
        if (ImGui::Selectable(deviceName(index).c_str(), settings.cameraIndex == static_cast<int>(index))) {
          settings.cameraIndex = static_cast<int>(index);
          attempt([&] { refreshCameraTriggerSources(); });
          changed = true;
        }
      ImGui::EndCombo();
    }
  } else {
    ImGui::InputInt("Device", &settings.cameraIndex);
    changed |= ImGui::IsItemDeactivatedAfterEdit();
  }
  if (!nativeCameraSelected()) changed |= ImGui::Checkbox("Auto exposure", &settings.autoExposure);
  ImGui::InputFloat(nativeCameraSelected() ? "Exposure (ms)" : "Exposure (device)", &settings.exposure);
  changed |= ImGui::IsItemDeactivatedAfterEdit();
  ImGui::InputFloat("Gain", &settings.gain);
  changed |= ImGui::IsItemDeactivatedAfterEdit();
  ImGui::SliderInt("Settle time (ms)", &settings.settleMilliseconds, 0, 1000);
  changed |= ImGui::IsItemDeactivatedAfterEdit();
  ImGui::SliderInt("Discard frames", &settings.discardFrames, 0, 8);
  changed |= ImGui::IsItemDeactivatedAfterEdit();
  ImGui::EndDisabled();
  return changed;
}

bool ApplicationController::cameraTriggerControls() {
  bool changed = false;
  ImGui::BeginDisabled(cameraDiscoveryJob.valid());
  ImGui::BeginDisabled(!nativeCameraSelected());
  const bool available = std::find(frameTriggerSources.begin(), frameTriggerSources.end(), settings.frameTriggerSource.data()) !=
      frameTriggerSources.end();
  const std::string sourceName = !settings.hardwareTrigger ? "Software" :
      available ? settings.frameTriggerSource.data() : std::string(settings.frameTriggerSource.data()) + " (unavailable)";
  if (ImGui::BeginCombo("Trigger source", sourceName.c_str())) {
    if (ImGui::Selectable("Software", !settings.hardwareTrigger)) {
      changed |= settings.hardwareTrigger;
      settings.hardwareTrigger = false;
    }
    for (const auto &source : frameTriggerSources)
      if (ImGui::Selectable(source.c_str(), settings.hardwareTrigger && source == settings.frameTriggerSource.data())) {
        attempt([&] {
          if (source.size() >= settings.frameTriggerSource.size()) throw std::runtime_error("Camera trigger source name is too long");
          changed |= !settings.hardwareTrigger || source != settings.frameTriggerSource.data();
          std::snprintf(settings.frameTriggerSource.data(), settings.frameTriggerSource.size(), "%s", source.c_str());
          settings.hardwareTrigger = true;
        });
      }
    if (frameTriggerSources.empty())
      ImGui::TextDisabled(triggerSourcesError.empty() ? "No external sources available" : "Trigger input discovery failed");
    ImGui::EndCombo();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip(nativeCameraSelected() ?
        "Start each frame with a software command or a signal on the selected camera input." :
        "OpenCV webcam capture supports software triggering only.");
  if (!triggerSourcesError.empty()) ImGui::TextWrapped("%s", triggerSourcesError.c_str());
  ImGui::BeginDisabled(!settings.hardwareTrigger);
  int activation = settings.triggerRisingEdge ? 0 : 1;
  const char *activations[] = {"Rising edge", "Falling edge"};
  if (ImGui::Combo("Trigger activation", &activation, activations, IM_ARRAYSIZE(activations))) {
    settings.triggerRisingEdge = activation == 0;
    changed = true;
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Capture on the rising or falling edge of the input signal.");
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  return changed;
}

void ApplicationController::stopCapture() {
  sequencePending = false;
  scan.stop();
  scanAfterProjectorOpen = false;
  submitAcquisition(acquisition.stopSequence());
  stopReconstruction();
  frames.clear();
}
}
