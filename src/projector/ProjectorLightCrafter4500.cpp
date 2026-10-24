#include "ProjectorLightCrafter4500.h"
#include "lc4500tools.h"
#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_usb.h"
#include "LC4500API/dlpc350_firmware.h"

#include <fstream>
#include <filesystem>
#include <atomic>
#include <iterator>
#include <numeric>
#include <thread>
#include <opencv2/imgcodecs.hpp>

namespace slstudio {
namespace {
std::atomic_bool occupied{false};

void check(int result, const char *operation) {
  if (result < 0) throw std::runtime_error(std::string("LC4500: ") + operation);
}

struct Connection {
  explicit Connection(std::chrono::milliseconds timeout) {
    if (timeout.count() <= 0 || timeout > std::chrono::minutes(1))
      throw std::invalid_argument("Projector timeout must be in (0, 60000] ms");
    if (occupied.exchange(true)) throw std::runtime_error("LC4500 is already open");
    try {
      check(DLPC350_USB_Init(), "HID initialization failed");
      wait([] { return DLPC350_USB_Open() == 0; }, timeout, "USB connection");
    } catch (...) {
      DLPC350_USB_Close();
      DLPC350_USB_Exit();
      occupied.store(false);
      throw;
    }
  }
  ~Connection() { DLPC350_USB_Close(); DLPC350_USB_Exit(); occupied.store(false); }
  Connection(const Connection &) = delete;
  Connection &operator=(const Connection &) = delete;

  void stopSequence(std::chrono::milliseconds timeout) const {
    bool pattern = false;
    check(DLPC350_GetMode(&pattern), "display mode query failed");
    if (!pattern) return;
    unsigned int action = 0;
    check(DLPC350_GetPatternDisplay(&action), "sequence status failed");
    if (action == 0) return;
    check(DLPC350_PatternDisplay(0), "sequence stop failed");
    // The stop acknowledgement precedes completion of the sequencer shutdown.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    wait([] {
      unsigned int actual = 0;
      check(DLPC350_GetPatternDisplay(&actual), "sequence stop status failed");
      return actual == 0;
    }, timeout, "sequence stop");
  }

  void setMode(bool pattern, std::chrono::milliseconds timeout) const {
    bool actual = false;
    check(DLPC350_GetMode(&actual), "display mode query failed");
    if (actual == pattern) return;
    stopSequence(timeout);
    // Leave internal test input before entering pattern mode, not before video mode.
    if (pattern) check(DLPC350_SetInputSource(0, 0), "HDMI input selection failed");
    check(DLPC350_SetMode(pattern), "display mode change failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wait([pattern] {
      bool current = !pattern;
      check(DLPC350_GetMode(&current), "display mode status failed");
      return current == pattern;
    }, timeout, "display mode change");
  }

  template <typename Predicate>
  void wait(Predicate ready, std::chrono::milliseconds timeout, const char *operation) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!ready()) {
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error(std::string("LC4500 timeout: ") + operation);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

void waitForValidation(const Connection &connection, std::chrono::milliseconds timeout) {
  connection.wait([] {
    bool ready = false;
    unsigned int flags = 0;
    check(DLPC350_CheckPatLutValidate(&ready, &flags), "LUT validation query failed");
    if (ready && flags != 0)
      throw std::runtime_error("LC4500 sequence validation failed; flags: " + std::to_string(flags));
    return ready;
  }, timeout, "LUT validation");
}

void validateSequence(const Connection &connection, std::chrono::milliseconds timeout) {
  check(DLPC350_StartPatLutValidate(), "LUT validation start failed");
  waitForValidation(connection, timeout);
}

LightCrafter4500Status readStatus() {
  LightCrafter4500Status status;
  check(DLPC350_GetPowerMode(&status.standby), "power status failed");
  check(DLPC350_GetMode(&status.patternMode), "display mode query failed");
  bool sequencer = false, red = false, green = false, blue = false;
  check(DLPC350_GetLedEnables(&sequencer, &red, &green, &blue), "LED status failed");
  status.ledsDisabled = !sequencer && !red && !green && !blue;
  if (status.patternMode) {
    unsigned int action = 0;
    check(DLPC350_GetPatternDisplay(&action), "sequence status failed");
    switch (action) {
      case 0: status.playback = LightCrafter4500Playback::Stopped; break;
      case 1: status.playback = LightCrafter4500Playback::Paused; break;
      case 2: status.playback = LightCrafter4500Playback::Running; break;
      default: throw std::runtime_error("LC4500 returned an unknown playback state");
    }
    check(DLPC350_CheckPatLutValidate(&status.validationReady, &status.validationFlags),
        "LUT validation query failed");
  }
  return status;
}

class ProjectorLightCrafter4500 final : public Projector {
public:
  explicit ProjectorLightCrafter4500(const ProjectorOpenSettings &settings) : connection(settings.timeout), settings(settings) {
    check(DLPC350_SetPowerMode(false), "wake failed");
    connection.wait([] {
      bool standby = true;
      check(DLPC350_GetPowerMode(&standby), "power status failed");
      return !standby;
    }, settings.timeout, "wake");
    initializeLedDrive();
    blank();
  }
  ~ProjectorLightCrafter4500() override { if (!recoveryRequired) { try { blank(); } catch (...) {} } }
  cv::Size resolution() const override { return {1824, 1140}; }
  ProjectorGeometry geometry() const override { return {resolution(), {912, 1140}, PixelLayout::Diamond}; }
  ProjectorCapabilities capabilities() const override { return {true, true, true}; }

  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    auto prepared = prepareMonochromeProjectorPatterns(patterns, geometry(), 48);
    unsigned int available = 0;
    check(DLPC350_GetNumImagesInFlash(&available), "flash image count failed");
    if ((prepared.size() + 2) / 3 > available)
      throw std::runtime_error("LC4500 flash does not contain enough resident images");
    stopSequence();
    nativePatterns = std::move(prepared);
    configured = false;
  }

  void programPatterns(const std::vector<cv::Mat> &patterns,
                       const ProjectorProgrammingProgress &progress) override {
    const auto report = [&](float fraction, const char *stage) {
      if (progress) progress(fraction, stage);
    };
    report(0, "Preparing patterns");
    if (recoveryRequired) throw std::runtime_error("LC4500 requires flash recovery before further operations");
    if (settings.flashBackupDirectory.empty())
      throw std::runtime_error("Select an LC4500 metadata backup directory before programming flash");
    auto prepared = prepareMonochromeProjectorPatterns(patterns, geometry(), 48);
    const auto splash = packLightCrafter4500Patterns(prepared);
    unsigned int appVersion = 0, apiVersion = 0, softwareVersion = 0, sequenceVersion = 0;
    check(DLPC350_GetVersion(&appVersion, &apiVersion, &softwareVersion, &sequenceVersion), "firmware version failed");
    if (appVersion != 0x04040000)
      throw std::runtime_error("LC4500 pattern programming supports firmware 4.4.0 only");
    stopSequence();
    report(0.05f, "Reading flash metadata");
    std::vector<unsigned char> originalTable(sizeof(FLASH_TABLE));
    const auto readWord = [&](unsigned int offset) {
      unsigned int value = 0;
      check(DLPC350_MemRead(lc4500FlashBase + sectorSize + offset, &value), "metadata sector read failed");
      if ((offset == 0 && value != FLASHTABLE_APP_SIGNATURE) ||
          (offset == 4 && value != lc4500FlashBase) || (offset == 8 && value != 0x13))
        throw std::runtime_error("Unsupported LC4500 flash table; no flash was erased");
      for (unsigned int byte = 0; byte < 4; ++byte)
        originalTable[offset + byte] = static_cast<unsigned char>(value >> (byte * 8));
    };
    for (unsigned int offset = 0; offset < originalTable.size(); offset += 4) {
      readWord(offset);
      report(0.05f + 0.05f * (offset + 4) / originalTable.size(), "Reading flash metadata");
    }
    auto update = prepareLightCrafter4500FlashUpdate(appVersion, originalTable, splash);
    if (update.tableSector != originalTable) {
      const auto patchedHeader = update.tableSector;
      originalTable.resize(sectorSize);
      for (unsigned int offset = sizeof(FLASH_TABLE); offset < sectorSize; offset += 4) {
        readWord(offset);
        report(0.1f + 0.2f * (offset + 4) / sectorSize, "Reading metadata sector for backup");
      }
      update.tableSector = originalTable;
      std::copy(patchedHeader.begin(), patchedHeader.end(), update.tableSector.begin());
    }
    report(0.3f, "Saving metadata backup");
    const auto backup = saveMetadataBackup(originalTable);
    nativePatterns.clear();
    configured = false;
    bool programming = false;
    bool tableTouched = false;
    try {
      report(0.32f, "Entering flash programming mode");
      check(DLPC350_EnterProgrammingMode(), "entering programming mode failed");
      programming = true;
      reconnect(true);
      unsigned short manufacturer = 0;
      unsigned long long device = 0;
      check(DLPC350_GetFlashManID(&manufacturer), "flash manufacturer query failed");
      check(DLPC350_GetFlashDevID(&device), "flash device query failed");
      if (manufacturer != 0x0020 || device != 0x227e)
        throw std::runtime_error("Refusing to program an unrecognized LC4500 flash device");
      check(DLPC350_SetFlashType(0), "flash type settings failed");
      report(0.35f, "Verifying original metadata");
      verify(sectorSize, originalTable);
      for (unsigned int offset = 0; offset < update.patternData.size(); offset += sectorSize) {
        report(0.36f + 0.09f * offset / update.patternData.size(), "Erasing pattern flash");
        erase(update.patternOffset + offset);
      }
      report(0.45f, "Uploading pattern data");
      upload(update.patternOffset, update.patternData.data(), static_cast<unsigned int>(update.patternData.size()),
          [&](float fraction, const char *) { report(0.45f + 0.4f * fraction, "Uploading pattern data"); });
      report(0.86f, "Verifying pattern checksum");
      verify(update.patternOffset, update.patternData);
      if (update.tableSector != originalTable) {
        report(0.9f, "Updating flash metadata");
        tableTouched = true;
        erase(sectorSize);
        upload(sectorSize, update.tableSector.data(), sectorSize);
        verify(sectorSize, update.tableSector);
        tableTouched = false;
      }
      report(0.95f, "Restarting projector");
      check(DLPC350_ExitProgrammingMode(), "leaving programming mode failed");
      programming = false;
      reconnect(false);
      stopSequence();
      check(DLPC350_SetLedEnables(false, false, false, false), "post-programming blanking failed");
      initializeLedDrive();
    } catch (const std::exception &error) {
      if (programming) {
        if (tableTouched) {
          try {
            report(0.9f, "Recovering original flash metadata");
            erase(sectorSize);
            upload(sectorSize, originalTable.data(), sectorSize);
            verify(sectorSize, originalTable);
          } catch (const std::exception &restoreError) {
            recoveryRequired = true;
            throw std::runtime_error(std::string(error.what()) + "; metadata recovery failed: " +
                restoreError.what() + "; device left in programming mode; backup: " + backup.string());
          }
        }
        if (DLPC350_ExitProgrammingMode() < 0) {
          recoveryRequired = true;
          throw std::runtime_error(std::string(error.what()) +
              "; leaving programming mode also failed; backup: " + backup.string());
        }
      } else recoveryRequired = true;
      throw;
    }
    nativePatterns = std::move(prepared);
    report(1, "Programming complete");
  }

  void present(std::size_t index) override {
    if (index >= nativePatterns.size()) throw std::out_of_range("LC4500 pattern index");
    configure(ProjectorSequenceSettings{}, index);
    startSequence();
  }
  void blank(bool white = false) override {
    stopSequence();
    if (white) {
      setMode(false);
      check(DLPC350_SetTPGColor(1023, 1023, 1023, 1023, 1023, 1023), "white field settings failed");
      check(DLPC350_SetTPGSelect(0), "solid field selection failed");
      check(DLPC350_SetInputSource(1, 0), "test-pattern input selection failed");
    }
    check(DLPC350_SetLedEnables(false, white, white, white), "blanking failed");
    configured = false;
  }
  void showSolid(std::uint8_t level) override {
    stopSequence();
    setMode(false);
    const auto value = static_cast<unsigned short>((static_cast<unsigned int>(level) * 1023u + 127u) / 255u);
    check(DLPC350_SetTPGColor(value, value, value, value, value, value), "solid-field settings failed");
    check(DLPC350_SetTPGSelect(0), "solid field selection failed");
    check(DLPC350_SetInputSource(1, 0), "test-pattern input selection failed");
    check(DLPC350_SetLedEnables(false, true, true, true), "solid-field LED settings failed");
    configured = false;
  }
  void configureSequence(const ProjectorSequenceSettings &timing) override {
    configure(timing, nativePatterns.size());
  }
  void startSequence() override {
    if (!configured) throw std::logic_error("Configure LC4500 timing before starting");
    check(DLPC350_PatternDisplay(2), "sequence start failed");
  }
  void stopSequence() override {
    if (recoveryRequired) throw std::runtime_error("LC4500 requires flash recovery before further operations");
    connection.stopSequence(settings.timeout);
  }

private:
  static constexpr unsigned int sectorSize = lc4500FlashSectorSize;
  Connection connection;
  ProjectorOpenSettings settings;
  std::vector<cv::Mat1b> nativePatterns;
  bool configured = false;
  bool recoveryRequired = false;

  void initializeLedDrive() {
    check(DLPC350_SetLEDPWMInvert(false), "LED polarity failed");
    const auto current = lightCrafter4500LedPwm(50);
    check(DLPC350_SetLedCurrents(current, current, current), "LED current settings failed");
  }

  std::filesystem::path saveMetadataBackup(const std::vector<unsigned char> &bytes) const {
    const std::filesystem::path parent(settings.flashBackupDirectory);
    std::filesystem::create_directories(parent);
    const auto directory = parent / ("lc4500-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(directory))
      throw std::runtime_error("Cannot create a unique LC4500 metadata backup directory");
    std::filesystem::permissions(directory, std::filesystem::perms::owner_all);
    const auto path = directory / (bytes.size() == sectorSize ? "flash-table-sector.bin" : "flash-table-header.bin");
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    file.close();
    if (!file) throw std::runtime_error("Cannot save LC4500 metadata backup: " + path.string());
    return path;
  }
  void reconnect(bool programming) {
    DLPC350_USB_Close();
    connection.wait([programming] {
      if (DLPC350_USB_Open() < 0) return false;
      unsigned char status = 0;
      unsigned int app = 0, api = 0, software = 0, sequence = 0;
      const bool ready = programming
          ? DLPC350_GetBLStatus(&status) >= 0 && (status & 0x80) != 0 && (status & STAT_BIT_FLASH_BUSY) == 0
          : DLPC350_GetVersion(&app, &api, &software, &sequence) >= 0 && app == 0x04040000;
      if (!ready) DLPC350_USB_Close();
      return ready;
    }, settings.timeout, "USB reconnect");
  }
  void verify(unsigned int address, const std::vector<unsigned char> &bytes) {
    check(DLPC350_SetFlashAddr(address), "checksum address failed");
    check(DLPC350_SetUploadSize(static_cast<unsigned int>(bytes.size())), "checksum size failed");
    check(DLPC350_CalculateFlashChecksum(), "checksum calculation failed");
    waitForFlash();
    unsigned int actual = 0;
    check(DLPC350_GetFlashChecksum(&actual), "checksum query failed");
    const auto expected = std::accumulate(bytes.begin(), bytes.end(), std::uint32_t{0});
    if (actual != expected) throw std::runtime_error("LC4500 flash checksum mismatch; programming stopped");
  }
  void waitForFlash() {
    connection.wait([] {
      unsigned char status = 0;
      check(DLPC350_GetBLStatus(&status), "flash status failed");
      return (status & STAT_BIT_FLASH_BUSY) == 0;
    }, settings.timeout, "flash operation");
  }
  void erase(unsigned int address) {
    check(DLPC350_SetFlashAddr(address), "flash address failed");
    check(DLPC350_FlashSectorErase(), "flash erase failed");
    waitForFlash();
  }
  void upload(unsigned int address, unsigned char *bytes, unsigned int size,
              const ProjectorProgrammingProgress &progress = {}) {
    check(DLPC350_SetFlashAddr(address), "upload address failed");
    check(DLPC350_SetUploadSize(size), "upload size failed");
    unsigned int offset = 0;
    while (offset < size) {
      const auto sent = DLPC350_UploadData(bytes + offset, size - offset);
      if (sent <= 0 || static_cast<unsigned int>(sent) > size - offset)
        throw std::runtime_error("LC4500 flash upload made no valid progress");
      offset += static_cast<unsigned int>(sent);
      if (progress) progress(static_cast<float>(offset) / size, "Uploading");
    }
    waitForFlash();
  }
  void setMode(bool pattern) {
    connection.setMode(pattern, settings.timeout);
  }
  void configure(const ProjectorSequenceSettings &timing, std::size_t selected) {
    validateLightCrafter4500Sequence(timing);
    if (nativePatterns.empty()) throw std::logic_error("Prepare LC4500 patterns before configuring timing");
    const bool single = selected < nativePatterns.size();
    const auto count = single ? std::size_t(1) : nativePatterns.size();
    const auto images = single ? std::size_t(1) : (count + 2) / 3;
    stopSequence();
    configured = false;
    setMode(true);
    check(DLPC350_SetLedEnables(true, true, true, true), "sequencer LED selection failed");
    check(DLPC350_ClearPatLut(), "LUT reset failed");
    for (std::size_t index = 0; index < count; ++index)
      check(DLPC350_AddToPatLut(0, static_cast<int>((single ? selected : index) % 3),
          8, 7, false, true, index % 3 == 0, false), "pattern LUT entry failed");
    check(DLPC350_SetPatternDisplayMode(false), "flash pattern mode failed");
    check(DLPC350_SetPatternConfig(static_cast<unsigned int>(count), timing.repeat,
        static_cast<unsigned int>(count), static_cast<unsigned int>(images)), "sequence settings failed");
    check(DLPC350_SetExposure_FramePeriod(static_cast<unsigned int>(timing.exposure.count()),
        static_cast<unsigned int>(timing.framePeriod.count())), "sequence timing failed");
    check(DLPC350_SetPatternTriggerMode(1), "internal trigger mode failed");
    check(DLPC350_SetTrigOutConfig(1, false, 0xbb, 0xbb), "camera trigger output failed");
    check(DLPC350_SetTrigOutConfig(2, false, 0xbb, 0), "sequence trigger output failed");
    check(DLPC350_SendPatLut(), "pattern LUT upload failed");
    std::vector<unsigned char> imageLut(images);
    if (single) imageLut[0] = static_cast<unsigned char>(selected / 3);
    else std::iota(imageLut.begin(), imageLut.end(), 0);
    check(DLPC350_SendImageLut(imageLut.data(), static_cast<unsigned int>(imageLut.size())), "image LUT upload failed");
    validateSequence(connection, settings.timeout);
    configured = true;
  }
};
}

std::unique_ptr<Projector> openLightCrafter4500(const ProjectorOpenSettings &settings) {
  return std::make_unique<ProjectorLightCrafter4500>(settings);
}

LightCrafter4500Firmware lightCrafter4500Firmware(const ProjectorOpenSettings &settings) {
  Connection connection(settings.timeout);
  LightCrafter4500Firmware versions{};
  check(DLPC350_GetVersion(&versions[0], &versions[1], &versions[2], &versions[3]), "firmware version query failed");
  return versions;
}

LightCrafter4500Status lightCrafter4500Status(const ProjectorOpenSettings &settings) {
  Connection connection(settings.timeout);
  return readStatus();
}

LightCrafter4500Status controlLightCrafter4500(LightCrafter4500Action action,
                                             const ProjectorOpenSettings &settings) {
  Connection connection(settings.timeout);
  if (action == LightCrafter4500Action::Blank) {
    check(DLPC350_SetLedEnables(false, false, false, false), "blanking failed");
    connection.stopSequence(settings.timeout);
    return readStatus();
  }
  const auto current = readStatus();
  if (current.standby || !current.patternMode)
    throw std::logic_error("LC4500 playback requires an awake projector in pattern mode");
  switch (action) {
    case LightCrafter4500Action::Start:
      if (current.playback == LightCrafter4500Playback::Stopped)
        validateSequence(connection, settings.timeout);
      else
        waitForValidation(connection, settings.timeout);
      check(DLPC350_PatternDisplay(2), "sequence start failed");
      break;
    case LightCrafter4500Action::Pause:
      if (current.playback != LightCrafter4500Playback::Running)
        throw std::logic_error("LC4500 pause requires a running sequence");
      check(DLPC350_PatternDisplay(1), "sequence pause failed");
      break;
    case LightCrafter4500Action::Stop:
      connection.stopSequence(settings.timeout);
      break;
    default:
      throw std::invalid_argument("Unknown LC4500 playback action");
  }
  if (action != LightCrafter4500Action::Stop) {
    const auto expected = action == LightCrafter4500Action::Start ? 2u : 1u;
    connection.wait([expected] {
      unsigned int actual = 0;
      check(DLPC350_GetPatternDisplay(&actual), "sequence status failed");
      return actual == expected;
    }, settings.timeout, "playback state change");
  }
  if (action == LightCrafter4500Action::Start)
    check(DLPC350_SetLedEnables(true, false, false, false), "sequence LED control failed");
  return readStatus();
}

void configureLightCrafter4500(const LightCrafter4500Settings &requestedSettings, const ProjectorOpenSettings &settings) {
  Connection connection(settings.timeout);
  try {
    check(DLPC350_SetPowerMode(false), "wake failed");
    connection.wait([] {
      bool standby = true;
      check(DLPC350_GetPowerMode(&standby), "power status failed");
      return !standby;
    }, settings.timeout, "wake");
    connection.stopSequence(settings.timeout);
    if (requestedSettings.standby) {
      check(DLPC350_SetLedEnables(false, false, false, false), "LED blanking failed");
      check(DLPC350_SetPowerMode(true), "standby failed");
      return;
    }
    check(DLPC350_SetLedEnables(false, false, false, false), "mode-change blanking failed");
    check(DLPC350_SetLEDPWMInvert(false), "LED polarity failed");
    check(DLPC350_SetLedCurrents(lightCrafter4500LedPwm(static_cast<std::uint8_t>(requestedSettings.ledCurrent[0])),
        lightCrafter4500LedPwm(static_cast<std::uint8_t>(requestedSettings.ledCurrent[1])),
        lightCrafter4500LedPwm(static_cast<std::uint8_t>(requestedSettings.ledCurrent[2]))),
        "LED current settings failed");
    check(DLPC350_SetLongAxisImageFlip(requestedSettings.flipLongAxis), "long-axis orientation failed");
    check(DLPC350_SetShortAxisImageFlip(requestedSettings.flipShortAxis), "short-axis orientation failed");
    const bool hdmiPatterns = requestedSettings.displayMode == LightCrafter4500Mode::HdmiPatterns;
    const bool flashPatterns = requestedSettings.displayMode == LightCrafter4500Mode::FlashPatterns;
    const bool patternMode = hdmiPatterns || flashPatterns;
    connection.setMode(patternMode, settings.timeout);
    if (!patternMode) {
      const bool test = requestedSettings.displayMode == LightCrafter4500Mode::TestPattern;
      if (test) {
        check(DLPC350_SetTPGColor(1023, 1023, 1023, 0, 0, 0), "test-pattern colors failed");
        check(DLPC350_SetTPGSelect(static_cast<unsigned int>(requestedSettings.testPattern)), "test-pattern selection failed");
      }
      check(DLPC350_SetInputSource(test ? 1 : 0, 0), "video input selection failed");
      check(DLPC350_SetLedEnables(false, (requestedSettings.ledSelection & 1) != 0,
          (requestedSettings.ledSelection & 2) != 0, (requestedSettings.ledSelection & 4) != 0), "LED selection failed");
      return;
    }
    check(DLPC350_SetPatternDisplayMode(hdmiPatterns), "pattern source selection failed");
    if (hdmiPatterns) {
      check(DLPC350_ClearPatLut(), "pattern LUT reset failed");
      for (int index = 0; index < 2; ++index)
        check(DLPC350_AddToPatLut(index == 0 ? 1 : 3, 1, 8, requestedSettings.ledSelection,
            false, false, index == 0, false), "HDMI pattern entry failed");
      check(DLPC350_SetPatternConfig(2, true, 2, 1), "HDMI sequence settings failed");
    }
    check(DLPC350_SetExposure_FramePeriod(static_cast<unsigned int>(requestedSettings.exposure.count()),
        static_cast<unsigned int>(requestedSettings.framePeriod.count())), "HDMI sequence timing failed");
    check(DLPC350_SetPatternTriggerMode(hdmiPatterns ? 0 : 1), "pattern trigger settings failed");
    check(DLPC350_SetTrigOutConfig(1, requestedSettings.triggerInverted, 0xbb, 0xbb), "camera trigger output failed");
    check(DLPC350_SetTrigOutConfig(2, requestedSettings.triggerInverted, 0xbb, 0), "sequence trigger output failed");
    if (hdmiPatterns) {
      check(DLPC350_SendPatLut(), "HDMI pattern LUT upload failed");
      unsigned char image = 0;
      check(DLPC350_SendImageLut(&image, 1), "HDMI image LUT upload failed");
    }
    validateSequence(connection, settings.timeout);
    check(DLPC350_SetLedEnables(false, false, false, false), "post-configuration blanking failed");
  } catch (...) {
    DLPC350_PatternDisplay(0);
    DLPC350_SetLedEnables(false, false, false, false);
    throw;
  }
}
}