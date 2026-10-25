#include "ProjectorLightCrafter4500.h"
#include "lc4500tools.h"
#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_usb.h"
#include "LC4500API/dlpc350_firmware.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>

unsigned char g_OutputBuffer[USB_MAX_PACKET_SIZE + 1]{};
unsigned char g_InputBuffer[USB_MAX_PACKET_SIZE + 1]{};

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

struct Device {
  bool connected = false, pattern = true, stopRequested = false;
  bool stopStuck = false, modeStuck = false, failModeAck = false, nackMode = false;
  unsigned int action = 2, source = 0;
  int stopPolls = 0, modeWrites = 0, hdmiWrites = 0, testSelections = 0;
  bool unsafeModeChange = false, earlyModePoll = false;
  bool programming = false, wrongFlash = false, corruptPatternChecksum = false;
  bool failTableOnce = false, failTableAlways = false;
  unsigned int firmware = 0x04040000, readFailure = 0, flashAddress = 0, uploadSize = 0, uploadOffset = 0;
  unsigned int checksum = 0, memoryReads = 0, programEntries = 0, programExits = 0, tableErases = 0;
  std::vector<unsigned int> erased;
  std::vector<unsigned char> flash;
  unsigned int mailbox = 0, exposure = 0, patternPeriod = 0, triggerMode = 0, ledEnables = 0;
  bool hdmiPatterns = false;
  bool standby = false, validationPending = false, actionStuck = false;
  unsigned int validationFlags = 0, validationStarts = 0, writes = 0, playbackWrites = 0;
  unsigned int failedRead = 0, failedWrite = 0;
  std::vector<unsigned char> patternLut, imageLut, patternConfig, ledCurrents;
  std::vector<unsigned char> solidColor;
  std::vector<unsigned int> solidCommands;
  std::chrono::steady_clock::time_point modeChanged{};
} device;
bool reading = false;
unsigned int command = 0;
unsigned char value = 0;
std::vector<unsigned char> request;
std::size_t requestSize = 0;

unsigned int word(const std::vector<unsigned char> &bytes, std::size_t offset) {
  return bytes.at(offset) | (static_cast<unsigned int>(bytes.at(offset + 1)) << 8) |
      (static_cast<unsigned int>(bytes.at(offset + 2)) << 16) |
      (static_cast<unsigned int>(bytes.at(offset + 3)) << 24);
}

void put(std::vector<unsigned char> &bytes, std::size_t offset, unsigned int value) {
  for (unsigned int index = 0; index < 4; ++index)
    bytes.at(offset + index) = static_cast<unsigned char>(value >> (index * 8));
}

std::vector<unsigned char> table(bool aligned = true) {
  std::vector<unsigned char> bytes(slstudio::lc4500FlashSectorSize);
  for (std::size_t index = 0; index < bytes.size(); ++index)
    bytes[index] = static_cast<unsigned char>(index * 17);
  std::fill(bytes.begin(), bytes.begin() + sizeof(FLASH_TABLE), 0xff);
  put(bytes, 0, FLASHTABLE_APP_SIGNATURE);
  put(bytes, 4, slstudio::lc4500FlashBase);
  put(bytes, 8, 0x13);
  put(bytes, 12, 0x180000);
  put(bytes, offsetof(FLASH_TABLE, AppCode), slstudio::lc4500FlashBase + 0x30000);
  put(bytes, offsetof(FLASH_TABLE, AppCode) + 4, 0x60000);
  put(bytes, offsetof(FLASH_TABLE, ASIC_Config_Data), slstudio::lc4500FlashBase + 0x150000);
  put(bytes, offsetof(FLASH_TABLE, ASIC_Config_Data) + 4, 0x5000);
  put(bytes, offsetof(FLASH_TABLE, Splash_Data), slstudio::lc4500FlashBase + (aligned ? 0x160000 : 0x15f000));
  put(bytes, offsetof(FLASH_TABLE, Splash_Data) + 4, 0x10000);
  return bytes;
}

void reset(bool pattern = true, unsigned int source = 0) {
  device = Device{};
  device.pattern = pattern;
  device.action = pattern ? 2 : 0;
  device.source = source;
  request.clear();
  requestSize = 0;
}

void flashDevice(bool aligned = true) {
  reset(false, 1);
  device.flash.assign(slstudio::lc4500PatternFlashLimit, 0xff);
  const auto snapshot = table(aligned);
  std::copy(snapshot.begin(), snapshot.end(), device.flash.begin() + slstudio::lc4500FlashSectorSize);
}

struct TemporaryDirectory {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("slstudio-lc4500-tests-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  TemporaryDirectory() { require(std::filesystem::create_directory(path), "Cannot create test backup directory"); }
  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
    if (error) std::cerr << "Cannot remove LC4500 test backups: " << error.message() << '\n';
  }
};

slstudio::LightCrafter4500Settings testSettings() {
  slstudio::LightCrafter4500Settings requestedSettings;
  requestedSettings.displayMode = slstudio::LightCrafter4500Mode::TestPattern;
  return requestedSettings;
}
}

int DLPC350_USB_Init() { return 0; }
int DLPC350_USB_Exit() { return 0; }
int DLPC350_USB_Open() { device.connected = true; return 0; }
int DLPC350_USB_Close() { device.connected = false; return 0; }
int DLPC350_USB_IsConnected() { return device.connected; }
int DLPC350_USB_IsPresent() { return 1; }

int DLPC350_USB_Write() {
  if (!device.connected) return -1;
  if (request.empty()) {
    reading = (g_OutputBuffer[1] & 0x80) != 0;
    requestSize = g_OutputBuffer[3] | (g_OutputBuffer[4] << 8);
    const auto count = std::min(requestSize, std::size_t{60});
    request.assign(g_OutputBuffer + 5, g_OutputBuffer + 5 + count);
  } else {
    const auto count = std::min(requestSize - request.size(), std::size_t{64});
    request.insert(request.end(), g_OutputBuffer + 1, g_OutputBuffer + 1 + count);
  }
  if (request.size() < requestSize) return USB_MAX_PACKET_SIZE + 1;
  command = request.at(0) | (request.at(1) << 8);
  value = request.size() > 2 ? request[2] : 0;
  auto message = std::move(request);
  request.clear();
  if (!reading) {
    ++device.writes;
    if (command == 0x1a07 || command == 0x1203 || command == 0x1a00 || command == 0x1204)
      device.solidCommands.push_back(command);
    if (command == 0x1204) device.solidColor.assign(message.begin() + 2, message.end());
    if (command == 0x0200) device.standby = value != 0;
    if (command == 0x1a1a) ++device.validationStarts;
    if (command == 0x1a33) device.mailbox = value;
    if (command == 0x1a34 && device.mailbox == 2)
      device.patternLut.assign(message.begin() + 2, message.end());
    if (command == 0x1a34 && device.mailbox == 1)
      device.imageLut.assign(message.begin() + 2, message.end());
    if (command == 0x1a31) device.patternConfig.assign(message.begin() + 2, message.end());
    if (command == 0x1a29) {
      device.exposure = word(message, 2);
      device.patternPeriod = word(message, 6);
    }
    if (command == 0x1a22) device.hdmiPatterns = value == 0;
    if (command == 0x1a23) device.triggerMode = value;
    if (command == 0x1a07) device.ledEnables = value;
    if (command == 0x0b01) device.ledCurrents.assign(message.begin() + 2, message.end());
    if (command == 0x1a24) {
      ++device.playbackWrites;
      if (value == 0) {
        if (!device.stopRequested) device.stopPolls = 0;
        device.stopRequested = true;
      } else {
        device.stopRequested = false;
        if (!device.actionStuck) device.action = value;
      }
    }
    if (command == 0x1a00) {
      device.source = value & 7;
      if (device.source == 0) ++device.hdmiWrites;
    }
    if (command == 0x1203) ++device.testSelections;
    if (command == 0x1a1b) {
      ++device.modeWrites;
      device.unsafeModeChange = device.action != 0 || (value != 0 && device.source == 1);
      if (device.unsafeModeChange) return -1;
      if (!device.modeStuck && !device.failModeAck && !device.nackMode) device.pattern = value != 0;
      device.modeChanged = std::chrono::steady_clock::now();
    }
    if (command == 0x3001) {
      require(value == 1, "Invalid enter-programming payload");
      device.programming = true;
      ++device.programEntries;
    }
    if (command == 0x0030) {
      require(value == 2, "Invalid exit-programming payload");
      device.programming = false;
      device.pattern = false;
      device.action = 0;
      ++device.programExits;
    }
    if (command == 0x0029) device.flashAddress = word(message, 2);
    if (command == 0x002c) {
      device.uploadSize = word(message, 2);
      device.uploadOffset = 0;
    }
    if (command == 0x0028) {
      require(device.programming && message.size() == 3 && value == 0, "Invalid erase command");
      require(device.flashAddress % slstudio::lc4500FlashSectorSize == 0, "Unaligned sector erase");
      const auto start = device.flash.begin() + device.flashAddress;
      std::fill(start, start + slstudio::lc4500FlashSectorSize, 0xff);
      device.erased.push_back(device.flashAddress);
      if (device.flashAddress == slstudio::lc4500FlashSectorSize) ++device.tableErases;
    }
    if (command == 0x0025) {
      require(device.programming && device.uploadOffset + message.size() - 2 <= device.uploadSize,
          "Invalid programming data range");
      if (device.flashAddress == slstudio::lc4500FlashSectorSize &&
          (device.failTableAlways || (device.failTableOnce && device.tableErases == 1))) return -1;
      std::copy(message.begin() + 2, message.end(), device.flash.begin() + device.flashAddress + device.uploadOffset);
      device.uploadOffset += static_cast<unsigned int>(message.size() - 2);
    }
    if (command == 0x0026) {
      require(device.programming && message.size() == 3 && value == 1, "Invalid checksum command");
      const auto start = device.flash.begin() + device.flashAddress;
      device.checksum = std::accumulate(start, start + device.uploadSize, std::uint32_t{0});
      if (device.corruptPatternChecksum && device.flashAddress >= 0x160000) ++device.checksum;
    }
  } else if (command == 0x1a16) {
    device.flashAddress = word(message, 2);
  }
  return USB_MAX_PACKET_SIZE + 1;
}

int DLPC350_USB_Read() {
  if (!device.connected) return -1;
  if (reading && command == device.failedRead) return -1;
  std::fill(std::begin(g_InputBuffer), std::end(g_InputBuffer), 0);
  if (!reading) {
    if (command == device.failedWrite) return -1;
    if (command == 0x1a1b && device.failModeAck) return -1;
    if (command == 0x1a1b && device.nackMode) g_InputBuffer[0] = 0x20;
    return USB_MAX_PACKET_SIZE;
  }
  g_InputBuffer[2] = 1;
  if (command == 0x0200) {
    g_InputBuffer[4] = device.standby;
  } else if (command == 0x1a07) {
    g_InputBuffer[4] = static_cast<unsigned char>(device.ledEnables);
  } else if (command == 0x0205) {
    g_InputBuffer[2] = 16;
    const unsigned int versions[] = {device.firmware, 0x04030000, 0, 0};
    for (unsigned int index = 0; index < 4; ++index)
      for (unsigned int byte = 0; byte < 4; ++byte)
        g_InputBuffer[4 + index * 4 + byte] = static_cast<unsigned char>(versions[index] >> (byte * 8));
  } else if (command == 0x1a16) {
    if (device.flashAddress == device.readFailure || device.flashAddress < slstudio::lc4500FlashBase) return -1;
    const auto offset = device.flashAddress - slstudio::lc4500FlashBase;
    if (offset > device.flash.size() || device.flash.size() - offset < 4) return -1;
    std::copy(device.flash.begin() + offset, device.flash.begin() + offset + 4, g_InputBuffer + 4);
    g_InputBuffer[2] = 4;
    ++device.memoryReads;
  } else if (command == 0x0015) {
    if (!device.programming) return -1;
    g_InputBuffer[2] = 16;
    g_InputBuffer[4] = 0x80;
    const auto result = value == 0x0c ? 0x20u : value == 0x0d ? (device.wrongFlash ? 0x1234u : 0x227eu) : device.checksum;
    for (unsigned int byte = 0; byte < 4; ++byte)
      g_InputBuffer[10 + byte] = static_cast<unsigned char>(result >> (byte * 8));
  } else if (command == 0x1a42) {
    if (device.flash.empty()) g_InputBuffer[4] = 16;
    else {
      const auto address = word(device.flash, slstudio::lc4500FlashSectorSize + offsetof(FLASH_TABLE, Splash_Data));
      g_InputBuffer[4] = static_cast<unsigned char>(word(device.flash, address - slstudio::lc4500FlashBase + 8));
    }
  } else if (command == 0x1a1b) {
    if (device.modeWrites && std::chrono::steady_clock::now() - device.modeChanged <
        std::chrono::milliseconds(100)) {
      device.earlyModePoll = true;
      return -1;
    }
    g_InputBuffer[4] = device.pattern;
  } else if (command == 0x1a24) {
    if (device.stopRequested && !device.stopStuck && ++device.stopPolls >= 2) device.action = 0;
    g_InputBuffer[4] = static_cast<unsigned char>(device.action);
  } else if (command == 0x1a00) {
    g_InputBuffer[4] = static_cast<unsigned char>(device.source);
  } else if (command == 0x1a1a) {
    g_InputBuffer[4] = static_cast<unsigned char>(device.validationPending ? 0x80 : device.validationFlags);
  }
  return USB_MAX_PACKET_SIZE;
}

int main() {
  try {
    for (const auto *line : {"", "   ", "; comment", "\t\r\n", "123 456"}) {
      std::string valid = "DEFAULT.LEDCURRENT_RED\t0x12 ; comment\r\n";
      require(DLPC350_Frmw_ParseIniLines(valid.data()) == 0, "Valid upstream INI syntax was rejected");
      char token[128]{};
      uint32 values[1]{};
      int count = 0;
      DLPC350_Frmw_GetCurrentIniLineParam(token, values, &count);
      require(std::string(token) == "DEFAULT.LEDCURRENT_RED" && count == 1 && values[0] == 0x12,
          "Whitespace-delimited INI parameters were not preserved");
      std::string invalid = line;
      require(DLPC350_Frmw_ParseIniLines(invalid.data()) < 0,
          "Empty INI lines must not reuse a previously parsed token");
      DLPC350_Frmw_GetCurrentIniLineParam(token, values, &count);
      require(token[0] == '\0' && count == 0, "Rejected INI lines retained stale parser state");
    }
    slstudio::ProjectorOpenSettings settings;
    settings.timeout = std::chrono::milliseconds(30);
    auto requestedSettings = testSettings();
    reset();
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(!device.pattern && device.action == 0 && device.stopPolls >= 2,
        "Video mode must wait until the running pattern sequence has stopped");
    require(device.modeWrites == 1 && !device.unsafeModeChange && !device.earlyModePoll,
        "Mode transition must be sent once and settle before status polling");
    require(device.source == 1 && device.hdmiWrites == 0 && device.testSelections == 1,
        "Internal test patterns must not force an HDMI input transition");

    reset(false, 1);
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(device.modeWrites == 0 && device.hdmiWrites == 0 && device.testSelections == 1,
        "Changing a test pattern must not rewrite the display mode or select HDMI");
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::HdmiVideo;
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(device.modeWrites == 0 && device.source == 0,
        "HDMI video must change only the source when already in video mode");

    reset(false, 1);
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::HdmiPatterns;
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(device.pattern && device.source == 0 && device.modeWrites == 1 && !device.unsafeModeChange,
        "Entering pattern mode must first leave internal test-pattern input");
    requestedSettings.framePeriod = requestedSettings.exposure + std::chrono::microseconds(231);
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(device.patternLut.size() == 6 && (device.patternLut[2] & 2) && (device.patternLut[5] & 2),
        "Generated HDMI patterns must insert black vectors when timing has a dark interval");

    for (int leds = 1; leds <= 7; ++leds) {
      reset(false, 0);
      requestedSettings = slstudio::LightCrafter4500Settings{};
      requestedSettings.ledSelection = leds;
      requestedSettings.ledCurrent = {13, 27, 45};
      slstudio::configureLightCrafter(requestedSettings, settings);
      const auto depthAndLeds = static_cast<unsigned char>(8 | (leds << 4));
      require(device.patternLut == std::vector<unsigned char>{
          0x05, depthAndLeds, 0x04, 0x07, depthAndLeds, 0x00},
          "HDMI must swap/trigger only the first exposure and repeat the same plane without another swap");
      require(device.hdmiPatterns && device.triggerMode == 0 &&
          device.exposure == 8333 && device.patternPeriod == 8333 &&
          device.patternConfig == std::vector<unsigned char>{1, 1, 1, 0} &&
          device.imageLut == std::vector<unsigned char>{0},
          "HDMI defaults must repeat two 8333 us patterns synchronized to VSYNC");
      require(device.ledEnables == 0 && device.action == 0 &&
          device.ledCurrents == std::vector<unsigned char>{242, 228, 210},
          "Applying HDMI patterns must leave playback stopped and dark, preserving brightness");
      const auto writes = device.writes;
      const auto status = slstudio::lightCrafter4500Status(settings);
      require(status.patternMode && !status.standby && status.ledsDisabled &&
          status.playback == slstudio::LightCrafter4500Playback::Stopped && status.validationReady &&
          status.validationFlags == 0 && device.writes == writes && !device.connected,
          "Status must read actual state without configuring, starting or blanking the projector");
      const auto lut = device.patternLut;
      const auto started = slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings);
      require(started.playback == slstudio::LightCrafter4500Playback::Running &&
          !started.ledsDisabled && device.ledEnables == 8 && device.patternLut == lut && !device.connected,
          "Start must enable sequencer-controlled LEDs without rewriting the selected LUT LEDs");
      const auto validations = device.validationStarts;
      const auto paused = slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Pause, settings);
      require(paused.playback == slstudio::LightCrafter4500Playback::Paused,
          "Pause must read back the paused state");
      slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings);
      require(device.action == 2 && device.validationStarts == validations && device.patternLut == lut,
          "Resume must not stop, revalidate or reconfigure the paused sequence");
      const auto stopped = slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Stop, settings);
      require(stopped.playback == slstudio::LightCrafter4500Playback::Stopped && !stopped.ledsDisabled,
          "Stop must not claim to blank the LEDs");
      slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings);
      require(device.validationStarts == validations + 1, "Restart from stopped must validate the LUT");
      const auto blanked = slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Blank, settings);
      require(blanked.ledsDisabled && blanked.playback == slstudio::LightCrafter4500Playback::Stopped,
          "Blank must stop playback and disable all LEDs");
      slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings);
      require(device.action == 2 && device.ledEnables == 8 && device.patternLut == lut,
          "Start after Blank must restore the applied sequence and its LED selection");
      require(device.programEntries == 0 && device.erased.empty(),
          "Applying the HDMI configuration must not program flash");
    }

    auto rejects = [](auto operation, const char *message) {
      bool rejected = false;
      try { operation(); }
      catch (const std::exception &) { rejected = true; }
      require(rejected && !device.connected, message);
    };
    reset();
    device.patternLut = {5, 24, 4};
    device.imageLut = {1};
    device.patternConfig = {0, 1, 0, 0};
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::FlashPatterns;
    const auto depthAndLeds = static_cast<unsigned char>(8 | (requestedSettings.ledSelection << 4));
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(device.patternLut == std::vector<unsigned char>{
            0, depthAndLeds, 6, 4, depthAndLeds, 2, 8, depthAndLeds, 2,
            0, depthAndLeds, 6, 4, depthAndLeds, 2, 8, depthAndLeds, 2} &&
        device.imageLut == std::vector<unsigned char>{1, 0} &&
        device.patternConfig == std::vector<unsigned char>{5, 1, 5, 1} &&
        device.exposure == 8333 && device.patternPeriod == 8564 &&
        !device.hdmiPatterns && device.triggerMode == 1 && device.action == 0 && device.ledEnables == 0,
        "Applying stored patterns must build an internal sequence and leave playback stopped and dark");
    for (const auto failedRead : {0x0200u, 0x1a1bu, 0x1a07u, 0x1a24u, 0x1a1au}) {
      reset();
      device.failedRead = failedRead;
      rejects([&] { slstudio::lightCrafter4500Status(settings); },
          "Status query failures must be reported and release USB ownership");
      require(device.writes == 0, "Failed status queries must not change device state");
      device.failedRead = 0;
      slstudio::lightCrafter4500Status(settings);
    }
    reset(false, 1);
    device.ledEnables = 7;
    const auto videoStatus = slstudio::lightCrafter4500Status(settings);
    require(!videoStatus.patternMode && !videoStatus.ledsDisabled && device.writes == 0,
        "Video status must not change input or LED state");
    for (const auto action : {slstudio::LightCrafter4500Action::Start,
        slstudio::LightCrafter4500Action::Pause, slstudio::LightCrafter4500Action::Stop})
      rejects([&] { slstudio::controlLightCrafter4500(action, settings); },
          "Pattern playback actions must reject video mode");
    require(slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Blank, settings).ledsDisabled &&
        device.modeWrites == 0 && device.testSelections == 0 && device.playbackWrites == 0,
        "Video Blank must disable LEDs without changing mode, input or pattern playback");
    reset();
    device.standby = true;
    rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings); },
        "Start must reject standby");
    require(device.writes == 0, "A rejected standby command must not wake or configure the device");
    reset();
    device.action = 0;
    rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Pause, settings); },
        "Pause must reject a stopped sequence");
    for (const auto flags : {1u, 2u, 3u, 9u, 31u, 32u}) {
      reset();
      device.action = 0;
      device.validationFlags = flags;
      require(slstudio::lightCrafter4500Status(settings).validationFlags == flags,
          "Status must expose actual sequence validation flags");
      rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings); },
          "Start must reject all reported sequence validation errors");
      require(device.playbackWrites == 0 && device.ledEnables == 0,
          "Invalid sequences must not start or enable LEDs");
    }
    for (const auto flags : {4u, 8u, 16u, 28u}) {
      reset();
      device.action = 0;
      device.validationFlags = flags;
      const auto status = slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings);
      require(status.validationPassed() && status.validationFlags == flags &&
          status.playback == slstudio::LightCrafter4500Playback::Running,
          "Documented validation warnings must remain visible without blocking playback");
    }
    reset();
    device.action = 1;
    device.validationPending = true;
    require(!slstudio::lightCrafter4500Status(settings).validationReady,
        "Pending validation must not be reported as valid");
    rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings); },
        "Resume must time out if validation never finishes");
    require(device.playbackWrites == 0, "Failed validation must not disturb paused playback");
    reset();
    device.action = 0;
    device.failedWrite = 0x1a24;
    rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings); },
        "Start must report a failed USB acknowledgement");
    require(device.ledEnables == 0, "Failed Start must not enable LEDs");
    reset();
    device.action = 0;
    device.actionStuck = true;
    rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Start, settings); },
        "Start must confirm actual state and report a playback transition timeout");
    reset();
    device.action = 3;
    rejects([&] { slstudio::lightCrafter4500Status(settings); }, "Unknown playback states must be reported");
    reset();
    device.ledEnables = 8;
    device.stopStuck = true;
    rejects([&] { slstudio::controlLightCrafter4500(slstudio::LightCrafter4500Action::Blank, settings); },
        "Blank must report a failed sequence stop");
    require(device.ledEnables == 0, "Blank must disable LEDs even when stopping the sequence fails");
    auto invalidTimeout = settings;
    invalidTimeout.timeout = std::chrono::milliseconds(0);
    rejects([&] { slstudio::lightCrafter4500Status(invalidTimeout); },
        "Status must reject invalid connection timeouts");

    for (int failure = 0; failure < 4; ++failure) {
      reset();
      device.stopStuck = failure == 0;
      device.modeStuck = failure == 1;
      device.failModeAck = failure == 2;
      device.nackMode = failure == 3;
      bool rejected = false;
      try {
        slstudio::configureLightCrafter(testSettings(), settings);
      } catch (const std::runtime_error &error) {
        const std::string expected = failure == 0 ? "sequence stop" :
            failure == 1 ? "display mode change" : "display mode change failed";
        rejected = std::string(error.what()).find(expected) != std::string::npos;
      }
      require(rejected, "Failed stop/mode transitions must report the failing operation");
      require(device.testSelections == 0 && !device.connected,
          "Failed transitions must not continue configuring and must release the connection");
      if (failure == 0) require(device.modeWrites == 0, "A failed sequence stop must prevent mode changes");
      reset(false, 1);
      slstudio::configureLightCrafter(testSettings(), settings);
      require(device.testSelections == 1, "A failed operation must release exclusive device ownership");
    }
    reset(false, 1);
    auto projector = slstudio::openProjector(
        {"lc4500", "LC4500", slstudio::OutputBackend::LightCrafter4500}, settings);
    for (const unsigned int level : {0u, 128u, 255u}) {
      device.solidCommands.clear();
      projector->showSolid(static_cast<std::uint8_t>(level));
      const int value = static_cast<int>((level * 1023 + 127) / 255);
      require(device.solidColor.size() == 12 && device.ledEnables == 8,
          "Solid fields must use sequencer-controlled illumination");
      for (std::size_t channel = 0; channel < 6; ++channel)
        require((device.solidColor[channel * 2] | (device.solidColor[channel * 2 + 1] << 8)) == value,
            "Solid fields must set all foreground/background RGB channels to the requested ten-bit value");
      require(device.solidCommands == std::vector<unsigned int>{0x1a07, 0x1204, 0x1203, 0x1a00, 0x1a07},
          "Solid-field updates must blank LEDs until color and source changes have finished");
    }
    projector->blank(true);
    require(device.modeWrites == 0 && device.hdmiWrites == 0,
        "Direct solid-field and white projection must not re-enter video mode or force HDMI");
    bool busy = false;
    try { slstudio::lightCrafter4500Status(settings); }
    catch (const std::runtime_error &) { busy = true; }
    require(busy && device.connected, "A status query must not steal or close an owned projector connection");
    projector.reset();
    slstudio::lightCrafter4500Status(settings);
    const auto images = std::vector<cv::Mat3b>{cv::Mat3b(4, 6, cv::Vec3b(12, 34, 56))};
    const auto original = table(false);
    const auto plan = slstudio::prepareLightCrafter4500FlashUpdate(0x04040000, original, images);
    require(plan.patternOffset == 0x160000 && word(plan.patternData, 0) == 0x12345678 &&
        word(plan.patternData, 4) == 0x87654321 && word(plan.patternData, 8) == 1 &&
        word(plan.patternData, 12) >= slstudio::lc4500FlashBase + plan.patternOffset,
        "Template-free image generation must create a valid directory at the relocated address");
    for (std::size_t index = 0; index < original.size(); ++index)
      if (index < offsetof(FLASH_TABLE, Splash_Data) || index >= offsetof(FLASH_TABLE, Splash_Data) + 4)
        require(plan.tableSector[index] == original[index], "Unrelated table-sector bytes changed");
    for (int failure = 0; failure < 7; ++failure) {
      auto invalid = table();
      unsigned int firmware = 0x04040000;
      if (failure == 0) firmware = 0x04030000;
      if (failure == 1) put(invalid, 0, 0);
      if (failure == 2) put(invalid, 8, 0x99);
      if (failure == 3) invalid.resize(sizeof(FLASH_TABLE) - 1);
      if (failure == 4) put(invalid, offsetof(FLASH_TABLE, Splash_Data), 0xfffffff0);
      if (failure >= 5) {
        put(invalid, offsetof(FLASH_TABLE, Batch_File), slstudio::lc4500FlashBase + (failure == 5 ? 0x160000 : 0x15f000));
        put(invalid, offsetof(FLASH_TABLE, Batch_File) + 4, 0x3000);
      }
      bool rejected = false;
      try { slstudio::prepareLightCrafter4500FlashUpdate(firmware, invalid, images); }
      catch (const std::runtime_error &) { rejected = true; }
      require(rejected, "Unsafe firmware/table layouts must be rejected before programming");
    }
    auto limited = table();
    put(limited, offsetof(FLASH_TABLE, Splash_Data),
        slstudio::lc4500FlashBase + slstudio::lc4500PatternFlashLimit - slstudio::lc4500FlashSectorSize);
    cv::Mat3b oversized(1140, 912);
    cv::randu(oversized, 0, 255);
    bool overflowRejected = false;
    try { slstudio::prepareLightCrafter4500FlashUpdate(0x04040000, limited, {oversized}); }
    catch (const std::runtime_error &) { overflowRejected = true; }
    require(overflowRejected, "Pattern payloads must not cross the supported flash capacity");
    TemporaryDirectory backups;
    settings.flashBackupDirectory = backups.path.string();
    settings.timeout = std::chrono::milliseconds(100);
    const std::vector<cv::Mat> patterns{cv::Mat1b(1140, 1824, static_cast<unsigned char>(42))};
    for (int relocation = 0; relocation < 2; ++relocation) {
      flashDevice(relocation == 0);
      const auto snapshot = table(relocation == 0);
      projector = slstudio::openProjector({"lc4500", "LC4500", slstudio::OutputBackend::LightCrafter4500}, settings);
      put(device.flash, (relocation == 0 ? 0x160000 : 0x15f000) + 8, 1);
      projector->setPatterns(patterns);
      require(device.programEntries == 0 && device.erased.empty(),
          "Preparing resident patterns must not upload or erase flash");
      float lastProgress = -1;
      bool uploadProgress = false, metadataProgress = false;
      projector->programPatterns(patterns, [&](float fraction, const char *stage) {
        require(fraction >= lastProgress && fraction <= 1, "Programming progress must be monotonic and bounded");
        require(stage && *stage, "Programming progress must name the current stage");
        lastProgress = fraction;
        if (std::string(stage) == "Uploading pattern data" && fraction > 0.45f) uploadProgress = true;
        if (std::string(stage) == "Reading metadata sector for backup") metadataProgress = true;
      });
      require(lastProgress == 1 && uploadProgress && metadataProgress == (relocation != 0),
          "Programming must report upload, metadata backup and successful completion");
      require(device.programEntries == 1 && device.programExits == 1 && !device.programming,
          "Successful programming must verify and return to the application");
      require(device.tableErases == static_cast<unsigned int>(relocation) &&
          device.memoryReads == (relocation ? slstudio::lc4500FlashSectorSize : sizeof(FLASH_TABLE)) / 4,
          "Aligned pattern updates must read only the header and never erase the table sector");
      for (const auto address : device.erased)
        require(address >= 0x160000 || (relocation && address == slstudio::lc4500FlashSectorSize),
            "Programming erased a protected firmware sector");
      for (std::size_t index = 0; index < snapshot.size(); ++index)
        if (!relocation || index < offsetof(FLASH_TABLE, Splash_Data) || index >= offsetof(FLASH_TABLE, Splash_Data) + 4)
          require(device.flash[slstudio::lc4500FlashSectorSize + index] == snapshot[index],
              "Metadata-sector neighbors were not preserved");
      projector->setPatterns(patterns);
      const auto modeWrites = device.modeWrites;
      for (const int darkTime : {1, 230}) {
        bool rejected = false;
        try {
          projector->configureSequence({std::chrono::microseconds(8333),
              std::chrono::microseconds(8333 + darkTime)});
        } catch (const std::invalid_argument &) { rejected = true; }
        require(rejected && device.modeWrites == modeWrites && !device.pattern,
            "Invalid sequence timing must fail before changing the device mode");
      }
      bool shortExposureRejected = false;
      try { projector->configureSequence({std::chrono::microseconds(8332), std::chrono::microseconds(8332)}); }
      catch (const std::invalid_argument &) { shortExposureRejected = true; }
      require(shortExposureRejected && device.modeWrites == modeWrites && !device.pattern,
          "Eight-bit flash sequences must enforce the minimum exposure before device I/O");
      projector->configureSequence({std::chrono::microseconds(8333), std::chrono::microseconds(8564)});
      require((device.patternLut.at(2) & 2) != 0,
          "Generated resident-pattern sequences must insert black vectors during dark intervals");
      projector->configureSequence({std::chrono::microseconds(8333), std::chrono::microseconds(8333)});
      require(device.exposure == 8333 && device.patternPeriod == 8564 && (device.patternLut.at(2) & 2),
          "Flash playback must separate camera trigger pulses even when exposure equals the requested period");
      projector.reset();
    }
    reset();
    projector = slstudio::openProjector({"lc4500", "LC4500", slstudio::OutputBackend::LightCrafter4500}, settings);
    for (const auto count : {6, 7, 8, 12}) {
      projector->setPatterns(std::vector<cv::Mat>(count, patterns.front()));
      for (const bool repeat : {false, true}) {
        projector->configureSequence({std::chrono::microseconds(16667), std::chrono::microseconds(16667), repeat});
        const unsigned int expected = count == 6 ? 16898 :
            repeat && count == 7 ? 200000 : repeat && count == 8 ? 100000 : 66667;
        require(device.exposure == 16667 && device.patternPeriod == expected &&
            device.patternLut.size() == static_cast<std::size_t>(3 * count) &&
            device.patternConfig.at(1) == static_cast<unsigned char>(repeat),
            "Scan and calibration sequences must share trigger-gap and flash-loading timing");
        for (int index = 0; index < count; ++index)
          require(device.patternLut.at(3 * index + 2) & 2,
              "Every flash pattern must insert black during the trigger gap");
        projector->configureSequence({std::chrono::microseconds(16667), std::chrono::microseconds(250000), repeat});
        require(device.patternPeriod == 250000, "Flash timing must preserve longer requested periods");
      }
    }
    projector->configureSequence({std::chrono::microseconds(66666), std::chrono::microseconds(66666)});
    require(device.patternPeriod == 66897, "Flash-loading timing must not leave a sub-minimum trigger gap");
    projector.reset();
    reset(false, 0);
    requestedSettings = slstudio::LightCrafter4500Settings{};
    requestedSettings.displayMode = slstudio::LightCrafter4500Mode::FlashPatterns;
    requestedSettings.exposure = std::chrono::microseconds(16667);
    requestedSettings.framePeriod = requestedSettings.exposure;
    slstudio::configureLightCrafter(requestedSettings, settings);
    require(device.exposure == 16667 && device.patternPeriod == 16898 && device.patternLut.size() == 18,
        "Stored flash settings must use the same trigger gap as capture");
    for (int index = 0; index < 6; ++index)
      require(device.patternLut.at(3 * index + 2) & 2,
          "Stored flash settings must insert black between all six exposures");
    for (int failure = 0; failure < 6; ++failure) {
      flashDevice(failure < 4);
      if (failure == 0) device.firmware = 0x04030000;
      if (failure == 1) device.readFailure = slstudio::lc4500FlashBase + slstudio::lc4500FlashSectorSize + 12;
      if (failure == 2) device.wrongFlash = true;
      if (failure == 3) device.corruptPatternChecksum = true;
      if (failure == 4) device.failTableOnce = true;
      if (failure == 5) device.failTableAlways = true;
      projector = slstudio::openProjector({"lc4500", "LC4500", slstudio::OutputBackend::LightCrafter4500}, settings);
      bool rejected = false;
      float lastProgress = -1;
      try { projector->programPatterns(patterns, [&](float fraction, const char *) { lastProgress = fraction; }); }
      catch (const std::runtime_error &) { rejected = true; }
      require(rejected, "Firmware/read/identity/write/verification failures must not report success");
      require(lastProgress < 1, "Failed programming must not report completed progress");
      if (failure < 3) require(device.erased.empty(), "Preflight failures must prevent all flash erases");
      if (failure == 3) require(device.tableErases == 0, "A bad payload checksum must prevent metadata changes");
      if (failure == 4) {
        const auto restored = table(false);
        require(device.tableErases == 2 && !device.programming &&
            std::equal(restored.begin(), restored.end(), device.flash.begin() + slstudio::lc4500FlashSectorSize),
            "Failed metadata programming must restore and verify the original sector");
      }
      if (failure == 5)
        require(device.programming && device.programExits == 0,
            "Failed metadata recovery must leave the bootloader running rather than boot corrupted data");
      projector.reset();
    }
    std::size_t backupCount = 0;
    for (const auto &entry : std::filesystem::recursive_directory_iterator(backups.path))
      if (entry.is_regular_file()) {
        std::ifstream file(entry.path(), std::ios::binary);
        const std::vector<unsigned char> saved{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
        require(saved.size() == sizeof(FLASH_TABLE) || saved.size() == slstudio::lc4500FlashSectorSize,
            "Metadata backup has an unexpected size");
        require(word(saved, 0) == FLASHTABLE_APP_SIGNATURE, "Metadata backup was not written before programming");
        ++backupCount;
      }
    require(backupCount == 6, "Every attempted flash operation must save its original metadata before programming");
    std::cout << "LC4500 simulated mode, packet, template-free programming, bounds, backup and recovery tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
