#include "ProjectorLightCrafter3000.h"
#include "LC3000API/lcr_cmd.h"
#include "LC3000API/lcr_packetizer.h"
#include "LC3000API/tcp_client.h"

#include <filesystem>
#include <atomic>
#include <random>
#include <opencv2/imgcodecs.hpp>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace slstudio {
namespace {
std::atomic_bool occupied{false};
#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket invalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket invalidSocket = -1;
#endif
struct Connection;
Connection *activeConnection = nullptr;

void check(ErrorCode_t result, const char *operation) {
  if (result != SUCCESS) throw std::runtime_error(std::string("LC3000: ") + operation);
}

struct Connection {
  Socket socket = invalidSocket;
  const std::chrono::milliseconds timeout;
  const std::string address;

  explicit Connection(const ProjectorOpenSettings &settings) : timeout(settings.timeout), address(settings.networkAddress) {
    if (occupied.exchange(true)) throw std::runtime_error("LC3000 is already open");
    activeConnection = this;
    try { check(LCR_CMD_Open(), "connect failed"); }
    catch (...) { closeSocket(); activeConnection = nullptr; occupied.store(false); throw; }
  }
  ~Connection() { LCR_CMD_Close(); closeSocket(); activeConnection = nullptr; occupied.store(false); }
  Connection(const Connection &) = delete;
  Connection &operator=(const Connection &) = delete;

  int closeSocket() {
    if (socket != invalidSocket) {
#ifdef _WIN32
      closesocket(socket);
      WSACleanup();
#else
      close(socket);
#endif
      socket = invalidSocket;
    }
    return 0;
  }

  int openSocket(unsigned long port) {
    if (socket != invalidSocket) return -1;
#ifdef _WIN32
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return -1;
#endif
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
    addrinfo *resolved = nullptr;
    const auto service = std::to_string(port);
    if (getaddrinfo(address.c_str(), service.c_str(), &hints, &resolved) != 0) {
#ifdef _WIN32
      WSACleanup();
#endif
      return -1;
    }
    socket = ::socket(resolved->ai_family, resolved->ai_socktype, resolved->ai_protocol);
    if (socket == invalidSocket) {
      freeaddrinfo(resolved);
#ifdef _WIN32
      WSACleanup();
#endif
      return -1;
    }
#ifdef _WIN32
    u_long nonblocking = 1;
    const bool configured = ioctlsocket(socket, FIONBIO, &nonblocking) == 0;
#else
    const bool configured = socket < FD_SETSIZE && fcntl(socket, F_SETFL, O_NONBLOCK) == 0;
#endif
    const bool connected = configured && (connect(socket, resolved->ai_addr,
        static_cast<int>(resolved->ai_addrlen)) == 0 ||
        waitForSocket(true, std::chrono::steady_clock::now() + timeout));
    freeaddrinfo(resolved);
    int error = 0;
#ifdef _WIN32
    int length = sizeof(error);
    const auto status = getsockopt(socket, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&error), &length);
#else
    socklen_t length = sizeof(error);
    const auto status = getsockopt(socket, SOL_SOCKET, SO_ERROR, &error, &length);
#endif
    if (!connected || status != 0 || error != 0) { closeSocket(); return -1; }
    return 0;
  }

  int sendBytes(unsigned char *buffer, int length) {
    if (socket == invalidSocket || length < 0) return 1;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (length > 0) {
      if (!waitForSocket(true, deadline)) return 1;
#ifdef MSG_NOSIGNAL
      const auto sent = send(socket, reinterpret_cast<char *>(buffer), length, MSG_NOSIGNAL);
#else
      const auto sent = send(socket, reinterpret_cast<char *>(buffer), length, 0);
#endif
      if (sent < 0 && retrySocket()) continue;
      if (sent <= 0) return 1;
      buffer += sent;
      length -= static_cast<int>(sent);
    }
    return 0;
  }

  int receiveBytes(unsigned char *buffer, int length) {
    if (socket == invalidSocket || length < 0) return 1;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (length > 0) {
      if (!waitForSocket(false, deadline)) return 1;
      const auto received = recv(socket, reinterpret_cast<char *>(buffer), length, 0);
      if (received < 0 && retrySocket()) continue;
      if (received <= 0) return 1;
      buffer += received;
      length -= static_cast<int>(received);
    }
    return 0;
  }

private:
  bool waitForSocket(bool writing, std::chrono::steady_clock::time_point deadline) const {
    for (;;) {
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) return false;
      timeval wait{static_cast<long>(remaining.count() / 1000000), static_cast<long>(remaining.count() % 1000000)};
      fd_set sockets;
      FD_ZERO(&sockets);
      FD_SET(socket, &sockets);
      const auto result = select(static_cast<int>(socket + 1), writing ? nullptr : &sockets,
          writing ? &sockets : nullptr, nullptr, &wait);
      if (result >= 0) return result > 0;
#ifdef _WIN32
      if (WSAGetLastError() != WSAEINTR) return false;
#else
      if (errno != EINTR) return false;
#endif
    }
  }

  static bool retrySocket() {
#ifdef _WIN32
    const auto error = WSAGetLastError();
    return error == WSAEWOULDBLOCK || error == WSAEINTR;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
  }
};

struct TemporaryBitmap {
  std::filesystem::path directory;
  TemporaryBitmap() {
    std::random_device random;
    for (int attempt = 0; attempt < 20; ++attempt) {
      const auto candidate = std::filesystem::temp_directory_path() /
          ("slstudio-lc3000-" + std::to_string(random()));
      if (std::filesystem::create_directory(candidate)) { directory = candidate; return; }
    }
    throw std::runtime_error("Cannot create LC3000 upload staging directory");
  }
  ~TemporaryBitmap() {
    std::error_code error;
    std::filesystem::remove_all(directory, error);
  }
};

class ProjectorLightCrafter3000 final : public Projector {
public:
  explicit ProjectorLightCrafter3000(const ProjectorOpenSettings &openSettings) : connection(openSettings) {
    check(LCR_CMD_SetPowerMode(PWR_NORMAL), "wake failed");
    LCR_LEDCurrent_t currents{255, 255, 255};
    check(LCR_CMD_SetLEDCurrent(&currents), "LED settings failed");
    LCR_DisplaySetting_t display{0, 0, 1};
    check(LCR_CMD_SetDisplaySetting(&display), "orientation settings failed");
    LCR_CamTriggerSetting_t trigger{};
    trigger.Enable = 1;
    trigger.Polarity = TRIGGER_EDGE_POS;
    trigger.PulseWidth = 200;
    check(LCR_CMD_SetCamTriggerSetting(&trigger), "trigger settings failed");
    settings.BitDepth = 8;
    settings.PatternType = PTN_TYPE_NORMAL;
    settings.InputTriggerType = TRIGGER_TYPE_SW;
    settings.ExposureTime = 16667;
    settings.LEDSelect = LED_RED;
    settings.Repeat = 1;
    blank();
  }
  ~ProjectorLightCrafter3000() override { try { blank(); } catch (...) {} }
  cv::Size resolution() const override { return {1216, 684}; }
  ProjectorGeometry geometry() const override { return {resolution(), {608, 684}, PixelLayout::Diamond}; }
  ProjectorCapabilities capabilities() const override { return {true, true, true}; }

  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    auto prepared = prepareMonochromeProjectorPatterns(patterns, geometry(), 96);
    prepareSequence(prepared.size());
    count = prepared.size();
  }

  void programPatterns(const std::vector<cv::Mat> &patterns,
                       const ProjectorProgrammingProgress &progress) override {
    if (progress) progress(0, "Preparing patterns");
    auto prepared = prepareMonochromeProjectorPatterns(patterns, geometry(), 96);
    prepareSequence(prepared.size());
    TemporaryBitmap staging;
    const auto path = (staging.directory / "pattern.bmp").string();
    for (std::size_t index = 0; index < prepared.size(); ++index) {
      if (!cv::imwrite(path, prepared[index])) throw std::runtime_error("Cannot encode LC3000 pattern");
      check(LCR_CMD_DefinePatternBMP(static_cast<LCR_PatternCount_t>(index), path.c_str()), "pattern upload failed");
      if (progress) progress(static_cast<float>(index + 1) / prepared.size(), "Uploading patterns");
    }
    count = prepared.size();
  }

  void present(std::size_t index) override {
    if (index >= count) throw std::out_of_range("LC3000 pattern index");
    stopSequence();
    configured = false;
    settings.InputTriggerType = TRIGGER_TYPE_SW;
    check(LCR_CMD_SetPatternSeqSetting(&settings), "single-pattern settings failed");
    check(LCR_CMD_SetDisplayMode(DISP_MODE_PTN_SEQ), "pattern mode failed");
    if (LCR_CMD_PKT_CommandInit(LCR_CMD_PKT_TYPE_WRITE, 0x0405) != 0 ||
        LCR_CMD_PKT_PutInt(static_cast<uint32>(index), 2) != 0 || LCR_CMD_PKT_SendCommand() != 0)
      throw std::runtime_error("LC3000: pattern presentation failed");
  }

  void blank(bool white = false) override {
    stopSequence();
    check(LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE), "static-image mode failed");
    check(LCR_CMD_DisplayStaticColor(white ? 0xffffff : 0), "blanking failed");
  }
  void showSolid(std::uint8_t level) override {
    stopSequence();
    configured = false;
    check(LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE), "static-image mode failed");
    const auto color = static_cast<unsigned int>(level) * 0x010101u;
    check(LCR_CMD_DisplayStaticColor(color), "solid-field settings failed");
  }

  void configureSequence(const ProjectorSequenceSettings &requestedSettings) override {
    validateProjectorSequence(requestedSettings);
    if (!count) throw std::logic_error("Prepare LC3000 patterns first");
    stopSequence();
    configured = false;
    settings.InputTriggerType = TRIGGER_TYPE_AUTO;
    settings.ExposureTime = static_cast<uint32>(requestedSettings.exposure.count());
    settings.AutoTriggerPeriod = static_cast<uint32>(requestedSettings.framePeriod.count());
    settings.Repeat = requestedSettings.repeat;
    check(LCR_CMD_SetPatternSeqSetting(&settings), "sequence settings failed");
    configured = true;
  }
  void startSequence() override {
    if (!configured) throw std::logic_error("Configure LC3000 timing before starting");
    check(LCR_CMD_SetDisplayMode(DISP_MODE_PTN_SEQ), "pattern mode failed");
    check(LCR_CMD_StartPatternSeq(1), "sequence start failed");
  }
  void stopSequence() override { check(LCR_CMD_StartPatternSeq(0), "sequence stop failed"); }

private:
  void prepareSequence(std::size_t patternCount) {
    stopSequence();
    count = 0;
    configured = false;
    settings.NumPatterns = static_cast<LCR_PatternCount_t>(patternCount);
    settings.InputTriggerType = TRIGGER_TYPE_SW;
    check(LCR_CMD_SetPatternSeqSetting(&settings), "pattern settings failed");
  }

  Connection connection;
  LCR_PatternSeqSetting_t settings{};
  std::size_t count = 0;
  bool configured = false;
};
}

std::unique_ptr<Projector> openLightCrafter3000(const ProjectorOpenSettings &settings) {
  return std::make_unique<ProjectorLightCrafter3000>(settings);
}

void configureLightCrafter3000(const LightCrafter3000Settings &requestedSettings, const ProjectorOpenSettings &settings) {
  Connection connection(settings);
  try {
    check(LCR_CMD_SetPowerMode(PWR_NORMAL), "wake failed");
    check(LCR_CMD_StartPatternSeq(0), "sequence stop failed");
    if (requestedSettings.standby) {
      check(LCR_CMD_SetPowerMode(PWR_STANDBY), "standby failed");
      return;
    }
    LCR_LEDCurrent_t currents{static_cast<uint16>(requestedSettings.ledCurrent[0]),
        static_cast<uint16>(requestedSettings.ledCurrent[1]), static_cast<uint16>(requestedSettings.ledCurrent[2])};
    check(LCR_CMD_SetLEDCurrent(&currents), "LED settings failed");
    LCR_DisplaySetting_t display{static_cast<uint8>(requestedSettings.rotate), static_cast<uint8>(requestedSettings.flipLongAxis),
        static_cast<uint8>(requestedSettings.flipShortAxis)};
    check(LCR_CMD_SetDisplaySetting(&display), "orientation settings failed");
    check(LCR_CMD_SetDisplayMode(DISP_MODE_PTN_SEQ), "HDMI preparation failed");
    check(LCR_CMD_SetDisplayMode(DISP_MODE_VIDEO), "HDMI mode failed");
    const LCR_VideoSetting_t video{608, 684, 0, 0, 608, 684};
    check(LCR_CMD_SetVideoSetting(&video), "HDMI video geometry failed");
    LCR_VideoModeSetting_t mode{static_cast<uint8>(requestedSettings.frameRate), 8, 2};
    check(LCR_CMD_SetVideoMode(&mode), "HDMI video timing failed");
    LCR_CamTriggerSetting_t trigger{};
    trigger.Enable = requestedSettings.triggerEnabled;
    trigger.Polarity = requestedSettings.triggerInverted ? TRIGGER_EDGE_NEG : TRIGGER_EDGE_POS;
    trigger.Delay = static_cast<uint32>(requestedSettings.triggerDelayUs);
    trigger.PulseWidth = static_cast<uint32>(requestedSettings.triggerPulseUs);
    check(LCR_CMD_SetCamTriggerSetting(&trigger), "camera trigger settings failed");
  } catch (...) {
    LCR_CMD_StartPatternSeq(0);
    LCR_CMD_SetDisplayMode(DISP_MODE_IMAGE);
    LCR_CMD_DisplayStaticColor(0);
    throw;
  }
}
}

extern "C" {
int TCP_Connect(char *, unsigned long port) {
  return slstudio::activeConnection ? slstudio::activeConnection->openSocket(port) : -1;
}
int TCP_Disconnect(int) {
  return slstudio::activeConnection ? slstudio::activeConnection->closeSocket() : 0;
}
int TCP_Send(int, unsigned char *buffer, int length) {
  return slstudio::activeConnection ? slstudio::activeConnection->sendBytes(buffer, length) : 1;
}
int TCP_Receive(int, unsigned char *buffer, int length) {
  return slstudio::activeConnection ? slstudio::activeConnection->receiveBytes(buffer, length) : 1;
}
}