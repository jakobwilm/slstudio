#include "ProjectorFactory.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <roapi.h>
#include <d3d11_4.h>
#include <dxgi1_4.h>
#include <windows.devices.display.core.interop.h>
#include <winrt/Windows.Devices.Display.h>
#include <winrt/Windows.Devices.Display.Core.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.Metadata.h>
#include <winrt/Windows.Graphics.h>
#include <winrt/Windows.Graphics.DirectX.h>
#include <winrt/Windows.Graphics.DirectX.Direct3D11.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace slstudio {
namespace {

namespace core = winrt::Windows::Devices::Display::Core;
namespace directx = winrt::Windows::Graphics::DirectX;
using winrt::Windows::Devices::Display::DisplayMonitorUsageKind;

// Balance only our own initialization, including S_FALSE. An existing STA is
// usable here: these synchronous APIs do not need a new apartment or a dispatcher.
class Apartment final {
public:
  Apartment() {
    const HRESULT result = RoInitialize(RO_INIT_MULTITHREADED);
    if (result != RPC_E_CHANGED_MODE)
      winrt::check_hresult(result);
    initialized = SUCCEEDED(result);
  }
  ~Apartment() { if (initialized) RoUninitialize(); }
  Apartment(const Apartment &) = delete;
  Apartment &operator=(const Apartment &) = delete;
private:
  bool initialized = false;
};

struct ManagerOwner {
  core::DisplayManager value{nullptr};
  ~ManagerOwner() { close(); }
  void close() noexcept {
    if (value) {
      try { value.Close(); }
      catch (const winrt::hresult_error &error) {
        std::fprintf(stderr, "Display.Core Close failed (HRESULT 0x%08lX)\n",
                     static_cast<unsigned long>(static_cast<std::uint32_t>(error.code().value)));
      } catch (const std::exception &error) {
        std::fprintf(stderr, "Display.Core Close failed: %s\n", error.what());
      } catch (...) {
        std::fprintf(stderr, "Display.Core Close failed with an unknown exception\n");
      }
      value = nullptr;
    }
  }
};

std::runtime_error apiError(const winrt::hresult_error &error) {
  return std::runtime_error("Display.Core: " + winrt::to_string(error.message()) +
                           " (HRESULT " + std::to_string(error.code().value) + ")");
}

bool supported() {
  // TryExecuteTask, source status and flip-completion fences require Windows 11.
  return winrt::Windows::Foundation::Metadata::ApiInformation::IsApiContractPresent(
      L"Windows.Foundation.UniversalApiContract", 14);
}

bool eligible(const core::DisplayTarget &target) {
  const auto usage = target.UsageKind();
  return target.IsConnected() && !target.IsStale() &&
         (usage == DisplayMonitorUsageKind::HeadMounted ||
          usage == DisplayMonitorUsageKind::SpecialPurpose) &&
         !target.StableMonitorId().empty();
}

std::string outputId(const core::DisplayTarget &target) {
  return "display-core:" + winrt::to_string(target.StableMonitorId());
}

void checkStateResult(const core::DisplayManagerResultWithState &result,
                      const char *operation) {
  winrt::check_hresult(result.ExtendedErrorCode());
  if (result.ErrorCode() != core::DisplayManagerResult::Success || !result.State())
    throw std::runtime_error(std::string("Display.Core ") + operation +
                             " failed (status " +
                             std::to_string(static_cast<int>(result.ErrorCode())) + ")");
}

class ProjectorDisplayCore final : public Projector {
public:
  explicit ProjectorDisplayCore(const ProjectorOutputInfo &output) {
    try {
      if (output.backend != OutputBackend::DisplayCore || !output.exclusive)
        throw std::invalid_argument("Not an exclusive Display.Core projector output");
      if (!supported())
        throw std::runtime_error("Display.Core projector requires Windows 11 or later");
      configure(output.id);
      createResources();
      blank();
    } catch (const winrt::hresult_error &error) {
      owner.close();
      throw apiError(error);
    } catch (...) {
      owner.close();
      throw;
    }
  }

  ~ProjectorDisplayCore() override {
    // present() drains every accepted flip before returning. Close also revokes
    // the still-scanning primary before any shared GPU resources are released.
    owner.close();
  }

  cv::Size resolution() const override { return size; }

  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    try {
      checkActive();
      std::vector<Buffer> prepared;
      prepared.reserve(patterns.size());
      for (const auto &pattern : patterns) {
        if (pattern.size() != size ||
            (pattern.type() != CV_8UC1 && pattern.type() != CV_8UC3))
          throw std::invalid_argument("Projector patterns must match output pixels");
        cv::Mat bgra;
        cv::cvtColor(pattern, bgra, pattern.channels() == 1
                                        ? cv::COLOR_GRAY2BGRA : cv::COLOR_RGB2BGRA);
        prepared.push_back(createBuffer(bgra));
      }
      context->Flush();
      waitFor(rendered, renderValue);
      blank();
      buffers.swap(prepared);
    } catch (const winrt::hresult_error &error) {
      owner.close();
      throw apiError(error);
    }
  }

  void present(std::size_t index) override {
    if (index >= buffers.size())
      throw std::out_of_range("Projector pattern index");
    submit(buffers[index]);
  }

  void blank(bool white = false) override {
    submit(white ? whiteBuffer : blackBuffer);
  }

  void showSolid(std::uint8_t level) override {
    if (level == 0 || level == 255) {
      blank(level == 255);
      return;
    }
    try {
      auto prepared = createBuffer(cv::Mat(size, CV_8UC4, cv::Scalar(level, level, level, 255)));
      context->Flush();
      waitFor(rendered, renderValue);
      submit(prepared);
      solidBuffer = std::move(prepared);
    } catch (const winrt::hresult_error &error) {
      owner.close();
      throw apiError(error);
    }
  }

private:
  struct Fence {
    winrt::com_ptr<ID3D11Fence> d3d;
    core::DisplayFence display{nullptr};
    winrt::handle event;
  };
  struct Buffer {
    core::DisplaySurface surface{nullptr};
    core::DisplayScanout scanout{nullptr};
    winrt::com_ptr<ID3D11Texture2D> texture;
    core::DisplayTask task{nullptr};
  };

  void configure(const std::string &id) {
    // Mode queries are transactional. Retry only actual topology races, not
    // access denial, unsupported hardware or driver failures.
    for (int attempt = 0; attempt < 3; ++attempt) {
      owner.close();
      owner.value = core::DisplayManager::Create(
          core::DisplayManagerOptions::EnforceSourceOwnership);
      target = nullptr;
      for (const auto &candidate : owner.value.GetCurrentTargets())
        if (eligible(candidate) && outputId(candidate) == id) {
          target = candidate;
          break;
        }
      if (!target)
        throw std::runtime_error("Specialized Display.Core projector is no longer available");
      const auto targets = winrt::single_threaded_vector<core::DisplayTarget>({target});
      const auto acquired = owner.value.TryAcquireTargetsAndCreateEmptyState(targets);
      if (acquired.ErrorCode() == core::DisplayManagerResult::TargetStale)
        continue;
      checkStateResult(acquired, "exclusive acquisition");
      auto state = acquired.State();
      auto path = state.ConnectTarget(target);
      path.IsInterlaced(false);
      path.Scaling(core::DisplayPathScaling::Identity);
      path.Rotation(core::DisplayRotation::None);
      path.SourcePixelFormat(directx::DirectXPixelFormat::B8G8R8A8UIntNormalized);
      const core::DisplayWireFormat wireFormat{
          core::DisplayWireFormatPixelEncoding::Rgb444, 8,
          core::DisplayWireFormatColorSpace::BT709, core::DisplayWireFormatEotf::Sdr,
          core::DisplayWireFormatHdrMetadata::None};
      path.WireFormat(wireFormat);
      core::DisplayModeInfo chosen{nullptr};
      double bestDistance = std::numeric_limits<double>::infinity();
      for (const auto &mode : path.FindModes(core::DisplayModeQueryOptions::OnlyPreferredResolution)) {
        const auto sourceSize = mode.SourceResolution();
        const auto targetSize = mode.TargetResolution();
        if (!sourceSize || !targetSize ||
            sourceSize.Value().Width != targetSize.Value().Width ||
            sourceSize.Value().Height != targetSize.Value().Height ||
            mode.IsInterlaced() || mode.IsStereo() || !mode.IsWireFormatSupported(wireFormat))
          continue;
        const auto rate = mode.PresentationRate().VerticalSyncRate;
        if (!rate.Denominator)
          continue;
        const double distance = std::abs(static_cast<double>(rate.Numerator) /
                                        rate.Denominator - 60.0);
        if (distance < bestDistance) {
          chosen = mode;
          bestDistance = distance;
        }
      }
      if (!chosen)
        throw std::runtime_error("Display.Core projector has no preferred progressive BGRA8 mode");
      path.ApplyPropertiesFromMode(chosen);
      path.Scaling(core::DisplayPathScaling::Identity);
      path.Rotation(core::DisplayRotation::None);
      path.WireFormat(wireFormat);
      const auto applied = state.TryApply(core::DisplayStateApplyOptions::FailIfStateChanged);
      if (applied.Status() == core::DisplayStateOperationStatus::SystemStateChanged)
        continue;
      winrt::check_hresult(applied.ExtendedErrorCode());
      if (applied.Status() != core::DisplayStateOperationStatus::Success)
        throw std::runtime_error("Display.Core mode setting failed (status " +
                                 std::to_string(static_cast<int>(applied.Status())) + ")");
      const auto current = owner.value.TryAcquireTargetsAndReadCurrentState(targets);
      checkStateResult(current, "reading applied mode");
      path = current.State().GetPathForTarget(target);
      const auto dimensions = path.SourceResolution();
      const auto targetDimensions = path.TargetResolution();
      const auto actualFormat = path.WireFormat();
      if (!dimensions || dimensions.Value().Width <= 0 || dimensions.Value().Height <= 0 ||
          !targetDimensions || dimensions.Value().Width != targetDimensions.Value().Width ||
          dimensions.Value().Height != targetDimensions.Value().Height ||
          path.Scaling() != core::DisplayPathScaling::Identity ||
          path.Rotation() != core::DisplayRotation::None ||
          path.SourcePixelFormat() != directx::DirectXPixelFormat::B8G8R8A8UIntNormalized ||
          !actualFormat || actualFormat.PixelEncoding() != core::DisplayWireFormatPixelEncoding::Rgb444 ||
          actualFormat.BitsPerChannel() != 8 ||
          actualFormat.ColorSpace() != core::DisplayWireFormatColorSpace::BT709 ||
          actualFormat.Eotf() != core::DisplayWireFormatEotf::Sdr ||
          actualFormat.HdrMetadata() != core::DisplayWireFormatHdrMetadata::None)
        throw std::runtime_error("Display.Core requires unscaled, unrotated RGB8 SDR output");
      size = {dimensions.Value().Width, dimensions.Value().Height};
      return;
    }
    throw std::runtime_error("Display.Core topology kept changing during exclusive acquisition");
  }

  Fence createFence() {
    Fence fence;
    winrt::check_hresult(d3d->CreateFence(0, D3D11_FENCE_FLAG_SHARED,
                                        __uuidof(ID3D11Fence), fence.d3d.put_void()));
    winrt::handle shared;
    winrt::check_hresult(fence.d3d->CreateSharedHandle(nullptr, GENERIC_ALL, nullptr,
                                                     shared.put()));
    winrt::com_ptr<::IInspectable> inspectable;
    winrt::check_hresult(device.as<IDisplayDeviceInterop>()->OpenSharedHandle(
        shared.get(), __uuidof(::IInspectable), inspectable.put_void()));
    fence.display = inspectable.as<core::DisplayFence>();
    fence.event = winrt::handle{CreateEventW(nullptr, FALSE, FALSE, nullptr)};
    if (!fence.event)
      winrt::throw_last_error();
    return fence;
  }

  void createResources() {
    device = owner.value.CreateDisplayDevice(target.Adapter());
    source = device.CreateScanoutSource(target);
    pool = device.CreateTaskPool();
    winrt::com_ptr<IDXGIFactory4> factory;
    winrt::check_hresult(CreateDXGIFactory1(__uuidof(IDXGIFactory4), factory.put_void()));
    const auto adapterId = device.RenderAdapterId();
    const LUID luid{adapterId.LowPart, adapterId.HighPart};
    winrt::com_ptr<IDXGIAdapter> adapter;
    winrt::check_hresult(factory->EnumAdapterByLuid(luid, __uuidof(IDXGIAdapter),
                                                   adapter.put_void()));
    winrt::com_ptr<ID3D11Device> baseDevice;
    winrt::com_ptr<ID3D11DeviceContext> baseContext;
    winrt::check_hresult(D3D11CreateDevice(
        adapter.get(), D3D_DRIVER_TYPE_UNKNOWN, nullptr, D3D11_CREATE_DEVICE_BGRA_SUPPORT,
        nullptr, 0, D3D11_SDK_VERSION, baseDevice.put(), nullptr, baseContext.put()));
    d3d = baseDevice.as<ID3D11Device5>();
    context = baseContext.as<ID3D11DeviceContext4>();
    rendered = createFence();
    flippedTo = createFence();
    flippedAway = createFence();
    blackBuffer = createBuffer(cv::Mat(size, CV_8UC4, cv::Scalar(0, 0, 0, 255)));
    whiteBuffer = createBuffer(cv::Mat(size, CV_8UC4, cv::Scalar(255, 255, 255, 255)));
    context->Flush();
    waitFor(rendered, renderValue);
  }

  Buffer createBuffer(const cv::Mat &bgra) {
    checkActive();
    winrt::Windows::Graphics::DirectX::Direct3D11::Direct3DMultisampleDescription samples{};
    samples.Count = 1;
    const core::DisplayPrimaryDescription description{
        static_cast<std::uint32_t>(size.width), static_cast<std::uint32_t>(size.height),
        directx::DirectXPixelFormat::B8G8R8A8UIntNormalized,
        directx::DirectXColorSpace::RgbFullG22NoneP709, false, samples};
    Buffer buffer;
    buffer.surface = device.CreatePrimary(target, description);
    buffer.scanout = device.CreateSimpleScanout(source, buffer.surface, 0, 1);
    winrt::handle shared;
    winrt::check_hresult(device.as<IDisplayDeviceInterop>()->CreateSharedHandle(
        buffer.surface.as<::IInspectable>().get(), nullptr, GENERIC_ALL,
        nullptr, shared.put()));
    winrt::check_hresult(d3d->OpenSharedResource1(
        shared.get(), __uuidof(ID3D11Texture2D), buffer.texture.put_void()));
    context->UpdateSubresource(buffer.texture.get(), 0, nullptr, bgra.data,
                               static_cast<UINT>(bgra.step), 0);
    winrt::check_hresult(context->Signal(rendered.d3d.get(), ++renderValue));
    buffer.task = pool.CreateTask();
    buffer.task.SetScanout(buffer.scanout);
    buffer.task.SetWait(rendered.display, renderValue);
    buffer.task.SetSignal(core::DisplayTaskSignalKind::OnPresentFlipTo, flippedTo.display);
    buffer.task.SetSignal(core::DisplayTaskSignalKind::OnPresentFlipAway, flippedAway.display);
    return buffer;
  }

  void checkActive() {
    if (!owner.value)
      throw std::runtime_error("Display.Core projector is closed after ownership or presentation failure");
    try {
      if (source && source.Status() != core::DisplaySourceStatus::Active)
        throw std::runtime_error("Display.Core projector ownership, connection or display power was lost");
      if (d3d)
        winrt::check_hresult(d3d->GetDeviceRemovedReason());
    } catch (const winrt::hresult_error &error) {
      owner.close();
      throw apiError(error);
    } catch (...) {
      owner.close();
      throw;
    }
  }

  void waitFor(const Fence &fence, std::uint64_t value) {
    if (!value)
      return;
    winrt::check_hresult(fence.d3d->SetEventOnCompletion(value, fence.event.get()));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    for (;;) {
      checkActive();
      const auto completed = fence.d3d->GetCompletedValue();
      if (completed == std::numeric_limits<std::uint64_t>::max())
        throw std::runtime_error("Display.Core GPU fence was invalidated");
      if (completed >= value)
        return;
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("Display.Core projector frame completion timed out");
      const DWORD result = WaitForSingleObject(fence.event.get(), 50);
      if (result == WAIT_FAILED)
        winrt::throw_last_error();
    }
  }

  void submit(const Buffer &buffer) {
    try {
      checkActive();
      const auto previous = lastPresent;
      const bool changed = displayedSurface != buffer.surface;
      const auto result = pool.TryExecuteTask(buffer.task);
      if (result.PresentStatus() != core::DisplayPresentStatus::Success ||
          result.SourceStatus() != core::DisplaySourceStatus::Active)
        throw std::runtime_error("Display.Core rejected projector frame (present status " +
                                 std::to_string(static_cast<int>(result.PresentStatus())) +
                                 ", source status " +
                                 std::to_string(static_cast<int>(result.SourceStatus())) + ")");
      lastPresent = result.PresentId();
      waitFor(flippedTo, lastPresent);
      if (changed) waitFor(flippedAway, previous);
      displayedSurface = buffer.surface;
    } catch (const winrt::hresult_error &error) {
      owner.close();
      throw apiError(error);
    } catch (...) {
      // Do not leave an accepted, unfinished flip or an exclusive target alive
      // after a failed presentation, and never retry using an invalid source.
      owner.close();
      throw;
    }
  }

  Apartment apartment;
  ManagerOwner owner;
  core::DisplayTarget target{nullptr};
  core::DisplayDevice device{nullptr};
  core::DisplaySource source{nullptr};
  core::DisplayTaskPool pool{nullptr};
  core::DisplaySurface displayedSurface{nullptr};
  winrt::com_ptr<ID3D11Device5> d3d;
  winrt::com_ptr<ID3D11DeviceContext4> context;
  Fence rendered, flippedTo, flippedAway;
  std::vector<Buffer> buffers;
  Buffer blackBuffer, whiteBuffer, solidBuffer;
  cv::Size size;
  std::uint64_t renderValue = 0, lastPresent = 0;
};

}

std::vector<ProjectorOutputInfo> displayCoreProjectorOutputs() {
  try {
    Apartment apartment;
    if (!supported())
      return {};
    ManagerOwner owner;
    owner.value = core::DisplayManager::Create(
        core::DisplayManagerOptions::EnforceSourceOwnership);
    std::vector<ProjectorOutputInfo> outputs;
    for (const auto &target : owner.value.GetCurrentTargets()) {
      if (!eligible(target))
        continue;
      const auto monitor = target.TryGetMonitor();
      const auto name = monitor ? winrt::to_string(monitor.DisplayName()) : std::string{};
      outputs.push_back({outputId(target),
                         (name.empty() ? "Specialized display" : name) +
                             " (Exclusive Display.Core)",
                         OutputBackend::DisplayCore, true});
    }
    return outputs;
  } catch (const winrt::hresult_error &error) {
    throw apiError(error);
  }
}

std::unique_ptr<Projector> openDisplayCoreProjector(const ProjectorOutputInfo &output) {
  try {
    return std::make_unique<ProjectorDisplayCore>(output);
  } catch (const winrt::hresult_error &error) {
    throw apiError(error);
  }
}

}
