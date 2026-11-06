#include "ProjectorFactory.h"

#import <AppKit/AppKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#import <QuartzCore/CATransaction.h>
#import <dispatch/dispatch.h>

#include <atomic>
#include <limits>
#include <opencv2/imgproc.hpp>

#if !__has_feature(objc_arc)
#error "ProjectorCoreGraphics.mm requires Objective-C ARC (-fobjc-arc)"
#endif

namespace slstudio {
namespace {

void requireMainThread() {
  if (![NSThread isMainThread])
    throw std::logic_error("CoreGraphics projector operations require the main thread");
}

bool suitableDisplay(CGDirectDisplayID display) {
  return CGDisplayIsOnline(display) && CGDisplayIsActive(display) &&
         !CGDisplayIsMain(display) && !CGDisplayIsInMirrorSet(display) &&
         !CGDisplayIsAsleep(display);
}

std::string displayIdentifier(CGDirectDisplayID display) {
  CFUUIDRef uuid = CGDisplayCreateUUIDFromDisplayID(display);
  if (!uuid)
    throw std::runtime_error("Cannot obtain CoreGraphics display UUID");
  NSString *text = CFBridgingRelease(CFUUIDCreateString(kCFAllocatorDefault, uuid));
  CFRelease(uuid);
  if (!text)
    throw std::runtime_error("Cannot format CoreGraphics display UUID");
  const char *identifier = text.UTF8String;
  if (!identifier)
    throw std::runtime_error("Cannot encode CoreGraphics display UUID as UTF-8");
  return std::string(identifier);
}

std::vector<CGDirectDisplayID> onlineDisplays() {
  uint32_t count = 0;
  if (CGGetOnlineDisplayList(0, nullptr, &count) != kCGErrorSuccess)
    throw std::runtime_error("Cannot enumerate CoreGraphics displays");
  std::vector<CGDirectDisplayID> displays(count);
  if (count && CGGetOnlineDisplayList(count, displays.data(), &count) != kCGErrorSuccess)
    throw std::runtime_error("Cannot enumerate CoreGraphics displays");
  displays.resize(count);
  return displays;
}

NSScreen *screenForDisplay(CGDirectDisplayID display) {
  for (NSScreen *screen in NSScreen.screens)
    if ([screen.deviceDescription[@"NSScreenNumber"] unsignedIntValue] == display)
      return screen;
  return nil;
}

struct Capture {
  explicit Capture(std::function<void(std::function<void()>)> dispatch)
      : mainThread(std::move(dispatch)) {}
  std::function<void(std::function<void()>)> mainThread;
  CGDirectDisplayID display = kCGNullDirectDisplay;
  bool captured = false;
  bool cursorHidden = false;
  bool callbackRegistered = false;
  std::atomic<bool> invalidated{false};
  NSWindow *window = nil;
  CAMetalLayer *layer = nil;
  id<MTLDevice> device = nil;
  id<MTLCommandQueue> queue = nil;
  dispatch_semaphore_t completed = dispatch_semaphore_create(0);
  dispatch_semaphore_t presented = dispatch_semaphore_create(0);
  std::vector<id<MTLTexture>> textures;

  static void reconfigured(CGDirectDisplayID display, CGDisplayChangeSummaryFlags flags,
                           void *context) {
    auto &capture = *static_cast<Capture *>(context);
    if (display == capture.display && !(flags & kCGDisplayBeginConfigurationFlag))
      capture.invalidated.store(true);
  }

  ~Capture() {
    if (callbackRegistered)
      CGDisplayRemoveReconfigurationCallback(reconfigured, this);
    auto closeWindow = [&] {
      @autoreleasepool {
        [window orderOut:nil];
        window.contentView.layer = nil;
        [window close];
        window = nil;
      }
    };
    if (window) {
      if (mainThread) mainThread(closeWindow);
      else closeWindow();
    }
    textures.clear();
    queue = nil;
    layer = nil;
    device = nil;
    window = nil;
    if (cursorHidden)
      CGDisplayShowCursor(display);
    if (captured) {
      const auto error = CGDisplayRelease(display);
      if (error != kCGErrorSuccess)
        NSLog(@"SLStudio: CGDisplayRelease failed (%d)", static_cast<int>(error));
    }
  }
};

class ProjectorCoreGraphics final : public Projector {
public:
  ProjectorCoreGraphics(CGDirectDisplayID display,
                        std::function<void(std::function<void()>)> dispatch) : mainThread(std::move(dispatch)) {
    @autoreleasepool {
      if (!suitableDisplay(display) || CGDisplayIsCaptured(display))
        throw std::runtime_error("Secondary CoreGraphics display is no longer available");
      CGDisplayModeRef mode = CGDisplayCopyDisplayMode(display);
      if (!mode)
        throw std::runtime_error("Cannot read CoreGraphics display mode");
      const auto width = CGDisplayModeGetPixelWidth(mode);
      const auto height = CGDisplayModeGetPixelHeight(mode);
      CGDisplayModeRelease(mode);
      if (!width || !height || width > std::numeric_limits<int>::max() ||
          height > std::numeric_limits<int>::max())
        throw std::runtime_error("Invalid CoreGraphics display pixel dimensions");
      if (width != CGDisplayPixelsWide(display) || height != CGDisplayPixelsHigh(display) ||
          CGDisplayRotation(display) != 0)
        throw std::runtime_error("CoreGraphics projector requires an unscaled, unrotated display mode");
      size = {static_cast<int>(width), static_cast<int>(height)};

      capture = std::make_unique<Capture>(mainThread);
      capture->display = display;
      const auto error = CGDisplayCapture(display);
      if (error != kCGErrorSuccess)
        throw std::runtime_error("Cannot exclusively capture CoreGraphics display (" +
                                 std::to_string(error) + ")");
      capture->captured = true;
      if (CGDisplayRegisterReconfigurationCallback(Capture::reconfigured, capture.get()) !=
          kCGErrorSuccess)
        throw std::runtime_error("Cannot monitor captured CoreGraphics display");
      capture->callbackRegistered = true;
      if (CGDisplayHideCursor(display) != kCGErrorSuccess)
        throw std::runtime_error("Cannot hide cursor for captured CoreGraphics display");
      capture->cursorHidden = true;
      capture->device = CGDirectDisplayCopyCurrentMetalDevice(display);
      capture->queue = [capture->device newCommandQueue];
      if (!capture->device || !capture->queue)
        throw std::runtime_error("Captured display does not have a usable Metal device");

      // Only AppKit window/layer setup needs the GUI; display capture and Metal stay on acquisition.
      onMainThread([&] {
        @autoreleasepool {
          NSScreen *screen = screenForDisplay(display);
          if (!screen || !suitableDisplay(display))
            throw std::runtime_error("Secondary CoreGraphics display is no longer available");
          // With an explicit screen, the initial content rectangle is screen-relative.
          capture->window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0,
                                                                             screen.frame.size.width,
                                                                             screen.frame.size.height)
                                                        styleMask:NSWindowStyleMaskBorderless
                                                          backing:NSBackingStoreBuffered
                                                            defer:NO
                                                           screen:screen];
          if (!capture->window)
            throw std::runtime_error("Cannot create captured projector window");
          capture->window.releasedWhenClosed = NO;
          capture->window.hasShadow = NO;
          capture->window.opaque = YES;
          capture->window.backgroundColor = NSColor.blackColor;
          capture->window.hidesOnDeactivate = NO;
          capture->window.ignoresMouseEvents = YES;
          capture->window.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces |
                                               NSWindowCollectionBehaviorStationary |
                                               NSWindowCollectionBehaviorIgnoresCycle;
          // SDL2's Cocoa exclusive path likewise presents a window at shielding level.
          capture->window.level = CGShieldingWindowLevel() + 1;
          capture->layer = [CAMetalLayer layer];
          capture->layer.device = capture->device;
          capture->layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
          capture->layer.framebufferOnly = NO;
          capture->layer.opaque = YES;
          capture->layer.colorspace = nullptr;
          capture->layer.wantsExtendedDynamicRangeContent = NO;
          capture->layer.displaySyncEnabled = YES;
          capture->layer.allowsNextDrawableTimeout = YES;
          capture->layer.contentsScale = screen.backingScaleFactor;
          capture->layer.drawableSize = CGSizeMake(width, height);
          capture->window.contentView.wantsLayer = YES;
          capture->window.contentView.layer = capture->layer;
          [capture->window orderFrontRegardless];
          [capture->window displayIfNeeded];
          [CATransaction flush];
        }
      });
      blank();
    }
  }

  ~ProjectorCoreGraphics() override = default;

  cv::Size resolution() const override { return size; }

  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    @autoreleasepool {
      checkCapture();
      for (const auto &pattern : patterns)
        if (pattern.size() != size || (pattern.type() != CV_8UC1 && pattern.type() != CV_8UC3))
          throw std::invalid_argument("Projector patterns must match output pixels");
      std::vector<id<MTLTexture>> textures;
      std::vector<id<MTLTexture>> stagingTextures;
      textures.reserve(patterns.size());
      stagingTextures.reserve(patterns.size());
      for (const auto &pattern : patterns) {
        cv::Mat bgra;
        cv::cvtColor(pattern, bgra,
                     pattern.channels() == 1 ? cv::COLOR_GRAY2BGRA : cv::COLOR_RGB2BGRA);
        auto *descriptor = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                        width:size.width
                                       height:size.height
                                    mipmapped:NO];
        descriptor.storageMode = [capture->device supportsFamily:MTLGPUFamilyApple1]
                                     ? MTLStorageModeShared : MTLStorageModeManaged;
        id<MTLTexture> staging = [capture->device newTextureWithDescriptor:descriptor];
        if (!staging)
          throw std::runtime_error("Cannot allocate Metal projector upload texture");
        [staging replaceRegion:MTLRegionMake2D(0, 0, size.width, size.height)
                   mipmapLevel:0
                     withBytes:bgra.data
                   bytesPerRow:bgra.step[0]];
        descriptor.storageMode = MTLStorageModePrivate;
        id<MTLTexture> texture = [capture->device newTextureWithDescriptor:descriptor];
        if (!texture)
          throw std::runtime_error("Cannot allocate Metal projector pattern");
        stagingTextures.push_back(staging);
        textures.push_back(texture);
      }
      id<MTLCommandBuffer> upload = [capture->queue commandBuffer];
      id<MTLBlitCommandEncoder> encoder = [upload blitCommandEncoder];
      if (!upload || !encoder)
        fail("Cannot prepare Metal projector upload commands");
      for (std::size_t index = 0; index < textures.size(); ++index) {
        [encoder copyFromTexture:stagingTextures[index]
                     sourceSlice:0
                     sourceLevel:0
                    sourceOrigin:MTLOriginMake(0, 0, 0)
                      sourceSize:MTLSizeMake(size.width, size.height, 1)
                       toTexture:textures[index]
                destinationSlice:0
                destinationLevel:0
               destinationOrigin:MTLOriginMake(0, 0, 0)];
      }
      [encoder endEncoding];
      finishCommand(upload);
      capture->textures.swap(textures);
    }
  }

  void present(std::size_t index) override {
    @autoreleasepool {
      checkCapture();
      if (index >= capture->textures.size())
        throw std::out_of_range("Projector pattern index");
      draw(capture->textures[index], false);
    }
  }

  void blank(bool white = false) override {
    @autoreleasepool {
      checkCapture();
      draw(nil, white);
    }
  }

private:
  void onMainThread(std::function<void()> action) const {
    if (mainThread) mainThread(std::move(action));
    else {
      requireMainThread();
      action();
    }
  }
  [[noreturn]] void fail(const std::string &message) {
    capture->invalidated.store(true);
    throw std::runtime_error(message);
  }

  void checkCapture() {
    if (capture->invalidated.load())
      fail("Exclusive CoreGraphics projector display reported a reconfiguration");
    if (!suitableDisplay(capture->display))
      fail("Exclusive CoreGraphics projector display is no longer online, active, secondary, unmirrored, and awake");
    if (!CGDisplayIsCaptured(capture->display))
      fail("Exclusive CoreGraphics projector display is no longer captured");
    onMainThread([&] {
      if (!capture->window.visible)
        fail("Exclusive CoreGraphics projector window is not visible");
      if (capture->window.miniaturized)
        fail("Exclusive CoreGraphics projector window is minimized");

      const auto backing = [capture->window.contentView
          convertRectToBacking:capture->window.contentView.bounds];
      if (backing.size.width != size.width || backing.size.height != size.height)
        fail("Exclusive CoreGraphics projector window backing size changed: expected " +
             std::to_string(size.width) + "x" + std::to_string(size.height) + ", got " +
             std::to_string(backing.size.width) + "x" + std::to_string(backing.size.height));

      if ([capture->window.screen.deviceDescription[@"NSScreenNumber"] unsignedIntValue] !=
          capture->display)
        fail("Exclusive CoreGraphics projector window moved to another display");
    });
    id<MTLDevice> currentDevice = CGDirectDisplayCopyCurrentMetalDevice(capture->display);
    if (!currentDevice || currentDevice.registryID != capture->device.registryID)
      fail("Exclusive CoreGraphics projector display Metal device changed");
  }

  void finishCommand(id<MTLCommandBuffer> command) {
    const auto completed = capture->completed;
    [command addCompletedHandler:^(id<MTLCommandBuffer>) {
      dispatch_semaphore_signal(completed);
    }];
    [command commit];
    if (dispatch_semaphore_wait(completed, dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC)) != 0)
      fail("Timed out completing Metal projector commands");
    if (command.status != MTLCommandBufferStatusCompleted) {
      NSString *description = command.error.localizedDescription;
      fail("Metal projector command failed: " +
           std::string(description ? description.UTF8String : "unknown error"));
    }
  }

  void draw(id<MTLTexture> texture, bool white) {
    const auto presented = capture->presented;
    // Drain drawable/command autoreleases before waiting; retaining them can defer the
    // presentation callback after idle gaps and prevent acquisition from reaching the camera trigger.
    @autoreleasepool {
      id<CAMetalDrawable> drawable = [capture->layer nextDrawable];
      if (!drawable || drawable.texture.width != static_cast<NSUInteger>(size.width) ||
          drawable.texture.height != static_cast<NSUInteger>(size.height))
        fail("Cannot acquire a native-pixel Metal projector drawable");
      id<MTLCommandBuffer> command = [capture->queue commandBuffer];
      if (!command)
        fail("Cannot create Metal projector command buffer");
      if (texture) {
        id<MTLBlitCommandEncoder> encoder = [command blitCommandEncoder];
        if (!encoder)
          fail("Cannot create Metal projector blit encoder");
        [encoder copyFromTexture:texture
                     sourceSlice:0
                     sourceLevel:0
                    sourceOrigin:MTLOriginMake(0, 0, 0)
                      sourceSize:MTLSizeMake(size.width, size.height, 1)
                       toTexture:drawable.texture
                destinationSlice:0
                destinationLevel:0
               destinationOrigin:MTLOriginMake(0, 0, 0)];
        [encoder endEncoding];
      } else {
        auto *pass = [MTLRenderPassDescriptor renderPassDescriptor];
        pass.colorAttachments[0].texture = drawable.texture;
        pass.colorAttachments[0].loadAction = MTLLoadActionClear;
        pass.colorAttachments[0].storeAction = MTLStoreActionStore;
        const double value = white ? 1.0 : 0.0;
        pass.colorAttachments[0].clearColor = MTLClearColorMake(value, value, value, 1.0);
        id<MTLRenderCommandEncoder> encoder = [command renderCommandEncoderWithDescriptor:pass];
        if (!encoder)
          fail("Cannot create Metal projector clear encoder");
        [encoder endEncoding];
      }
      if (@available(macOS 10.15.4, *)) {
        [drawable addPresentedHandler:^(id<MTLDrawable>) {
          dispatch_semaphore_signal(presented);
        }];
      } else {
        fail("Exclusive CoreGraphics projection requires macOS 10.15.4 or newer");
      }
      [command presentDrawable:drawable];
      finishCommand(command);
    }
    // Publish this thread's layer transaction before waiting for presentation.
    [CATransaction flush];
    if (dispatch_semaphore_wait(presented, dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC)) != 0)
      fail("Timed out presenting captured CoreGraphics projector");
    checkCapture();
  }

  cv::Size size;
  std::function<void(std::function<void()>)> mainThread;
  std::unique_ptr<Capture> capture;
};
}

std::vector<ProjectorOutputInfo> coreGraphicsProjectorOutputs() {
  requireMainThread();
  @autoreleasepool {
    std::vector<ProjectorOutputInfo> outputs;
    if (@available(macOS 10.15.4, *)) {
      for (const auto display : onlineDisplays()) {
        if (!suitableDisplay(display) || CGDisplayIsCaptured(display))
          continue;
        NSScreen *screen = screenForDisplay(display);
        if (!screen || !CGDirectDisplayCopyCurrentMetalDevice(display))
          continue;
        auto id = displayIdentifier(display);
        outputs.push_back({id, std::string(screen.localizedName.UTF8String) +
                                  " (Exclusive capture)",
                           OutputBackend::CoreGraphics, true});
      }
    }
    return outputs;
  }
}

std::unique_ptr<Projector> openCoreGraphicsProjector(const ProjectorOutputInfo &output,
    const ProjectorOpenSettings &settings) {
  @autoreleasepool {
    if (@available(macOS 10.15.4, *)) {
      if (output.backend == OutputBackend::CoreGraphics)
        for (const auto display : onlineDisplays())
          if (displayIdentifier(display) == output.id)
            return std::make_unique<ProjectorCoreGraphics>(display, settings.mainThread);
      throw std::runtime_error("Secondary CoreGraphics projector is no longer available");
    }
    throw std::runtime_error("Exclusive CoreGraphics projection requires macOS 10.15.4 or newer");
  }
}
}
