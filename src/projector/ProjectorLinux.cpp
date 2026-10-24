#include "ProjectorFactory.h"
#include "drm-lease-v1-client-protocol.h"

#ifdef SLSTUDIO_WITH_XRANDR_LEASE
#include <X11/Xlib.h>
#include <X11/Xlib-xcb.h>
#include <X11/extensions/Xrandr.h>
#include <xcb/randr.h>
#endif
#include <wayland-client.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <sys/mman.h>
#include <unistd.h>
#include <poll.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <stdexcept>

namespace slstudio {
namespace {

struct Lease {
  int fd = -1;
  virtual ~Lease() { if (fd >= 0) close(fd); }
  virtual void check() {}
};

class WaylandLease final : public Lease {
  struct Device;
  struct Connector {
    wp_drm_lease_connector_v1 *proxy = nullptr;
    Device *device = nullptr;
    std::string name;
    std::uint32_t id = 0;
    bool available = true;
    ~Connector() { if (proxy) wp_drm_lease_connector_v1_destroy(proxy); }
  };
  struct Device {
    wp_drm_lease_device_v1 *proxy = nullptr;
    std::uint32_t globalName = 0;
    std::vector<std::unique_ptr<Connector>> connectors;
    ~Device() {
      connectors.clear();
      if (proxy) wp_drm_lease_device_v1_destroy(proxy);
    }
  };
public:
  WaylandLease() {
    display = wl_display_connect(nullptr);
    if (!display)
      return;
    registry = wl_display_get_registry(display);
    static const wl_registry_listener listener = {
      [](void *data, wl_registry *registry, std::uint32_t name,
         const char *interface, std::uint32_t) {
        if (std::strcmp(interface, wp_drm_lease_device_v1_interface.name) != 0)
          return;
        auto &self = *static_cast<WaylandLease *>(data);
        auto device = std::make_unique<Device>();
        device->globalName = name;
        device->proxy = static_cast<wp_drm_lease_device_v1 *>(
            wl_registry_bind(registry, name, &wp_drm_lease_device_v1_interface, 1));
        static const wp_drm_lease_device_v1_listener deviceListener = {
          [](void *, wp_drm_lease_device_v1 *, int fd) { close(fd); },
          [](void *data, wp_drm_lease_device_v1 *, wp_drm_lease_connector_v1 *proxy) {
            auto &device = *static_cast<Device *>(data);
            auto connector = std::make_unique<Connector>();
            connector->proxy = proxy;
            connector->device = &device;
            static const wp_drm_lease_connector_v1_listener connectorListener = {
              [](void *data, wp_drm_lease_connector_v1 *, const char *name) {
                static_cast<Connector *>(data)->name = name;
              },
              [](void *, wp_drm_lease_connector_v1 *, const char *) {},
              [](void *data, wp_drm_lease_connector_v1 *, std::uint32_t id) {
                static_cast<Connector *>(data)->id = id;
              },
              [](void *, wp_drm_lease_connector_v1 *) {},
              [](void *data, wp_drm_lease_connector_v1 *) {
                static_cast<Connector *>(data)->available = false;
              }
            };
            wp_drm_lease_connector_v1_add_listener(proxy, &connectorListener, connector.get());
            device.connectors.push_back(std::move(connector));
          },
          [](void *, wp_drm_lease_device_v1 *) {},
          [](void *data, wp_drm_lease_device_v1 *proxy) {
            wp_drm_lease_device_v1_destroy(proxy);
            static_cast<Device *>(data)->proxy = nullptr;
          }
        };
        wp_drm_lease_device_v1_add_listener(device->proxy, &deviceListener, device.get());
        self.devices.push_back(std::move(device));
      },
      [](void *data, wl_registry *, std::uint32_t name) {
        auto &self = *static_cast<WaylandLease *>(data);
        for (const auto &device : self.devices) {
          if (device->globalName != name) continue;
          for (const auto &connector : device->connectors) connector->available = false;
          if (self.leasedDevice == device.get()) self.finished = true;
        }
      }
    };
    wl_registry_add_listener(registry, &listener, this);
    for (int round = 0; round < 3; ++round)
      if (wl_display_roundtrip(display) < 0)
        break;
  }

  ~WaylandLease() override {
    if (lease) wp_drm_lease_v1_destroy(lease);
    if (display) {
      for (const auto &device : devices)
        if (device->proxy) wp_drm_lease_device_v1_release(device->proxy);
      wl_display_roundtrip(display);
    }
    devices.clear();
    if (registry) wl_registry_destroy(registry);
    if (display) {
      wl_display_flush(display);
      wl_display_disconnect(display);
    }
  }

  std::vector<ProjectorOutputInfo> outputs() const {
    std::vector<ProjectorOutputInfo> result;
    for (std::size_t index = 0; index < devices.size(); ++index)
      for (const auto &connector : devices[index]->connectors)
        if (connector->available)
          result.push_back({std::to_string(devices[index]->globalName) + ":" + std::to_string(connector->id),
                             connector->name + " (DRM lease)", OutputBackend::WaylandLease, true});
    return result;
  }

  void acquire(const std::string &id) {
    Connector *selected = nullptr;
    for (std::size_t index = 0; index < devices.size(); ++index)
      for (const auto &connector : devices[index]->connectors)
        if (connector->available && id == std::to_string(devices[index]->globalName) + ":" + std::to_string(connector->id))
          selected = connector.get();
    if (!selected)
      throw std::runtime_error("Compositor is not offering the requested projector connector");
    leasedDevice = selected->device;
    auto *request = wp_drm_lease_device_v1_create_lease_request(selected->device->proxy);
    wp_drm_lease_request_v1_request_connector(request, selected->proxy);
    lease = wp_drm_lease_request_v1_submit(request);
    static const wp_drm_lease_v1_listener listener = {
      [](void *data, wp_drm_lease_v1 *, int fd) { static_cast<WaylandLease *>(data)->fd = fd; },
      [](void *data, wp_drm_lease_v1 *) { static_cast<WaylandLease *>(data)->finished = true; }
    };
    wp_drm_lease_v1_add_listener(lease, &listener, this);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (fd < 0 && !finished && std::chrono::steady_clock::now() < deadline)
      dispatch(100);
    if (fd < 0 || finished)
      throw std::runtime_error("Compositor denied or did not grant the DRM lease");
  }

  void check() override {
    dispatch(0);
    if (finished)
      throw std::runtime_error("Projector DRM lease was revoked");
  }

private:
  void dispatch(int timeout) {
    while (wl_display_prepare_read(display) != 0)
      if (wl_display_dispatch_pending(display) < 0)
        throw std::runtime_error("Wayland connection lost");
    wl_display_flush(display);
    pollfd descriptor{wl_display_get_fd(display), POLLIN, 0};
    const int result = poll(&descriptor, 1, timeout);
    if (result > 0 && (descriptor.revents & POLLIN)) {
      if (wl_display_read_events(display) < 0)
        throw std::runtime_error("Wayland connection lost");
    } else {
      wl_display_cancel_read(display);
      if (result < 0 && errno != EINTR)
        throw std::runtime_error("Wayland lease event polling failed");
      if (descriptor.revents & (POLLHUP | POLLERR))
        throw std::runtime_error("Wayland connection lost");
    }
    if (wl_display_dispatch_pending(display) < 0)
      throw std::runtime_error("Wayland connection lost");
  }
  wl_display *display = nullptr;
  wl_registry *registry = nullptr;
  wp_drm_lease_v1 *lease = nullptr;
  std::vector<std::unique_ptr<Device>> devices;
  bool finished = false;
  Device *leasedDevice = nullptr;
};

#ifdef SLSTUDIO_WITH_XRANDR_LEASE
class XRandRLease final : public Lease {
public:
  XRandRLease() {
    display = XOpenDisplay(nullptr);
    int major = 1, minor = 6;
    if (display && (!XRRQueryVersion(display, &major, &minor) ||
        major < 1 || (major == 1 && minor < 6))) {
      XCloseDisplay(display);
      display = nullptr;
    }
  }
  ~XRandRLease() override {
    if (display) {
      if (lease) {
        xcb_randr_free_lease(XGetXCBConnection(display), lease, true);
        xcb_flush(XGetXCBConnection(display));
      }
      XCloseDisplay(display);
    }
  }
  std::vector<ProjectorOutputInfo> outputs() const {
    std::vector<ProjectorOutputInfo> result;
    if (!display) return result;
    auto *resources = XRRGetScreenResourcesCurrent(display, DefaultRootWindow(display));
    if (!resources) return result;
    for (int index = 0; index < resources->noutput; ++index) {
      auto *output = XRRGetOutputInfo(display, resources, resources->outputs[index]);
      if (!output) continue;
      if (output->connection == RR_Connected && output->crtc == None && output->nmode > 0)
        result.push_back({std::to_string(resources->outputs[index]),
                           std::string(output->name, output->nameLen) + " (DRM lease)",
                           OutputBackend::XRandRLease, true});
      XRRFreeOutputInfo(output);
    }
    XRRFreeScreenResources(resources);
    return result;
  }
  void acquire(const std::string &id) {
    if (!display) throw std::runtime_error("XRandR leasing is unavailable");
    RROutput selected = std::stoul(id);
    auto *resources = XRRGetScreenResourcesCurrent(display, DefaultRootWindow(display));
    if (!resources) throw std::runtime_error("Cannot query XRandR outputs");
    auto *output = XRRGetOutputInfo(display, resources, selected);
    RRCrtc crtc = None;
    if (output && output->crtc == None && output->connection == RR_Connected) {
      for (int index = 0; index < output->ncrtc; ++index) {
        auto *candidate = XRRGetCrtcInfo(display, resources, output->crtcs[index]);
        if (candidate && candidate->noutput == 0) crtc = output->crtcs[index];
        if (candidate) XRRFreeCrtcInfo(candidate);
        if (crtc) break;
      }
    }
    if (output) XRRFreeOutputInfo(output);
    XRRFreeScreenResources(resources);
    if (!crtc) throw std::runtime_error("Detach the projector from the desktop before requesting an XRandR lease");
    auto *connection = XGetXCBConnection(display);
    lease = xcb_generate_id(connection);
    const xcb_randr_crtc_t selectedCrtc = crtc;
    const xcb_randr_output_t selectedOutput = selected;
    const auto cookie = xcb_randr_create_lease(connection, DefaultRootWindow(display),
                                               lease, 1, 1, &selectedCrtc, &selectedOutput);
    xcb_generic_error_t *error = nullptr;
    auto *reply = xcb_randr_create_lease_reply(connection, cookie, &error);
    if (reply && !error && reply->nfd == 1)
      fd = xcb_randr_create_lease_reply_fds(connection, reply)[0];
    std::free(error);
    std::free(reply);
    if (fd < 0) throw std::runtime_error("X server denied the projector DRM lease");
  }
private:
  Display *display = nullptr;
  xcb_randr_lease_t lease = 0;
};
#endif

class ScanoutBuffer {
public:
  ScanoutBuffer(int descriptor, cv::Size size) : fd(descriptor) {
    drm_mode_create_dumb create{};
    create.width = size.width;
    create.height = size.height;
    create.bpp = 32;
    if (drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0)
      throw std::runtime_error("Driver does not support projector scanout buffers");
    handle = create.handle;
    pitch = create.pitch;
    bytes = create.size;
    std::uint32_t handles[4]{handle}, pitches[4]{pitch}, offsets[4]{};
    if (drmModeAddFB2(fd, size.width, size.height, DRM_FORMAT_XRGB8888,
                       handles, pitches, offsets, &framebuffer, 0) != 0) {
      release();
      throw std::runtime_error("Cannot register projector framebuffer");
    }
    drm_mode_map_dumb map{};
    map.handle = handle;
    if (drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map) == 0)
      memory = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map.offset);
    if (memory == MAP_FAILED) {
      release();
      throw std::runtime_error("Cannot map projector scanout buffer");
    }
  }
  ~ScanoutBuffer() { release(); }
  ScanoutBuffer(const ScanoutBuffer &) = delete;
  ScanoutBuffer &operator=(const ScanoutBuffer &) = delete;
  void upload(const cv::Mat &pattern) {
    for (int row = 0; row < pattern.rows; ++row) {
      auto *destination = reinterpret_cast<std::uint32_t *>(static_cast<char *>(memory) + row * pitch);
      for (int col = 0; col < pattern.cols; ++col) {
        if (pattern.channels() == 1) {
          const auto value = pattern.at<unsigned char>(row, col);
          destination[col] = (value << 16) | (value << 8) | value;
        } else {
          const auto value = pattern.at<cv::Vec3b>(row, col);
          destination[col] = (value[0] << 16) | (value[1] << 8) | value[2];
        }
      }
    }
  }
  std::uint32_t framebuffer = 0;
private:
  void release() {
    if (memory != MAP_FAILED) munmap(memory, bytes);
    if (framebuffer) drmModeRmFB(fd, framebuffer);
    if (handle) {
      drm_mode_destroy_dumb destroy{};
      destroy.handle = handle;
      drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
    memory = MAP_FAILED;
    framebuffer = handle = 0;
  }
  int fd;
  std::uint32_t handle = 0, pitch = 0;
  std::size_t bytes = 0;
  void *memory = MAP_FAILED;
};

class LeasedProjector final : public Projector {
public:
  explicit LeasedProjector(std::unique_ptr<Lease> acquired) : lease(std::move(acquired)) {
    auto *objects = drmModeGetLease(lease->fd);
    auto *resources = drmModeGetResources(lease->fd);
    if (objects && resources) {
      auto included = [&](std::uint32_t id) {
        return std::find(objects->objects, objects->objects + objects->count, id) != objects->objects + objects->count;
      };
      for (int index = 0; index < resources->count_crtcs; ++index)
        if (included(resources->crtcs[index])) { crtc = resources->crtcs[index]; break; }
      for (int index = 0; index < resources->count_connectors; ++index) {
        if (!included(resources->connectors[index])) continue;
        auto *candidate = drmModeGetConnector(lease->fd, resources->connectors[index]);
        if (candidate && candidate->count_modes > 0) {
          connector = candidate->connector_id;
          mode = candidate->modes[0];
          for (int modeIndex = 0; modeIndex < candidate->count_modes; ++modeIndex)
            if (candidate->modes[modeIndex].type & DRM_MODE_TYPE_PREFERRED)
              mode = candidate->modes[modeIndex];
        }
        if (candidate) drmModeFreeConnector(candidate);
        if (connector) break;
      }
    }
    if (objects) drmFree(objects);
    if (resources) drmModeFreeResources(resources);
    if (!connector || !crtc)
      throw std::runtime_error("Lease lacks a usable connector, CRTC, or video mode");
    black = std::make_unique<ScanoutBuffer>(lease->fd, resolution());
    black->upload(cv::Mat(resolution(), CV_8UC1, cv::Scalar(0)));
    white = std::make_unique<ScanoutBuffer>(lease->fd, resolution());
    white->upload(cv::Mat(resolution(), CV_8UC1, cv::Scalar(255)));
    blank();
  }
  ~LeasedProjector() override {
    drmModeSetCrtc(lease->fd, crtc, 0, 0, 0, nullptr, 0, nullptr);
  }
  cv::Size resolution() const override { return {mode.hdisplay, mode.vdisplay}; }
  void setPatterns(const std::vector<cv::Mat> &patterns) override {
    std::vector<std::unique_ptr<ScanoutBuffer>> uploaded;
    for (const auto &pattern : patterns) {
      if (pattern.size() != resolution() || (pattern.type() != CV_8UC1 && pattern.type() != CV_8UC3))
        throw std::invalid_argument("Projector patterns must match native output pixels");
      auto buffer = std::make_unique<ScanoutBuffer>(lease->fd, resolution());
      buffer->upload(pattern);
      uploaded.push_back(std::move(buffer));
    }
    blank();
    buffers = std::move(uploaded);
  }
  void present(std::size_t index) override { flip(buffers.at(index)->framebuffer); }
  void blank(bool bright = false) override { flip(bright ? white->framebuffer : black->framebuffer); }
private:
  void flip(std::uint32_t framebuffer) {
    lease->check();
    if (!active) {
      if (drmModeSetCrtc(lease->fd, crtc, framebuffer, 0, 0, &connector, 1, &mode) != 0)
        throw std::runtime_error("DRM projector modeset failed");
      active = true;
      return;
    }
    pending = true;
    if (drmModePageFlip(lease->fd, crtc, framebuffer, DRM_MODE_PAGE_FLIP_EVENT, &pending) != 0)
      throw std::runtime_error("DRM projector page flip failed");
    drmEventContext events{};
    events.version = 2;
    events.page_flip_handler = [](int, unsigned int, unsigned int, unsigned int, void *data) {
      *static_cast<bool *>(data) = false;
    };
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (pending) {
      pollfd descriptor{lease->fd, POLLIN, 0};
      const int result = poll(&descriptor, 1, 100);
      lease->check();
      if (result < 0 && errno == EINTR) continue;
      if (result < 0 || (descriptor.revents & (POLLHUP | POLLERR)) ||
          std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("Projector page flip interrupted or timed out");
      if (descriptor.revents & POLLIN)
        if (drmHandleEvent(lease->fd, &events) != 0)
          throw std::runtime_error("Cannot read projector page flip completion");
    }
  }
  std::unique_ptr<Lease> lease;
  std::unique_ptr<ScanoutBuffer> black, white;
  std::vector<std::unique_ptr<ScanoutBuffer>> buffers;
  std::uint32_t crtc = 0, connector = 0;
  drmModeModeInfo mode{};
  bool active = false, pending = false;
};
}

std::vector<ProjectorOutputInfo> leasedProjectorOutputs() {
  WaylandLease wayland;
  auto result = wayland.outputs();
  if (std::getenv("WAYLAND_DISPLAY")) return result;
#ifdef SLSTUDIO_WITH_XRANDR_LEASE
  XRandRLease xrandr;
  auto xoutputs = xrandr.outputs();
  result.insert(result.end(), xoutputs.begin(), xoutputs.end());
#endif
  return result;
}

std::unique_ptr<Projector> openLeasedProjector(const ProjectorOutputInfo &output) {
  std::unique_ptr<Lease> lease;
  if (output.backend == OutputBackend::WaylandLease) {
    auto wayland = std::make_unique<WaylandLease>();
    wayland->acquire(output.id);
    lease = std::move(wayland);
#ifdef SLSTUDIO_WITH_XRANDR_LEASE
  } else if (output.backend == OutputBackend::XRandRLease) {
    auto xrandr = std::make_unique<XRandRLease>();
    xrandr->acquire(output.id);
    lease = std::move(xrandr);
  #endif
  } else {
    throw std::invalid_argument("Exclusive projector requires a DRM lease backend");
  }
  return std::make_unique<LeasedProjector>(std::move(lease));
}
}