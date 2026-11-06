#include "GnomeDisplayReservation.h"
#include <gio/gio.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace slstudio {
namespace {
struct VariantDeleter {
  void operator()(GVariant *value) const { g_variant_unref(value); }
};
struct ProxyDeleter {
  void operator()(GDBusProxy *value) const { g_object_unref(value); }
};
using Variant = std::unique_ptr<GVariant, VariantDeleter>;
using Proxy = std::unique_ptr<GDBusProxy, ProxyDeleter>;

Variant child(GVariant *value, gsize index) { return Variant(g_variant_get_child_value(value, index)); }
std::string text(GVariant *value, gsize index) {
  const auto item = child(value, index);
  return g_variant_get_string(item.get(), nullptr);
}

Proxy displayProxy() {
  GError *error = nullptr;
  Proxy proxy(g_dbus_proxy_new_for_bus_sync(G_BUS_TYPE_SESSION, G_DBUS_PROXY_FLAGS_DO_NOT_AUTO_START,
      nullptr, "org.gnome.Mutter.DisplayConfig", "/org/gnome/Mutter/DisplayConfig",
      "org.gnome.Mutter.DisplayConfig", nullptr, &error));
  if (error) g_error_free(error);
  return proxy;
}

Variant call(GDBusProxy *proxy, const char *method, GVariant *parameters, int timeout) {
  GError *error = nullptr;
  Variant result(g_dbus_proxy_call_sync(proxy, method, parameters, G_DBUS_CALL_FLAGS_NO_AUTO_START,
      timeout, nullptr, &error));
  if (!result) {
    const std::string message = error ? error->message : "GNOME display configuration failed";
    if (error) g_error_free(error);
    throw std::runtime_error(message);
  }
  return result;
}

bool flag(GVariant *properties, const char *name) {
  gboolean value = false;
  g_variant_lookup(properties, name, "b", &value);
  return value;
}

bool primaryMonitor(GVariant *state, GVariant *spec) {
  const auto logical = child(state, 2);
  for (gsize index = 0; index < g_variant_n_children(logical.get()); ++index) {
    const auto monitor = child(logical.get(), index);
    if (!g_variant_get_boolean(child(monitor.get(), 4).get())) continue;
    const auto specs = child(monitor.get(), 5);
    for (gsize specIndex = 0; specIndex < g_variant_n_children(specs.get()); ++specIndex)
      if (g_variant_equal(child(specs.get(), specIndex).get(), spec)) return true;
  }
  return false;
}

Variant findMonitor(GVariant *state, GVariant *spec) {
  const auto monitors = child(state, 1);
  for (gsize index = 0; index < g_variant_n_children(monitors.get()); ++index) {
    auto monitor = child(monitors.get(), index);
    if (g_variant_equal(child(monitor.get(), 0).get(), spec)) return monitor;
  }
  throw std::runtime_error("Projector display is no longer connected");
}

struct Configuration {
  Variant logical;
  Variant properties;
};

Configuration configuration(GVariant *state, GVariant *reserved = nullptr) {
  g_auto(GVariantBuilder) logical;
  g_variant_builder_init(&logical, G_VARIANT_TYPE("a(iiduba(ssa{sv}))"));
  const auto current = child(state, 2);
  for (gsize index = 0; index < g_variant_n_children(current.get()); ++index) {
    const auto item = child(current.get(), index);
    const auto specs = child(item.get(), 5);
    g_auto(GVariantBuilder) monitors;
    g_variant_builder_init(&monitors, G_VARIANT_TYPE("a(ssa{sv})"));
    unsigned int count = 0;
    for (gsize specIndex = 0; specIndex < g_variant_n_children(specs.get()); ++specIndex) {
      const auto spec = child(specs.get(), specIndex);
      if (reserved && g_variant_equal(spec.get(), reserved)) continue;
      const auto monitor = findMonitor(state, spec.get());
      const auto modes = child(monitor.get(), 1);
      std::string modeId;
      for (gsize modeIndex = 0; modeIndex < g_variant_n_children(modes.get()); ++modeIndex) {
        const auto mode = child(modes.get(), modeIndex);
        if (flag(child(mode.get(), 6).get(), "is-current")) modeId = text(mode.get(), 0);
      }
      if (modeId.empty()) throw std::runtime_error("Cannot determine the current desktop display mode");
      g_auto(GVariantBuilder) properties;
      g_variant_builder_init(&properties, G_VARIANT_TYPE_VARDICT);
      const auto original = child(monitor.get(), 2);
      guint32 color = 0;
      if (g_variant_lookup(original.get(), "color-mode", "u", &color))
        g_variant_builder_add(&properties, "{sv}", "color-mode", g_variant_new_uint32(color));
      gboolean underscanning = false;
      if (g_variant_lookup(original.get(), "is-underscanning", "b", &underscanning))
        g_variant_builder_add(&properties, "{sv}", "underscanning", g_variant_new_boolean(underscanning));
      g_variant_builder_add(&monitors, "(ss@a{sv})", text(spec.get(), 0).c_str(), modeId.c_str(),
          g_variant_builder_end(&properties));
      ++count;
    }
    if (!count) continue;
    g_variant_builder_add(&logical, "(iidub@a(ssa{sv}))",
        g_variant_get_int32(child(item.get(), 0).get()), g_variant_get_int32(child(item.get(), 1).get()),
        g_variant_get_double(child(item.get(), 2).get()), g_variant_get_uint32(child(item.get(), 3).get()),
        g_variant_get_boolean(child(item.get(), 4).get()), g_variant_builder_end(&monitors));
  }
  g_auto(GVariantBuilder) leases;
  g_variant_builder_init(&leases, G_VARIANT_TYPE("a(ssss)"));
  const auto monitors = child(state, 1);
  for (gsize index = 0; index < g_variant_n_children(monitors.get()); ++index) {
    const auto monitor = child(monitors.get(), index);
    const auto spec = child(monitor.get(), 0);
    if (flag(child(monitor.get(), 2).get(), "is-for-lease") ||
        (reserved && g_variant_equal(spec.get(), reserved)))
      g_variant_builder_add_value(&leases, spec.get());
  }
  g_auto(GVariantBuilder) properties;
  g_variant_builder_init(&properties, G_VARIANT_TYPE_VARDICT);
  guint32 layout = 1;
  g_variant_lookup(child(state, 3).get(), "layout-mode", "u", &layout);
  g_variant_builder_add(&properties, "{sv}", "layout-mode", g_variant_new_uint32(layout));
  g_variant_builder_add(&properties, "{sv}", "monitors-for-lease", g_variant_builder_end(&leases));
  return {Variant(g_variant_ref_sink(g_variant_builder_end(&logical))),
          Variant(g_variant_ref_sink(g_variant_builder_end(&properties)))};
}

void apply(GDBusProxy *proxy, GVariant *state, const Configuration &config, int timeout) {
  call(proxy, "ApplyMonitorsConfig", g_variant_new("(uu@a(iiduba(ssa{sv}))@a{sv})",
      g_variant_get_uint32(child(state, 0).get()), 1,
      g_variant_ref(config.logical.get()), g_variant_ref(config.properties.get())), timeout);
}
}

std::vector<ProjectorOutputInfo> gnomeLeaseCandidates() {
  try {
    const auto proxy = displayProxy();
    if (!proxy) return {};
    const Variant allowed(g_dbus_proxy_get_cached_property(proxy.get(), "ApplyMonitorsConfigAllowed"));
    if (!allowed || !g_variant_get_boolean(allowed.get())) return {};
    const auto state = call(proxy.get(), "GetCurrentState", nullptr, 2000);
    const auto monitors = child(state.get(), 1);
    std::vector<ProjectorOutputInfo> result;
    for (gsize index = 0; index < g_variant_n_children(monitors.get()); ++index) {
      const auto monitor = child(monitors.get(), index);
      const auto spec = child(monitor.get(), 0);
      const auto properties = child(monitor.get(), 2);
      if (flag(properties.get(), "is-builtin") || flag(properties.get(), "is-for-lease") ||
          primaryMonitor(state.get(), spec.get())) continue;
      const auto modes = child(monitor.get(), 1);
      if (!g_variant_n_children(modes.get())) continue;
      const auto connector = text(spec.get(), 0);
      ProjectorOutputInfo output{connector, connector + " (DRM lease)", OutputBackend::GnomeLease, true, {}, connector};
      for (gsize modeIndex = 0; modeIndex < g_variant_n_children(modes.get()); ++modeIndex) {
        const auto mode = child(modes.get(), modeIndex);
        const int width = g_variant_get_int32(child(mode.get(), 1).get());
        const int height = g_variant_get_int32(child(mode.get(), 2).get());
        const double refresh = g_variant_get_double(child(mode.get(), 3).get());
        const bool interlaced = flag(child(mode.get(), 6).get(), "is-interlaced");
        output.displayModes.push_back({std::to_string(width) + "x" + std::to_string(height) + "@" +
            std::to_string(std::lround(refresh * 1000)) + (interlaced ? "i" : ""),
            cv::format("%dx%d @ %.3f Hz%s", width, height, refresh, interlaced ? " (interlaced)" : "")});
      }
      result.push_back(std::move(output));
    }
    return result;
  } catch (const std::exception &) {
    return {};
  }
}

struct GnomeDisplayReservation::State {
  Proxy proxy;
  Configuration original, expected;
  int timeout = 5000;
  bool active = false;

  ~State() {
    if (!active) return;
    try {
      const auto current = call(proxy.get(), "GetCurrentState", nullptr, timeout);
      const auto config = configuration(current.get());
      if (expected.logical && (!g_variant_equal(expected.logical.get(), config.logical.get()) ||
          !g_variant_equal(expected.properties.get(), config.properties.get()))) {
        std::cerr << "Desktop configuration changed; leaving the projector reserved for leasing\n";
        return;
      }
      apply(proxy.get(), current.get(), original, timeout);
    } catch (const std::exception &error) {
      std::cerr << "Cannot restore projector desktop configuration: " << error.what() << '\n';
    }
  }
};

GnomeDisplayReservation::GnomeDisplayReservation(const std::string &connector, std::chrono::milliseconds timeout)
    : state(std::make_unique<State>()) {
  state->timeout = static_cast<int>(timeout.count());
  state->proxy = displayProxy();
  if (!state->proxy) throw std::runtime_error("GNOME display configuration is unavailable");
  const auto current = call(state->proxy.get(), "GetCurrentState", nullptr, state->timeout);
  const auto monitors = child(current.get(), 1);
  for (gsize index = 0; index < g_variant_n_children(monitors.get()); ++index) {
    const auto monitor = child(monitors.get(), index);
    const auto spec = child(monitor.get(), 0);
    if (text(spec.get(), 0) != connector) continue;
    const auto properties = child(monitor.get(), 2);
    if (flag(properties.get(), "is-builtin") || primaryMonitor(current.get(), spec.get()))
      throw std::runtime_error("Cannot reserve a built-in or primary desktop display");
    if (flag(properties.get(), "is-for-lease")) return;
    state->original = configuration(current.get());
    const auto reserved = configuration(current.get(), spec.get());
    apply(state->proxy.get(), current.get(), reserved, state->timeout);
    state->active = true;
    const auto configured = call(state->proxy.get(), "GetCurrentState", nullptr, state->timeout);
    state->expected = configuration(configured.get());
    return;
  }
  throw std::runtime_error("Projector display is no longer connected");
}

GnomeDisplayReservation::~GnomeDisplayReservation() = default;
}