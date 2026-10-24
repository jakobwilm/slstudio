#include "Ros2Output.h"

#include <cmath>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#ifdef SLSTUDIO_WITH_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

namespace slstudio {

struct Ros2Output::Impl {
  struct Pending {
    PointCloudConstPtr cloud;
    std::chrono::system_clock::time_point timestamp;
  };
  Ros2OutputSettings settings;
#ifdef SLSTUDIO_WITH_ROS2
  std::shared_ptr<rclcpp::Context> context = std::make_shared<rclcpp::Context>();
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
#endif
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::optional<Pending> pending;
  std::exception_ptr error;
  bool stopped = false;
  std::thread thread;

  explicit Impl(const Ros2OutputSettings &requestedSettings) : settings(requestedSettings) {
#ifdef SLSTUDIO_WITH_ROS2
    context->init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("slstudio", rclcpp::NodeOptions().context(context));
    publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        settings.topic, rclcpp::SensorDataQoS().keep_last(1));
    thread = std::thread([this] { run(); });
#else
    throw std::runtime_error("ROS 2 support was not built (SLSTUDIO_WITH_ROS2=OFF)");
#endif
  }
  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopped = true;
      pending.reset();
    }
    changed.notify_one();
    if (thread.joinable()) thread.join();
  }
#ifdef SLSTUDIO_WITH_ROS2
  void publish(const Pending &next) {
    const auto &points = next.cloud->points;
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        next.timestamp.time_since_epoch()).count();
    if (points.size() > std::numeric_limits<std::uint32_t>::max() / 16 ||
        timestamp < 0 || timestamp / 1000000000 > std::numeric_limits<std::int32_t>::max())
      throw std::invalid_argument("Point cloud size or timestamp exceeds PointCloud2 limits");
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = settings.frameId;
    message.header.stamp.sec = static_cast<std::int32_t>(timestamp / 1000000000);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(timestamp % 1000000000);
    message.height = 1;
    message.width = static_cast<std::uint32_t>(points.size());
    for (const auto *name : {"x", "y", "z", "rgb"}) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = static_cast<std::uint32_t>(message.fields.size() * 4);
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      message.fields.push_back(field);
    }
    const std::uint16_t endian = 1;
    message.is_bigendian = *reinterpret_cast<const std::uint8_t *>(&endian) == 0;
    message.point_step = 16;
    message.row_step = message.width * message.point_step;
    message.is_dense = true;
    message.data.resize(message.row_step);
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto &point = points[index];
      const float coordinates[] = {static_cast<float>(point.x * settings.metresPerUnit),
                                   static_cast<float>(point.y * settings.metresPerUnit),
                                   static_cast<float>(point.z * settings.metresPerUnit)};
      for (const auto coordinate : coordinates)
        if (!std::isfinite(coordinate)) message.is_dense = false;
      const std::uint32_t rgb = (std::uint32_t(point.r) << 16) | (std::uint32_t(point.g) << 8) | point.b;
      auto *destination = message.data.data() + index * 16;
      std::memcpy(destination, coordinates, sizeof(coordinates));
      std::memcpy(destination + 12, &rgb, sizeof(rgb));
    }
    publisher->publish(message);
  }
  void run() {
    try {
      for (;;) {
        Pending next;
        {
          std::unique_lock<std::mutex> lock(mutex);
          changed.wait(lock, [&] { return stopped || pending.has_value(); });
          if (stopped) return;
          next = std::move(*pending);
          pending.reset();
        }
        publish(next);
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      error = std::current_exception();
      stopped = true;
      pending.reset();
    }
  }
#endif
};

bool Ros2Output::available() {
#ifdef SLSTUDIO_WITH_ROS2
  return true;
#else
  return false;
#endif
}

Ros2Output::Ros2Output(const Ros2OutputSettings &settings) {
  if (settings.topic.empty() || settings.frameId.empty() ||
      !std::isfinite(settings.metresPerUnit) || settings.metresPerUnit <= 0)
    throw std::invalid_argument("ROS 2 output requires a topic, frame ID and positive metres-per-unit scale");
  impl = std::make_unique<Impl>(settings);
}
Ros2Output::~Ros2Output() = default;

void Ros2Output::submit(PointCloudConstPtr cloud, std::chrono::system_clock::time_point timestamp) {
  if (!cloud) throw std::invalid_argument("Cannot publish a null point cloud");
  {
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (impl->error) std::rethrow_exception(impl->error);
    impl->pending = Impl::Pending{std::move(cloud), timestamp};
  }
  impl->changed.notify_one();
}

void Ros2Output::checkError() const {
  std::lock_guard<std::mutex> lock(impl->mutex);
  if (impl->error) std::rethrow_exception(impl->error);
}
}
