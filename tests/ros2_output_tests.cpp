#include "Ros2Output.h"
#include "ros2_output_tests.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#ifdef SLSTUDIO_WITH_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
template <typename Function> void rejects(Function action, const char *expected) {
  try { action(); }
  catch (const std::exception &error) {
    require(std::string(error.what()).find(expected) != std::string::npos, error.what());
    return;
  }
  throw std::runtime_error("Invalid output operation was accepted");
}

void testSettings() {
  slstudio::Ros2OutputSettings settings;
  for (const auto scale : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
    settings.metresPerUnit = scale;
    rejects([&] { slstudio::Ros2Output output(settings); }, "positive");
  }
  settings.metresPerUnit = 1;
  settings.topic.clear();
  rejects([&] { slstudio::Ros2Output output(settings); }, "topic");
  settings.topic = "/points";
  settings.frameId.clear();
  rejects([&] { slstudio::Ros2Output output(settings); }, "frame ID");
}

#ifdef SLSTUDIO_WITH_ROS2
template <typename Function> void await(Function predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("ROS 2 output test timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void testPublishing() {
  require(slstudio::Ros2Output::available(), "ROS 2 output is unavailable in a ROS-enabled build");
  slstudio::Ros2OutputSettings settings{"/slstudio/test", "camera", 0.001};
  auto invalid = settings;
  invalid.topic = "/invalid topic";
  rejects([&] { slstudio::Ros2Output output(invalid); }, "topic");
  auto node = std::make_shared<rclcpp::Node>("slstudio_output_test");
  sensor_msgs::msg::PointCloud2::SharedPtr received;
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      settings.topic, rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::SharedPtr message) { received = std::move(message); });
  auto cloud = std::make_shared<PointCloud>();
  cloud->points = {{1000, 2000, 3000, 11, 22, 33},
                   {std::numeric_limits<float>::quiet_NaN(), 0, 0, 1, 2, 3}};
  const auto timestamp = std::chrono::system_clock::time_point(
      std::chrono::nanoseconds(1700000000123456789LL));
  {
    slstudio::Ros2Output output(settings);
    rejects([&] { output.submit(nullptr, timestamp); }, "null");
    await([&] {
      output.checkError();
      output.submit(cloud, timestamp);
      rclcpp::spin_some(node);
      return received != nullptr;
    });
    require(received->header.frame_id == "camera" &&
            received->header.stamp.sec == 1700000000 &&
            received->header.stamp.nanosec == 123456789, "Incorrect header");
    require(received->height == 1 && received->width == 2 &&
            received->point_step == 16 && received->row_step == 32 &&
            received->data.size() == 32 && !received->is_dense, "Incorrect cloud layout");
    const std::uint16_t endian = 1;
    require(received->is_bigendian == (*reinterpret_cast<const std::uint8_t *>(&endian) == 0),
            "Incorrect byte order");
    require(received->fields.size() == 4, "Incorrect field count");
    const char *names[] = {"x", "y", "z", "rgb"};
    for (std::size_t index = 0; index < 4; ++index)
      require(received->fields[index].name == names[index] &&
              received->fields[index].offset == index * 4 &&
              received->fields[index].datatype == sensor_msgs::msg::PointField::FLOAT32 &&
              received->fields[index].count == 1, "Incorrect point fields");
    float coordinates[3]{};
    std::uint32_t rgb = 0;
    std::memcpy(coordinates, received->data.data(), sizeof(coordinates));
    std::memcpy(&rgb, received->data.data() + 12, sizeof(rgb));
    require(coordinates[0] == 1 && coordinates[1] == 2 && coordinates[2] == 3 &&
            rgb == 0x0b1621, "Incorrect scaled XYZ or packed RGB");
    auto empty = std::make_shared<PointCloud>();
    await([&] {
      output.checkError();
      output.submit(empty, timestamp);
      rclcpp::spin_some(node);
      return received->width == 0;
    });
    require(received->height == 1 && received->row_step == 0 &&
            received->data.empty() && received->is_dense, "Incorrect empty cloud layout");

    PointCloudConstPtr latest;
    for (int index = 1; index <= 1000; ++index) {
      auto next = std::make_shared<PointCloud>();
      next->points.push_back({float(index * 1000), 0, 0, 0, 0, 0});
      latest = next;
      output.submit(latest, timestamp);
    }
    await([&] {
      output.checkError();
      rclcpp::spin_some(node);
      if (received->width != 1) return false;
      float x = 0;
      std::memcpy(&x, received->data.data(), sizeof(x));
      return x == 1000;
    });
    output.submit(cloud, std::chrono::system_clock::time_point(std::chrono::seconds(-1)));
    await([&] {
      try { output.checkError(); }
      catch (const std::exception &error) {
        require(std::string(error.what()).find("PointCloud2 limits") != std::string::npos, error.what());
        return true;
      }
      return false;
    });
    rejects([&] { output.submit(cloud, timestamp); }, "PointCloud2 limits");
  }
  require(rclcpp::ok(), "Stopping output shut down the caller's ROS context");
  for (int index = 0; index < 2; ++index) {
    slstudio::Ros2Output output(settings);
    output.submit(cloud, timestamp);
  }
}
#endif
}

int runRos2OutputTests() {
  try {
    testSettings();
#ifdef SLSTUDIO_WITH_ROS2
    rclcpp::init(0, nullptr);
    testPublishing();
    rclcpp::shutdown();
#else
    require(!slstudio::Ros2Output::available(), "ROS 2 was enabled in a ROS-free build");
    rejects([] { slstudio::Ros2Output output(slstudio::Ros2OutputSettings{}); }, "not built");
#endif
    std::cout << "ROS 2 output tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
#ifdef SLSTUDIO_WITH_ROS2
    if (rclcpp::ok()) rclcpp::shutdown();
#endif
    return 1;
  }
}

#ifdef SLSTUDIO_ROS2_OUTPUT_TEST_MAIN
int main() { return runRos2OutputTests(); }
#endif
