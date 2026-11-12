#include <chrono>
#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// One network reader. Keep all fields, points, frame and acquisition timestamps.
// Native forwarding avoids converting a 1.4 MB payload to/from Python objects.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("companion_lidar_ingress");
  const auto source = node->declare_parameter<std::string>("source", "/rover_argo_N1/Instance1/livox/lidar");
  const auto target = node->declare_parameter<std::string>("target", source + "_onboard");
  if (source == target) throw std::invalid_argument("source and target must differ");
  using Cloud = sensor_msgs::msg::PointCloud2;
  auto pub = node->create_publisher<Cloud>(target, rclcpp::SensorDataQoS().keep_last(1));
  size_t count = 0;
  auto sub = node->create_subscription<Cloud>(source, rclcpp::QoS(1).reliable(),
    [&pub, &count](Cloud::ConstSharedPtr msg) { pub->publish(*msg); ++count; });
  auto timer = node->create_wall_timer(std::chrono::seconds(10), [&]() {
    RCLCPP_INFO(node->get_logger(), "MID360 recebido: %.1f Hz (janela de 10 s)", count / 10.);
    count = 0;
  });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
