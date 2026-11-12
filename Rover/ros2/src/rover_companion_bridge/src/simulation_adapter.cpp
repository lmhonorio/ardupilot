#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

// Gazebo measures the whole batch at one instant. Keep exactly the Python
// adapter's timing and sampling, without thousands of Python point objects.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("companion_simulation_adapter");
  const auto source = node->declare_parameter<std::string>("source", "/rover_argo_N1/Instance1/livox/lidar_onboard");
  const auto target = node->declare_parameter<std::string>("target", "/fast_lio/lidar");
  const auto visual = node->declare_parameter<std::string>("visual", "/mid360/points3d");
  const auto stride = node->declare_parameter<int>("stride", 8);
  if (stride < 1 || stride > 16) throw std::invalid_argument("stride must be 1..16");
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Livox = livox_ros_driver2::msg::CustomMsg;
  auto pub = node->create_publisher<Livox>(target, rclcpp::QoS(1));
  auto xyz_pub = node->create_publisher<Cloud>(visual, rclcpp::QoS(1));
  auto sub = node->create_subscription<Cloud>(source, rclcpp::SensorDataQoS().keep_last(1),
    [=](Cloud::ConstSharedPtr msg) {
      constexpr uint32_t offset_ns = 100000000;
      const int64_t ns = int64_t(msg->header.stamp.sec) * 1000000000 + msg->header.stamp.nanosec;
      if (ns <= offset_ns) return;
      int offsets[3] = {-1,-1,-1};
      for (const auto &f : msg->fields) {
        for (int k=0; k<3; ++k) {
          if (f.name == std::string(1, "xyz"[k]) && f.count == 1 &&
              f.datatype == sensor_msgs::msg::PointField::FLOAT32 && f.offset+4 <= msg->point_step)
            offsets[k] = static_cast<int>(f.offset);
        }
      }
      if (msg->is_bigendian || offsets[0]<0 || offsets[1]<0 || offsets[2]<0 ||
          uint64_t(msg->width)*msg->point_step > msg->row_step ||
          uint64_t(msg->row_step)*msg->height > msg->data.size()) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                            "Invalid simulated XYZ layout; scan rejected");
        return;
      }
      Livox out;
      out.header = msg->header;
      out.timebase = ns-offset_ns;
      out.header.stamp.sec = out.timebase/1000000000;
      out.header.stamp.nanosec = out.timebase%1000000000;
      size_t valid = 0;
      for (uint32_t row=0; row<msg->height; ++row) {
        for (uint32_t col=0; col<msg->width; ++col) {
          const auto *data = msg->data.data()+size_t(row)*msg->row_step+size_t(col)*msg->point_step;
          float p[3];
          for (int k=0; k<3; ++k) std::memcpy(&p[k], data+offsets[k], 4);
          const float range = std::sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
          if (!std::isfinite(range) || range<=.35f || range>=50.f) continue;
          if (valid++ % stride) continue;
          livox_ros_driver2::msg::CustomPoint point;
          point.x=p[0]; point.y=p[1]; point.z=p[2];
          point.offset_time=offset_ns; point.reflectivity=1; point.tag=0; point.line=0;
          out.points.push_back(point);
        }
      }
      if (out.points.size()<10) return;
      out.point_num = out.points.size();
      Cloud xyz; xyz.header=msg->header;
      sensor_msgs::PointCloud2Modifier modifier(xyz);
      modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,
                                     "y",1,sensor_msgs::msg::PointField::FLOAT32,
                                     "z",1,sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(out.points.size());
      sensor_msgs::PointCloud2Iterator<float> x(xyz,"x"), y(xyz,"y"), z(xyz,"z");
      for (const auto &p : out.points) { *x=p.x; *y=p.y; *z=p.z; ++x; ++y; ++z; }
      xyz_pub->publish(xyz); pub->publish(out);
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
