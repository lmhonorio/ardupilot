// Native Gazebo Classic transport -> framed binary stream for Python 3.8.
// stdout is binary only. Header: "GZS1", uint32 JSON bytes, uint32 data bytes
// (little endian). Camera payload: raw pixels. LiDAR: float64 ranges, row major.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <vector>
#include <cstring>

std::mutex outputMutex;
std::atomic<bool> running{true};
void stop(int) { running = false; }
void writeU32(uint32_t value) {
  for (int i = 0; i < 4; ++i) std::cout.put((value >> (8 * i)) & 255);
}
void writePacket(const std::string &meta, const char *data, size_t size) {
  std::lock_guard<std::mutex> lock(outputMutex);
  std::cout.write("GZS1", 4);
  writeU32(meta.size()); writeU32(size);
  std::cout.write(meta.data(), meta.size()); std::cout.write(data, size);
  std::cout.flush();
}
void camera(ConstImageStampedPtr &msg) {
  const auto &im = msg->image();
  std::ostringstream m;
  m << "{\"type\":\"camera\",\"sec\":" << msg->time().sec()
    << ",\"nsec\":" << msg->time().nsec() << ",\"width\":" << im.width()
    << ",\"height\":" << im.height() << ",\"step\":" << im.step()
    << ",\"format\":" << im.pixel_format() << "}";
  writePacket(m.str(), im.data().data(), im.data().size());
}
void lidar(ConstLaserScanStampedPtr &msg) {
  const auto &s = msg->scan();
  std::ostringstream m;
  m << std::setprecision(17)
    << "{\"type\":\"lidar\",\"sec\":" << msg->time().sec()
    << ",\"nsec\":" << msg->time().nsec() << ",\"count\":" << s.count()
    << ",\"vertical_count\":" << s.vertical_count()
    << ",\"angle_min\":" << s.angle_min() << ",\"angle_step\":" << s.angle_step()
    << ",\"vertical_angle_min\":" << s.vertical_angle_min()
    << ",\"vertical_angle_step\":" << s.vertical_angle_step()
    << ",\"range_min\":" << s.range_min() << ",\"range_max\":" << s.range_max() << "}";
  // Encode explicitly in little endian, preserving infinity/no-return values.
  std::vector<char> payload(s.ranges_size() * 8);
  for (int i = 0; i < s.ranges_size(); ++i) {
    double r = s.ranges(i); uint64_t bits; std::memcpy(&bits, &r, 8);
    for (int j = 0; j < 8; ++j) payload[i * 8 + j] = (bits >> (j * 8)) & 255;
  }
  writePacket(m.str(), payload.data(), payload.size());
}
int main(int argc, char **argv) {
  gazebo::client::setup(argc, argv);
  std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("rover_simple");
  auto imageSub = node->Subscribe("~/rover_mid360/cam_link/front_camera/image", camera);
  auto lidarSub = node->Subscribe("~/rover_mid360/mid360_link/mid360/scan", lidar);
  while (running && std::cout.good()) gazebo::common::Time::MSleep(50);
  gazebo::client::shutdown();
}
