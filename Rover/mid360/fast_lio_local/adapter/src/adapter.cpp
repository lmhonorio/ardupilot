#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>

class Adapter : public rclcpp::Node {
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  template<class T> static T get(const uint8_t* ptr) {T v; std::memcpy(&v,ptr,sizeof(v));return v;}
 public:
  Adapter():Node("mid360_lio_timing") {
    pub=create_publisher<livox_ros_driver2::msg::CustomMsg>("/lio_test/livox",rclcpp::QoS(40).reliable());
    sub=create_subscription<sensor_msgs::msg::PointCloud2>("/lio_test/points",rclcpp::QoS(40).reliable(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){convert(*m);});
  }
  void convert(const sensor_msgs::msg::PointCloud2& m) {
    if(m.is_bigendian || m.point_step<26 || m.row_step<uint64_t(m.width)*m.point_step || m.data.size()!=uint64_t(m.height)*m.row_step)
      throw std::runtime_error("Invalid native Livox cloud layout");
    std::map<std::string,sensor_msgs::msg::PointField> f;
    for(const auto& field:m.fields)f[field.name]=field;
    for(const auto& name:{"x","y","z","intensity","tag","line","timestamp"}){
      const auto it=f.find(name); if(it==f.end() || it->second.count!=1)throw std::runtime_error("Missing Livox field");
      bool byte=std::string(name)=="tag" || std::string(name)=="line";
      bool stamp=std::string(name)=="timestamp";
      if(it->second.datatype!=(byte?2:stamp?8:7) || it->second.offset+(byte?1:stamp?8:4)>m.point_step)
        throw std::runtime_error("Unexpected Livox field format");
    }
    livox_ros_driver2::msg::CustomMsg out;
    out.header=m.header;out.timebase=uint64_t(m.header.stamp.sec)*1000000000ULL+m.header.stamp.nanosec;
    out.lidar_id=0;out.points.reserve(uint64_t(m.width)*m.height);
    for(uint32_t row=0;row<m.height;++row)for(uint32_t col=0;col<m.width;++col){
      auto p=m.data.data()+row*m.row_step+col*m.point_step;
      // Driver PointCloud2 exports absolute nanoseconds as FLOAT64.
      // CustomMsg requires offset nanoseconds from this cloud's timebase.
      double absolute_ns=get<double>(p+f.at("timestamp").offset);
      double t=absolute_ns-static_cast<double>(out.timebase);
      if(t<0 && t>=-512.0)t=0; // FLOAT64 rounding at epoch-scale nanoseconds
      if(!std::isfinite(t) || t<0 || t>500000000.0)throw std::runtime_error("Invalid per-point time: expected absolute nanoseconds consistent with header");
      livox_ros_driver2::msg::CustomPoint q;
      q.offset_time=static_cast<uint32_t>(std::llround(t));
      q.x=get<float>(p+f.at("x").offset);q.y=get<float>(p+f.at("y").offset);q.z=get<float>(p+f.at("z").offset);
      const float intensity=get<float>(p+f.at("intensity").offset);
      if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(intensity))continue;
      q.reflectivity=static_cast<uint8_t>(std::max(0.f,std::min(255.f,intensity)));
      q.tag=*(p+f.at("tag").offset);q.line=*(p+f.at("line").offset);
      if(q.line>=4)throw std::runtime_error("MID360 line out of range");
      out.points.push_back(q);
    }
    out.point_num=out.points.size();pub->publish(out);
  }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Adapter>());}catch(const std::exception& e){fprintf(stderr,"timing_adapter: %s\n",e.what());return 1;}rclcpp::shutdown();}
