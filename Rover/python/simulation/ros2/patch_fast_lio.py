#!/usr/bin/env python3
"""Apply the explicit Jazzy transport fix to the pinned ROS 2 FAST-LIO port."""
from pathlib import Path
import sys
source=Path(sys.argv[1])/'src/laserMapping.cpp'
s=source.read_text()
old='this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk)'
new='this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(200), imu_cbk)'
if old not in s and new not in s:
 raise SystemExit('Fonte FAST-LIO mudou: revise QoS IMU antes de compilar')
s=s.replace(old,new)
# Publish the covariance from this estimate, not the previous scan's covariance.
before='    pubOdomAftMapped->publish(odomAftMapped);\n    auto P = kf.get_P();'
after='    auto P = kf.get_P();'
if before in s:
    s=s.replace(before,after)
    anchor='    geometry_msgs::msg::TransformStamped trans;'
    if s.count(anchor)!=1:raise SystemExit('Revise local de publicacao da covariancia FAST-LIO')
    s=s.replace(anchor,'    pubOdomAftMapped->publish(odomAftMapped);\n\n'+anchor)
source.write_text(s)
