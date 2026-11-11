#!/usr/bin/env python3
"""Real DDS/QoS/clock smoke test. No simulated or physical vehicle commands."""
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import rclpy

ros.init_node('rover_io_smoke')
peer=rclpy.create_node('rover_io_peer')
clock=peer.create_publisher(Clock,'/clock',10)
pub=peer.create_publisher(String,'/rover/test/sensor',qos_profile_sensor_data)
received=[]
sub=ros.Subscriber('/rover/test/sensor',String,lambda m:received.append(m.data))
end=time.monotonic()+8
while time.monotonic()<end:
    clock.publish(Clock(clock=Time(sec=42,nanosec=125000000)))
    pub.publish(String(data='best_effort_ok'))
    rclpy.spin_once(peer,timeout_sec=.03)
    if received and ros.seconds(ros.now())==42.125:break
else:raise SystemExit('FAIL: DDS sensor QoS or simulated clock')
assert abs(ros.age(Time(sec=42,nanosec=0))-.125)<1e-9
print('PASS: real DDS best-effort subscription, /clock and sensor age')
sub.unregister();peer.destroy_node();ros.signal_shutdown('test complete')
