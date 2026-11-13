#!/usr/bin/env python3
"""Transporta mensagens ROS serializadas por stdout/SSH; nao converte pontos."""
import struct
import sys
import threading
import zlib
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu


def main():
    rclpy.init(); node = rclpy.create_node('mid360_ssh_source')
    pending = {}; cv = threading.Condition(); closed = threading.Event()
    def receive(channel, raw):
        with cv:
            pending[channel] = raw  # bounded: only latest unsent message per topic
            cv.notify()
    def send():
        try:
            sys.stdout.buffer.write(b'MID360v1'); sys.stdout.buffer.flush()
            while not closed.is_set():
                with cv:
                    cv.wait_for(lambda: pending or closed.is_set(), timeout=1)
                    batch = list(pending.items()); pending.clear()
                for channel, raw in batch:
                    packed = zlib.compress(raw, 1)
                    sys.stdout.buffer.write(struct.pack('!BI', channel, len(packed)))
                    sys.stdout.buffer.write(packed); sys.stdout.buffer.flush()
        except (BrokenPipeError, OSError):
            closed.set()
    worker = threading.Thread(target=send, daemon=True); worker.start()
    node.create_subscription(PointCloud2, '/mid360/points', lambda m: receive(1, m), qos_profile_sensor_data, raw=True)
    node.create_subscription(Imu, '/mid360/imu', lambda m: receive(2, m), qos_profile_sensor_data, raw=True)
    try:
        while rclpy.ok() and not closed.is_set(): rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt: pass
    finally:
        closed.set()
        with cv: cv.notify_all()
        worker.join(timeout=1); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
