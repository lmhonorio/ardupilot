#!/usr/bin/env python3
"""Only visualization outputs cross SSH. Estimator inputs remain on the Pi."""
import os
import struct
import sys
import threading
import time
import zlib

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = rclpy.create_node('lio_pi_visual_source')
    pending = {}
    condition = threading.Condition()
    closed = threading.Event()
    progress = [time.monotonic()]
    last_path = [0.0]

    def receive(channel, raw):
        if len(raw) > 16 * 1024 * 1024:
            raise RuntimeError('Resultado excede limite do transporte visual')
        with condition:
            pending[channel] = raw  # Never queues an ever-growing visualization backlog.
            condition.notify()

    def path(message):
        if time.monotonic() - last_path[0] < 1:
            return
        last_path[0] = time.monotonic()
        message.poses = message.poses[-3000:]
        receive(3, serialize_message(message))

    def send():
        try:
            sys.stdout.buffer.write(b'LIOPIv01')
            sys.stdout.buffer.flush()
            while not closed.is_set():
                with condition:
                    condition.wait_for(lambda: pending or closed.is_set(), timeout=1)
                    batch = list(pending.items())
                    pending.clear()
                for channel, raw in batch:
                    packed = zlib.compress(raw, 1)
                    sys.stdout.buffer.write(struct.pack('!BI', channel, len(packed)))
                    sys.stdout.buffer.write(packed)
                    sys.stdout.buffer.flush()
                    progress[0] = time.monotonic()
        except (BrokenPipeError, OSError):
            closed.set()

    def watchdog():
        while not closed.wait(2):
            if time.monotonic() - progress[0] > 30:
                os.write(2, b'Sem resultados FAST-LIO ou leitor SSH por 30s. Encerrando somente a ponte.\n')
                os._exit(2)

    worker = threading.Thread(target=send, daemon=True)
    worker.start()
    threading.Thread(target=watchdog, daemon=True).start()
    node.create_subscription(PointCloud2, '/lio_pi/cloud_registered', lambda m: receive(1, m), qos_profile_sensor_data, raw=True)
    node.create_subscription(Odometry, '/lio_pi/odometry', lambda m: receive(2, m), qos_profile_sensor_data, raw=True)
    node.create_subscription(Path, '/lio_pi/path', path, qos_profile_sensor_data)
    try:
        while rclpy.ok() and not closed.is_set():
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        closed.set()
        with condition:
            condition.notify_all()
        worker.join(timeout=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
