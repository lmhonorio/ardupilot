#!/usr/bin/env python3
"""Publish exact Gazebo simulation timestamps at a bounded ROS clock rate."""
import argparse


class ClockGate:
    def __init__(self, hz=50.):
        if not 1 <= hz <= 250:
            raise ValueError('clock rate must be between 1 and 250 Hz')
        self.period = round(1e9 / hz)
        self.last = None

    def accept(self, stamp):
        # Publish resets immediately, including a reset back to zero. Never
        # extrapolate time while Gazebo is paused or disconnected.
        if self.last is None or stamp < self.last or stamp-self.last >= self.period:
            self.last = stamp
            return True
        return False


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock as GzClock
    p=argparse.ArgumentParser();p.add_argument('--hz',type=float,default=50.)
    a=p.parse_args();gate=ClockGate(a.hz)
    rclpy.init(args=[]);node=rclpy.create_node('rover_clock_bridge')
    pub=node.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    gz=Node()
    def receive(msg):
        if gate.accept(msg.sim.sec*10**9+msg.sim.nsec):
            clock=Clock();clock.clock.sec=msg.sim.sec;clock.clock.nanosec=msg.sim.nsec
            pub.publish(clock)
    if not gz.subscribe(GzClock,'/clock',receive):
        raise RuntimeError('Nao foi possivel assinar /clock do Gazebo')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        gz.unsubscribe('/clock');node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
