"""ROS 2 I/O for synchronous MAVProxy controllers.

One rclpy node is spun in a background executor. Algorithms retain their
blocking control loops; all transport, clocks, QoS and messages are ROS 2.
This is deliberately a small project API, not a replacement rospy module.
"""
import atexit
from collections import deque
import math
import os
import threading
import time

_node = None
_executor = None
_thread = None
_last_log = {}
_failure = None
_closed = False


def init_node(name, anonymous=False, disable_signals=False):
    global _node, _executor, _thread
    if _node is not None:
        return _node
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    suffix = '_%d' % os.getpid() if anonymous else ''
    sim = os.environ.get('ROVER_USE_SIM_TIME', 'true').lower() == 'true'
    _node = rclpy.create_node(name + suffix, parameter_overrides=[Parameter('use_sim_time', value=sim)])
    _executor = SingleThreadedExecutor()
    _executor.add_node(_node)
    def spin():
        global _failure
        try:
            _executor.spin()
        except Exception as exc:
            if rclpy.ok() and not _closed:
                _failure = exc
                _node.get_logger().error('Callback ROS 2 falhou: %s' % exc)
    _thread = threading.Thread(target=spin, daemon=True)
    _thread.start()
    atexit.register(signal_shutdown, 'exit')
    return _node


def node():
    if _node is None:
        raise RuntimeError('Chame init_node antes de acessar ROS 2')
    return _node


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def nanoseconds(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def now():
    return node().get_clock().now().to_msg()


def age(stamp):
    return (node().get_clock().now().nanoseconds - nanoseconds(stamp)) / 1e9


def stamp_from_seconds(value):
    from builtin_interfaces.msg import Time
    ns = round(value * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def Duration(value):
    from rclpy.duration import Duration as RclDuration
    return RclDuration(seconds=value).to_msg()


def get_param(name, default=None):
    key = name.lstrip('/')
    return node().get_parameter(key).value if node().has_parameter(key) else default


def is_shutdown():
    import rclpy
    if _failure is not None:
        raise RuntimeError('Executor ROS 2 interrompido') from _failure
    return not rclpy.ok()


def signal_shutdown(reason=''):
    global _closed
    import rclpy
    if _closed:
        return
    _closed = True
    if _executor is not None:
        _executor.shutdown(timeout_sec=2)
    if _thread is not None and threading.current_thread() is not _thread:
        _thread.join(timeout=3)
    if _node is not None:
        _node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def sensor_qos(depth=5):
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)


class Subscriber:
    def __init__(self, topic, kind, callback, queue_size=5, buff_size=None):
        self.handle = node().create_subscription(kind, topic, callback, sensor_qos(queue_size))

    def unregister(self):
        node().destroy_subscription(self.handle)

    def get_num_connections(self):
        return node().count_publishers(self.handle.topic_name)


class Publisher:
    def __init__(self, topic, kind, queue_size=5, latch=False):
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=queue_size)
        if latch:
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.kind = kind
        self.handle = node().create_publisher(kind, topic, qos)

    def publish(self, value):
        if not isinstance(value, self.kind):
            value = self.kind(data=value)
        self.handle.publish(value)


def get_published_topics(namespace='/'):
    return [(name, kinds[0]) for name, kinds in node().get_topic_names_and_types()
            if name.startswith(namespace) and kinds]


def _log(level, interval, message, *args):
    text = message % args if args else str(message)
    key = (level, message)
    current = time.monotonic()
    if current - _last_log.get(key, -float('inf')) >= interval:
        getattr(node().get_logger(), level)(text)
        _last_log[key] = current


def logwarn_throttle(interval, message, *args):
    _log('warning', interval, message, *args)


def loginfo_throttle(interval, message, *args):
    _log('info', interval, message, *args)


def logerr(message, *args):
    _log('error', 0, message, *args)


class JointFeedback:
    """Measured joint positions from Harmonic; never substitutes commands for feedback."""
    def __init__(self, topic='/rover_argo_N1/Instance1/joint_states'):
        from sensor_msgs.msg import JointState
        self._data = None
        self._wall = 0
        self._history = deque(maxlen=300)
        self.sub = Subscriber(topic, JointState, self._receive)

    def _receive(self, msg):
        self._data, self._wall = msg, time.monotonic()
        self._history.append(msg)

    def position(self, name, stamp=None):
        msg = self._data
        if msg is None or time.monotonic() - self._wall > .7 or not -.05 <= age(msg.header.stamp) <= .5:
            raise RuntimeError('Feedback de juntas Harmonic ausente/desatualizado')
        if stamp is not None:
            t=seconds(stamp)
            samples=[(seconds(m.header.stamp),dict(zip(m.name,m.position)).get(name))
                     for m in list(self._history)]
            samples=[(s,p) for s,p in samples if p is not None and abs(s-t)<.12]
            if not samples:raise RuntimeError('Sem junta sincronizada com a imagem: '+name)
            before=[s for s in samples if s[0]<=t];after=[s for s in samples if s[0]>=t]
            if before and after:
                a=max(before);b=min(after)
                value=a[1] if b[0]==a[0] else a[1]+(b[1]-a[1])*(t-a[0])/(b[0]-a[0])
            else:value=min(samples,key=lambda s:abs(s[0]-t))[1]
            if not math.isfinite(value):raise RuntimeError('Posicao da junta invalida: '+name)
            return value
        positions = dict(zip(msg.name, msg.position))
        if name not in positions:
            raise RuntimeError('Junta nao publicada: ' + name)
        if not math.isfinite(positions[name]):
            raise RuntimeError('Posicao da junta invalida: ' + name)
        return positions[name]

    def wait(self, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not is_shutdown():
            msg = self._data
            if (msg is not None and time.monotonic()-self._wall <= .7
                    and -.05 <= age(msg.header.stamp) <= .5):
                return
            time.sleep(.05)
        raise TimeoutError('Sem joint_states recentes do Gazebo Harmonic')

    def close(self):
        self.sub.unregister()
