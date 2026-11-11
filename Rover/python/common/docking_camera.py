"""ROS 2 camera input shared by QR docking and read-only diagnostics."""
import math
import threading
import time

from . import ros2_runtime as ros

DEFAULT_NAMESPACE = '/rover_argo_N1/Instance1'


def add_camera_arguments(parser, diagnostic=True):
    parser.add_argument('--namespace', default=DEFAULT_NAMESPACE,
                        help='Namespace ROS 2 do rover; independente do sysid MAVLink')
    if diagnostic:
        parser.add_argument('--diagnostico', action='store_true',
                            help='Verifica imagem, calibracao e juntas sem conectar ao MAVProxy')


def validate_pair(image, info):
    if not image.header.frame_id or image.header.frame_id != info.header.frame_id:
        raise RuntimeError('Image e CameraInfo com frames diferentes ou vazios')
    if (image.width, image.height) != (info.width, info.height):
        raise RuntimeError('Resolucao de CameraInfo diferente da imagem')
    if abs(ros.nanoseconds(image.header.stamp)-ros.nanoseconds(info.header.stamp)) > 500000000:
        raise RuntimeError('CameraInfo nao corresponde ao instante da imagem')
    if (len(info.k) != 9 or not all(math.isfinite(v) for v in (*info.k, *info.d))
            or info.k[0] <= 0 or info.k[4] <= 0 or info.k[8] != 1):
        raise RuntimeError('CameraInfo sem calibracao valida')
    # solvePnP uses the OpenCV pinhole model, not fisheye/equidistant.
    if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
        raise RuntimeError('Modelo de distorcao nao suportado: '+info.distortion_model)


class DockingCamera:
    def __init__(self, namespace=DEFAULT_NAMESPACE, image_topic=None, info_topic=None):
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image, CameraInfo
        self.namespace = '/' + namespace.strip('/')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image = self.info = None
        self.wall = 0.
        self.subs = [ros.Subscriber(image_topic or self.namespace+'/camera/image_raw',
                                    Image, self._image, queue_size=1),
                     ros.Subscriber(info_topic or self.namespace+'/camera/camera_info',
                                    CameraInfo, self._info, queue_size=1)]

    def _image(self, msg):
        with self.lock:
            self.image, self.wall = msg, time.monotonic()

    def _info(self, msg):
        with self.lock:
            self.info = msg

    def snapshot(self, after_wall=0.):
        with self.lock:
            image, info, wall = self.image, self.info, self.wall
        if image is None or info is None or time.monotonic()-wall > .7:
            raise RuntimeError('Imagem ou CameraInfo indisponivel/desatualizada')
        if wall <= after_wall:
            raise RuntimeError('Aguardando imagem apos mover camera')
        if not -.05 <= ros.age(image.header.stamp) <= .5:
            raise RuntimeError('Timestamp da imagem invalido/desatualizado; confira /clock')
        validate_pair(image, info)
        # Conversion stays outside the ROS executor so it cannot delay /clock/joints.
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8').copy()
        return frame, info, image.header.stamp

    def close(self):
        for sub in self.subs:
            sub.unregister()


def diagnose(namespace, timeout=10.):
    """Read only: no MAVLink connection, arm, mode or pan/tilt commands."""
    ros.init_node('docking_camera_diagnostic', anonymous=True)
    camera = DockingCamera(namespace)
    joints = ros.JointFeedback(camera.namespace+'/joint_states')
    last = 'Sem mensagens'
    try:
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and not ros.is_shutdown():
            try:
                frame, info, stamp = camera.snapshot()
                pan = joints.position('pt_support_s23_pan_joint')
                tilt = joints.position('pt_support_s23_tilt_joint')
                print('ROS 2 OK: camera %dx%d frame=%s stamp=%.3f pan=%.3f tilt=%.3f' %
                      (info.width, info.height, info.header.frame_id, ros.seconds(stamp), pan, tilt), flush=True)
                from .qr_detector import QRDetector
                detector = QRDetector()
                text, _, _ = detector.detectAndDecode(frame)
                print(detector.status, flush=True)
                print('QR visivel: '+(text or 'nenhum (recepcao OK; docking nao validado)'), flush=True)
                return 0
            except RuntimeError as exc:
                last = str(exc)
            time.sleep(.05)
        print('Diagnostico ROS 2: '+last, flush=True)
        return 1
    finally:
        camera.close()
        joints.close()
        ros.signal_shutdown('diagnostico concluido')
