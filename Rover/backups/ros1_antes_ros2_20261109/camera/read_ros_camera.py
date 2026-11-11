#!/usr/bin/python3
"""View the ROS Argo rover cameras using ROS Noetic and Python 3.8."""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import threading
import time

CAMERAS = {
    'samsung': 'camera/image_raw',
    'insta360': 'camera/insta360/image_raw',
}


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=CAMERAS, default='samsung')
    parser.add_argument('--namespace', default='/rover_argo_N1/Instance1',
                        help='Namespace do rover; troque Instance1 para outro rover')
    parser.add_argument('--topic', help='Topico sensor_msgs/Image; substitui --camera')
    parser.add_argument('--list', action='store_true', help='Listar topicos de imagem e sair')
    parser.add_argument('--headless', action='store_true', help='Receber sem abrir janela')
    parser.add_argument('--snapshot', type=Path, help='Salvar primeiro frame em PNG e sair')
    parser.add_argument('--save-dir', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'gazebo_simple/run/camera',
                        help='Diretorio para imagens salvas com a tecla S')
    parser.add_argument('--timeout', type=float, default=30,
                        help='Segundos sem frames antes de sair; 0 aguarda indefinidamente')
    args = parser.parse_args(argv)
    if args.timeout < 0:
        parser.error('--timeout deve ser >= 0')
    if args.snapshot and args.snapshot.suffix.lower() != '.png':
        parser.error('--snapshot deve terminar em .png')
    return args


def main(argv=None):
    args = arguments(argv)
    try:
        import cv2
        import rospy
        from cv_bridge import CvBridge, CvBridgeError
        from sensor_msgs.msg import Image
    except ImportError as exc:
        print('Dependencia indisponivel: %s\nCarregue o ROS e execute com /usr/bin/python3.8.' % exc,
              file=sys.stderr)
        return 1

    try:
        rospy.get_master().getPid()
    except Exception as exc:
        print('ROS master indisponivel. Inicie o roslaunch primeiro: %s' % exc,
              file=sys.stderr)
        return 1
    rospy.init_node('rover_camera_reader', anonymous=True)
    if args.list:
        topics = [(name, kind) for name, kind in rospy.get_published_topics('/')
                  if kind == 'sensor_msgs/Image']
        for name, _ in sorted(topics):
            print(name)
        if not topics:
            print('Nenhum topico sensor_msgs/Image anunciado.')
        return 0

    topic = args.topic or '/%s/%s' % (args.namespace.strip('/'), CAMERAS[args.camera])
    bridge = CvBridge()
    lock = threading.Lock()
    latest = {'frame': None, 'received': time.monotonic(), 'count': 0,
              'frame_id': '', 'encoding': ''}

    def receive(message):
        try:
            frame = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5, 'Falha ao converter imagem: %s', exc)
            return
        with lock:
            latest.update(frame=frame, received=time.monotonic(),
                          count=latest['count'] + 1, frame_id=message.header.frame_id,
                          encoding=message.encoding)

    # Keep only the latest image; enough receive buffer for 1920x1080 RGB.
    subscriber = rospy.Subscriber(topic, Image, receive, queue_size=1, buff_size=32 * 1024 * 1024)
    show = not args.headless and args.snapshot is None
    window = 'Camera do rover — Q/ESC: sair | S: salvar'
    print('Lendo %s' % topic, flush=True)
    if show:
        print('Teclas na janela: S salva PNG; Q ou ESC encerra.', flush=True)
    previous_count = 0
    report_at = time.monotonic()
    result = 0

    def save(path, frame):
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), frame):
            raise OSError('Nao foi possivel salvar %s' % path)
        print('Imagem salva: %s' % path.resolve(), flush=True)

    try:
        if show:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 960, 540)
        while not rospy.is_shutdown():
            with lock:
                frame, count = latest['frame'], latest['count']
                received, frame_id, encoding = latest['received'], latest['frame_id'], latest['encoding']
            now = time.monotonic()
            if args.timeout and now - received > args.timeout:
                print('Sem frames por %.0f s. Confira o topico com --list e se o Gazebo esta rodando.'
                      % args.timeout, file=sys.stderr)
                result = 1
                break
            if frame is not None:
                if previous_count == 0:
                    print('%dx%d | origem: %s | encoding ROS: %s' %
                          (frame.shape[1], frame.shape[0], frame_id, encoding), flush=True)
                if args.snapshot:
                    save(args.snapshot, frame)
                    break
                if show and count != previous_count:
                    cv2.imshow(window, frame)
                previous_count = count
            if show:
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), ord('Q'), 27):
                    break
                if key in (ord('s'), ord('S')) and frame is not None:
                    name = datetime.now().strftime('camera_%Y%m%d_%H%M%S_%f.png')
                    save(args.save_dir / name, frame)
                # OpenCV 4.2/GTK returns -1 for WND_PROP_VISIBLE even
                # while open. AUTOSIZE is 0/1 for an existing window and
                # -1 once it has been closed (also for WINDOW_NORMAL).
                if cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0:
                    break
            if now >= report_at:
                print('Frames: %d | atraso desde ultimo frame: %.1f s' %
                      (count, now - received), flush=True)
                report_at = now + 5
            # Wall time also works while Gazebo /clock is paused.
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
    except (OSError, cv2.error) as exc:
        print('Erro: %s' % exc, file=sys.stderr)
        result = 1
    finally:
        subscriber.unregister()
        if show:
            cv2.destroyAllWindows()
        rospy.signal_shutdown('Leitor de camera encerrado')
    return result


if __name__ == '__main__':
    sys.exit(main())
