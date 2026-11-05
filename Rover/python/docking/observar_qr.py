#!/usr/bin/env python3.8
"""Read-only QR detection for the simulated docking station."""
import argparse
import math
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

EXPECTED = 'ROVER_DOCKING_STATION_01'
# The printed code occupies 0.6706 of the 0.36 m panel; rest is white border.
CODE_SIZE_M = 0.36 * (397.0 / 592.0)


def estimate(corners, fx, cx):
    p = corners.reshape(4, 2)
    if not math.isfinite(fx) or fx <= 0:
        raise ValueError('CameraInfo sem distancia focal valida')
    width = (math.dist(p[0], p[1]) + math.dist(p[2], p[3])) / 2
    if width <= 1:
        raise ValueError('QR pequeno demais')
    center = float(p[:, 0].mean())
    return CODE_SIZE_M * fx / width, math.atan2(center-cx, fx)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--topic', default='/rover_argo_N1/Instance1/camera/image_raw')
    p.add_argument('--camera-info', default='/rover_argo_N1/Instance1/camera/camera_info')
    p.add_argument('--headless', action='store_true')
    args = p.parse_args()
    rospy.init_node('dock_qr_observer')
    bridge = CvBridge(); lock = threading.Lock(); state = {'image':None,'info':None,'wall':0}
    def receive_image(msg):
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with lock:
            state.update(image=frame, stamp=msg.header.stamp, wall=time.monotonic())
    def receive_info(msg):
        with lock: state['info']=msg
    subs = [rospy.Subscriber(args.topic, Image, receive_image, queue_size=1, buff_size=32*1024*1024),
            rospy.Subscriber(args.camera_info, CameraInfo, receive_info, queue_size=1)]
    detector=cv2.QRCodeDetector(); report=0; last_stamp=None
    try:
        print('Observacao sem comandos; QR esperado:', EXPECTED, flush=True)
        while not rospy.is_shutdown():
            with lock: frame=state['image']; info=state['info']; wall=state['wall']; stamp=state.get('stamp')
            now=time.monotonic()
            if frame is None or info is None or now-wall > 1 or stamp is None or not -.05 <= (rospy.Time.now()-stamp).to_sec() <= .5:
                if now>=report: print('Aguardando imagem e CameraInfo recentes',flush=True);report=now+2
                time.sleep(.1); continue
            if last_stamp == stamp: time.sleep(.03); continue
            last_stamp=stamp
            text, points, _ = detector.detectAndDecode(frame)
            if text == EXPECTED and points is not None:
                distance, bearing = estimate(points, info.K[0], info.K[2])
                if now>=report:
                    print('QR OK | distancia aproximada=%.2f m | angulo=%.1f graus (direita positivo)'%(distance,math.degrees(bearing)),flush=True);report=now+1
                cv2.polylines(frame,[points.astype('int32').reshape(-1,1,2)],True,(0,255,0),3)
            elif now>=report:
                print('QR da estacao nao identificado',flush=True);report=now+2
            if not args.headless:
                cv2.imshow('QR docking (apenas observacao; Q encerra)',frame)
                if cv2.waitKey(1)&255 in (ord('q'),27): break
            time.sleep(.02)
    except KeyboardInterrupt: pass
    finally:
        for sub in subs: sub.unregister()
        if not args.headless: cv2.destroyAllWindows()

if __name__=='__main__': main()
