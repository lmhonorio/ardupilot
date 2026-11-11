#!/usr/bin/env python3
"""Read-only QR detection for the simulated docking station."""
import argparse
import math
from pathlib import Path
import signal
import sys
import time

import cv2
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
from common.qr_detector import QRDetector
from common.docking_camera import DockingCamera, add_camera_arguments
from common.docking_stations import DEFAULT_STATIONS, load_stations, select_station

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
    p.add_argument('--topic', help='Sobrescreve o topico de imagem do namespace')
    p.add_argument('--camera-info', help='Sobrescreve o topico CameraInfo do namespace')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--estacoes',default=str(DEFAULT_STATIONS))
    p.add_argument('--estacao',help='Observar apenas uma estacao cadastrada')
    add_camera_arguments(p, diagnostic=False)
    args = p.parse_args()
    try:
        stations=load_stations(args.estacoes)
        if args.estacao:stations=[select_station(stations,args.estacao)]
    except (OSError,ValueError) as exc:p.error(str(exc))
    expected={s['qr']:s['nome'] for s in stations}
    signal.signal(signal.SIGTERM, lambda sig, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    ros.init_node('dock_qr_observer')
    camera = DockingCamera(args.namespace, args.topic, args.camera_info)
    detector=QRDetector(); report=0; last_stamp=None
    try:
        print('Observacao sem comandos; QR cadastrados:', expected, flush=True)
        while not ros.is_shutdown():
            now=time.monotonic()
            try:
                frame, info, stamp = camera.snapshot()
            except RuntimeError as exc:
                if now>=report: print(str(exc),flush=True);report=now+2
                time.sleep(.1); continue
            if last_stamp == stamp: time.sleep(.03); continue
            last_stamp=stamp
            text, points, _ = detector.detectAndDecode(frame)
            if text in expected and points is not None:
                distance, bearing = estimate(points, info.k[0], info.k[2])
                if now>=report:
                    print('QR OK | estacao=%s | distancia aproximada=%.2f m | angulo=%.1f graus (direita positivo)'%(expected[text],distance,math.degrees(bearing)),flush=True);report=now+1
                cv2.polylines(frame,[points.astype('int32').reshape(-1,1,2)],True,(0,255,0),3)
            elif now>=report:
                print(('Codigo lido nao cadastrado: '+text) if text else detector.status,flush=True);report=now+2
            if not args.headless:
                cv2.imshow('QR docking (apenas observacao; Q encerra)',frame)
                if cv2.waitKey(1)&255 in (ord('q'),27): break
            time.sleep(.02)
    except KeyboardInterrupt: pass
    finally:
        camera.close()
        ros.signal_shutdown('observador encerrado')
        if not args.headless: cv2.destroyAllWindows()

if __name__=='__main__': main()
