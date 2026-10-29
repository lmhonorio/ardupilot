#!/usr/bin/env python3.8
"""Read the Classic camera and 3D ray sensor with NumPy/OpenCV, without ROS."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import time

import cv2
import numpy as np


def read_exact(stream, count, deadline):
    data = bytearray()
    while len(data) < count:
        remaining = None if deadline is None else deadline - time.monotonic()
        if remaining is not None and remaining <= 0:
            raise TimeoutError("Sem dados completos; confirme que o mundo esta executando.")
        if not select.select([stream], [], [], remaining)[0]:
            raise TimeoutError("Tempo limite aguardando sensores do Gazebo.")
        chunk = os.read(stream.fileno(), count - len(data))
        if not chunk:
            raise EOFError("Leitor nativo encerrado.")
        data.extend(chunk)
    return bytes(data)


def decode_camera(meta, payload):
    if meta['format'] != 3:  # gazebo::common::Image::RGB_INT8
        raise ValueError("Esperava RGB8 (formato 3), recebido: %s" % meta['format'])
    rows = np.frombuffer(payload, dtype=np.uint8).reshape(meta['height'], meta['step'])
    rgb = rows[:, :meta['width'] * 3].reshape(meta['height'], meta['width'], 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def decode_lidar(meta, payload):
    ranges = np.frombuffer(payload, dtype='<f8').reshape(meta['vertical_count'], meta['count'])
    azimuth = meta['angle_min'] + np.arange(meta['count']) * meta['angle_step']
    elevation = meta['vertical_angle_min'] + np.arange(meta['vertical_count']) * meta['vertical_angle_step']
    valid = np.isfinite(ranges) & (ranges >= meta['range_min']) & (ranges < meta['range_max'])
    with np.errstate(invalid='ignore'):
        xyz = np.stack(np.broadcast_arrays(
            ranges * np.cos(elevation[:, None]) * np.cos(azimuth),
            ranges * np.cos(elevation[:, None]) * np.sin(azimuth),
            ranges * np.sin(elevation[:, None])), axis=-1)
    return ranges, xyz[valid].astype(np.float32)


def decode_qr(detector, image):
    value, corners, _ = detector.detectAndDecode(image)
    # At the initial pose the marker is small. Retry its detected region
    # with a white margin and nearest-neighbor enlargement.
    if not value and corners is not None:
        x, y, width, height = cv2.boundingRect(corners.astype(np.float32))
        margin = max(10, width // 4)
        crop = image[max(0, y - margin):y + height + margin,
                     max(0, x - margin):x + width + margin]
        enlarged = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
        value, _, _ = detector.detectAndDecode(enlarged)
    return value, corners


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', required=True)
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parents[2] / 'gazebo_simple/run/sensors')
    parser.add_argument('--no-gui', action='store_true', help='Salvar dados sem abrir janela OpenCV')
    parser.add_argument('--once', action='store_true', help='Salvar uma imagem e uma varredura e sair')
    parser.add_argument('--timeout', type=float, default=30, help='Limite total em segundos; 0 = ilimitado')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    detector = cv2.QRCodeDetector()
    proc = subprocess.Popen([args.bridge], stdout=subprocess.PIPE, bufsize=0)
    deadline = time.monotonic() + args.timeout if args.timeout else None
    seen, last_saved = set(), {}
    print('Aguardando camera e LiDAR. Saida:', args.output_dir, flush=True)
    try:
        while True:
            magic, meta_len, payload_len = struct.unpack('<4sII', read_exact(proc.stdout, 12, deadline))
            if magic != b'GZS1' or meta_len > 65536 or payload_len > 16 * 1024 * 1024:
                raise ValueError('Cabecalho binario invalido do leitor nativo.')
            meta = json.loads(read_exact(proc.stdout, meta_len, deadline))
            payload = read_exact(proc.stdout, payload_len, deadline)
            kind = meta['type']
            # Save at most once per wall-clock second per sensor; frames are independent.
            now = time.monotonic()
            if now - last_saved.get(kind, 0) < 1:
                continue
            last_saved[kind] = now
            stamp = meta['sec'] + meta['nsec'] * 1e-9
            if kind == 'camera':
                image = decode_camera(meta, payload)
                value, corners = decode_qr(detector, image)
                if not cv2.imwrite(str(args.output_dir / 'camera.png'), image):
                    raise OSError('Nao foi possivel salvar camera.png')
                print('Camera: %dx%d, t=%.3f s, QR=%s' %
                      (meta['width'], meta['height'], stamp, value or '(nao detectado)'), flush=True)
                if not args.no_gui:
                    if corners is not None:
                        cv2.polylines(image, [corners.astype(np.int32)], True, (0, 255, 0), 2)
                    cv2.imshow('Camera Gazebo', image)
                    if cv2.waitKey(1) & 255 == ord('q'):
                        break
            elif kind == 'lidar':
                ranges, xyz = decode_lidar(meta, payload)
                np.savez_compressed(args.output_dir / 'lidar.npz', ranges=ranges, xyz=xyz,
                                    metadata=json.dumps(meta))
                minimum = np.linalg.norm(xyz, axis=1).min() if len(xyz) else math.nan
                print('LiDAR: %s, %d retornos validos, menor distancia=%.2f m, t=%.3f s' %
                      (ranges.shape, len(xyz), minimum, stamp), flush=True)
            (args.output_dir / (kind + '.json')).write_text(json.dumps(meta, indent=2))
            seen.add(kind)
            if args.once and seen >= {'camera', 'lidar'}:
                break
    except KeyboardInterrupt:
        pass
    except (TimeoutError, EOFError, ValueError) as exc:
        parser.exit(1, str(exc) + '\n')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        proc.stdout.close()
        if not args.no_gui:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
