#!/usr/bin/env python3
"""Recebe CDR via SSH e republica no computador; sem alterar coordenadas/tempo."""
import argparse
import os
import re
import select
import struct
import subprocess
import time
import zlib
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--remote', default=os.environ.get('ROVER_REMOTE', 'rover@192.168.0.167'))
    a = p.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+', a.remote): p.error('Destino SSH invalido')
    cmd = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5',
           '-o', 'ServerAliveCountMax=2', a.remote,
           'docker exec rover-mid360 /opt/rover/Rover/companion/entrypoint.sh python /mid360/enviar.py']
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
    rclpy.init(); node = rclpy.create_node('mid360_ssh_receiver')
    pubs = {1: node.create_publisher(PointCloud2, '/mid360/points', qos_profile_sensor_data),
            2: node.create_publisher(Imu, '/mid360/imu', qos_profile_sensor_data)}
    def read(size):
        result = bytearray(); deadline = time.monotonic()+20
        while len(result)<size:
            if time.monotonic()>deadline: raise TimeoutError('Sem dados recentes do sensor via SSH')
            if not select.select([proc.stdout], [], [], .5)[0]: continue
            data = os.read(proc.stdout.fileno(), size-len(result))
            if not data: raise EOFError('Ponte SSH encerrou')
            result.extend(data)
        return bytes(result)
    try:
        if read(8) != b'MID360v1': raise ValueError('Cabecalho de transporte invalido')
        print('MID360 fisico via SSH: /mid360/points e /mid360/imu; dominio 21.', flush=True)
        while rclpy.ok():
            channel, size = struct.unpack('!BI', read(5))
            if channel not in pubs or not 0<size<=8*1024*1024: raise ValueError('Quadro invalido')
            decoder = zlib.decompressobj(); raw = decoder.decompress(read(size), 16*1024*1024)
            if not decoder.eof or decoder.unconsumed_tail or decoder.unused_data: raise ValueError('Mensagem excede limite ou incompleta')
            pubs[channel].publish(raw)
    except KeyboardInterrupt: pass
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        proc.stdout.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
