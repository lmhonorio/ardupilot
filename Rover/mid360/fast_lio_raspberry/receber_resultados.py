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
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from mapa_pc import MapView
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from tf2_ros import TransformBroadcaster


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--remote', default=os.environ.get('ROVER_REMOTE', 'rover@192.168.0.167'))
    a = p.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+', a.remote): p.error('Destino SSH invalido')
    cmd = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5',
           '-o', 'ServerAliveCountMax=2', a.remote,
           'docker exec rover-mid360-fastlio /opt/rover/Rover/companion/entrypoint.sh python /test/enviar_resultados.py']
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
    rclpy.init(); node = rclpy.create_node('lio_pi_visual_receiver')
    pubs = {1: node.create_publisher(PointCloud2, '/lio_pi/cloud_registered', qos_profile_sensor_data),
            2: node.create_publisher(Odometry, '/lio_pi/odometry', qos_profile_sensor_data),
            3: node.create_publisher(Path, '/lio_pi/path', 5)}
    map_view = MapView(node)
    transforms = TransformBroadcaster(node)
    def read(size):
        result = bytearray(); deadline = time.monotonic()+35
        while len(result)<size:
            if time.monotonic()>deadline: raise TimeoutError('Sem dados recentes do sensor via SSH')
            if not select.select([proc.stdout], [], [], .5)[0]: continue
            data = os.read(proc.stdout.fileno(), size-len(result))
            if not data: raise EOFError('Ponte SSH encerrou')
            result.extend(data)
        return bytes(result)
    try:
        if read(8) != b'LIOPIv01': raise ValueError('Cabecalho de transporte invalido')
        print('Resultados FAST-LIO da Raspberry: /lio_pi/cloud_registered, odometry, path; mapa no PC /lio_pi/map_pc. Dominio 23.', flush=True)
        while rclpy.ok():
            channel, size = struct.unpack('!BI', read(5))
            if channel not in pubs or not 0<size<=8*1024*1024: raise ValueError('Quadro invalido')
            decoder = zlib.decompressobj(); raw = decoder.decompress(read(size), 16*1024*1024)
            if not decoder.eof or decoder.unconsumed_tail or decoder.unused_data: raise ValueError('Mensagem excede limite ou incompleta')
            pubs[channel].publish(raw)
            if channel == 1: map_view.add(raw)
            if channel == 2:
                odom = deserialize_message(raw, Odometry)
                tf = TransformStamped()
                tf.header = odom.header
                tf.child_frame_id = odom.child_frame_id or 'body'
                tf.transform.translation.x = odom.pose.pose.position.x
                tf.transform.translation.y = odom.pose.pose.position.y
                tf.transform.translation.z = odom.pose.pose.position.z
                tf.transform.rotation = odom.pose.pose.orientation
                transforms.sendTransform(tf)
    except KeyboardInterrupt: pass
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        proc.stdout.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
