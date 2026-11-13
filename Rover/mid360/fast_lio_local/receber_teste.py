#!/usr/bin/env python3
import argparse,json,os,re,select,struct,subprocess,time,zlib
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu


def main():
    p=argparse.ArgumentParser();p.add_argument('--stats',required=True);a=p.parse_args()
    remote=os.environ.get('ROVER_REMOTE','rover@192.168.0.167')
    if not re.fullmatch(r'[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+',remote):p.error('Destino SSH invalido')
    proc=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',remote,
        'docker exec rover-mid360 /opt/rover/Rover/companion/entrypoint.sh python /mid360/fast_lio_local/enviar_teste.py'],stdout=subprocess.PIPE,bufsize=0)
    rclpy.init();node=rclpy.create_node('mid360_lio_test_receiver')
    pubs={1:node.create_publisher(PointCloud2,'/lio_test/points',QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)),
          2:node.create_publisher(Imu,'/lio_test/imu',QoSProfile(depth=2048,reliability=ReliabilityPolicy.RELIABLE))}
    seq={1:0,2:0};begin=time.monotonic();last=begin
    def read(size):
        data=bytearray();end=time.monotonic()+10
        while len(data)<size:
            if time.monotonic()>end:raise TimeoutError('Sem dados por 10 s; interrompendo LIO')
            if not select.select([proc.stdout],[],[],.2)[0]:continue
            part=os.read(proc.stdout.fileno(),size-len(data))
            if not part:raise EOFError('Fonte do teste encerrou')
            data.extend(part)
        return bytes(data)
    try:
        if read(8)!=b'LIO360v1':raise ValueError('Protocolo inesperado')
        while rclpy.ok():
            ch,number,size=struct.unpack('!BII',read(9))
            if ch not in pubs or number!=seq[ch]+1 or not 0<size<=8*1024*1024:raise ValueError('Lacuna de sequencia/quadro invalido')
            decoder=zlib.decompressobj();raw=decoder.decompress(read(size),16*1024*1024)
            if not decoder.eof or decoder.unconsumed_tail or decoder.unused_data:raise ValueError('Quadro incompleto')
            pubs[ch].publish(raw);seq[ch]=number
            if time.monotonic()-last>=1:
                Path(a.stats).write_text(json.dumps({'clouds':seq[1],'imu':seq[2],'elapsed_s':time.monotonic()-begin,'sequence_errors':0})+'\n');last=time.monotonic()
    except KeyboardInterrupt:pass
    finally:
        proc.terminate()
        try:proc.wait(timeout=4)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        proc.stdout.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
