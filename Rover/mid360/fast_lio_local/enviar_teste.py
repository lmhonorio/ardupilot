#!/usr/bin/env python3
"""FIFO exclusivo do ensaio LIO: nenhuma substituicao silenciosa de mensagens."""
from collections import deque
import os
import struct
import sys
import threading
import time
import zlib
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu


def main():
    rclpy.init();node=rclpy.create_node('mid360_lio_test_source')
    queue=deque();cv=threading.Condition();stop=threading.Event();failure=[];size=0;seq={1:0,2:0};progress=[time.monotonic()]
    def receive(channel,raw):
        nonlocal size
        with cv:
            if stop.is_set():return
            if size+len(raw)>32*1024*1024 or len(queue)>=4096:
                failure.append('FIFO excedeu limite: teste interrompido, sem descarte silencioso');stop.set();cv.notify_all();return
            seq[channel]+=1;queue.append((time.monotonic(),channel,seq[channel],raw));size+=len(raw);cv.notify()
    def send():
        nonlocal size
        try:
            sys.stdout.buffer.write(b'LIO360v1');sys.stdout.buffer.flush()
            while not stop.is_set():
                with cv:
                    cv.wait_for(lambda:queue or stop.is_set(),timeout=1)
                    if not queue:continue
                    stamp,ch,number,raw=queue.popleft();size-=len(raw)
                if time.monotonic()-stamp>3:raise RuntimeError('Atraso na FIFO maior que 3 s')
                data=zlib.compress(raw,1)
                sys.stdout.buffer.write(struct.pack('!BII',ch,number,len(data))+data);sys.stdout.buffer.flush();progress[0]=time.monotonic()
        except Exception as exc:failure.append(str(exc));stop.set()
    worker=threading.Thread(target=send,daemon=True);worker.start()
    node.create_subscription(PointCloud2,'/mid360/points',lambda m:receive(1,m),QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE),raw=True)
    node.create_subscription(Imu,'/mid360/imu',lambda m:receive(2,m),QoSProfile(depth=2048,reliability=ReliabilityPolicy.RELIABLE),raw=True)
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-progress[0]>8:
                # A disconnected SSH client can leave docker-exec stdout blocked.
                # Exit this source only; never leave a stale subscription running.
                os.write(2,b'Fonte MID360 sem progresso por 8 s: confira driver, sensor e endereco 192.168.1.50 na Pi.\n')
                os._exit(2)
    except KeyboardInterrupt:pass
    finally:
        stop.set()
        with cv:cv.notify_all()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if failure:print(failure[-1],file=sys.stderr);return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
