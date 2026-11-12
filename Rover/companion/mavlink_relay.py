#!/usr/bin/env python3
"""Reuse a local MAVProxy --out while moving its consumers to another host."""
import argparse
import select
import socket
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--remote',default='192.168.0.198');p.add_argument('--listen',type=int,default=14557);p.add_argument('--port',type=int,default=14600)
    a=p.parse_args();peer=(socket.gethostbyname(a.remote),a.port)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('0.0.0.0',a.listen))
    upstream=None;last_upstream=0
    print('MAVProxy local UDP %d <-> %s:%d'%(a.listen,*peer),flush=True)
    try:
        while True:
            if not select.select([sock],[],[],1)[0]:continue
            data,addr=sock.recvfrom(65535)
            if addr[0]=='127.0.0.1':
                upstream=addr;last_upstream=time.monotonic();sock.sendto(data,peer)
            elif addr==peer and upstream and time.monotonic()-last_upstream<3:
                sock.sendto(data,upstream)
    except KeyboardInterrupt:pass
    finally:sock.close()

if __name__=='__main__':main()
