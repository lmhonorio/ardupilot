#!/usr/bin/env python3
"""Observe ARP/Livox UDP e, opcionalmente, consulte a sub-rede por ARP.

Nao altera IPs nem configuracoes do sensor. Linux; exige CAP_NET_RAW.
Uma resposta ARP identifica um endereco, nao confirma que seja um MID360.
"""
import argparse
import ipaddress
import json
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--scan', help='Sub-rede IPv4 privada /24 para consulta ARP')
    parser.add_argument('--source-ip', help='IPv4 ja existente na interface')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('--seconds deve estar entre 1 e 120')
    network = ipaddress.ip_network(args.scan) if args.scan else None
    if network and (network.version != 4 or network.prefixlen != 24 or not network.is_private or not args.source_ip):
        parser.error('Use sub-rede IPv4 privada /24 e --source-ip')
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
    sock.bind((args.interface, 0))
    sock.settimeout(.2)
    mac = sock.getsockname()[4]
    if network:
        source = socket.inet_aton(args.source_ip)
        for address in network.hosts():
            arp = struct.pack('!HHBBH6s4s6s4s', 1, 0x0800, 6, 4, 1,
                              mac, source, b'\0' * 6, address.packed)
            sock.send(b'\xff' * 6 + mac + b'\x08\x06' + arp)
            time.sleep(.003)
    seen = set()
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if addr[2] == socket.PACKET_OUTGOING or len(data) < 42:
                continue
            event = None
            if data[12:14] == b'\x08\x06':
                op = struct.unpack_from('!H', data, 20)[0]
                ip = socket.inet_ntoa(data[28:32])
                if op == 2 or ip.startswith('192.168.1.'):
                    event = {'kind': 'arp', 'ip': ip,
                             'mac': ':'.join('%02x' % x for x in data[22:28]), 'op': op}
            elif data[12:14] == b'\x08\x00' and data[23] == 17:
                ihl = (data[14] & 15) * 4
                offset = 14 + ihl
                if ihl < 20 or len(data) < offset + 8:
                    continue
                # Nao interpretar fragmentos posteriores como cabecalho UDP.
                if struct.unpack_from('!H', data, 20)[0] & 0x1fff:
                    continue
                src, dst = struct.unpack_from('!HH', data, offset)
                if 56000 <= src <= 56501 or 56000 <= dst <= 56501:
                    event = {'kind': 'livox_udp_candidate', 'ip': socket.inet_ntoa(data[26:30]),
                             'destination': socket.inet_ntoa(data[30:34]), 'source_port': src,
                             'destination_port': dst}
            if event:
                key = json.dumps(event, sort_keys=True)
                if key not in seen:
                    seen.add(key)
                    print(key, flush=True)
    finally:
        sock.close()
    print(json.dumps({'finished': True, 'observations': len(seen)}), flush=True)


if __name__ == '__main__':
    main()
