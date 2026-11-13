#!/usr/bin/env python3
"""Endereco secundario temporario na Ethernet da Pi; sem trocar rota padrao."""
import argparse
import ipaddress
import json
from pathlib import Path
import socket
import struct
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['up', 'down'])
    p.add_argument('--interface', default='eth0')
    p.add_argument('--address', default='192.168.1.50/24')
    p.add_argument('--state', default='/mid360/run/network.json')
    a = p.parse_args()
    interface = ipaddress.ip_interface(a.address)
    if interface.version != 4 or not interface.ip.is_private or interface.network.prefixlen != 24:
        p.error('Use endereco IPv4 privado /24')
    addresses = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show', 'dev', a.interface]))
    existing = addresses[0]['addr_info']
    present = any(x['local'] == str(interface.ip) and x['prefixlen'] == 24 for x in existing)
    state = Path(a.state)
    record = {'interface': a.interface, 'address': a.address}
    if a.action == 'down':
        if not state.exists() or json.loads(state.read_text()) != record:
            raise SystemExit('Nenhum endereco criado por este script para remover.')
        if present:
            subprocess.run(['ip', 'addr', 'del', a.address, 'dev', a.interface], check=True)
        state.unlink()
        print('Endereco secundario removido; rede original preservada.')
        return
    if present:
        print('Endereco ja presente:', a.address)
        return
    if not existing:
        raise SystemExit('Interface sem IPv4 original; conferir cabeamento/configuracao primeiro.')
    if state.exists() and json.loads(state.read_text()) != record:
        raise SystemExit('Ha outro endereco registrado; remova-o antes de trocar.')
    # ARP duplicate address detection before using the candidate host address.
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806)) as sock:
        sock.bind((a.interface, 0)); sock.settimeout(.2)
        mac = sock.getsockname()[4]
        frame = b'\xff'*6 + mac + b'\x08\x06' + struct.pack(
            '!HHBBH6s4s6s4s', 1, 0x0800, 6, 4, 1, mac, b'\0'*4, b'\0'*6, interface.ip.packed)
        for _ in range(3):
            sock.send(frame)
            end = time.monotonic() + 1
            while time.monotonic() < end:
                try: data, addr = sock.recvfrom(2048)
                except socket.timeout: continue
                if len(data) < 42 or data[22:28] == mac: continue
                if data[28:32] == interface.ip.packed or (data[28:32] == b'\0'*4 and data[38:42] == interface.ip.packed):
                    raise SystemExit('Conflito ARP: endereco em uso; nao foi alterada a rede.')
    subprocess.run(['ip', 'addr', 'add', a.address, 'dev', a.interface], check=True)
    state.parent.mkdir(parents=True, exist_ok=True)
    state.write_text(json.dumps(record, indent=2)+'\n')
    print('Adicionado somente endereco secundario temporario:', a.address, a.interface)


if __name__ == '__main__':
    main()
