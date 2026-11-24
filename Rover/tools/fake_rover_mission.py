#!/usr/bin/env python3
"""Rover MAVLink minimo para testar missao do mode_panel sem hardware.

Escuta UDP (padrao 14580), manda heartbeat + GPS para o GCS, e responde
MISSION_REQUEST_LIST / REQUEST_INT / COUNT / ITEM_INT / ACK.

  py -3 fake_rover_mission.py
  py -3 fake_rover_mission.py --gcs 127.0.0.1:14550 --bind 0.0.0.0:14580
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

import esc_monitor as em
import mission_core as mission

SYSID = 1
COMPID = 1
MAV_TYPE_GROUND_ROVER = 10
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_STATE_ACTIVE = 4
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

# Seed mission: home + two waypoints (1 m north, then 2 m east of that).
LAT0 = -35.363261
LON0 = 149.165230
HDG0 = 45.0


def offset(lat, lon, bearing, dist):
    R = 6371000.0
    brng = math.radians(bearing % 360)
    p1 = math.radians(lat)
    l1 = math.radians(lon)
    d = dist / R
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(brng))
    l2 = l1 + math.atan2(
        math.sin(brng) * math.sin(d) * math.cos(p1),
        math.cos(d) - math.sin(p1) * math.sin(p2),
    )
    return math.degrees(p2), ((math.degrees(l2) + 540) % 360) - 180


def seed_mission():
    a = offset(LAT0, LON0, 0, 1.0)
    b = offset(a[0], a[1], 90, 2.0)
    items = [
        mission.make_item(16, LAT0, LON0, z=0.0, current=1, frame=0),
        mission.make_item(16, a[0], a[1], z=-1.0),
        mission.make_item(16, b[0], b[1], z=90.0),
    ]
    return mission.renumber(items)


class RoverEncoder:
    def __init__(self):
        self.seq = 0
        mission.register_mission_messages(em.CRC_EXTRA)

    def pack(self, msgid, payload, sysid=SYSID, compid=COMPID):
        header = bytes([len(payload), self.seq & 0xFF, sysid, compid, msgid])
        self.seq = (self.seq + 1) & 0xFF
        crc = em.x25_crc(header + payload + bytes([em.CRC_EXTRA[msgid]]))
        return b"\xfe" + header + payload + struct.pack("<H", crc)

    def heartbeat(self, mode=10, armed=False):
        base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (128 if armed else 0)
        return self.pack(em.MSG_HEARTBEAT, struct.pack(
            "<IBBBBB", int(mode), MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA,
            base, MAV_STATE_ACTIVE, 3))

    def global_position(self, lat, lon, hdg, t_ms=0):
        payload = struct.pack(
            "<IiiiihhhH",
            int(t_ms) & 0xFFFFFFFF,
            int(round(lat * 1e7)),
            int(round(lon * 1e7)),
            0, 0, 0, 0, 0,
            int(round(hdg * 100)) & 0xFFFF,
        )
        return self.pack(em.MSG_GLOBAL_POSITION_INT, payload)

    def mission_count(self, n, target_sys, target_comp):
        return self.pack(mission.MSG_MISSION_COUNT, struct.pack("<HBB", n, target_sys, target_comp))

    def mission_item(self, item, target_sys, target_comp):
        return self.pack(mission.MSG_MISSION_ITEM_INT, item.payload(target_sys, target_comp))

    def mission_request_int(self, seq, target_sys, target_comp):
        return self.pack(mission.MSG_MISSION_REQUEST_INT, struct.pack("<HBB", seq, target_sys, target_comp))

    def mission_ack(self, target_sys, target_comp, result=0):
        return self.pack(mission.MSG_MISSION_ACK, struct.pack("<BBB", target_sys, target_comp, result))

    def command_ack(self, command, result=0):
        return self.pack(em.MSG_COMMAND_ACK, struct.pack("<HB", int(command), int(result)))


def parse_hostport(text):
    host, port = text.rsplit(":", 1)
    return host, int(port)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Fake rover for mission tests")
    ap.add_argument("--bind", default="0.0.0.0:14580")
    ap.add_argument("--gcs", default="127.0.0.1:14550",
                    help="onde o mode_panel escuta")
    ap.add_argument("--lat", type=float, default=LAT0)
    ap.add_argument("--lon", type=float, default=LON0)
    ap.add_argument("--hdg", type=float, default=HDG0)
    args = ap.parse_args(argv)

    bind_host, bind_port = parse_hostport(args.bind)
    gcs_host, gcs_port = parse_hostport(args.gcs)
    gcs = (gcs_host, gcs_port)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_host, bind_port))
    sock.settimeout(0.2)

    enc = RoverEncoder()
    parser = em.MavlinkParser()
    items = seed_mission()
    # Upload state
    upload_n = None
    upload_got = {}
    peer = gcs
    last_hb = 0.0
    t0 = time.monotonic()

    print("fake rover em %s -> gcs %s  (%d itens iniciais)" % (args.bind, args.gcs, len(items)), flush=True)
    print("Ctrl+C encerra.", flush=True)

    try:
        while True:
            now = time.monotonic()
            if now - last_hb >= 1.0:
                last_hb = now
                t_ms = int((now - t0) * 1000)
                sock.sendto(enc.heartbeat(), peer)
                sock.sendto(enc.global_position(args.lat, args.lon, args.hdg, t_ms), peer)

            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows: ICMP port unreachable when GCS is not up yet.
                continue
            peer = addr
            for msgid, sysid, compid, payload in parser.feed(data):
                # Ignore our own echoes if any
                if sysid == SYSID:
                    continue
                target_sys, target_comp = sysid, compid

                if msgid == mission.MSG_MISSION_REQUEST_LIST:
                    sock.sendto(enc.mission_count(len(items), target_sys, target_comp), peer)
                    print("MISSION_COUNT %d -> %s" % (len(items), peer), flush=True)

                elif msgid == mission.MSG_MISSION_REQUEST_INT:
                    seq = struct.unpack("<H", payload[:2])[0]
                    if 0 <= seq < len(items):
                        sock.sendto(enc.mission_item(items[seq], target_sys, target_comp), peer)

                elif msgid == mission.MSG_MISSION_REQUEST:
                    seq = struct.unpack("<H", payload[:2])[0]
                    if 0 <= seq < len(items):
                        sock.sendto(enc.mission_item(items[seq], target_sys, target_comp), peer)

                elif msgid == mission.MSG_MISSION_COUNT:
                    upload_n = struct.unpack("<H", payload[:2])[0]
                    upload_got = {}
                    print("GCS quer enviar %d itens" % upload_n, flush=True)
                    if upload_n == 0:
                        items = []
                        sock.sendto(enc.mission_ack(target_sys, target_comp, 0), peer)
                        upload_n = None
                    else:
                        sock.sendto(enc.mission_request_int(0, target_sys, target_comp), peer)

                elif msgid == mission.MSG_MISSION_ITEM_INT and upload_n is not None:
                    it = mission.Item.from_payload(payload)
                    upload_got[it.seq] = it
                    print("recebeu item %d/%d" % (it.seq + 1, upload_n), flush=True)
                    if len(upload_got) >= upload_n:
                        items = [upload_got[i] for i in range(upload_n)]
                        mission.renumber(items)
                        sock.sendto(enc.mission_ack(target_sys, target_comp, 0), peer)
                        print("missão aceita (%d itens)" % len(items), flush=True)
                        upload_n = None
                        upload_got = {}
                    else:
                        nxt = min(i for i in range(upload_n) if i not in upload_got)
                        sock.sendto(enc.mission_request_int(nxt, target_sys, target_comp), peer)

                elif msgid == mission.MSG_MISSION_ACK:
                    pass

                elif msgid == em.MSG_COMMAND_LONG:
                    cmd = struct.unpack("<7fHBBB", payload[:33])[7]
                    sock.sendto(enc.command_ack(cmd, 0), peer)

    except KeyboardInterrupt:
        print("\nfim. missão final: %d itens" % len(items), flush=True)
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
