#!/usr/bin/env python3
"""
Monitor ao vivo do rover pelo HereLink: motores CAN (driver CAN do rover.lua) + navegacao.

Grava em CSV (formato longo: hora_pc, t_s, boot_ms, nome, valor, texto):
  ESCn_I / ESCn_ICMD / ESCn_RPM, ESC_TGT_L / ESC_TGT_R   (modules/can_report.lua, ESC_GCS_HZ > 0)
  STATUSTEXT                                              mensagens da controladora
  MODE (valor = numero, texto = nome) e ARMED             mudancas de modo e arme
  ATT_*   atitude: roll, pitch, yaw (graus), yawrate (graus/s)
  GPI_*   posicao global: lat, lon, vn, ve (m/s), hdg (graus)
  NAV_*   NAV_CONTROLLER_OUTPUT: xtrack (m), nav_brg, tgt_brg (graus), wp_dist (m)
  VFR_*   gspd (m/s), hdg (graus), thr (%)
  GPS_*   fix, sats, eph (cm), vel (m/s), cog (graus)
  BAT_V / BAT_I  tensao (V) e corrente (A) da bateria que o ArduPilot le (BMS)
  ESCn_T  temperatura do motor (graus C, 1 Hz)
  SRV_n   SERVO_OUTPUT_RAW (us), RC_n RC_CHANNELS (us), WP_SEQ (missao atual)
  PIDS_* / PIDV_*   PID_TUNING de direcao (graus/s) e velocidade (m/s):
                    des, ach, ff, p, i, d  (precisa GCS_PID_MASK = 3)
  PARAM:<nome>      parametros: copia completa no inicio + mudancas durante o log
                    (por exemplo os ganhos que o QuickTune grava)

Tambem salva <csv>.parm com todos os parametros (formato do Mission Planner).
Analise do log (mesmo arquivo):
  python esc_monitor.py analyze esc_log_....csv            # relatorio por modo
  python esc_monitor.py analyze esc_log_....csv --plot     # + PNG ao lado do CSV
  python esc_monitor.py analyze esc_log_....csv --from 120 --to 300   # so um trecho (t_s)
O relatorio aponta zigue-zague (periodo/amplitude do xtrack), atraso e ganho do yaw
rate, termos do PID de direcao, saturacao e atraso das rodas, e os parametros
alterados durante o log (ganhos do QuickTune).

O MAVLink e decodificado aqui mesmo (sem pymavlink); o grafico usa matplotlib.

Por padrao escuta UDP 14550 e, do mesmo socket, manda heartbeat para o
HereLink em 192.168.43.1:14552: o controle so transmite para um cliente que
ja falou com ele (--kick none desliga, --kick HOST:PORTA muda o destino).
O monitor pede as mensagens acima na taxa --rate (MAV_CMD_SET_MESSAGE_INTERVAL);
isso vale so ate a placa reiniciar e nao grava parametro.

Exemplos:
  python esc_monitor.py                              # escuta UDP 14550 e chama o HereLink
  python esc_monitor.py --no-plot                    # so console + CSV
  python esc_monitor.py --rate 20                    # navegacao a 20 Hz
  python esc_monitor.py --no-params                  # nao baixa os parametros
  python esc_monitor.py --conn udp:0.0.0.0:14551     # porta encaminhada pelo Mission Planner
"""

import re
import argparse
import collections
import csv
import datetime
import math
import socket
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Minimal MAVLink v1/v2 decoder
# ---------------------------------------------------------------------------
MSG_HEARTBEAT = 0
MSG_SYS_STATUS = 1
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_REQUEST_LIST = 21
MSG_PARAM_VALUE = 22
MSG_GPS_RAW_INT = 24
MSG_ATTITUDE = 30
MSG_GLOBAL_POSITION_INT = 33
MSG_SERVO_OUTPUT_RAW = 36
MSG_MISSION_CURRENT = 42
MSG_NAV_CONTROLLER_OUTPUT = 62
MSG_RC_CHANNELS = 65
MSG_VFR_HUD = 74
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_PID_TUNING = 194
MSG_NAMED_VALUE_FLOAT = 251
MSG_STATUSTEXT = 253
# "Lua: Time: <us> Mem: <total> + <run>" (SCR_DEBUG_OPTS bit 1)
LUA_MEM_RE = re.compile(r"^Lua: Time: (\d+) Mem: (-?\d+) \+ (-?\d+)")
MSG_FILE_TRANSFER_PROTOCOL = 110

# msgid: (crc_extra, full base payload length)
MSG_INFO = {
    MSG_HEARTBEAT: (50, 9),
    MSG_SYS_STATUS: (124, 31),
    MSG_PARAM_REQUEST_READ: (214, 20),
    MSG_PARAM_REQUEST_LIST: (159, 2),
    MSG_PARAM_VALUE: (220, 25),
    MSG_GPS_RAW_INT: (24, 30),
    MSG_ATTITUDE: (39, 28),
    MSG_GLOBAL_POSITION_INT: (104, 28),
    MSG_SERVO_OUTPUT_RAW: (222, 21),
    MSG_MISSION_CURRENT: (28, 2),
    MSG_NAV_CONTROLLER_OUTPUT: (183, 26),
    MSG_RC_CHANNELS: (118, 42),
    MSG_VFR_HUD: (20, 20),
    MSG_COMMAND_LONG: (152, 33),
    MSG_COMMAND_ACK: (143, 3),
    MSG_PID_TUNING: (98, 25),
    MSG_NAMED_VALUE_FLOAT: (170, 18),
    MSG_STATUSTEXT: (83, 51),
    # MAVFtp. The payload is 254 bytes; MAVLink 2 drops the trailing zeros.
    MSG_FILE_TRANSFER_PROTOCOL: (84, 254),
}
CRC_EXTRA = {k: v[0] for k, v in MSG_INFO.items()}
# MAVLink 2 trims trailing zero bytes, so pad back to the base length
PAYLOAD_LEN = {k: v[1] for k, v in MSG_INFO.items()}

# Streams asked from the vehicle: msgid -> rate factor (1 = --rate)
STREAMS = {
    MSG_ATTITUDE: 1.0,
    MSG_SYS_STATUS: 0.2,
    MSG_GLOBAL_POSITION_INT: 1.0,
    MSG_NAV_CONTROLLER_OUTPUT: 1.0,
    MSG_PID_TUNING: 1.0,
    MSG_VFR_HUD: 0.5,
    MSG_SERVO_OUTPUT_RAW: 1.0,
    MSG_RC_CHANNELS: 0.5,
    MSG_GPS_RAW_INT: 0.5,
    MSG_MISSION_CURRENT: 0.2,
}
MAV_CMD_SET_MESSAGE_INTERVAL = 511

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_MODE_FLAG_SAFETY_ARMED = 0x80

GCS_SYSID = 254   # not 255, so it does not clash with Mission Planner
GCS_COMPID = 190

ROVER_MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW",
               7: "SIMPLE", 8: "DOCK", 9: "CIRCLE", 10: "AUTO", 11: "RTL", 12: "SMART_RTL",
               15: "GUIDED", 16: "INITIALISING"}
PID_AXIS = {5: "PIDS", 4: "PIDV", 2: "PIDP"}   # MAVLink PID_TUNING_STEER = 5, ACCZ = 4 (speed), PITCH = 2


def x25_crc(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavlinkParser:
    """Feed raw bytes, get (msgid, sysid, compid, payload) for known messages."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf.extend(data)
        out = []
        buf = self.buf
        while True:
            # resync on the next start byte
            i = 0
            while i < len(buf) and buf[i] not in (0xFE, 0xFD):
                i += 1
            if i:
                del buf[:i]
            if len(buf) < 2:
                break
            if buf[0] == 0xFE:
                hdr_len, length = 6, buf[1]
                total = hdr_len + length + 2
                if len(buf) < total:
                    break
                sysid, compid, msgid = buf[3], buf[4], buf[5]
            else:
                if len(buf) < 10:
                    break
                hdr_len, length = 10, buf[1]
                signed = buf[2] & 0x01
                total = hdr_len + length + 2 + (13 if signed else 0)
                if len(buf) < total:
                    break
                sysid, compid = buf[5], buf[6]
                msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            extra = CRC_EXTRA.get(msgid)
            if extra is None:
                # Not a message we decode, so its CRC cannot be checked. Skip it
                # whole only when a new frame starts right after it (or the data
                # ends there); otherwise this was a false start byte, so drop
                # one byte and resync without swallowing the next real frame.
                if len(buf) == total or buf[total] in (0xFE, 0xFD):
                    del buf[:total]
                else:
                    del buf[:1]
                continue
            crc = x25_crc(bytes(buf[1:hdr_len + length]) + bytes([extra]))
            got = buf[hdr_len + length] | (buf[hdr_len + length + 1] << 8)
            if crc != got:
                self.bad_crc += 1
                del buf[:1]
                continue
            payload = bytes(buf[hdr_len:hdr_len + length])
            payload = payload.ljust(PAYLOAD_LEN[msgid], b"\0")
            del buf[:total]
            out.append((msgid, sysid, compid, payload))
        return out


class Encoder:
    """MAVLink v1 frames from the monitor's GCS id."""

    def __init__(self):
        self.seq = 0

    def pack(self, msgid, payload):
        header = bytes([len(payload), self.seq & 0xFF, GCS_SYSID, GCS_COMPID, msgid])
        self.seq += 1
        crc = x25_crc(header + payload + bytes([CRC_EXTRA[msgid]]))
        return b"\xfe" + header + payload + struct.pack("<H", crc)

    def heartbeat(self):
        return self.pack(MSG_HEARTBEAT, struct.pack("<IBBBBB", 0, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 3))

    def set_interval(self, sysid, compid, msgid, hz):
        interval_us = int(1e6 / hz) if hz > 0 else -1
        payload = struct.pack("<7fHBBB", float(msgid), float(interval_us), 0, 0, 0, 0, 0,
                              MAV_CMD_SET_MESSAGE_INTERVAL, sysid, compid, 0)
        return self.pack(MSG_COMMAND_LONG, payload)

    def param_list(self, sysid, compid):
        return self.pack(MSG_PARAM_REQUEST_LIST, struct.pack("<BB", sysid, compid))

    def param_read_index(self, sysid, compid, index):
        return self.pack(MSG_PARAM_REQUEST_READ, struct.pack("<hBB16s", index, sysid, compid, b""))

    def param_read_name(self, sysid, compid, name):
        return self.pack(MSG_PARAM_REQUEST_READ, struct.pack("<hBB16s", -1, sysid, compid, name.encode()))


def decode(msgid, p):
    """Return a dict of named fields for one message (None when not used)."""
    if msgid == MSG_NAMED_VALUE_FLOAT:
        boot_ms, value, name = struct.unpack("<If10s", p[:18])
        return {"boot_ms": boot_ms, "value": value,
                "name": name.split(b"\0", 1)[0].decode("ascii", "replace")}
    if msgid == MSG_COMMAND_ACK:
        command, result = struct.unpack("<HB", p[:3])
        return {"command": command, "result": result}
    if msgid == MSG_STATUSTEXT:
        return {"severity": p[0],
                # ArduPilot text is UTF-8 (e.g. the degree sign); a 50-byte chunk can
                # cut a character in half, so undecodable bytes are replaced
                "text": p[1:51].split(b"\0", 1)[0].decode("utf-8", "replace")}
    if msgid == MSG_HEARTBEAT:
        custom_mode, mav_type, autopilot, base_mode, _, _ = struct.unpack("<IBBBBB", p[:9])
        return {"type": mav_type, "autopilot": autopilot, "mode": custom_mode,
                "armed": bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)}
    if msgid == MSG_PARAM_VALUE:
        value, count, index, pid, ptype = struct.unpack("<fHH16sB", p[:25])
        return {"value": value, "count": count, "index": index, "type": ptype,
                "name": pid.split(b"\0", 1)[0].decode("ascii", "replace")}
    if msgid == MSG_SYS_STATUS:
        f = struct.unpack("<IIIHHhHHHHHHb", p[:31])
        vals = []
        if f[4] not in (0, 65535):
            vals.append(("BAT_V", f[4] / 1000.0))
        if f[5] != -1:
            vals.append(("BAT_I", f[5] / 100.0))   # battery monitor (BMS) current, A
        return {"values": vals} if vals else None
    if msgid == MSG_ATTITUDE:
        t, roll, pitch, yaw, _rs, _ps, ys = struct.unpack("<I6f", p[:28])
        return {"boot_ms": t, "values": (("ATT_roll", math.degrees(roll)), ("ATT_pitch", math.degrees(pitch)),
                                         ("ATT_yaw", math.degrees(yaw) % 360.0),
                                         ("ATT_yawrate", math.degrees(ys)))}
    if msgid == MSG_GLOBAL_POSITION_INT:
        t, lat, lon, _alt, _ralt, vx, vy, _vz, hdg = struct.unpack("<IiiiihhhH", p[:28])
        vals = [("GPI_lat", lat * 1e-7), ("GPI_lon", lon * 1e-7), ("GPI_vn", vx / 100.0), ("GPI_ve", vy / 100.0)]
        if hdg != 65535:
            vals.append(("GPI_hdg", hdg / 100.0))
        return {"boot_ms": t, "values": vals}
    if msgid == MSG_NAV_CONTROLLER_OUTPUT:
        _nr, _np, _ae, _se, xtrack, nav_brg, tgt_brg, wp_dist = struct.unpack("<5fhhH", p[:26])
        return {"values": (("NAV_xtrack", xtrack), ("NAV_nav_brg", nav_brg % 360), ("NAV_tgt_brg", tgt_brg % 360),
                           ("NAV_wp_dist", wp_dist))}
    if msgid == MSG_VFR_HUD:
        _as, gs, _alt, _cl, hdg, thr = struct.unpack("<4fhH", p[:20])
        return {"values": (("VFR_gspd", gs), ("VFR_hdg", hdg), ("VFR_thr", thr))}
    if msgid == MSG_GPS_RAW_INT:
        _t, _lat, _lon, _alt, eph, _epv, vel, cog, fix, sats = struct.unpack("<QiiiHHHHBB", p[:30])
        vals = [("GPS_fix", fix), ("GPS_sats", sats), ("GPS_eph", eph)]
        if vel != 65535:
            vals.append(("GPS_vel", vel / 100.0))
        if cog != 65535:
            vals.append(("GPS_cog", cog / 100.0))
        return {"values": vals}
    if msgid == MSG_SERVO_OUTPUT_RAW:
        f = struct.unpack("<I8HB", p[:21])
        if f[9] != 0:   # only the first 8 outputs
            return None
        return {"values": tuple(("SRV_%d" % (i + 1), f[1 + i]) for i in range(8) if f[1 + i])}
    if msgid == MSG_RC_CHANNELS:
        f = struct.unpack("<I18HBB", p[:42])
        return {"boot_ms": f[0], "values": tuple(("RC_%d" % (i + 1), f[1 + i]) for i in range(8) if f[1 + i])}
    if msgid == MSG_MISSION_CURRENT:
        return {"values": (("WP_SEQ", struct.unpack("<H", p[:2])[0]),)}
    if msgid == MSG_PID_TUNING:
        des, ach, ff, kp, ki, kd, axis = struct.unpack("<6fB", p[:25])
        pre = PID_AXIS.get(axis)
        if pre is None:
            return None
        return {"values": ((pre + "_des", des), (pre + "_ach", ach), (pre + "_ff", ff),
                           (pre + "_p", kp), (pre + "_i", ki), (pre + "_d", kd))}
    return None


def fmt_value(name, value):
    if name in ("GPI_lat", "GPI_lon"):
        return "%.7f" % value
    if isinstance(value, int):
        return str(value)
    return "%.4f" % value


# ---------------------------------------------------------------------------
# Transport: UDP listen, UDP out, or TCP client
# ---------------------------------------------------------------------------
HERELINK_KICK = "192.168.43.1:14552"


class Link:
    def __init__(self, conn, kick=HERELINK_KICK):
        self.kick = None
        if kick and kick.lower() != "none" and conn.startswith("udp:"):
            khost, _, kport = kick.rpartition(":")
            if kport.isdigit():
                self.kick = (khost, int(kport))
        kind, _, rest = conn.partition(":")
        host, _, port = rest.rpartition(":")
        if not port.isdigit():
            raise ValueError("conexao invalida: %s (use udp:HOST:PORTA, udpout:HOST:PORTA ou tcp:HOST:PORTA)" % conn)
        self.kind = kind
        self.addr = (host or "0.0.0.0", int(port))
        self.peer = None
        if kind == "udp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # No SO_REUSEADDR: on Windows it lets this socket share a port that
            # Mission Planner already uses and silently take its packets.
            # Failing here is clearer.
            try:
                self.sock.bind(self.addr)
            except OSError as exc:
                raise OSError("%s; a porta %d ja esta em uso (Mission Planner? mav_param.py?). Encaminhe pelo MP "
                              "(Ctrl+F > MAVLink > UDP Outbound) para outra porta e use --conn udp:0.0.0.0:<porta>"
                              % (exc, self.addr[1]))
        elif kind == "udpout":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.peer = self.addr
        elif kind == "tcp":
            self.sock = socket.create_connection(self.addr, timeout=5)
        else:
            raise ValueError("tipo de conexao desconhecido: %s" % kind)
        self.sock.settimeout(0.1)

    def recv(self):
        try:
            if self.kind == "tcp":
                data = self.sock.recv(8192)
                if not data:
                    raise ConnectionError("conexao TCP fechada")
                return data
            data, addr = self.sock.recvfrom(8192)
            if self.kind == "udp":
                self.peer = addr   # answer the sender so it keeps streaming to us
            return data
        except socket.timeout:
            return b""

    def send(self, data):
        try:
            if self.kind == "tcp":
                self.sock.sendall(data)
                return
            if self.peer is not None:
                self.sock.sendto(data, self.peer)
            # Also call the HereLink MAVLink port: it only streams to a client
            # it has heard from (sent from this socket, so replies come back here)
            if self.kick is not None and self.kick != self.peer:
                self.sock.sendto(data, self.kick)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Data store shared between the reader thread and the plot
# ---------------------------------------------------------------------------
class Store:
    def __init__(self, window_s):
        self.window_s = window_s
        self.lock = threading.Lock()
        self.series = collections.defaultdict(collections.deque)
        self.events = collections.deque(maxlen=200)   # (t, severity, text)
        self.armed = None
        self.mode = None
        self.last_rx = None      # last ESC value
        self.last_any = None     # last vehicle message
        self.latest = {}         # name -> last value (any message)
        self.param_status = ""
        self.t0 = time.monotonic()

    def now(self):
        return time.monotonic() - self.t0

    def add_value(self, name, value, esc=False):
        t = self.now()
        with self.lock:
            self.latest[name] = value
            d = self.series[name]
            d.append((t, value))
            while d and d[0][0] < t - self.window_s:
                d.popleft()
            if esc:
                self.last_rx = t

    def add_event(self, severity, text):
        with self.lock:
            self.events.append((self.now(), severity, text))

    def snapshot(self):
        with self.lock:
            return ({k: list(v) for k, v in self.series.items()},
                    list(self.events), self.armed, self.last_rx)


SEVERITY = ["EMERG", "ALERT", "CRIT", "ERRO", "AVISO", "NOTICE", "INFO", "DEBUG"]


class ParamFetcher:
    """Download the whole parameter list once, then report changes."""

    def __init__(self, enabled):
        self.enabled = enabled
        self.values = {}       # name -> value
        self.by_index = {}     # index -> name
        self.count = None
        self.started = None
        self.last_rx = 0.0
        self.last_req = 0.0
        self.done = not enabled
        self.rounds = 0

    def missing(self):
        if self.count is None:
            return []
        return [i for i in range(self.count) if i not in self.by_index]

    def poll(self, now, link, enc, sysid, compid, store):
        """Send what is needed; returns True when the list just completed."""
        if self.done or sysid is None:
            return False
        if self.started is None:
            self.started = self.last_rx = now
            link.send(enc.param_list(sysid, compid))
            store.param_status = "baixando parametros..."
            print("Baixando parametros...")
            return False
        if self.count is not None and not self.missing():
            self.done = True
            return True
        # list stream stalled: ask the gaps by index, a few per call
        if now - self.last_rx > 2.0 and now - self.last_req > 0.5:
            self.last_req = now
            if self.count is None:
                if now - self.started > 10.0:
                    link.send(enc.param_list(sysid, compid))
                    self.started = now
                    self.rounds += 1
            else:
                for idx in self.missing()[:10]:
                    link.send(enc.param_read_index(sysid, compid, idx))
                self.rounds += 1
            if self.rounds > 120:
                print("Parametros incompletos: %d de %s" % (len(self.by_index), self.count))
                self.done = True
                return True
        return False


def write_parm(path, values):
    with open(path, "w", encoding="utf-8") as f:
        for name in sorted(values):
            f.write("%s,%.7g\n" % (name, values[name]))


def reader(link, store, csv_writer, csv_file, stop, args, parm_path):
    parser = MavlinkParser()
    enc = Encoder()
    last_hb = 0.0
    last_streams = -100.0
    vehicle_sys = None
    vehicle_comp = 1
    last_value_ms = {}   # name -> boot_ms of the last value kept
    recent = {}          # (msgid, payload) -> monotonic time seen (HereLink duplicates)
    params = ParamFetcher(not args.no_params)
    store.params = params   # main() saves a partial list if the monitor is closed early
    acks = set()
    while not stop.is_set():
        now = time.monotonic()
        if now - last_hb >= 1.0:
            last_hb = now
            link.send(enc.heartbeat())
        if vehicle_sys is not None and args.rate > 0 and now - last_streams >= 15.0:
            # repeated: survives a board reboot and a lost uplink packet
            last_streams = now
            for msgid, factor in STREAMS.items():
                link.send(enc.set_interval(vehicle_sys, vehicle_comp, msgid, args.rate * factor))
        if params.poll(now, link, enc, vehicle_sys, vehicle_comp, store):
            write_parm(parm_path, params.values)
            store.param_status = "%d parametros" % len(params.values)
            print("Parametros: %d salvos em %s" % (len(params.values), parm_path))
        try:
            data = link.recv()
        except (OSError, ConnectionError) as exc:
            print("Erro de conexao: %s" % exc, file=sys.stderr)
            stop.set()
            break
        if not data:
            continue
        rows = []
        for msgid, sysid, compid, payload in parser.feed(data):
            if sysid == GCS_SYSID:
                continue
            msg = decode(msgid, payload)
            if msg is None:
                continue
            wall = datetime.datetime.now().isoformat(timespec="milliseconds")
            t_s = "%.3f" % store.now()
            if msgid == MSG_HEARTBEAT:
                if msg["autopilot"] == MAV_AUTOPILOT_INVALID or msg["type"] == MAV_TYPE_GCS:
                    continue
                if vehicle_sys is None:
                    vehicle_sys, vehicle_comp = sysid, compid
                if sysid != vehicle_sys:
                    continue
                store.last_any = store.now()
                if store.armed != msg["armed"]:
                    store.armed = msg["armed"]
                    print("[%s] %s" % (wall[11:], "ARMADO" if msg["armed"] else "DESARMADO"))
                    rows.append([wall, t_s, "", "ARMED", int(msg["armed"]), ""])
                    store.add_event(6, "ARMADO" if msg["armed"] else "DESARMADO")
                if store.mode != msg["mode"]:
                    store.mode = msg["mode"]
                    name = ROVER_MODES.get(msg["mode"], str(msg["mode"]))
                    print("[%s] modo %s" % (wall[11:], name))
                    rows.append([wall, t_s, "", "MODE", msg["mode"], name])
                    store.add_event(6, "modo " + name)
                continue
            if vehicle_sys is not None and sysid != vehicle_sys:
                continue
            store.last_any = store.now()
            if msgid == MSG_NAMED_VALUE_FLOAT:
                if not msg["name"].startswith("ESC"):
                    continue
                # The HereLink can deliver every message twice (its 14550 stream
                # and the replies to our 14552 heartbeat): same name and boot time
                if last_value_ms.get(msg["name"]) == msg["boot_ms"]:
                    continue
                last_value_ms[msg["name"]] = msg["boot_ms"]
                store.add_value(msg["name"], msg["value"], esc=True)
                rows.append([wall, t_s, msg["boot_ms"], msg["name"], "%.4f" % msg["value"], ""])
                continue
            # Same duplicate path for everything else: an identical message seen
            # within the last half second is the copy
            key = (msgid, payload)
            if now - recent.get(key, -10.0) < 0.5:
                continue
            recent[key] = now
            if len(recent) > 2000:
                recent.clear()
            if msgid == MSG_COMMAND_ACK:
                if msg["command"] == MAV_CMD_SET_MESSAGE_INTERVAL:
                    # the acks of all streams look identical, so report each result once
                    if msg["result"] not in acks:
                        acks.add(msg["result"])
                        print("[%s] pedido de taxa: %s" % (wall[11:], "aceito" if msg["result"] == 0
                              else "recusado (resultado %d)" % msg["result"]))
                        rows.append([wall, t_s, "", "RATE_ACK", msg["result"], ""])
                continue
            if msgid == MSG_STATUSTEXT:
                sev = SEVERITY[msg["severity"]] if msg["severity"] < len(SEVERITY) else str(msg["severity"])
                store.add_event(msg["severity"], msg["text"])
                rows.append([wall, t_s, "", "STATUSTEXT", msg["severity"], msg["text"]])
                print("[%s] %-6s %s" % (wall[11:], sev, msg["text"]))
                # ArduPilot's scripting run line (rover.lua asks for one at boot and
                # when a tune loads/unloads): memory in use as % of SCR_HEAP_SIZE
                m = LUA_MEM_RE.match(msg["text"])
                if m:
                    total, run = int(m.group(2)), int(m.group(3))
                    used = total - run
                    heap = params.values.get("SCR_HEAP_SIZE")
                    if heap:
                        pct = 100.0 * used / heap
                        print("[%s] MEM    Lua %.0f kB de %.0f kB (%.0f%%)" % (wall[11:], used / 1024.0,
                                                                         heap / 1024.0, pct))
                        rows.append([wall, t_s, "", "LUA_MEM_PCT", "%.1f" % pct, ""])
                    else:
                        print("[%s] MEM    Lua %.0f kB (SCR_HEAP_SIZE ainda nao lido)" % (wall[11:], used / 1024.0))
                    rows.append([wall, t_s, "", "LUA_MEM_B", used, ""])
            elif msgid == MSG_PARAM_VALUE:
                name, value = msg["name"], msg["value"]
                params.last_rx = now
                if msg["count"] and msg["count"] < 65535:
                    params.count = msg["count"]
                if msg["index"] < 65535:
                    params.by_index[msg["index"]] = name
                old = params.values.get(name)
                params.values[name] = value
                if old is None or abs(old - value) > 1e-7 * max(1.0, abs(value)):
                    rows.append([wall, t_s, "", "PARAM:" + name, "%.7g" % value, ""])
                    # ground pressure and run-time counters change by themselves: CSV only
                    if old is not None and not (name.endswith("_GND_PRESS") or name.startswith("STAT_")):
                        print("[%s] PARAM %s: %.6g -> %.6g" % (wall[11:], name, old, value))
                        if params.done:
                            write_parm(parm_path, params.values)
            else:
                boot = msg.get("boot_ms", "")
                for name, value in msg["values"]:
                    store.add_value(name, value)
                    rows.append([wall, t_s, boot, name, fmt_value(name, value), ""])
        if rows:
            csv_writer.writerows(rows)
            csv_file.flush()


# ---------------------------------------------------------------------------
# Output: live plot or console summary
# ---------------------------------------------------------------------------
def motor_ids(series):
    ids = set()
    for name in series:
        if name.startswith("ESC") and name[3:4].isdigit():
            ids.add(int(name[3]))
    return sorted(ids)


def nav_line(store):
    lat = store.latest
    parts = []
    if store.mode is not None:
        parts.append(ROVER_MODES.get(store.mode, str(store.mode)))
    for key, fmt in (("VFR_gspd", "v %.2fm/s"), ("NAV_xtrack", "xtrack %+.2fm"), ("NAV_wp_dist", "wp %.0fm"),
                     ("PIDS_des", "yawrate des %+.0f"), ("PIDS_ach", "ach %+.0f graus/s"),
                     ("GPS_sats", "sats %d")):
        if key in lat:
            parts.append(fmt % lat[key])
    if store.param_status:
        parts.append(store.param_status)
    return " | ".join(parts)


def run_console(store, stop, duration):
    end = time.monotonic() + duration if duration else None
    while not stop.is_set():
        if end and time.monotonic() >= end:
            break
        time.sleep(1.0)
        series, _, armed, last_rx = store.snapshot()
        if store.last_any is None:
            print("Aguardando MAVLink do rover... (rede/porta certas? Wi-Fi AndroidAP como rede Privada?)")
            continue
        parts = []
        for m in motor_ids(series):
            def last(name):
                d = series.get(name)
                return d[-1][1] if d else float("nan")
            parts.append("M%d %5.1fA/%5.1fA %6.0frpm" % (
                m, last("ESC%d_I" % m), last("ESC%d_ICMD" % m), last("ESC%d_RPM" % m))
                + (" %3.0fC" % last("ESC%d_T" % m) if ("ESC%d_T" % m) in store.latest else ""))
        if last_rx is None:
            parts.append("sem valores ESC (ESC_CAN = 1 e ESC_GCS_HZ > 0?)")
        print(" | ".join(parts) + ("   [armado]" if armed else ""))
        nl = nav_line(store)
        if nl:
            print("    " + nl)
    stop.set()


def run_plot(store, stop, window_s):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
    fig, (ax_i, ax_r, ax_n, ax_t) = plt.subplots(4, 1, sharex=True, figsize=(11, 10.5),
                                                 gridspec_kw={"height_ratios": (3, 3, 3, 2)})
    fig.canvas.manager.set_window_title("Rover monitor")
    status = fig.text(0.01, 0.01, "", fontsize=9, family="monospace")
    lines = {}

    def line(ax, key, **kw):
        if key not in lines:
            lines[key], = ax.plot([], [], **kw)
        return lines[key]

    ax_i.set_ylabel("Corrente (A)")
    ax_i.set_title("solida = medida, tracejada = comandada")
    ax_r.set_ylabel("Roda (rpm)")
    ax_n.set_ylabel("yaw rate (graus/s)")
    ax_t.set_ylabel("Motor (graus C)")
    ax_t.set_xlabel("tempo (s)")
    ax_b = ax_t.twinx()
    ax_b.set_ylabel("Bateria BMS (A)", color="gray")
    ax_x = ax_n.twinx()
    ax_x.set_ylabel("xtrack (m)", color="purple")
    for ax in (ax_i, ax_r, ax_n, ax_t):
        ax.grid(True, alpha=0.3)
    event_marks = []

    def set_xy(ln, d):
        ln.set_data([p[0] for p in d], [p[1] for p in d])

    def update(_):
        if stop.is_set():
            plt.close(fig)
            return []
        series, events, armed, last_rx = store.snapshot()
        now = store.now()
        for m in motor_ids(series):
            c = colors[(m - 1) % len(colors)]
            for key, ax, style, label in (
                    ("ESC%d_I" % m, ax_i, "-", "M%d medida" % m),
                    ("ESC%d_ICMD" % m, ax_i, "--", "M%d cmd" % m),
                    ("ESC%d_RPM" % m, ax_r, "-", "M%d" % m),
                    ("ESC%d_T" % m, ax_t, "-", "M%d" % m)):
                set_xy(line(ax, key, color=c, linestyle=style, label=label, linewidth=1.2), series.get(key, []))
        for key, style in (("ESC_TGT_L", ":"), ("ESC_TGT_R", "-.")):
            set_xy(line(ax_r, key, color="black", linestyle=style, label=key[-5:], linewidth=1), series.get(key, []))
        for key, c, label in (("PIDS_des", "black", "desejado"), ("PIDS_ach", "#2ca02c", "real"),
                              ("ATT_yawrate", "#aaaaaa", "giro (ATT)")):
            if key in series or key == "PIDS_des":
                set_xy(line(ax_n, key, color=c, label=label, linewidth=1.1), series.get(key, []))
        set_xy(line(ax_b, "BAT_I", color="gray", linestyle=":", linewidth=1, label="bateria"), series.get("BAT_I", []))
        set_xy(line(ax_x, "NAV_xtrack", color="purple", linewidth=1.2, label="xtrack"), series.get("NAV_xtrack", []))
        for mark in event_marks:
            mark.remove()
        event_marks.clear()
        for t, sev, text in events:
            if t < now - window_s:
                continue
            if "lost" in text or sev <= 3:
                for ax in (ax_i, ax_r, ax_n, ax_t):
                    event_marks.append(ax.axvline(t, color="red", alpha=0.5))
            elif text.startswith("modo "):
                for ax in (ax_i, ax_r, ax_n, ax_t):
                    event_marks.append(ax.axvline(t, color="gray", linestyle="--", alpha=0.6))
                event_marks.append(ax_n.text(t, 0.95, text[5:], transform=ax_n.get_xaxis_transform(),
                                             fontsize=8, color="gray", va="top"))
        for ax in (ax_i, ax_r, ax_n, ax_x, ax_t, ax_b):
            ax.set_xlim(max(0, now - window_s), max(window_s, now))
            ax.relim()
            ax.autoscale_view(scalex=False)
        for ax in (ax_i, ax_r, ax_n, ax_t):
            handles, _ = ax.get_legend_handles_labels()
            if handles and (ax.get_legend() is None or len(handles) != len(ax.get_legend().get_texts())):
                ax.legend(loc="upper left", fontsize=8, ncol=4)
        last_text = next((e[2] for e in reversed(events) if not e[2].startswith("modo ")), "")
        state = "sem dados" if store.last_any is None else ("ARMADO" if armed else "desarmado")
        status.set_text("%s | %s | ultima msg: %s" % (state, nav_line(store), last_text[:60]))
        return []

    anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    fig._esc_anim = anim  # keep a reference so it is not garbage collected
    plt.tight_layout(rect=(0, 0.03, 1, 1))
    plt.show()
    stop.set()


# ---------------------------------------------------------------------------
# Log analysis:  python esc_monitor.py analyze <csv> [--plot] [--from T] [--to T]
# ---------------------------------------------------------------------------
import os

try:
    import numpy as np
except ImportError:   # only the analysis needs numpy
    np = None

KEY_PARAM_PREFIXES = ("ESC_", "ATC_STR", "ATC_SPEED", "ATC_ACCEL", "ATC_DECEL", "ATC_TURN", "CRUISE_", "WP_",
                      "PSC_", "NAVL1_", "TURN_", "THRO_", "CANT_ACTIVE", "SCR_USER", "GCS_PID_MASK", "RTUN_",
                      "CTUN_", "YAWT_", "PILOT_STEER", "MOT_THR", "MOT_SLEWRATE", "CIRC_", "SPEED_", "AHRS_EKF",
                      "EK3_SRC", "GPS_TYPE", "COMPASS_USE")
SKIP_PARAMS = ("ESC_CMD_L", "ESC_CMD_R", "ESC_CMD_SEQ", "ESC_STAT", "ESC_FR1", "ESC_FR2", "ESC_FR3", "ESC_FR4",
               "ESC_LOC_P", "ESC_LOC_I", "ESC_LOC_MAX")
NAV_MODES = ("AUTO", "RTL", "SMART_RTL", "GUIDED", "LOITER")


class Log:
    def __init__(self, path, t_from=None, t_to=None):
        raw = collections.defaultdict(list)
        self.texts = []
        self.modes = []          # (t, name)
        self.armed = []          # (t, 0/1)
        self.params0 = {}        # first value seen
        self.param_changes = []  # (t, name, old, new)
        last_param = {}
        with open(path, newline="", encoding="utf-8-sig") as f:
            for row in csv.DictReader(f):
                try:
                    t = float(row["t_s"])
                except (TypeError, ValueError):
                    continue
                name = row["nome"]
                if name.startswith("PARAM:"):
                    pname = name[6:]
                    v = float(row["valor"])
                    if pname not in last_param:
                        self.params0[pname] = v
                    elif abs(last_param[pname] - v) > 1e-7 * max(1.0, abs(v)):
                        self.param_changes.append((t, pname, last_param[pname], v))
                    last_param[pname] = v
                    continue
                if (t_from is not None and t < t_from) or (t_to is not None and t > t_to):
                    if name == "MODE":   # keep the mode active at the window start
                        self.modes.append((t, row["texto"]))
                    continue
                if name == "STATUSTEXT":
                    self.texts.append((t, row["valor"], row["texto"]))
                elif name == "MODE":
                    self.modes.append((t, row["texto"]))
                elif name == "ARMED":
                    self.armed.append((t, int(float(row["valor"]))))
                else:
                    try:
                        raw[name].append((t, float(row["valor"])))
                    except ValueError:
                        pass
        self.params = dict(self.params0)
        self.params.update(last_param)
        self.s = {k: (np.array([p[0] for p in v]), np.array([p[1] for p in v])) for k, v in raw.items()}
        ts = [v[0] for v in self.s.values() if len(v[0])]
        self.t0 = min(a[0] for a in ts) if ts else 0.0
        self.t1 = max(a[-1] for a in ts) if ts else 0.0
        if t_from is not None:
            self.t0 = max(self.t0, t_from)
            # mode active at the start of the window
            before = [m for m in self.modes if m[0] <= t_from]
            self.modes = ([(t_from, before[-1][1])] if before else []) + [m for m in self.modes if m[0] > t_from]
        if t_to is not None:
            self.t1 = min(self.t1, t_to)
            self.modes = [m for m in self.modes if m[0] <= t_to]

    def has(self, name):
        return name in self.s and len(self.s[name][0]) > 1

    def get(self, name, a=None, b=None):
        if name not in self.s:
            return np.array([]), np.array([])
        t, v = self.s[name]
        if a is None:
            return t, v
        m = (t >= a) & (t <= b)
        return t[m], v[m]

    def p(self, name, default=None):
        return self.params.get(name, default)

    def segments(self, min_s=3.0):
        """[(mode, t_start, t_end)] from MODE rows (whole log as '?' if none)."""
        if not self.modes:
            return [("?", self.t0, self.t1)]
        out = []
        modes = sorted(self.modes)
        if modes[0][0] > self.t0 + 1:
            out.append(("?", self.t0, modes[0][0]))
        for i, (t, name) in enumerate(modes):
            end = modes[i + 1][0] if i + 1 < len(modes) else self.t1
            out.append((name, max(t, self.t0), end))
        return [s for s in out if s[2] - s[1] >= min_s]


# ---------------------------------------------------------------------------
# Signal helpers
# ---------------------------------------------------------------------------
def wrap180(x):
    return (np.asarray(x) + 180.0) % 360.0 - 180.0


def resample(t, v, grid, angle=False):
    if len(t) < 2:
        return None
    if angle:
        v = np.degrees(np.unwrap(np.radians(v)))
    out = np.interp(grid, t, v)
    # no data gaps longer than 1 s
    idx = np.searchsorted(t, grid)
    idx = np.clip(idx, 1, len(t) - 1)
    gap = (t[idx] - t[idx - 1]) > 1.0
    out[gap] = np.nan
    return out


def rate_hz(t):
    if len(t) < 2:
        return 0.0
    return (len(t) - 1) / max(1e-6, t[-1] - t[0])


def dominant(x, fs, fmin=0.05, fmax=2.0):
    """(frequency Hz, amplitude) of the strongest oscillation in x."""
    x = x[~np.isnan(x)]
    if len(x) < fs * 8:
        return None
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    win = np.hanning(len(x))
    spec = np.abs(np.fft.rfft(x * win))
    freqs = np.fft.rfftfreq(len(x), 1.0 / fs)
    band = (freqs >= fmin) & (freqs <= fmax)
    if not band.any():
        return None
    i = np.argmax(np.where(band, spec, 0))
    amp = 2.0 * spec[i] / win.sum()
    # share of the band energy in the peak +-1 bin: is it a clear oscillation?
    e = spec[band] ** 2
    peak = (spec[max(i - 1, 0):i + 2] ** 2).sum()
    return freqs[i], amp, peak / max(e.sum(), 1e-12)


def lag_s(ref, out, fs, max_lag=2.0):
    """Delay of out behind ref (s) and the correlation at that delay."""
    m = ~(np.isnan(ref) | np.isnan(out))
    if m.sum() < fs * 5:
        return None
    a = ref[m] - ref[m].mean()
    b = out[m] - out[m].mean()
    if a.std() < 1e-9 or b.std() < 1e-9:
        return None
    best = (-2.0, 0)
    for k in range(0, int(max_lag * fs) + 1):
        if k >= len(a) - 10:
            break
        c = np.corrcoef(a[:len(a) - k], b[k:])[0, 1]
        if c > best[0]:
            best = (c, k)
    return best[1] / fs, best[0]


def rms(x):
    x = np.asarray(x)
    x = x[~np.isnan(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


# ---------------------------------------------------------------------------
# Report sections
# ---------------------------------------------------------------------------
GROUPS = [("ESC (esc_can)", "ESC1_RPM"), ("Atitude", "ATT_yaw"), ("Posicao", "GPI_lat"),
          ("Navegacao", "NAV_xtrack"), ("PID direcao", "PIDS_des"), ("PID velocidade", "PIDV_des"),
          ("VFR", "VFR_gspd"), ("GPS", "GPS_sats"), ("Servos", "SRV_1"), ("RC", "RC_1"), ("Missao", "WP_SEQ"),
          ("Temperatura", "ESC1_T"), ("Bateria", "BAT_V")]


def report_inputs(log):
    print("== Dados no log: %.0f s (t_s %.0f..%.0f)" % (log.t1 - log.t0, log.t0, log.t1))
    missing = []
    parts = []
    for label, key in GROUPS:
        if log.has(key):
            parts.append("%s %.1f Hz" % (label, rate_hz(log.get(key)[0])))
        else:
            missing.append(label)
    print("   " + " | ".join(parts))
    if missing:
        print("   faltando: " + ", ".join(missing))
    if "PID direcao" in missing:
        print("   -> PID_TUNING exige GCS_PID_MASK = 3 (direcao + velocidade). Atual: %s"
              % fmt(log.p("GCS_PID_MASK")))
    if log.has("ESC1_RPM") and rate_hz(log.get("ESC1_RPM")[0]) < 8:
        print("   -> ESC a %.0f Hz: pouco para medir atraso das rodas. ESC_GCS_HZ 10..20 ajuda "
              "(atual %s)." % (rate_hz(log.get("ESC1_RPM")[0]), fmt(log.p("ESC_GCS_HZ"))))
    if not log.params:
        print("   sem parametros no log (monitor antigo ou --no-params)")


def fmt(v):
    if v is None:
        return "?"
    return ("%.4g" % v) if abs(v - round(v)) > 1e-9 else "%d" % round(v)


def report_params(log):
    if not log.params:
        return
    keys = sorted(k for k in log.params if k.startswith(KEY_PARAM_PREFIXES) and k not in SKIP_PARAMS)
    print("\n== Parametros (%d no log; relevantes abaixo)" % len(log.params))
    line = []
    for k in keys:
        line.append("%s=%s" % (k, fmt(log.params[k])))
        if len(line) == 5:
            print("   " + "  ".join(line))
            line = []
    if line:
        print("   " + "  ".join(line))
    if log.param_changes:
        print("\n== Parametros alterados durante o log")
        for t, name, old, new in log.param_changes:
            if name in SKIP_PARAMS or name.endswith("_GND_PRESS") or name.startswith("STAT_"):
                continue
            print("   t=%7.1f  %-18s %s -> %s" % (t, name, fmt(old), fmt(new)))


# M3508 P19 at the output shaft: torque constant and winding resistance (datasheet)
M3508_KT = 0.3      # N.m per A of torque current
M3508_R = 0.194     # ohm
ESC_EFF = 0.9


def battery_estimate(log, a=None, b=None, fs=5.0):
    """(t, battery A, total W) from the motor torque current and speed, or None.

    ESCn_I is the motor torque (phase) current from the C620, not battery current:
    P = Kt*|I|*w + R*I^2 per motor, divided by the ESC efficiency and the battery voltage.
    """
    if not log.has("ESC1_I"):
        return None
    a = log.t0 if a is None else a
    b = log.t1 if b is None else b
    grid = np.arange(a, b, 1.0 / fs)
    if len(grid) < 2:
        return None
    total = np.zeros_like(grid)
    for w in range(1, 5):
        ti, cur = log.get("ESC%d_I" % w)
        tr, rpm = log.get("ESC%d_RPM" % w)
        if len(ti) < 2 or len(tr) < 2:
            continue
        i = np.abs(np.interp(grid, ti, cur))
        omega = np.abs(np.interp(grid, tr, rpm)) * 2 * math.pi / 60.0
        total += M3508_KT * i * omega + M3508_R * i * i
    total /= ESC_EFF
    if log.has("BAT_V"):
        tv, vv = log.get("BAT_V")
        volts = np.interp(grid, tv, vv)
    else:
        volts = np.full_like(grid, np.nan)
    return grid, total / volts, total


def report_battery(log, a, b):
    est = battery_estimate(log, a, b)
    if est is None:
        return
    t, amps, watts = est
    phase = 0.0
    for w in range(1, 5):
        _, cur = log.get("ESC%d_I" % w, a, b)
        if len(cur):
            phase += float(np.mean(np.abs(cur)))
    msg = "   corrente dos motores (soma |ESCn_I|, fase/torque): %.1f A; potencia estimada %.0f W" % (
        phase, np.mean(watts))
    if not np.isnan(amps).all():
        msg += " -> bateria estimada %.1f A" % np.nanmean(amps)
    print(msg)
    _, bi = log.get("BAT_I", a, b)
    _, bv = log.get("BAT_V", a, b)
    if len(bi) or len(bv):
        print("   bateria (ArduPilot/BMS): %s%s" % (
            ("%.1f V " % np.mean(bv)) if len(bv) else "", ("%.1f A media" % np.mean(bi)) if len(bi) else ""))


def side_wheels(log):
    lmask = int(log.p("ESC_LMASK", 6) or 6)
    left = [i for i in range(1, 5) if lmask & (1 << (i - 1))]
    right = [i for i in range(1, 5) if i not in left]
    return left, right


def report_segment(log, mode, a, b):
    fs = 10.0
    grid = np.arange(a, b, 1.0 / fs)
    print("\n== %s  t=%.1f..%.1f (%.0f s)" % (mode, a, b, b - a))
    hints = []

    # speed
    t, v = log.get("VFR_gspd", a, b)
    if len(v) == 0:
        t, vn = log.get("GPI_vn", a, b)
        _, ve = log.get("GPI_ve", a, b)
        n = min(len(vn), len(ve))
        v = np.hypot(vn[:n], ve[:n])
    if len(v):
        wps = log.p("WP_SPEED")
        print("   velocidade: media %.2f m/s, max %.2f m/s%s" % (
            v.mean(), v.max(), (" (WP_SPEED %.2f)" % wps) if wps and mode in NAV_MODES else ""))

    # cross track and heading
    if log.has("NAV_xtrack") and mode in NAV_MODES:
        t, x = log.get("NAV_xtrack", a, b)
        if len(x) > 5:
            # remove each leg's mean so a leg change is not counted as oscillation
            if log.has("WP_SEQ"):
                ts, seq = log.get("WP_SEQ")
                leg = np.interp(t, ts, seq) if len(ts) > 1 else np.zeros_like(t)
                leg = np.round(leg)
                xd = x.copy()
                for s in np.unique(leg):
                    m = leg == s
                    xd[m] = x[m] - x[m].mean()
            else:
                xd = x - x.mean()
            print("   xtrack: rms %.2f m, max |%.2f| m" % (rms(x), np.abs(x).max()))
            xg = resample(t, xd, grid)
            d = dominant(xg, fs) if xg is not None else None
            if d:
                f, amp, share = d
                clear = share > 0.25
                print("   oscilacao do xtrack: periodo %.1f s, amplitude %.2f m%s" % (
                    1 / f, amp, "" if clear else " (fraca/nao periodica)"))
                if clear and amp > 0.1:
                    hints.append("rover zigue-zagueia: periodo %.1f s, +-%.2f m em torno da linha" % (1 / f, amp))
    if log.has("NAV_nav_brg") and log.has("ATT_yaw") and mode in NAV_MODES:
        nb = resample(*log.get("NAV_nav_brg"), grid, angle=True)
        yw = resample(*log.get("ATT_yaw"), grid, angle=True)
        if nb is not None and yw is not None:
            he = wrap180(nb - yw)
            print("   erro de rumo (nav_bearing - yaw): rms %.1f graus" % rms(he))

    # heading estimate vs where the rover really goes (GPS course while driving):
    # a steady offset makes the rover drift off the line and then correct (S path)
    if log.has("GPS_cog") and log.has("ATT_yaw"):
        tc, cog = log.get("GPS_cog", a, b)
        if len(tc) > 5:
            ty, yw = log.get("ATT_yaw")
            tv, vv = log.get("VFR_gspd")
            yaw_u = np.degrees(np.unwrap(np.radians(yw)))
            spd = np.interp(tc, tv, vv) if len(tv) > 1 else np.ones_like(tc)
            m = spd > 0.6
            if m.sum() >= 8:
                diff = wrap180(cog[m] - np.interp(tc[m], ty, yaw_u))
                med = float(np.median(diff))
                print("   direcao real (GPS) - rumo estimado (bussola/EKF): mediana %+.1f graus, "
                      "faixa central %+.1f..%+.1f (%d amostras andando)" % (
                          med, np.percentile(diff, 25), np.percentile(diff, 75), m.sum()))
                if abs(med) >= 3:
                    hints.append("o rover anda %.0f graus a %s de onde o rumo estimado aponta: erro de bussola "
                                 "(interferencia dos motores/cabos, alinhamento do GPS) faz ele sair da linha e "
                                 "corrigir em S" % (abs(med), "direita" if med > 0 else "esquerda"))

    # steering rate loop
    if log.has("PIDS_des"):
        td, des = log.get("PIDS_des", a, b)
        ta, ach = log.get("PIDS_ach", a, b)
        if len(des) > 10:
            dg = resample(td, des, grid)
            ag = resample(ta, ach, grid)
            err = dg - ag
            print("   yaw rate: desejado rms %.1f, real rms %.1f, erro rms %.1f graus/s" % (
                rms(dg), rms(ag), rms(err)))
            lg = lag_s(dg, ag, fs)
            if lg:
                print("   atraso real x desejado: %.2f s (correlacao %.2f)" % lg)
                if lg[0] >= 0.3 and lg[1] > 0.5:
                    hints.append("yaw rate real atrasa %.2f s do desejado: malha de direcao/rodas lenta" % lg[0])
            sd, sa = np.nanstd(dg), np.nanstd(ag)
            if sd > 1:
                ratio = sa / sd
                print("   ganho real/desejado: %.2f" % ratio)
                if ratio > 1.3:
                    hints.append("yaw rate real maior que o pedido (%.0f%%): ganho de direcao alto" % (ratio * 100))
                elif ratio < 0.6:
                    hints.append("yaw rate real menor que o pedido (%.0f%%): falta autoridade/ganho" % (ratio * 100))
            d = dominant(dg, fs)
            if d and d[2] > 0.25:
                print("   oscilacao do yaw rate desejado: periodo %.1f s, amplitude %.1f graus/s" % (1 / d[0], d[1]))
            terms = {}
            for k in ("ff", "p", "i", "d"):
                _, v = log.get("PIDS_" + k, a, b)
                if len(v):
                    terms[k] = float(np.std(v)), float(np.mean(v))
            if terms:
                print("   termos PID direcao (desvio padrao / media): " + ", ".join(
                    "%s %.3f/%+.3f" % (k.upper(), s, m) for k, (s, m) in terms.items()))
                imax = log.p("ATC_STR_RAT_IMAX")
                if "i" in terms and imax and abs(terms["i"][1]) > 0.8 * imax:
                    hints.append("integrador da direcao perto do IMAX (%.2f)" % imax)
                if "p" in terms and "ff" in terms and terms["p"][0] > 2 * terms["ff"][0] > 0:
                    hints.append("termo P domina o FF na direcao: P alto ou FF baixo")

    if log.has("PIDV_des"):
        _, vd = log.get("PIDV_des", a, b)
        _, va = log.get("PIDV_ach", a, b)
        if len(vd) > 10:
            n = min(len(vd), len(va))
            print("   velocidade (PID): desejada %.2f, real %.2f m/s, erro rms %.2f" % (
                np.mean(vd), np.mean(va), rms(vd[:n] - va[:n])))

    # CAN wheels
    if log.has("ESC_TGT_L"):
        esc_rpm = log.p("ESC_RPM")
        esc_cur = log.p("ESC_CUR")
        left, right = side_wheels(log)
        efs = max(2.0, min(20.0, rate_hz(log.get("ESC1_RPM")[0]) if log.has("ESC1_RPM") else 4.0))
        egrid = np.arange(a, b, 1.0 / efs)
        for side, key, wheels in (("esq", "ESC_TGT_L", left), ("dir", "ESC_TGT_R", right)):
            tt, tgt = log.get(key, a, b)
            if len(tgt) < 3:
                continue
            msg = "   rodas %s (ESC%s): alvo max |%.0f| rpm" % (side, "+".join(map(str, wheels)), np.abs(tgt).max())
            if esc_rpm:
                sat = np.mean(np.abs(tgt) >= 0.97 * esc_rpm)
                msg += ", no limite ESC_RPM %.0f%% do tempo" % (sat * 100)
                if sat > 0.05:
                    hints.append("lado %s satura em ESC_RPM %.0f%% do tempo: velocidade/curva acima do limite"
                                 % (side, sat * 100))
            print(msg)
            tg = resample(tt, tgt, egrid)
            for w in wheels:
                tw, rpm = log.get("ESC%d_RPM" % w, a, b)
                if len(rpm) < 3 or tg is None:
                    continue
                rg = resample(tw, rpm, egrid)
                # the wheel sign convention may be flipped against the side target
                sgn = 1.0 if np.nansum(rg * tg) >= 0 else -1.0
                e = rms(sgn * rg - tg)
                lg = lag_s(tg, sgn * rg, efs)
                ti, cur = log.get("ESC%d_I" % w, a, b)
                lim = (np.mean(np.abs(cur) >= 0.95 * esc_cur) * 100) if (esc_cur and len(cur)) else float("nan")
                print("     ESC%d: erro rms %.1f rpm%s, corrente media |%.1f| A, no limite %s" % (
                    w, e, (", atraso %.2f s" % lg[0]) if lg else "", np.mean(np.abs(cur)) if len(cur) else 0,
                    "?" if math.isnan(lim) else "%.0f%%" % lim))
                if lg and lg[0] >= 0.3 and lg[1] > 0.6:
                    hints.append("ESC%d segue o alvo com %.2f s de atraso: malha das rodas lenta" % (w, lg[0]))
                if lim > 5:
                    hints.append("ESC%d no limite de corrente %.0f%% do tempo" % (w, lim))
                _, temp = log.get("ESC%d_T" % w, a, b)
                if len(temp):
                    print("       temperatura do motor: inicio %.0f, max %.0f, fim %.0f graus C" % (
                        temp[0], temp.max(), temp[-1]))
                    if temp.max() >= 70:
                        hints.append("ESC%d: motor chegou a %.0f graus C" % (w, temp.max()))
                # ESC answers on CAN but delivers no current: protection or power/phase fault
                tc, cmd = log.get("ESC%d_ICMD" % w, a, b)
                if len(cmd) > 5 and len(cur) > 5:
                    curg = np.interp(tc, ti, cur)
                    dead = (np.abs(cmd) >= 3.0) & (np.abs(curg) < 0.5)
                    if dead.any():
                        # first run of at least 1 s
                        start = None
                        for k in range(len(tc)):
                            if dead[k]:
                                start = tc[k] if start is None else start
                                if tc[k] - start >= 1.0:
                                    break
                            else:
                                start = None
                        else:
                            start = None
                        if start is not None:
                            hints.append("ESC%d SEM CORRENTE a partir de t=%.1f s (pede >= 3 A, mede ~0): "
                                         "protecao do C620, temperatura ou fio/fase do motor" % (w, start))

    report_battery(log, a, b)

    texts = [x for x in log.texts if a <= x[0] <= b]
    if texts:
        print("   mensagens:")
        for t, sev, text in texts[:15]:
            print("     t=%7.1f  %s" % (t, text))
        if len(texts) > 15:
            print("     ... +%d" % (len(texts) - 15))
    if hints:
        print("   >> " + "\n   >> ".join(hints))


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def plot(log, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(6, 2, width_ratios=(1, 1.6))
    ax_xy = fig.add_subplot(gs[:, 0])
    axes = [fig.add_subplot(gs[i, 1]) for i in range(6)]
    for ax in axes[1:]:
        ax.sharex(axes[0])
    segs = log.segments(min_s=0.5)
    palette = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf"]
    mode_color = {}
    for m, _, _ in segs:
        mode_color.setdefault(m, palette[len(mode_color) % len(palette)])

    # track
    if log.has("GPI_lat"):
        t, lat = log.get("GPI_lat")
        _, lon = log.get("GPI_lon")
        n = min(len(lat), len(lon))
        t, lat, lon = t[:n], lat[:n], lon[:n]
        lat0, lon0 = lat[0], lon[0]
        y = (lat - lat0) * 111320.0
        x = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
        for m, a, b in segs:
            sel = (t >= a) & (t <= b)
            if sel.any():
                ax_xy.plot(x[sel], y[sel], ".-", ms=2, lw=0.8, color=mode_color[m],
                           label=m if m not in ax_xy.get_legend_handles_labels()[1] else None)
        ax_xy.set_aspect("equal", adjustable="datalim")
        ax_xy.legend(fontsize=8)
    ax_xy.set_title("trajeto (m, leste x norte)")
    ax_xy.grid(True, alpha=0.3)

    def series(ax, key, label, **kw):
        if log.has(key):
            t, v = log.get(key)
            ax.plot(t, v, label=label, lw=1, **kw)

    series(axes[0], "NAV_xtrack", "xtrack (m)", color="purple")
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].set_ylabel("xtrack m")
    series(axes[1], "PIDS_des", "yaw rate desejado", color="k")
    series(axes[1], "PIDS_ach", "yaw rate real", color="#2ca02c")
    if not log.has("PIDS_ach"):
        series(axes[1], "ATT_yawrate", "yaw rate (ATT)", color="#2ca02c")
    axes[1].set_ylabel("graus/s")
    if log.has("NAV_nav_brg") and log.has("ATT_yaw"):
        t, nb = log.get("NAV_nav_brg")
        ty, yw = log.get("ATT_yaw")
        he = wrap180(np.interp(ty, t, np.degrees(np.unwrap(np.radians(nb)))) - yw)
        axes[2].plot(ty, he, lw=1.4, color="#8c564b", label="nav_bearing - yaw (graus)")
    for k in ("ff", "p", "i"):
        if log.has("PIDS_" + k):
            t, v = log.get("PIDS_" + k)
            axes[2].plot(t, v * 100, lw=0.8, label="PID %s x100" % k.upper())
    axes[2].set_ylabel("graus / termos")
    left, right = side_wheels(log)
    colors = {1: "#1f77b4", 2: "#d62728", 3: "#2ca02c", 4: "#ff7f0e"}
    for w in range(1, 5):
        series(axes[3], "ESC%d_RPM" % w, "ESC%d" % w, color=colors[w])
    series(axes[3], "ESC_TGT_L", "alvo esq", color="k", linestyle=":")
    series(axes[3], "ESC_TGT_R", "alvo dir", color="k", linestyle="-.")
    axes[3].set_ylabel("rpm")
    series(axes[4], "VFR_gspd", "velocidade (m/s)", color="k")
    series(axes[4], "PIDV_des", "vel desejada", color="#9467bd", linestyle="--")
    for w in range(1, 5):
        if log.has("ESC%d_I" % w):
            t, v = log.get("ESC%d_I" % w)
            axes[4].plot(t, v / 10.0, lw=0.7, color=colors[w], alpha=0.6, label="I%d/10 (A)" % w)
    for w in range(1, 5):
        series(axes[5], "ESC%d_T" % w, "M%d" % w, color=colors[w])
    axes[5].set_ylabel("motor graus C")
    if log.has("BAT_I") or log.has("ESC1_I"):
        ax_b = axes[5].twinx()
        if log.has("BAT_I"):
            t, v = log.get("BAT_I")
            ax_b.plot(t, v, color="gray", lw=0.8, linestyle=":", label="bateria BMS (A)")
        est = battery_estimate(log)
        if est is not None and not np.isnan(est[1]).all():
            ax_b.plot(est[0], est[1], color="k", lw=0.8, alpha=0.7, label="bateria estimada (A)")
        ax_b.set_ylabel("bateria A", color="gray")
        if ax_b.get_legend_handles_labels()[0]:
            ax_b.legend(fontsize=7, loc="upper right")
    axes[5].set_xlabel("t_s")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=7, loc="upper left", ncol=4)
        for m, a, b in segs:
            ax.axvspan(a, b, color=mode_color[m], alpha=0.06)
    for m, a, b in segs:
        axes[0].text(a, 1.0, m, transform=axes[0].get_xaxis_transform(), fontsize=7, va="bottom")
    for t, _, text in log.texts:
        if "RTun" in text or "Tune" in text or "lost" in text:
            for ax in axes:
                ax.axvline(t, color="red", lw=0.6, alpha=0.4)
    axes[0].set_xlim(log.t0, log.t1)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print("\nGrafico: %s" % out_path)

def analyze_main(argv):
    ap = argparse.ArgumentParser(prog="esc_monitor.py analyze", description="Analise de um CSV gravado pelo monitor")
    ap.add_argument("csv")
    ap.add_argument("--plot", action="store_true", help="grava <csv>_analise.png")
    ap.add_argument("--from", dest="t_from", type=float, default=None, help="inicio do trecho (t_s)")
    ap.add_argument("--to", dest="t_to", type=float, default=None, help="fim do trecho (t_s)")
    ap.add_argument("--min-seg", type=float, default=3.0, help="ignora trechos de modo mais curtos (s)")
    args = ap.parse_args(argv)

    log = Log(args.csv, args.t_from, args.t_to)
    report_inputs(log)
    report_params(log)
    for mode, a, b in log.segments(args.min_seg):
        report_segment(log, mode, a, b)
    if args.plot:
        base = os.path.splitext(args.csv)[0]
        suffix = "" if args.t_from is None and args.t_to is None else "_%s_%s" % (
            int(args.t_from or log.t0), int(args.t_to or log.t1))
        plot(log, base + "_analise%s.png" % suffix)




def main():
    # A console that cannot show a character prints "?" instead of failing
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    if len(sys.argv) > 1 and sys.argv[1] == "analyze":
        if np is None:
            sys.exit("a analise precisa do numpy: pip install numpy")
        analyze_main(sys.argv[2:])
        return
    ap = argparse.ArgumentParser(description="Monitor ao vivo do rover (motores CAN + navegacao) via MAVLink")
    ap.add_argument("--conn", default="udp:0.0.0.0:14550",
                    help="udp:HOST:PORTA (escuta), udpout:HOST:PORTA ou tcp:HOST:PORTA (padrao: udp:0.0.0.0:14550)")
    ap.add_argument("--csv", default=None, help="arquivo CSV (padrao: Rover/tools/Logs/esc_log_DATA.csv)")
    ap.add_argument("--window", type=float, default=20.0, help="janela do grafico em segundos")
    ap.add_argument("--no-plot", action="store_true", help="sem grafico, so console e CSV")
    ap.add_argument("--duration", type=float, default=0, help="encerrar apos N segundos (0 = sem limite)")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="taxa (Hz) pedida para atitude, posicao, navegacao e PID (0 = nao pedir; padrao 10)")
    ap.add_argument("--no-params", action="store_true", help="nao baixar a lista de parametros no inicio")
    ap.add_argument("--kick", default=HERELINK_KICK,
                    help="com --conn udp, tambem manda heartbeat para HOST:PORTA para o HereLink "
                         "comecar a transmitir (padrao %s; 'none' desliga)" % HERELINK_KICK)
    args = ap.parse_args()

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Logs")
    os.makedirs(log_dir, exist_ok=True)
    csv_path = args.csv or os.path.join(log_dir, datetime.datetime.now().strftime("esc_log_%Y%m%d_%H%M%S.csv"))
    parm_path = csv_path[:-4] + ".parm" if csv_path.lower().endswith(".csv") else csv_path + ".parm"
    try:
        link = Link(args.conn, args.kick)
    except (OSError, ValueError) as exc:
        sys.exit("Nao foi possivel abrir %s: %s" % (args.conn, exc))
    print("Conectado em %s; gravando %s%s" % (args.conn, csv_path,
          ("; chamando HereLink em %s:%d" % link.kick) if link.kick else ""))

    store = Store(args.window)
    stop = threading.Event()
    # UTF-8 (with BOM so Excel reads it): the default Windows code page cannot
    # write the degree sign, and the reader thread died on it
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["hora_pc", "t_s", "boot_ms", "nome", "valor", "texto"])
        th = threading.Thread(target=reader, args=(link, store, writer, f, stop, args, parm_path), daemon=True)
        th.start()
        try:
            if args.no_plot:
                run_console(store, stop, args.duration)
            else:
                if args.duration:
                    threading.Timer(args.duration, stop.set).start()
                run_plot(store, stop, args.window)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            th.join(timeout=2)
    params = getattr(store, "params", None)
    if params is not None and params.values and not params.done:
        write_parm(parm_path, params.values)
        print("AVISO: parametros incompletos (%d de %s): o monitor foi fechado antes de terminar o download. "
              "Deixe rodando ate aparecer 'Parametros: N salvos'." % (len(params.values), params.count))
    print("CSV salvo em %s" % csv_path)
    print("Analise: python esc_monitor.py analyze %s --plot" % csv_path)


if __name__ == "__main__":
    main()
