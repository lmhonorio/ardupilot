#!/usr/bin/env python3
"""Interface local do rover. O navegador nao fala MAVLink: este processo
escuta a UDP, chama o HereLink e serve a pagina na rede local.

A pagina que abre e a de logs, com armar e desarmar. As outras trocam modo,
parametro ou preparam um ensaio (SCR_USER1). Nenhum ensaio arma sozinho.

  py -3 mode_panel.py
  py -3 mode_panel.py --conn udp:0.0.0.0:14551
  py -3 mode_panel.py --kick none

Feche o esc_monitor.py antes: os dois escutam a UDP 14550.
"""

import argparse
import collections
import json
import math
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import esc_monitor as mon

MSG_HEARTBEAT = 0
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_REQUEST_LIST = 21
MSG_PARAM_SET = 23
MSG_SET_MODE = 11
MSG_COMMAND_LONG = 76

CRC_EXTRA = {
    MSG_HEARTBEAT: 50,
    MSG_PARAM_REQUEST_READ: 214,
    MSG_PARAM_REQUEST_LIST: 159,
    MSG_PARAM_SET: 168,
    MSG_SET_MODE: 89,
    MSG_COMMAND_LONG: 152,
}

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8

# 255 is SYSID_MYGCS on this rover (the id Mission Planner uses). With the
# planner closed, commands from any other id are easy for the link to drop.
GCS_SYSID = 255
GCS_COMPID = 190
COMMAND_WAIT_S = 4.0
COMMAND_RETRY_S = 0.4
RATE_HZ = 10.0
WINDOW_S = 20.0
HERELINK_KICK = "192.168.43.1:14552"
HTTP_PORT = 8765
HMI_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hmi")

MODES = (
    ("MANUAL", 0),
    ("HOLD", 4),
    ("STEERING", 3),
    ("ACRO", 1),
    ("LOITER", 5),
    ("CIRCLE", 9),
    ("AUTO", 10),
    ("RTL", 11),
    ("GUIDED", 15),
    ("SIMPLE", 7),
    ("SMART_RTL", 12),
)
MODE_ID = {name: number for name, number in MODES}
MODE_NAME = {number: name for name, number in MODES}

ACK_TEXT = {
    0: "aceito",
    1: "recusado por agora",
    2: "negado",
    3: "sem suporte",
    4: "falhou",
    5: "em andamento",
    6: "cancelado",
}

SERIES = {
    "ESC1_I", "ESC2_I", "ESC3_I", "ESC4_I",
    "ESC1_ICMD", "ESC2_ICMD", "ESC3_ICMD", "ESC4_ICMD",
    "ESC1_RPM", "ESC2_RPM", "ESC3_RPM", "ESC4_RPM",
    "ESC_TGT_L", "ESC_TGT_R",
    "PIDS_des", "PIDS_ach", "ATT_yawrate",
    "NAV_xtrack",
    "ESC1_T", "ESC2_T", "ESC3_T", "ESC4_T",
    "BAT_I",
}
READOUT = ("VFR_gspd", "NAV_xtrack", "BAT_V", "BAT_I")

GROUPS = [
    {"id": "direcao", "title": "Direção", "params": [
        {"name": "ATC_STR_RAT_FF", "unit": "", "step": "0.001",
         "desc": "Parte da taxa pedida que segue direto para o comando, antes do P e do I."},
        {"name": "ATC_STR_RAT_P", "unit": "", "step": "0.001",
         "desc": "Correção proporcional do erro de taxa de guinada, em graus/s."},
        {"name": "ATC_STR_RAT_I", "unit": "", "step": "0.001",
         "desc": "Correção que cresce enquanto o erro de guinada não zera."},
        {"name": "ATC_STR_RAT_D", "unit": "", "step": "0.0001",
         "desc": "Freia a variação do erro. No rover costuma ficar baixo."},
        {"name": "ATC_STR_RAT_IMAX", "unit": "graus/s", "step": "0.1",
         "desc": "Teto do termo integral."},
        {"name": "ATC_STR_RAT_MAX", "unit": "graus/s", "step": "1",
         "desc": "Maior taxa de guinada que o controlador pode pedir."},
        {"name": "ATC_STR_RAT_FLTT", "unit": "Hz", "step": "0.1",
         "desc": "Filtro da taxa desejada."},
        {"name": "ATC_STR_RAT_FLTE", "unit": "Hz", "step": "0.1",
         "desc": "Filtro do erro de taxa."},
    ]},
    {"id": "velocidade", "title": "Velocidade", "params": [
        {"name": "ATC_SPEED_P", "unit": "", "step": "0.001",
         "desc": "Correção proporcional do erro de velocidade."},
        {"name": "ATC_SPEED_I", "unit": "", "step": "0.001",
         "desc": "Integral da velocidade. Cobre o erro que o P deixa."},
        {"name": "ATC_SPEED_D", "unit": "", "step": "0.001",
         "desc": "Derivativo da velocidade."},
        {"name": "ATC_SPEED_FF", "unit": "", "step": "0.001",
         "desc": "Feedforward da velocidade pedida."},
        {"name": "ATC_SPEED_IMAX", "unit": "", "step": "0.1",
         "desc": "Teto do integral de velocidade."},
        {"name": "CRUISE_SPEED", "unit": "m/s", "step": "0.1",
         "desc": "Velocidade de cruzeiro."},
        {"name": "CRUISE_THROTTLE", "unit": "%", "step": "1",
         "desc": "Acelerador que mantém a velocidade de cruzeiro."},
        {"name": "WP_SPEED", "unit": "m/s", "step": "0.1",
         "desc": "Velocidade usada no modo AUTO."},
        {"name": "MOT_SLEWRATE", "unit": "%/s", "step": "1",
         "desc": "Limite de variação do acelerador. 0 não limita."},
    ]},
    {"id": "motores", "title": "Motores", "params": [
        {"name": "ESC_CAN", "unit": "", "step": "1",
         "desc": "1 liga o controle dos motores pela CAN."},
        {"name": "ESC_MODE", "unit": "", "step": "1",
         "desc": "0 é o modo em que cada roda fecha o próprio PI. Os ensaios 4 e 5 exigem 0."},
        {"name": "ESC_RPM", "unit": "rpm", "step": "1",
         "desc": "Teto de rotação pedido às rodas."},
        {"name": "ESC_CUR", "unit": "A", "step": "0.5",
         "desc": "Teto de corrente de cada motor. O ensaio de roda usa esse valor no pulso."},
        {"name": "ESC_GCS_HZ", "unit": "Hz", "step": "1",
         "desc": "Taxa com que corrente, rotação e temperatura chegam nos gráficos."},
        {"name": "ESC_P", "unit": "", "step": "0.001",
         "desc": "Ganho P médio das rodas. Uma roda com Pn = 0 usa este valor."},
        {"name": "ESC_I", "unit": "", "step": "0.001",
         "desc": "Ganho I médio das rodas."},
        {"name": "ESC_FF", "unit": "", "step": "0.0001",
         "desc": "Feedforward médio, em corrente por rpm."},
        {"name": "ESC_IMAX", "unit": "", "step": "0.5",
         "desc": "Teto do integral de cada roda."},
        {"name": "ESC_ACC", "unit": "rpm/s", "step": "10",
         "desc": "Rampa da rotação pedida. 0 não limita."},
        {"name": "ESC_REV", "unit": "", "step": "1",
         "desc": "Bits que invertem o sentido de cada motor. 1, 2, 4 e 8 são as rodas 1 a 4."},
        {"name": "ESC_LMASK", "unit": "", "step": "1",
         "desc": "Bits dos motores do lado esquerdo. 6 é o par 2+3; 9 é o par 1+4."},
        {"name": "ESC_WT_RELAY", "unit": "%", "step": "1",
         "desc": "Fração de ESC_CUR usada no pulso do ensaio das rodas."},
        {"name": "ESC_WT_RPM", "unit": "rpm", "step": "1",
         "desc": "Patamar alto de rotação do ensaio das rodas."},
        {"name": "ESC_P1", "unit": "", "step": "0.001", "desc": "P da roda 1, gravado pela calibração."},
        {"name": "ESC_I1", "unit": "", "step": "0.001", "desc": "I da roda 1."},
        {"name": "ESC_FF1", "unit": "", "step": "0.0001", "desc": "Feedforward da roda 1."},
        {"name": "ESC_P2", "unit": "", "step": "0.001", "desc": "P da roda 2."},
        {"name": "ESC_I2", "unit": "", "step": "0.001", "desc": "I da roda 2."},
        {"name": "ESC_FF2", "unit": "", "step": "0.0001", "desc": "Feedforward da roda 2."},
        {"name": "ESC_P3", "unit": "", "step": "0.001", "desc": "P da roda 3."},
        {"name": "ESC_I3", "unit": "", "step": "0.001", "desc": "I da roda 3."},
        {"name": "ESC_FF3", "unit": "", "step": "0.0001", "desc": "Feedforward da roda 3."},
        {"name": "ESC_P4", "unit": "", "step": "0.001", "desc": "P da roda 4."},
        {"name": "ESC_I4", "unit": "", "step": "0.001", "desc": "I da roda 4."},
        {"name": "ESC_FF4", "unit": "", "step": "0.0001", "desc": "Feedforward da roda 4."},
    ]},
    {"id": "missao", "title": "Missão", "params": [
        {"name": "WP_RADIUS", "unit": "m", "step": "0.1",
         "desc": "Distância em que o waypoint conta como alcançado."},
        {"name": "WP_ACCEL", "unit": "m/s²", "step": "0.1",
         "desc": "Aceleração ao seguir a missão."},
        {"name": "WP_PIVOT_ANGLE", "unit": "graus", "step": "1",
         "desc": "Erro de proa a partir do qual o rover gira parado antes de seguir."},
        {"name": "WP_PIVOT_RATE", "unit": "graus/s", "step": "1",
         "desc": "Taxa desse giro parado."},
        {"name": "TURN_RADIUS", "unit": "m", "step": "0.1",
         "desc": "Raio mínimo de curva."},
        {"name": "CIRC_RADIUS", "unit": "m", "step": "0.5",
         "desc": "Raio do modo CIRCLE."},
        {"name": "CIRC_SPEED", "unit": "m/s", "step": "0.1",
         "desc": "Velocidade no CIRCLE. 0 usa WP_SPEED."},
        {"name": "GCS_PID_MASK", "unit": "", "step": "1",
         "desc": "3 envia o PID de direção e o de velocidade para o gráfico de guinada."},
    ]},
    {"id": "script", "title": "Script", "params": [
        {"name": "SCR_USER1", "unit": "", "step": "1",
         "desc": "Seletor de ensaio: 0 normal, 1 QuickTune, 2 yaw, 3 Circle, 4 rodas no chão, 5 rodas no ar."},
        {"name": "SCR_USER2", "unit": "", "step": "0.001",
         "desc": "P de yaw gravado pelo ensaio."},
        {"name": "SCR_USER3", "unit": "", "step": "0.001",
         "desc": "I de yaw gravado pelo ensaio."},
        {"name": "SCR_USER5", "unit": "", "step": "1",
         "desc": "Tipo de veículo do mixer. Este rover usa 2."},
        {"name": "YAWT_MAX_ANG", "unit": "graus", "step": "1",
         "desc": "Erro de proa em que o relé de yaw inverte o comando."},
        {"name": "YAWT_MAX_ANG_ERR", "unit": "graus", "step": "1",
         "desc": "Erro de proa que aborta o ensaio de yaw."},
        {"name": "YAWT_MIN_OUT", "unit": "%", "step": "1",
         "desc": "Comando com que o relé de yaw começa."},
        {"name": "YAWT_MAX_OUT", "unit": "%", "step": "1",
         "desc": "Maior comando que o relé de yaw pode usar."},
        {"name": "YAWT_MAX_CYCLE", "unit": "", "step": "1",
         "desc": "Ciclos do relé, contando os dois primeiros, que são descartados."},
        {"name": "YAWT_TIMEOUT", "unit": "s", "step": "1",
         "desc": "Tempo sem girar até a verificação de yaw parar."},
        {"name": "YAWT_GAIN_MULT", "unit": "", "step": "0.1",
         "desc": "Multiplicador do P e do I na hora de alinhar a proa."},
        {"name": "RTUN_ENABLE", "unit": "", "step": "1",
         "desc": "1 liga o QuickTune."},
        {"name": "RTUN_AXES", "unit": "", "step": "1",
         "desc": "3 ajusta direção e velocidade."},
        {"name": "RTUN_AUTO_SAVE", "unit": "s", "step": "1",
         "desc": "Espera depois do QuickTune antes de gravar os ganhos."},
    ]},
]

PROCESSES = [
    {"id": "ground", "title": "Calibração das rodas no chão", "mode": "HOLD",
     "params": (("ESC_CAN", 1), ("ESC_MODE", 0), ("SCR_USER1", 4)),
     "text": "Deixa o ensaio sincronizado pronto. O rover precisa de alguns metros livres à frente e atrás, com os sticks no centro. Os ganhos das rodas são gravados pelo script depois que você armar. Ao terminar, SCR_USER1 volta sozinho para 0.",
     "done": "Calibração no chão preparada. Pode armar."},
    {"id": "air", "title": "Calibração com as rodas no ar", "mode": "HOLD",
     "params": (("ESC_CAN", 1), ("ESC_MODE", 0), ("SCR_USER1", 5)),
     "text": "Uma roda por vez, só com o rover levantado. Depois de preparar, arme. Não use este ensaio no chão.",
     "done": "Calibração no ar preparada. Pode armar."},
    {"id": "quick", "title": "QuickTune", "mode": "CIRCLE",
     "params": (("RTUN_ENABLE", 1), ("RTUN_AXES", 3), ("SCR_USER1", 1)),
     "text": "Prepara o ajuste de direção e velocidade em CIRCLE. O QuickTune começa quando o rover arma. Confira o raio e a área antes.",
     "done": "QuickTune preparado. Pode armar."},
    {"id": "circle", "title": "Relé em Circle", "mode": "CIRCLE",
     "params": (("SCR_USER1", 3),),
     "text": "Refina ATC_STR_RAT_P e ATC_STR_RAT_I girando em CIRCLE. Depois de preparar, arme.",
     "done": "Relé em Circle preparado. Pode armar."},
    {"id": "yaw", "title": "Relé de yaw", "mode": "HOLD",
     "params": (("SCR_USER1", 2),),
     "text": "O rover gira no lugar para medir o P e o I de yaw. Área livre em volta. Depois de preparar, arme.",
     "done": "Relé de yaw preparado. Pode armar."},
    {"id": "normal", "title": "Voltar ao normal", "mode": None,
     "params": (("SCR_USER1", 0),),
     "text": "Tira o rover de qualquer ensaio. Não muda o modo de voo. Também serve para abortar.",
     "done": "SCR_USER1 = 0."},
]
PROCESS_BY_ID = {item["id"]: item for item in PROCESSES}
PARAM_NAMES = tuple(param["name"] for group in GROUPS for param in group["params"])
PARAM_OK = set(PARAM_NAMES)


def x25_crc(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class Encoder:
    def __init__(self):
        self.seq = 0

    def pack(self, msgid, payload):
        header = bytes([len(payload), self.seq & 0xFF, GCS_SYSID, GCS_COMPID, msgid])
        self.seq += 1
        crc = x25_crc(header + payload + bytes([CRC_EXTRA[msgid]]))
        return b"\xfe" + header + payload + struct.pack("<H", crc)

    def heartbeat(self):
        return self.pack(MSG_HEARTBEAT, struct.pack("<IBBBBB", 0, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 3))

    def command_long(self, sysid, compid, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        payload = struct.pack("<7fHBBB", float(p1), float(p2), float(p3), float(p4),
                              float(p5), float(p6), float(p7), int(command), sysid, compid, 0)
        return self.pack(MSG_COMMAND_LONG, payload)

    def command_mode(self, sysid, compid, custom_mode):
        base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        return self.command_long(sysid, compid, MAV_CMD_DO_SET_MODE, base, float(custom_mode))

    def mode_frame(self, sysid, custom_mode):
        return self.pack(MSG_SET_MODE, struct.pack(
            "<IBB", int(custom_mode), sysid, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))

    def set_interval(self, sysid, compid, msgid, hz):
        interval = int(1e6 / hz) if hz > 0 else -1
        return self.command_long(sysid, compid, MAV_CMD_SET_MESSAGE_INTERVAL, float(msgid), float(interval))

    def param_list(self, sysid, compid):
        return self.pack(MSG_PARAM_REQUEST_LIST, struct.pack("<BB", sysid, compid))

    def param_read_name(self, sysid, compid, name):
        return self.pack(MSG_PARAM_REQUEST_READ, struct.pack("<hBB16s", -1, sysid, compid, name.encode()))

    def param_set(self, sysid, compid, name, value):
        payload = struct.pack("<fBB16sB", float(value), sysid, compid, name.encode()[:16], 9)
        return self.pack(MSG_PARAM_SET, payload)

    def arm(self, sysid, compid, arm):
        return self.command_long(sysid, compid, MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if arm else 0.0)


class Link:
    def __init__(self, conn, kick=HERELINK_KICK):
        import socket
        self.socket = socket
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
            try:
                self.sock.bind(self.addr)
            except OSError as exc:
                raise OSError("%s; a porta %d ja esta em uso (esc_monitor ou outra copia desta pagina). "
                              "Feche o outro programa ou use --conn udp:0.0.0.0:<porta>" % (exc, self.addr[1]))
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
                self.peer = addr
            return data
        except self.socket.timeout:
            return b""

    def send(self, data):
        try:
            if self.kind == "tcp":
                self.sock.sendall(data)
                return
            if self.peer is not None:
                self.sock.sendto(data, self.peer)
            if self.kick is not None and self.kick != self.peer:
                self.sock.sendto(data, self.kick)
        except OSError:
            pass


class Vehicle:
    def __init__(self):
        self.lock = threading.RLock()
        self.t0 = time.monotonic()
        self.sysid = None
        self.compid = None
        self.mode = None
        self.armed = None
        self.last_hb = 0.0
        self.jobs = collections.deque()
        self.note = "aguardando o rover"
        self.text = ""
        self.messages = collections.deque(maxlen=400)
        self.status_parts = {}   # id -> [severity, {seq: text}, monotonic]
        self.params = {}
        self.latest = {}
        self.series = collections.defaultdict(lambda: collections.deque(maxlen=500))
        self.list_sent = False


def mode_name(number):
    if number is None:
        return None
    return MODE_NAME.get(number, "modo %s" % number)


def push_message(vehicle, sev, text):
    """Append one line to the messages screen. Caller holds vehicle.lock."""
    text = (text or "").strip()
    if not text:
        return
    if vehicle.messages and vehicle.messages[-1]["text"] == text:
        return
    vehicle.text = text
    vehicle.messages.append({
        "t": time.strftime("%H:%M:%S"),
        "sev": int(sev),
        "text": text,
    })


def add_value(vehicle, name, value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return
    if not math.isfinite(value):
        return
    t = round(time.monotonic() - vehicle.t0, 2)
    with vehicle.lock:
        vehicle.latest[name] = value
        if name in SERIES:
            vehicle.series[name].append((t, value))


def reader(link, vehicle, stop, tx, enc):
    parser = mon.MavlinkParser()
    last_hb = 0.0
    last_streams = -100.0
    last_param = 0.0
    recent = {}
    while not stop.is_set():
        now = time.monotonic()
        if now - last_hb >= 1.0:
            last_hb = now
            with tx:
                link.send(enc.heartbeat())
        with vehicle.lock:
            sysid, compid = vehicle.sysid, vehicle.compid
        if sysid is not None and now - last_streams >= 15.0:
            last_streams = now
            with tx:
                for msgid, factor in mon.STREAMS.items():
                    link.send(enc.set_interval(sysid, compid, msgid, RATE_HZ * factor))
        service_jobs(link, vehicle, tx, enc)
        if sysid is not None and now - last_param >= 2.0:
            last_param = now
            with vehicle.lock:
                missing = [name for name in PARAM_NAMES if name not in vehicle.params][:4]
                send_list = not vehicle.list_sent
                vehicle.list_sent = True
            with tx:
                if send_list:
                    link.send(enc.param_list(sysid, compid))
                for name in missing:
                    link.send(enc.param_read_name(sysid, compid, name))
        try:
            data = link.recv()
        except OSError:
            break
        for msgid, sysid_rx, compid_rx, payload in parser.feed(data):
            if sysid_rx == GCS_SYSID:
                continue
            if msgid != mon.MSG_HEARTBEAT:
                key = (msgid, payload)
                seen = recent.get(key)
                if seen is not None and now - seen < 0.4:
                    continue
                recent[key] = now
                if len(recent) > 500:
                    recent.clear()
            msg = mon.decode(msgid, payload)
            if msgid == mon.MSG_HEARTBEAT:
                if not msg or msg["type"] == MAV_TYPE_GCS or msg["autopilot"] == MAV_AUTOPILOT_INVALID:
                    continue
                with vehicle.lock:
                    if vehicle.sysid is None:
                        vehicle.sysid = sysid_rx
                        vehicle.compid = compid_rx
                    elif sysid_rx != vehicle.sysid:
                        continue
                    was_armed = vehicle.armed
                    was_mode = vehicle.mode
                    vehicle.mode = msg["mode"]
                    vehicle.armed = msg["armed"]
                    vehicle.last_hb = time.monotonic()
                    if vehicle.note == "aguardando o rover":
                        vehicle.note = ""
                    if was_armed is None or was_armed != msg["armed"]:
                        push_message(vehicle, 6, "ARMADO" if msg["armed"] else "DESARMADO")
                    if was_mode != msg["mode"]:
                        push_message(vehicle, 6, "Modo %s" % (mode_name(msg["mode"]) or msg["mode"]))
                continue
            if msgid == mon.MSG_STATUSTEXT:
                # Not tied to the first heartbeat. Mission Planner lists every
                # STATUSTEXT from the Pixhawk, including long lines sent in pieces.
                for sev, text in take_statustext(vehicle, payload):
                    with vehicle.lock:
                        push_message(vehicle, sev, text)
                continue
            with vehicle.lock:
                if vehicle.sysid is not None and sysid_rx != vehicle.sysid:
                    continue
            if not msg:
                continue
            if msgid == mon.MSG_COMMAND_ACK:
                with vehicle.lock:
                    fail_job(vehicle, msg["command"], msg["result"])
            elif msgid == mon.MSG_PARAM_VALUE:
                with vehicle.lock:
                    vehicle.params[msg["name"]] = msg["value"]
            elif msgid == mon.MSG_NAMED_VALUE_FLOAT:
                add_value(vehicle, msg["name"], msg["value"])
            elif "values" in msg:
                for name, value in msg["values"]:
                    add_value(vehicle, name, value)


def take_statustext(vehicle, payload):
    """Join MAVLink STATUSTEXT chunks the way Mission Planner does.

    A short message has id 0 and is shown at once. A long one is split into
    50-character pieces that share an id and increase chunk_seq.
    """
    if not payload:
        return []
    sev = int(payload[0])
    field = payload[1:51]
    cut = field.find(b"\0")
    raw = field if cut < 0 else field[:cut]
    try:
        text = raw.decode("utf-8")
    except UnicodeError:
        text = raw.decode("utf-8", "replace")
    mid = payload[51] | (payload[52] << 8) if len(payload) >= 53 else 0
    seq = payload[53] if len(payload) >= 54 else 0
    now = time.monotonic()
    out = []
    with vehicle.lock:
        if mid == 0:
            if text.strip():
                out.append((sev, text.strip()))
        else:
            slot = vehicle.status_parts.get(mid)
            if slot is None:
                slot = [sev, {}, now]
                vehicle.status_parts[mid] = slot
            slot[0] = sev
            slot[1][seq] = text
            slot[2] = now
            if len(raw) < 50:
                parts = [slot[1][i] for i in sorted(slot[1])]
                joined = "".join(parts).strip()
                if joined:
                    out.append((sev, joined))
                del vehicle.status_parts[mid]
        for key, slot in list(vehicle.status_parts.items()):
            if now - slot[2] <= 2.0:
                continue
            parts = [slot[1][i] for i in sorted(slot[1])]
            joined = "".join(parts).strip()
            if joined:
                out.append((slot[0], joined))
            del vehicle.status_parts[key]
    return out


def job_label(job):
    kind = job["kind"]
    if kind == "mode":
        return job["name"]
    if kind == "param":
        return "%s = %s" % (job["name"], ("%.6g" % job["value"]))
    return "armar" if job["arm"] else "desarmar"


def job_done(job):
    kind = job["kind"]
    if kind == "note":
        return job["text"]
    if kind == "mode":
        return "%s aceito" % job["name"]
    if kind == "param":
        return "%s = %s" % (job["name"], ("%.6g" % job["value"]))
    return "armação aceita" if job["arm"] else "desarme aceito"


def enqueue(vehicle, jobs, done):
    with vehicle.lock:
        if vehicle.sysid is None:
            vehicle.note = "sem rover"
            return False
        vehicle.jobs.clear()
        for job in jobs:
            job["deadline"] = None
            job["sent"] = 0.0
            vehicle.jobs.append(job)
        vehicle.jobs.append({"kind": "note", "text": done, "deadline": None, "sent": 0.0})
        vehicle.note = "pedindo %s" % job_label(jobs[0])
    return True


def job_succeeded(vehicle, job):
    kind = job["kind"]
    if kind == "note":
        return True
    if kind == "mode":
        return vehicle.mode == job["number"]
    if kind == "arm":
        return vehicle.armed == job["arm"]
    if kind == "param":
        got = vehicle.params.get(job["name"])
        if got is None:
            return False
        return abs(got - job["value"]) <= max(1e-4, abs(job["value"]) * 1e-4)
    return False


def fail_job(vehicle, command, result):
    """Abort the current job when the board refuses it. Caller holds the lock."""
    if result in (0, 1, 3, 5) or not vehicle.jobs:
        return
    job = vehicle.jobs[0]
    if job["kind"] == "mode" and command != MAV_CMD_DO_SET_MODE:
        return
    if job["kind"] == "arm" and command != MAV_CMD_COMPONENT_ARM_DISARM:
        return
    if job["kind"] not in ("mode", "arm"):
        return
    vehicle.note = "%s %s" % (job_label(job), ACK_TEXT.get(result, "resposta %d" % result))
    push_message(vehicle, 3, vehicle.note)
    vehicle.jobs.clear()
    print(vehicle.note, flush=True)


def send_job(link, enc, sysid, compid, job):
    kind = job["kind"]
    if kind == "mode":
        link.send(enc.command_mode(sysid, compid, job["number"]))
        link.send(enc.mode_frame(sysid, job["number"]))
        time.sleep(0.01)
        link.send(enc.mode_frame(sysid, job["number"]))
    elif kind == "param":
        link.send(enc.param_set(sysid, compid, job["name"], job["value"]))
    elif kind == "arm":
        link.send(enc.arm(sysid, compid, job["arm"]))


def service_jobs(link, vehicle, tx, enc):
    now = time.monotonic()
    with vehicle.lock:
        if not vehicle.jobs or vehicle.sysid is None:
            return
        job = vehicle.jobs[0]
        if job_succeeded(vehicle, job):
            vehicle.jobs.popleft()
            if vehicle.jobs and vehicle.jobs[0]["kind"] != "note":
                vehicle.note = "pedindo %s" % job_label(vehicle.jobs[0])
            elif vehicle.jobs:
                vehicle.note = vehicle.jobs.popleft()["text"]
                print(vehicle.note, flush=True)
            else:
                vehicle.note = job_done(job)
                print(vehicle.note, flush=True)
            return
        if job["deadline"] is None:
            job["deadline"] = now + COMMAND_WAIT_S
        elif now > job["deadline"]:
            vehicle.note = "sem resposta: %s" % job_label(job)
            push_message(vehicle, 4, vehicle.note)
            print(vehicle.note, flush=True)
            vehicle.jobs.clear()
            return
        if now - job["sent"] < COMMAND_RETRY_S:
            return
        job["sent"] = now
        sysid, compid = vehicle.sysid, vehicle.compid or 1
        outgoing = job
    with tx:
        send_job(link, enc, sysid, compid, outgoing)


def ask_mode(link, vehicle, tx, enc, name, number):
    return enqueue(vehicle, [{"kind": "mode", "name": name, "number": number}], "%s aceito" % name)


def set_param(link, vehicle, tx, enc, name, value, quiet=False):
    text = "%s = %s" % (name, ("%.6g" % value))
    return enqueue(vehicle, [{"kind": "param", "name": name, "value": float(value)}], text)


def set_arm(link, vehicle, tx, enc, arm):
    done = "armação aceita" if arm else "desarme aceito"
    return enqueue(vehicle, [{"kind": "arm", "arm": bool(arm)}], done)


def run_process(link, vehicle, tx, enc, proc):
    jobs = []
    if proc.get("mode"):
        jobs.append({"kind": "mode", "name": proc["mode"], "number": MODE_ID[proc["mode"]]})
    for name, value in proc["params"]:
        jobs.append({"kind": "param", "name": name, "value": float(value)})
    return enqueue(vehicle, jobs, proc["done"])


def snapshot(link, vehicle):
    with vehicle.lock:
        mode = vehicle.mode
        armed = vehicle.armed
        last = vehicle.last_hb
        seen = vehicle.sysid is not None
        note = vehicle.note
        text = vehicle.text
        messages = list(vehicle.messages)
        params = {name: vehicle.params[name] for name in PARAM_NAMES if name in vehicle.params}
        latest = {name: vehicle.latest[name] for name in READOUT if name in vehicle.latest}
    alive = seen and (time.monotonic() - last) < 3.0
    if not seen:
        link_text = "escutando UDP %d" % link.addr[1]
    elif alive:
        link_text = "rover ligado"
    else:
        link_text = "sem sinal"
    return {
        "seen": seen,
        "alive": alive,
        "mode": mode_name(mode) if seen else None,
        "armed": armed if seen else None,
        "link": link_text,
        "note": note,
        "text": text,
        "messages": messages,
        "params": params,
        "latest": latest,
    }


def telemetry(vehicle):
    now = time.monotonic() - vehicle.t0
    cut = now - WINDOW_S
    with vehicle.lock:
        raw = {name: [point for point in series if point[0] >= cut] for name, series in vehicle.series.items()}
    out = {}
    for name, points in raw.items():
        if len(points) > 400:
            step = len(points) / 400.0
            points = [points[int(i * step)] for i in range(400)]
        out[name] = [[point[0], round(point[1], 3)] for point in points]
    return {"t": round(now, 2), "window": WINDOW_S, "series": out}


def catalog():
    def sets_line(proc):
        parts = []
        if proc.get("mode"):
            parts.append("modo " + proc["mode"])
        parts.extend("%s=%s" % (name, value) for name, value in proc["params"])
        return "   ".join(parts)

    return {
        "modes": [{"name": name, "id": number} for name, number in MODES],
        "groups": GROUPS,
        "processes": [{
            "id": proc["id"],
            "title": proc["title"],
            "text": proc["text"],
            "sets": sets_line(proc),
        } for proc in PROCESSES],
    }


def json_body(payload, code=200):
    body = json.dumps(payload).encode("utf-8")
    return code, "application/json", body


class LocalServer(ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self):
        import socket
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            try:
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            except OSError:
                pass
        super().server_bind()


def make_handler(link, vehicle, tx, enc):
    root = os.path.realpath(HMI_DIR)
    types = {
        ".html": "text/html; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".js": "text/javascript; charset=utf-8",
        ".png": "image/png",
    }

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self):
            try:
                n = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                return None
            if n < 0 or n > 8000:
                return None
            raw = self.rfile.read(n) if n else b"{}"
            try:
                return json.loads(raw.decode("utf-8"))
            except (UnicodeError, json.JSONDecodeError):
                return None

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/state":
                code, ctype, body = json_body(snapshot(link, vehicle))
                self._send(code, ctype, body)
                return
            if path == "/telemetry":
                code, ctype, body = json_body(telemetry(vehicle))
                self._send(code, ctype, body)
                return
            if path == "/catalog":
                code, ctype, body = json_body(catalog())
                self._send(code, ctype, body)
                return
            rel = "index.html" if path == "/" else path.lstrip("/").replace("\\", "/")
            if not rel or ".." in rel.split("/"):
                self._send(404, "text/plain; charset=utf-8", b"")
                return
            full = os.path.realpath(os.path.join(root, rel))
            try:
                inside = os.path.commonpath([full, root]) == root
            except ValueError:
                inside = False
            if not inside or not os.path.isfile(full):
                self._send(404, "text/plain; charset=utf-8", b"")
                return
            with open(full, "rb") as handle:
                body = handle.read()
            self._send(200, types.get(os.path.splitext(full)[1], "application/octet-stream"), body)

        def do_POST(self):
            path = urlparse(self.path).path
            data = self._read_json()
            if data is None:
                self._send(400, "application/json", b'{"ok":false}')
                return
            if path == "/mode":
                name = data.get("name")
                number = MODE_ID.get(name)
                if number is None:
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = ask_mode(link, vehicle, tx, enc, name, number)
            elif path == "/param":
                name = data.get("name")
                try:
                    value = float(data.get("value"))
                except (TypeError, ValueError):
                    value = None
                if name not in PARAM_OK or value is None or not math.isfinite(value):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = set_param(link, vehicle, tx, enc, name, value)
            elif path == "/arm":
                if not isinstance(data.get("arm"), bool):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = set_arm(link, vehicle, tx, enc, data["arm"])
            elif path == "/process":
                proc = PROCESS_BY_ID.get(data.get("id"))
                if proc is None:
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = run_process(link, vehicle, tx, enc, proc)
            else:
                self._send(404, "text/plain; charset=utf-8", b"")
                return
            code, ctype, body = json_body({"ok": ok, "note": snapshot(link, vehicle)["note"]}, 200 if ok else 409)
            self._send(code, ctype, body)

    return Handler


def lan_addresses():
    """IPv4 addresses of this computer, skipping loopback and link-local."""
    import socket
    found = []
    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except OSError:
        return found
    for info in infos:
        ip = info[4][0]
        if ip.startswith("127.") or ip.startswith("169.254.") or ip in found:
            continue
        found.append(ip)
    return found


def main(argv=None):
    ap = argparse.ArgumentParser(description="Interface local do rover")
    ap.add_argument("--conn", default="udp:0.0.0.0:14550",
                    help="udp:HOST:PORTA, udpout:HOST:PORTA ou tcp:HOST:PORTA")
    ap.add_argument("--kick", default=HERELINK_KICK,
                    help="com udp, manda heartbeat para HOST:PORTA para o HereLink transmitir "
                         "(padrao %s; 'none' desliga)" % HERELINK_KICK)
    ap.add_argument("--http", type=int, default=HTTP_PORT,
                    help="porta da pagina neste computador e no celular na mesma rede (padrao %d)" % HTTP_PORT)
    args = ap.parse_args(argv)

    try:
        link = Link(args.conn, args.kick)
    except (OSError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1

    vehicle = Vehicle()
    stop = threading.Event()
    tx = threading.Lock()
    enc = Encoder()
    threading.Thread(target=reader, args=(link, vehicle, stop, tx, enc), daemon=True).start()

    try:
        httpd = LocalServer(("0.0.0.0", args.http), make_handler(link, vehicle, tx, enc))
    except OSError as exc:
        stop.set()
        link.sock.close()
        print("Nao foi possivel abrir a pagina na porta %d: %s" % (args.http, exc), file=sys.stderr)
        return 1

    print("Neste computador: http://127.0.0.1:%d  (GCS %d)" % (args.http, GCS_SYSID), flush=True)
    for ip in lan_addresses():
        print("No celular, mesma rede: http://%s:%d" % (ip, args.http), flush=True)
    print("MAVLink em %s. Ctrl+C encerra." % args.conn, flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        httpd.server_close()
        try:
            link.sock.close()
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
