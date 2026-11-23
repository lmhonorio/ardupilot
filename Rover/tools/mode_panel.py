#!/usr/bin/env python3
"""Interface local do rover. O navegador nao fala MAVLink: este processo
escuta a UDP, chama o HereLink e serve a pagina na rede local.

A pagina que abre e a de logs, com armar e desarmar. As outras trocam modo,
parametro ou preparam um ensaio (SCR_USER1). Nenhum ensaio arma sozinho.
O CSV so abre quando a pagina pede e grava o mesmo conteudo do log do esc_monitor:
telemetria, mensagens, parametros e mudancas de modo e de armamento.
A aba Scripts lista, copia, envia e exclui os .lua de /APM/scripts e
/APM/scripts/modules pelo MAVFtp, como o Mission Planner em Config, MAVFtp.
Também limpa /APM/STRG_BAK (backups de storage) sem listar os arquivos na tela.

  py -3 mode_panel.py
  py -3 mode_panel.py --conn udp:0.0.0.0:14551
  py -3 mode_panel.py --kick none

Feche o esc_monitor.py antes: os dois escutam a UDP 14550.
"""

import argparse
import base64
import collections
import csv
import datetime
import json
import math
import os
import re
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import esc_monitor as mon

MSG_HEARTBEAT = 0
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_REQUEST_LIST = 21
MSG_PARAM_SET = 23
MSG_SET_MODE = 11
MSG_COMMAND_LONG = 76
MSG_FILE_TRANSFER_PROTOCOL = 110

CRC_EXTRA = {
    MSG_HEARTBEAT: 50,
    MSG_PARAM_REQUEST_READ: 214,
    MSG_PARAM_REQUEST_LIST: 159,
    MSG_PARAM_SET: 168,
    MSG_SET_MODE: 89,
    MSG_COMMAND_LONG: 152,
    MSG_FILE_TRANSFER_PROTOCOL: 84,
}

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_PREFLIGHT_STORAGE = 245
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
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


LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
LOG_BASE = "esc_log"
LOG_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,60}$")


def log_filename(raw):
    """Same base as esc_monitor: esc_log_YYYYMMDD_HHMMSS.csv, or esc_log_<label>.csv."""
    name = re.sub(r"\s+", "_", (raw or "").strip())
    if name.lower().endswith(".csv"):
        name = name[:-4]
    if not name:
        return datetime.datetime.now().strftime(LOG_BASE + "_%Y%m%d_%H%M%S.csv")
    # Accept a full esc_log_* name or only the suffix after the shared base.
    low = name.lower()
    if low.startswith(LOG_BASE + "_"):
        name = name[len(LOG_BASE) + 1:]
    elif low == LOG_BASE:
        return datetime.datetime.now().strftime(LOG_BASE + "_%Y%m%d_%H%M%S.csv")
    if not LOG_NAME.match(name):
        return None
    return "%s_%s.csv" % (LOG_BASE, name)


class CanLog:
    """Same columns as the esc_monitor CSV."""

    def __init__(self, path, t0):
        self.path = path
        self.t0 = t0
        self.lock = threading.Lock()
        new = (not os.path.exists(path)) or os.path.getsize(path) == 0
        self.file = open(path, "a", newline="", encoding="utf-8-sig" if new else "utf-8")
        self.writer = csv.writer(self.file)
        self.rows = 0
        self.closed = False
        if new:
            self.writer.writerow(["hora_pc", "t_s", "boot_ms", "nome", "valor", "texto"])
            self.file.flush()

    def write(self, name, value, boot_ms="", text=""):
        wall = datetime.datetime.now().isoformat(timespec="milliseconds")
        t_s = "%.3f" % (time.monotonic() - self.t0)
        if isinstance(value, float):
            if name in ("GPI_lat", "GPI_lon"):
                shown = "%.7f" % value
            elif name.startswith("PARAM:"):
                shown = "%.7g" % value
            else:
                shown = "%.4f" % value
        else:
            shown = str(value)
        with self.lock:
            if self.closed:
                return
            self.writer.writerow([
                wall, t_s, boot_ms if boot_ms is not None else "", name, shown, text or "",
            ])
            self.rows += 1
            if self.rows % 40 == 0:
                self.file.flush()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.file.flush()
            self.file.close()


class Recorder:
    """One CSV at a time. Nothing is opened until start()."""

    def __init__(self, t0, directory=LOG_DIR):
        self.t0 = t0
        self.directory = directory
        self.lock = threading.Lock()
        self.log = None

    def status(self):
        with self.lock:
            log = self.log
            if log is None:
                return {"active": False, "name": "", "rows": 0, "bytes": 0}
            try:
                size = os.path.getsize(log.path)
            except OSError:
                size = 0
            return {
                "active": True,
                "name": os.path.basename(log.path),
                "rows": log.rows,
                "bytes": size,
            }

    def start(self, filename):
        os.makedirs(self.directory, exist_ok=True)
        path = os.path.join(self.directory, filename)
        with self.lock:
            if self.log is not None:
                return False, "já está gravando %s" % os.path.basename(self.log.path)
            existed = os.path.exists(path) and os.path.getsize(path) > 0
            self.log = CanLog(path, self.t0)
        if existed:
            return True, "continuando %s" % filename
        return True, "gravando %s" % filename

    def stop(self):
        with self.lock:
            log = self.log
            self.log = None
        if log is None:
            return False, "não estava gravando"
        rows = log.rows
        log.close()
        return True, "parou %s (%d linhas)" % (os.path.basename(log.path), rows)

    def write(self, name, value, boot_ms="", text=""):
        with self.lock:
            log = self.log
            if log is None:
                return
            log.write(name, value, boot_ms, text)

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
# Selector value that is only valid in this flight mode. 0 is always allowed.
ENSAIO_REQUIRES = {1: "CIRCLE", 2: "HOLD", 3: "CIRCLE", 4: "HOLD", 5: "HOLD"}
PROCESS_BY_ID = {item["id"]: item for item in PROCESSES}
PARAM_NAMES = tuple(param["name"] for group in GROUPS for param in group["params"])
PARAM_OK = set(PARAM_NAMES)
PARAM_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,15}$")


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

    def param_set(self, sysid, compid, name, value, ptype=9):
        payload = struct.pack("<fBB16sB", float(value), sysid, compid, name.encode()[:16], int(ptype) & 255)
        return self.pack(MSG_PARAM_SET, payload)

    def arm(self, sysid, compid, arm):
        return self.command_long(sysid, compid, MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if arm else 0.0)

    def ftp(self, sysid, compid, body):
        """FILE_TRANSFER_PROTOCOL. body is the 251-byte MAVFtp packet."""
        payload = bytes([0, sysid & 255, compid & 255]) + body
        return self.pack(MSG_FILE_TRANSFER_PROTOCOL, payload)


# Same opcodes and 80-byte writes as Mission Planner's MAVFtp (rwSize).
# The radio drops larger packets; the planner keeps this size for that reason.
FTP_NONE = 0
FTP_TERMINATE = 1
FTP_RESET = 2
FTP_LIST = 3
FTP_OPEN_RO = 4
FTP_READ = 5
FTP_CREATE = 6
FTP_WRITE = 7
FTP_REMOVE = 8
FTP_MKDIR = 9
FTP_RMDIR = 10
FTP_CRC = 14
STORAGE_BAK_DIR = "/APM/STRG_BAK"
FTP_ACK = 128
FTP_NACK = 129
FTP_CHUNK = 80
FTP_TIMEOUT = 4.0
FTP_RETRIES = 4
FTP_CRC_TIMEOUT = 15.0
FTP_MAX_BYTES = 400000
LUA_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,60}\.lua$")


def safe_lua_name(name):
    """A single file name from the card. No folders, so it cannot leave the directory."""
    if not isinstance(name, str) or not name or len(name) > 80:
        return False
    if not name.lower().endswith(".lua"):
        return False
    if name in (".", "..") or "/" in name or "\\" in name or "\x00" in name:
        return False
    return True
SCRIPT_DIRS = {
    "scripts": "/APM/scripts",
    "modules": "/APM/scripts/modules",
}
FTP_ERR_TEXT = {
    1: "falhou",
    2: "erro do cartão",
    3: "tamanho inválido",
    4: "sessão inválida",
    5: "sem sessão livre",
    6: "fim da lista",
    7: "comando desconhecido",
    8: "já existe",
    9: "arquivo protegido",
    10: "não encontrado",
}
FTP_ERRNO = {
    2: "ENOENT",
    6: "ENXIO",
    13: "EACCES",
    17: "EEXIST",
    28: "ENOSPC",
    30: "EROFS",
}


class FtpError(Exception):
    def __init__(self, text, retryable=False, missing=False, invalid_session=False, eof=False):
        super().__init__(text)
        self.retryable = retryable
        self.missing = missing
        self.invalid_session = invalid_session
        self.eof = eof


def ftp_crc32(data):
    """crc_crc32(0, data): the raw table CRC the flight controller reports."""
    crc = zlib.crc32(data, 0xFFFFFFFF) & 0xFFFFFFFF
    return crc ^ 0xFFFFFFFF


def parse_ftp_body(payload):
    """MAVLink FILE_TRANSFER_PROTOCOL payload, padded to 254 bytes."""
    if payload is None or len(payload) < 15:
        return None
    body = payload[3:254]
    if len(body) < 12:
        return None
    seq, session, opcode, size, req, _burst, _pad, offset = struct.unpack_from("<HBBBBBBI", body)
    size = min(size, 239)
    return {
        "seq": seq,
        "session": session,
        "opcode": opcode,
        "req": req,
        "offset": offset,
        "data": bytes(body[12:12 + size]),
    }


def parse_dir_entries(data):
    """Fname\\tsize and Dname entries, each ended by a NUL (ArduPilot adds two)."""
    found = []
    for part in data.split(b"\x00"):
        if not part:
            continue
        kind, body = part[:1], part[1:]
        if kind == b"F":
            name, sep, size = body.rpartition(b"\t")
            if not sep:
                continue
            try:
                size_n = int(size)
            except ValueError:
                continue
            found.append(("F", name.decode("ascii", "replace"), size_n))
        elif kind == b"D":
            name = body.split(b"\t", 1)[0].decode("ascii", "replace")
            found.append(("D", name, None))
    return found


def ftp_nack_error(reply):
    data = reply["data"]
    code = data[0] if data else 0
    errno = data[1] if len(data) > 1 else None
    if code == 6:
        return FtpError("fim da lista", eof=True)
    if code == 10 or (code == 2 and errno == 2):
        return FtpError("não encontrado", missing=True)
    if code == 8:
        return FtpError("já existe")
    if code == 4:
        return FtpError("sessão inválida", retryable=True, invalid_session=True)
    text = FTP_ERR_TEXT.get(code, "resposta %d" % code)
    if code == 2 and errno is not None:
        name = FTP_ERRNO.get(errno, "errno %d" % errno)
        text = "%s (%s)" % (text, name)
        if errno == 6:
            text = "o cartão não respondeu (ENXIO)"
    retryable = code in (1, 2, 5) or errno in (6, 11)
    return FtpError(text, retryable=retryable)


class FtpClient:
    """One MAVFtp session at a time, the way Mission Planner uploads a script.

    Reset the session, create the file, write 80-byte blocks, close, then
    compare the CRC the board computes with the one of the bytes we sent.
    """

    def __init__(self, link, tx, enc):
        self.link = link
        self.tx = tx
        self.enc = enc
        self.cv = threading.Condition()
        self.replies = collections.deque()
        self.op_lock = threading.Lock()
        self.ftp_seq = 0
        self.session = 0
        self.progress_lock = threading.Lock()
        self.progress = {
            "active": False,
            "action": "",
            "name": "",
            "folder": "",
            "phase": "",
            "done": 0,
            "total": 0,
            "index": 0,
            "count": 0,
            "t0": 0.0,
        }

    def push(self, payload):
        reply = parse_ftp_body(payload)
        if reply is None:
            return
        with self.cv:
            self.replies.append(reply)
            self.cv.notify()

    def begin_progress(self, action, name, folder="", total=0, index=1, count=1, phase=""):
        with self.progress_lock:
            self.progress = {
                "active": True,
                "action": action,
                "name": name or "",
                "folder": folder or "",
                "phase": phase or "",
                "done": 0,
                "total": int(total) if total else 0,
                "index": int(index) if index else 1,
                "count": int(count) if count else 1,
                "t0": time.monotonic(),
            }

    def update_progress(self, done=None, total=None, phase=None):
        with self.progress_lock:
            if not self.progress.get("active"):
                return
            if done is not None:
                self.progress["done"] = int(done)
            if total is not None:
                self.progress["total"] = int(total)
            if phase is not None:
                self.progress["phase"] = phase

    def end_progress(self):
        with self.progress_lock:
            self.progress["active"] = False
            self.progress["phase"] = ""

    def progress_snapshot(self):
        with self.progress_lock:
            snap = dict(self.progress)
        if snap.get("active") and snap.get("t0"):
            elapsed = max(0.0, time.monotonic() - snap["t0"])
            snap["elapsed"] = round(elapsed, 1)
            done = snap.get("done") or 0
            total = snap.get("total") or 0
            if done > 0 and total > done and elapsed >= 0.4:
                rate = done / elapsed
                snap["eta"] = round((total - done) / rate, 1) if rate > 0 else None
            else:
                snap["eta"] = None
        else:
            snap["elapsed"] = 0
            snap["eta"] = None
        snap.pop("t0", None)
        return snap

    def _wait(self, expect, session, timeout):
        deadline = time.monotonic() + timeout
        with self.cv:
            while True:
                while self.replies:
                    reply = self.replies.popleft()
                    if reply["seq"] == expect and reply["session"] == session:
                        return reply
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.cv.wait(remaining)

    def roundtrip(self, opcode, data=b"", offset=0, size=None, timeout=FTP_TIMEOUT, retries=FTP_RETRIES):
        data = bytes(data or b"")
        if len(data) > 239:
            raise FtpError("bloco grande demais")
        # ReadFile puts the byte count in size with an empty payload.
        wire_size = len(data) if size is None else int(size)
        if wire_size < 0 or wire_size > 239:
            raise FtpError("tamanho inválido")
        seq = self.ftp_seq & 0xFFFF
        expect = (seq + 1) & 0xFFFF
        session = self.session & 255
        header = struct.pack("<HBBBBBBI", seq, session, opcode, wire_size, 0, 0, 0, offset & 0xFFFFFFFF)
        body = (header + data).ljust(251, b"\0")
        with self.cv:
            self.replies.clear()
        last = None
        for _attempt in range(retries):
            frame = self.enc.ftp(self._sysid, self._compid, body)
            with self.tx:
                self.link.send(frame)
            deadline = time.monotonic() + timeout
            reply = None
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                got = self._wait(expect, session, remaining)
                if got is None:
                    break
                if got["req"] != opcode:
                    continue
                reply = got
                break
            if reply is None:
                last = FtpError("sem resposta do cartão", retryable=True)
                continue
            self.ftp_seq = expect
            if reply["opcode"] == FTP_NACK:
                raise ftp_nack_error(reply)
            if reply["opcode"] != FTP_ACK:
                raise FtpError("resposta inesperada", retryable=True)
            return reply
        raise last or FtpError("sem resposta do cartão", retryable=True)

    def _begin(self, vehicle):
        with vehicle.lock:
            if vehicle.sysid is None:
                raise FtpError("sem rover")
            if vehicle.jobs or vehicle.load is not None:
                raise FtpError("espere o pedido atual terminar")
            self._sysid = vehicle.sysid
            self._compid = vehicle.compid or 1

    def _call(self, vehicle, action):
        with self.op_lock:
            self._begin(vehicle)
            try:
                return action()
            except FtpError as exc:
                if not exc.retryable:
                    raise
                time.sleep(3.2 if exc.invalid_session else 0.4)
                try:
                    self.roundtrip(FTP_RESET, timeout=2.0, retries=2)
                except FtpError:
                    pass
                self._begin(vehicle)
                return action()

    def reset(self):
        self.roundtrip(FTP_RESET, timeout=2.0, retries=2)

    def list_dir(self, path):
        entries = []
        offset = 0
        encoded = path.encode("ascii")
        while True:
            try:
                reply = self.roundtrip(FTP_LIST, encoded, offset=offset)
            except FtpError as exc:
                if exc.eof:
                    break
                raise
            parts = [part for part in reply["data"].split(b"\x00") if part]
            batch = parse_dir_entries(reply["data"])
            if not parts:
                break
            entries.extend(batch)
            offset += len(parts)
            if offset > 500:
                break
        return entries

    def list_lua(self, path):
        files = []
        for kind, name, size in self.list_dir(path):
            if kind == "F" and safe_lua_name(name):
                files.append({"name": name, "size": size})
        files.sort(key=lambda item: item["name"].lower())
        return files

    def mkdir(self, path):
        try:
            self.roundtrip(FTP_MKDIR, path.encode("ascii"))
        except FtpError as exc:
            if "já existe" not in str(exc):
                raise

    def ensure_dir(self, path):
        built = ""
        for piece in path.strip("/").split("/"):
            built += "/" + piece
            try:
                self.list_dir(built)
            except FtpError as exc:
                if not exc.missing:
                    raise
                self.mkdir(built)

    def remove(self, path):
        self.reset()
        self.roundtrip(FTP_REMOVE, path.encode("ascii"))
        self.reset()

    def remove_file(self, path):
        """RemoveFile (opcode 8), same as Mission Planner — one path per call."""
        self.roundtrip(FTP_REMOVE, path.encode("ascii"))

    def clear_storage_bak(self):
        """List /APM/STRG_BAK and RemoveFile every STRG*.bak (and last_storage_bak).

        MAVFtp has no bulk delete: Mission Planner also removes one file at a time.
        Files are never returned to the UI — only a count.
        """
        self.update_progress(done=0, total=0, phase="listar")
        self.reset()
        try:
            entries = self.list_dir(STORAGE_BAK_DIR)
        except FtpError as exc:
            if exc.missing:
                return 0, 0, "pasta STRG_BAK já não existe"
            raise
        names = []
        for kind, name, _size in entries:
            if kind != "F":
                continue
            if name == "last_storage_bak":
                names.append(name)
            elif name.startswith("STRG") and name.endswith(".bak") and "/" not in name and "\\" not in name:
                names.append(name)
        total = len(names)
        self.update_progress(done=0, total=total, phase="apagar")
        if total == 0:
            return 0, 0, "STRG_BAK já está vazia"
        deleted = 0
        failed = 0
        for index, name in enumerate(names, start=1):
            path = STORAGE_BAK_DIR + "/" + name
            try:
                self.remove_file(path)
                deleted += 1
            except FtpError as exc:
                if exc.missing:
                    deleted += 1
                elif exc.invalid_session:
                    try:
                        self.reset()
                        self.remove_file(path)
                        deleted += 1
                    except FtpError:
                        failed += 1
                else:
                    failed += 1
            self.update_progress(done=index, total=total, phase="apagar")
        try:
            self.reset()
        except FtpError:
            pass
        if failed:
            return deleted, failed, "apagou %d; %d falharam" % (deleted, failed)
        return deleted, 0, "apagou %d arquivo%s de STRG_BAK" % (deleted, "" if deleted == 1 else "s")

    def get(self, path):
        """Read a whole file with OpenFileRO + ReadFile, then close the session."""
        self.reset()
        opened = self.roundtrip(FTP_OPEN_RO, path.encode("ascii"))
        if len(opened["data"]) < 4:
            raise FtpError("tamanho do arquivo ausente", retryable=True)
        size = struct.unpack("<I", opened["data"][:4])[0]
        if size > FTP_MAX_BYTES:
            try:
                self.roundtrip(FTP_TERMINATE, timeout=2.0, retries=2)
            except FtpError:
                pass
            raise FtpError("arquivo grande demais")
        chunks = []
        offset = 0
        while offset < size:
            want = min(FTP_CHUNK, size - offset)
            try:
                reply = self.roundtrip(FTP_READ, b"", offset=offset, size=want)
            except FtpError as exc:
                if exc.eof:
                    break
                raise
            block = reply["data"]
            if not block:
                break
            chunks.append(block)
            offset += len(block)
        try:
            self.roundtrip(FTP_TERMINATE, timeout=2.0, retries=2)
        except FtpError:
            self.reset()
        data = b"".join(chunks)
        if size and len(data) != size:
            raise FtpError("leitura incompleta (%d de %d)" % (len(data), size), retryable=True)
        return data

    def put(self, path, data):
        self.update_progress(done=0, total=len(data), phase="criar")
        self.reset()
        self.roundtrip(FTP_CREATE, path.encode("ascii"))
        offset = 0
        self.update_progress(done=0, total=len(data), phase="enviar")
        while offset < len(data):
            block = data[offset:offset + FTP_CHUNK]
            self.roundtrip(FTP_WRITE, block, offset=offset)
            offset += len(block)
            self.update_progress(done=offset, total=len(data), phase="enviar")
        self.update_progress(done=len(data), total=len(data), phase="crc")
        self.reset()
        reply = self.roundtrip(FTP_CRC, path.encode("ascii"), timeout=FTP_CRC_TIMEOUT, retries=2)
        if len(reply["data"]) < 4:
            raise FtpError("CRC sem resposta", retryable=True)
        remote = struct.unpack("<I", reply["data"][:4])[0]
        local = ftp_crc32(data)
        if remote != local:
            raise FtpError("o arquivo chegou diferente", retryable=True)
        self.update_progress(done=len(data), total=len(data), phase="pronto")


def script_paths(folder, name):
    directory = SCRIPT_DIRS.get(folder)
    if directory is None or not LUA_NAME.match(name or ""):
        return None, None
    return directory, directory + "/" + name


def script_list(vehicle):
    ftp = vehicle.ftp

    def read():
        out = {"ok": True, "scripts": [], "modules": [], "scripts_note": "", "modules_note": "", "note": ""}
        ftp.reset()
        for key, path in SCRIPT_DIRS.items():
            try:
                out[key] = ftp.list_lua(path)
            except FtpError as exc:
                if exc.missing:
                    out[key + "_note"] = "pasta ainda não existe"
                else:
                    out[key + "_note"] = str(exc)
                    out["ok"] = False
        return out

    try:
        return ftp._call(vehicle, read)
    except FtpError as exc:
        return {
            "ok": False, "scripts": [], "modules": [],
            "scripts_note": "", "modules_note": "", "note": str(exc),
        }


def script_put(vehicle, folder, name, data, index=1, count=1):
    directory, path = script_paths(folder, name)
    if path is None:
        return False, "nome inválido: use um arquivo .lua"
    if len(data) > FTP_MAX_BYTES:
        return False, "arquivo grande demais"
    ftp = vehicle.ftp
    ftp.begin_progress("put", name, folder=folder, total=len(data), index=index, count=count, phase="preparar")

    def write():
        ftp.ensure_dir(directory)
        ftp.put(path, data)

    try:
        ftp._call(vehicle, write)
    except FtpError as exc:
        ftp.end_progress()
        note = "%s: %s" % (name, exc)
        print("script: " + note, flush=True)
        return False, note
    ftp.end_progress()
    note = "%s enviado. Reinicie a placa para carregar." % name
    print("script: " + note, flush=True)
    with vehicle.lock:
        push_message(vehicle, 6, "script enviado: %s/%s" % (folder, name))
    return True, note


def script_progress(vehicle):
    ftp = getattr(vehicle, "ftp", None)
    if ftp is None:
        return {"active": False}
    return ftp.progress_snapshot()


def script_get(vehicle, folder, name):
    directory = SCRIPT_DIRS.get(folder)
    if directory is None or not safe_lua_name(name):
        return False, "nome inválido", b""
    path = directory + "/" + name
    ftp = vehicle.ftp

    def read():
        return ftp.get(path)

    try:
        data = ftp._call(vehicle, read)
    except FtpError as exc:
        note = "%s: %s" % (name, exc)
        print("script: " + note, flush=True)
        return False, note, b""
    note = "%s copiado" % name
    print("script: " + note, flush=True)
    return True, note, data


def script_remove(vehicle, folder, name):
    directory = SCRIPT_DIRS.get(folder)
    if directory is None or not safe_lua_name(name):
        return False, "nome inválido"
    path = directory + "/" + name
    ftp = vehicle.ftp

    def delete():
        ftp.remove(path)

    try:
        ftp._call(vehicle, delete)
    except FtpError as exc:
        note = "%s: %s" % (name, exc)
        print("script: " + note, flush=True)
        return False, note
    note = "%s excluído. Reinicie a placa para deixar de carregar." % name
    print("script: " + note, flush=True)
    with vehicle.lock:
        push_message(vehicle, 6, "script excluído: %s/%s" % (folder, name))
    return True, note


def storage_bak_clear(vehicle):
    ftp = vehicle.ftp
    ftp.begin_progress("clear", "STRG_BAK", folder="STRG_BAK", total=0, phase="listar")

    def wipe():
        return ftp.clear_storage_bak()

    try:
        deleted, failed, note = ftp._call(vehicle, wipe)
    except FtpError as exc:
        ftp.end_progress()
        note = "STRG_BAK: %s" % exc
        print("storage: " + note, flush=True)
        return False, note
    ftp.end_progress()
    print("storage: " + note, flush=True)
    with vehicle.lock:
        push_message(vehicle, 6, note)
    return failed == 0, note


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
        self.recorder = Recorder(self.t0)
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
        self.revert = None
        self.reset_inputs = False
        self.load = None
        self.last_ack = None
        self.want_refresh = False
        self.param_seen = {}
        self.param_type = {}
        self.param_epoch = 0
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
    rate_acks = set()
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
        service_load(link, vehicle, tx, enc)
        with vehicle.lock:
            refresh = bool(vehicle.want_refresh and vehicle.sysid is not None)
            if refresh:
                vehicle.want_refresh = False
                vehicle.params.clear()
                vehicle.param_seen.clear()
                vehicle.param_type.clear()
                vehicle.list_sent = True
                refresh_sys, refresh_comp = vehicle.sysid, vehicle.compid or 1
        if refresh:
            with tx:
                link.send(enc.param_list(refresh_sys, refresh_comp))
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
            if msgid == mon.MSG_FILE_TRANSFER_PROTOCOL:
                vehicle.ftp.push(payload)
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
                        vehicle.recorder.write("ARMED", int(bool(msg["armed"])))
                    if was_mode != msg["mode"]:
                        label = mode_name(msg["mode"]) or str(msg["mode"])
                        push_message(vehicle, 6, "Modo %s" % label)
                        vehicle.recorder.write("MODE", int(msg["mode"]), "", label)
                continue
            if msgid == mon.MSG_STATUSTEXT:
                # Not tied to the first heartbeat. Mission Planner lists every
                # STATUSTEXT from the Pixhawk, including long lines sent in pieces.
                for sev, text in take_statustext(vehicle, payload):
                    with vehicle.lock:
                        push_message(vehicle, sev, text)
                        vehicle.recorder.write("STATUSTEXT", int(sev), "", text)
                continue
            with vehicle.lock:
                if vehicle.sysid is not None and sysid_rx != vehicle.sysid:
                    continue
            if not msg:
                continue
            if msgid == mon.MSG_COMMAND_ACK:
                with vehicle.lock:
                    vehicle.last_ack = (msg["command"], msg["result"], time.monotonic())
                    fail_job(vehicle, msg["command"], msg["result"])
                    if msg["command"] == MAV_CMD_SET_MESSAGE_INTERVAL and msg["result"] not in rate_acks:
                        rate_acks.add(msg["result"])
                        vehicle.recorder.write("RATE_ACK", int(msg["result"]))
            elif msgid == mon.MSG_PARAM_VALUE:
                with vehicle.lock:
                    name, value = msg["name"], msg["value"]
                    old = vehicle.params.get(name)
                    vehicle.params[name] = value
                    vehicle.param_seen[name] = time.monotonic()
                    if msg.get("type"):
                        vehicle.param_type[name] = int(msg["type"])
                    if old is None or abs(old - value) > 1e-7 * max(1.0, abs(value)):
                        vehicle.recorder.write("PARAM:" + name, float(value))
            elif msgid == mon.MSG_NAMED_VALUE_FLOAT:
                add_value(vehicle, msg["name"], msg["value"])
                if msg["name"].startswith("ESC"):
                    vehicle.recorder.write(msg["name"], msg["value"], msg.get("boot_ms", ""))
            elif "values" in msg:
                boot = msg.get("boot_ms", "")
                for name, value in msg["values"]:
                    add_value(vehicle, name, value)
                    vehicle.recorder.write(name, value, boot)


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
    if kind == "command":
        return job["name"]
    if kind == "wait":
        return "espera"
    return "armar" if job["arm"] else "desarmar"


def job_done(job):
    kind = job["kind"]
    if kind == "note":
        return job["text"]
    if kind == "mode":
        return "%s aceito" % job["name"]
    if kind == "param":
        return "%s = %s" % (job["name"], ("%.6g" % job["value"]))
    if kind == "command":
        return job["name"]
    if kind == "wait":
        return "espera"
    return "armação aceita" if job["arm"] else "desarme aceito"


def param_blocked(vehicle, name, value):
    if name != "SCR_USER1":
        return None
    sel = int(round(value))
    need = ENSAIO_REQUIRES.get(sel)
    current = mode_name(vehicle.mode)
    if need and current != need:
        return "SCR_USER1=%d exige %s. A placa está em %s." % (sel, need, current or "?")
    return None


def remember_state(vehicle, jobs):
    params = {}
    for job in jobs:
        if job["kind"] == "param" and job["name"] in vehicle.params:
            params[job["name"]] = vehicle.params[job["name"]]
    vehicle.revert = {
        "mode": vehicle.mode,
        "mode_name": mode_name(vehicle.mode),
        "params": params,
    }


def enqueue(vehicle, jobs, done, remember=True, sev=None):
    with vehicle.lock:
        if vehicle.sysid is None:
            vehicle.note = "sem rover"
            return False
        if vehicle.load is not None:
            push_message(vehicle, 4, "carga do arquivo interrompida")
            vehicle.load = None
        if remember:
            remember_state(vehicle, jobs)
        else:
            vehicle.revert = None
        vehicle.jobs.clear()
        for job in jobs:
            job["deadline"] = None
            job["sent"] = 0.0
            job["opened"] = time.monotonic()
            vehicle.jobs.append(job)
        note = {"kind": "note", "text": done, "deadline": None, "sent": 0.0}
        if sev is not None:
            note["sev"] = sev
        vehicle.jobs.append(note)
        vehicle.note = "pedindo %s" % job_label(jobs[0])
        print("[debug] pedido: %s (modo atual %s)" % (
            ", ".join(job_label(job) for job in jobs), mode_name(vehicle.mode)), flush=True)
    return True


def give_up(vehicle, reason, sev=4):
    """Stop the pending change and command the previous mode. Caller holds the lock."""
    stayed = mode_name(vehicle.mode) or "?"
    text = "%s Continua em %s." % (reason, stayed)
    vehicle.note = text
    push_message(vehicle, sev, text)
    vehicle.reset_inputs = True
    print("[debug] " + text, flush=True)
    rev = vehicle.revert
    vehicle.revert = None
    vehicle.jobs.clear()
    back = []
    if rev and rev.get("mode") is not None and rev["mode"] != vehicle.mode:
        back.append({"kind": "mode", "name": rev["mode_name"] or str(rev["mode"]), "number": rev["mode"]})
    for name, old in (rev or {}).get("params", {}).items():
        if old is None:
            continue
        got = vehicle.params.get(name)
        if got is None or abs(got - old) > max(1e-4, abs(old) * 1e-4):
            back.append({"kind": "param", "name": name, "value": float(old)})
    if not back:
        return
    vehicle.note = "voltando para %s" % (rev.get("mode_name") or stayed)
    push_message(vehicle, 4, vehicle.note)
    for job in back:
        job["deadline"] = None
        job["sent"] = 0.0
        vehicle.jobs.append(job)
    vehicle.jobs.append({
        "kind": "note",
        "text": "voltou para %s" % (rev.get("mode_name") or stayed),
        "deadline": None,
        "sent": 0.0,
    })


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
    if kind == "command":
        ack = vehicle.last_ack
        return bool(ack and ack[0] == job["command"] and ack[1] == 0 and ack[2] >= job.get("opened", 0))
    if kind == "wait":
        return job.get("deadline") is not None and time.monotonic() >= job["deadline"]
    return False


def fail_job(vehicle, command, result):
    """Abort the current job when the board refuses it. Caller holds the lock."""
    if result in (0, 1, 3, 5) or not vehicle.jobs:
        return
    job = vehicle.jobs[0]
    if job["kind"] == "command":
        if command != job.get("command") or result in (0, 1, 5):
            return
        vehicle.note = "%s %s." % (job_label(job), ACK_TEXT.get(result, "resposta %d" % result))
        push_message(vehicle, 3, vehicle.note)
        vehicle.jobs.clear()
        vehicle.revert = None
        print("[debug] " + vehicle.note, flush=True)
        return
    if job["kind"] == "mode" and command != MAV_CMD_DO_SET_MODE:
        return
    if job["kind"] == "arm" and command != MAV_CMD_COMPONENT_ARM_DISARM:
        return
    if job["kind"] not in ("mode", "arm"):
        return
    reason = "%s %s." % (job_label(job), ACK_TEXT.get(result, "resposta %d" % result))
    give_up(vehicle, reason, 3)


def send_job(link, enc, sysid, compid, job):
    kind = job["kind"]
    if kind == "mode":
        link.send(enc.command_mode(sysid, compid, job["number"]))
        link.send(enc.mode_frame(sysid, job["number"]))
        time.sleep(0.01)
        link.send(enc.mode_frame(sysid, job["number"]))
    elif kind == "param":
        link.send(enc.param_set(sysid, compid, job["name"], job["value"], job.get("type", 9)))
    elif kind == "arm":
        link.send(enc.arm(sysid, compid, job["arm"]))
    elif kind == "command":
        link.send(enc.command_long(sysid, compid, job["command"], job.get("p1", 0)))


def service_jobs(link, vehicle, tx, enc):
    now = time.monotonic()
    with vehicle.lock:
        if not vehicle.jobs or vehicle.sysid is None:
            return
        job = vehicle.jobs[0]
        if job_succeeded(vehicle, job):
            if job.get("command") == MAV_CMD_PREFLIGHT_STORAGE:
                vehicle.want_refresh = True
            vehicle.jobs.popleft()
            if vehicle.jobs and vehicle.jobs[0]["kind"] != "note":
                vehicle.note = "pedindo %s" % job_label(vehicle.jobs[0])
            elif vehicle.jobs:
                done = vehicle.jobs.popleft()
                vehicle.note = done["text"]
                if done.get("sev") is not None:
                    push_message(vehicle, done["sev"], done["text"])
                print(vehicle.note, flush=True)
            else:
                vehicle.note = job_done(job)
                print(vehicle.note, flush=True)
            return
        if job["deadline"] is None:
            span = job["seconds"] if job["kind"] == "wait" else COMMAND_WAIT_S
            job["deadline"] = now + span
            if job["kind"] == "wait":
                return
        elif now > job["deadline"]:
            if job["kind"] == "command":
                reason = "Sem resposta ao pedir %s." % job_label(job)
                vehicle.note = reason
                push_message(vehicle, 4, reason)
                vehicle.jobs.clear()
                vehicle.revert = None
                print("[debug] " + reason, flush=True)
                return
            give_up(vehicle, "Sem resposta ao pedir %s." % job_label(job), 4)
            return
        if now - job["sent"] < COMMAND_RETRY_S:
            return
        job["sent"] = now
        sysid, compid = vehicle.sysid, vehicle.compid or 1
        outgoing = job
        print("[debug] enviando %s" % job_label(job), flush=True)
    with tx:
        send_job(link, enc, sysid, compid, outgoing)


def ask_mode(link, vehicle, tx, enc, name, number):
    return enqueue(vehicle, [{"kind": "mode", "name": name, "number": number}], "%s aceito" % name)


def set_param(link, vehicle, tx, enc, name, value, quiet=False):
    with vehicle.lock:
        reason = param_blocked(vehicle, name, value)
        if reason:
            vehicle.note = reason
            vehicle.reset_inputs = True
            push_message(vehicle, 4, reason)
            print("[debug] bloqueado: " + reason, flush=True)
            return False
        ptype = int(vehicle.param_type.get(name, 9))
    text = "%s = %s" % (name, ("%.6g" % float(value)))
    return enqueue(vehicle, [{
        "kind": "param", "name": name, "value": float(value), "type": ptype,
    }], text)


def set_arm(link, vehicle, tx, enc, arm):
    done = "armação aceita" if arm else "desarme aceito"
    return enqueue(vehicle, [{"kind": "arm", "arm": bool(arm)}], done)


def set_recording(vehicle, action, name):
    """Open or close the CAN CSV. Caller does not hold the lock."""
    with vehicle.lock:
        if action == "stop":
            ok, note = vehicle.recorder.stop()
        elif action == "start":
            filename = log_filename(name)
            if filename is None:
                vehicle.note = "use letras, números, _ ou - (base esc_log_)"
                push_message(vehicle, 4, vehicle.note)
                return False
            ok, note = vehicle.recorder.start(filename)
        else:
            return False
        vehicle.note = note
        push_message(vehicle, 6 if ok else 4, note)
        print("[debug] " + note, flush=True)
    return ok


def values_close(got, value):
    return got is not None and abs(got - value) <= max(1e-4, abs(value) * 1e-4)


def _refuse_if_busy(vehicle, action):
    """Caller holds the lock. Armed changes and a missing link stop the action."""
    if vehicle.armed:
        vehicle.note = "desarme antes de %s" % action
        push_message(vehicle, 4, vehicle.note)
        return True
    if vehicle.sysid is None:
        vehicle.note = "sem rover"
        return True
    return False


def refresh_params(vehicle):
    with vehicle.lock:
        if vehicle.sysid is None:
            vehicle.note = "sem rover"
            return False
        if vehicle.load is not None:
            push_message(vehicle, 4, "carga do arquivo interrompida")
            vehicle.load = None
        vehicle.want_refresh = True
        vehicle.param_epoch += 1
        vehicle.note = "lendo parâmetros de novo"
        push_message(vehicle, 6, "atualizando a lista de parâmetros")
    return True


def confirm_stored(vehicle):
    """Kept so an old page still gets a clear answer."""
    return start_write(vehicle, {})


def ask_reset(link, vehicle, tx, enc):
    """Mission Planner sets FORMAT_VERSION to 0, waits, then reboots.

    The running values stay until the board starts again and sees the
    format mismatch, which is what loads the firmware defaults.
    """
    with vehicle.lock:
        if _refuse_if_busy(vehicle, "voltar ao padrão de fábrica"):
            return False
        if vehicle.load is not None:
            push_message(vehicle, 4, "gravação interrompida")
            vehicle.load = None
        jobs = [{
            "kind": "param",
            "name": "FORMAT_VERSION",
            "value": 0.0,
            "type": int(vehicle.param_type.get("FORMAT_VERSION", 4)),
        }]
        if "SYSID_SW_MREV" in vehicle.params:
            jobs.append({
                "kind": "param",
                "name": "SYSID_SW_MREV",
                "value": 0.0,
                "type": int(vehicle.param_type.get("SYSID_SW_MREV", 4)),
            })
        jobs.append({"kind": "wait", "seconds": 1.0})
        jobs.append({
            "kind": "command",
            "command": MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            "p1": 1.0,
            "name": "reiniciar",
        })
    return enqueue(
        vehicle, jobs, "reiniciando no padrão de fábrica", remember=False, sev=4)


def start_write(vehicle, changes):
    """Send the edited values, the way Mission Planner's Write Params does."""
    with vehicle.lock:
        if vehicle.sysid is None:
            vehicle.note = "sem rover"
            return False
        if vehicle.jobs or vehicle.load is not None:
            vehicle.note = "espere o comando atual terminar"
            push_message(vehicle, 4, vehicle.note)
            return False
        pairs = []
        if isinstance(changes, dict):
            for name, raw in changes.items():
                if not isinstance(name, str) or not PARAM_NAME.match(name):
                    continue
                try:
                    value = float(raw)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(value):
                    continue
                pairs.append((name, value))
        if len(pairs) > 4000:
            vehicle.note = "muitos parâmetros"
            push_message(vehicle, 4, vehicle.note)
            return False
        if not pairs:
            vehicle.note = "nenhum parâmetro alterado"
            push_message(vehicle, 6, vehicle.note)
            return True
        vehicle.load = {
            "pairs": pairs,
            "i": 0,
            "fails": 0,
            "name": None,
            "deadline": 0.0,
            "sent": 0.0,
            "verb": "gravando",
            "refresh": False,
        }
        vehicle.note = "gravando %d parâmetros" % len(pairs)
        push_message(vehicle, 6, vehicle.note)
    return True


def ask_reboot(link, vehicle, tx, enc):
    with vehicle.lock:
        if _refuse_if_busy(vehicle, "reiniciar"):
            return False
    return enqueue(
        vehicle,
        [{"kind": "command", "command": MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, "p1": 1.0, "name": "reiniciar"}],
        "reinício aceito",
        remember=False,
        sev=6,
    )


def parse_parm(text):
    ordered = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue
        line = line.split("#", 1)[0].strip()
        if "," in line:
            name, val = line.split(",", 1)
            val = val.split(",")[0]
        else:
            bits = line.split()
            if len(bits) < 2:
                continue
            name, val = bits[0], bits[1]
        name = name.strip()
        if not PARAM_NAME.match(name):
            continue
        try:
            value = float(val.strip())
        except ValueError:
            continue
        if not math.isfinite(value):
            continue
        ordered[name] = value
    return list(ordered.items())


def parm_text(params):
    lines = ["# parametros do rover"]
    for name in sorted(params):
        lines.append("%s,%.7g" % (name, params[name]))
    return ("\n".join(lines) + "\n").encode("utf-8")


def start_load(vehicle, pairs):
    with vehicle.lock:
        if _refuse_if_busy(vehicle, "carregar o arquivo"):
            return False
        if not pairs:
            vehicle.note = "arquivo sem parâmetros"
            push_message(vehicle, 4, vehicle.note)
            return False
        if len(pairs) > 4000:
            vehicle.note = "arquivo grande demais"
            push_message(vehicle, 4, vehicle.note)
            return False
        if vehicle.jobs:
            vehicle.note = "espere o comando atual terminar"
            push_message(vehicle, 4, vehicle.note)
            return False
        vehicle.load = {
            "pairs": pairs,
            "i": 0,
            "fails": 0,
            "name": None,
            "deadline": 0.0,
            "sent": 0.0,
            "verb": "carregando",
            "refresh": True,
        }
        vehicle.note = "carregando %d parâmetros" % len(pairs)
        push_message(vehicle, 6, vehicle.note)
    return True


def service_load(link, vehicle, tx, enc):
    """Send one PARAM_SET from a loaded file. Values already on the board are skipped."""
    now = time.monotonic()
    send = None
    with vehicle.lock:
        load = vehicle.load
        if not load or vehicle.jobs or vehicle.sysid is None:
            return
        total = len(load["pairs"])
        verb = load.get("verb") or "carregando"
        while load["i"] < total:
            name, value = load["pairs"][load["i"]]
            if load["name"] != name:
                reason = param_blocked(vehicle, name, value)
                if reason:
                    load["fails"] += 1
                    load["i"] += 1
                    load["name"] = None
                    push_message(vehicle, 4, reason)
                    print("[debug] bloqueado: " + reason, flush=True)
                    continue
                if values_close(vehicle.params.get(name), value):
                    load["i"] += 1
                    load["name"] = None
                    if load["i"] % 25 == 0:
                        vehicle.note = "%s %d de %d" % (verb, load["i"], total)
                    continue
                load["name"] = name
                load["sent"] = now
                load["deadline"] = now + 2.0
                send = (vehicle.sysid, vehicle.compid or 1, name, value,
                        int(vehicle.param_type.get(name, 9)))
                break
            echoed = vehicle.param_seen.get(name, 0.0) >= load["sent"]
            if echoed and values_close(vehicle.params.get(name), value):
                load["i"] += 1
                load["name"] = None
                if load["i"] % 25 == 0:
                    vehicle.note = "%s %d de %d" % (verb, load["i"], total)
                continue
            if now <= load["deadline"]:
                return
            if not values_close(vehicle.params.get(name), value):
                load["fails"] += 1
                print("[debug] não gravou %s" % name, flush=True)
            load["i"] += 1
            load["name"] = None
        if send is None and load["i"] >= total:
            fails = load["fails"]
            done = total - fails
            if verb == "gravando":
                msg = "gravados %d, falhou %d" % (done, fails) if fails else "%d parâmetros gravados" % done
            else:
                msg = "arquivo carregado: %d ok, %d sem resposta" % (done, fails)
            vehicle.note = msg
            push_message(vehicle, 4 if fails else 6, msg)
            vehicle.load = None
            if load.get("refresh"):
                vehicle.want_refresh = True
                vehicle.param_epoch += 1
            return
    if send is not None:
        with tx:
            link.send(enc.param_set(*send))


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
        params = dict(vehicle.params)
        latest = {name: vehicle.latest[name] for name in READOUT if name in vehicle.latest}
        blocked = []
        reset_inputs = vehicle.reset_inputs
        vehicle.reset_inputs = False
        param_epoch = vehicle.param_epoch
        recording = vehicle.recorder.status()
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
        "blocked_modes": blocked,
        "reset_inputs": reset_inputs,
        "param_epoch": param_epoch,
        "recording": recording,
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
        ".json": "application/json; charset=utf-8",
        ".png": "image/png",
    }

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _send(self, code, ctype, body, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in extra or ():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self, limit=8000):
            try:
                n = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                return None
            if n < 0 or n > limit:
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
            if path == "/scripts":
                code, ctype, body = json_body(script_list(vehicle))
                self._send(code, ctype, body)
                return
            if path == "/scripts/progress":
                code, ctype, body = json_body(script_progress(vehicle))
                self._send(code, ctype, body)
                return
            if path == "/params.parm":
                with vehicle.lock:
                    body = parm_text(vehicle.params)
                self._send(200, "text/plain; charset=utf-8", body, (
                    ("Content-Disposition", 'attachment; filename="rover.parm"'),
                ))
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
            if path == "/storage/clear-bak":
                self._read_json(8000)
                ok, note = storage_bak_clear(vehicle)
                code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path in ("/scripts/put", "/scripts/remove", "/scripts/get"):
                data = self._read_json(900000 if path == "/scripts/put" else 8000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                folder = data.get("folder")
                name = data.get("name")
                if path == "/scripts/remove":
                    ok, note = script_remove(vehicle, folder, name if isinstance(name, str) else "")
                    code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                elif path == "/scripts/get":
                    ok, note, blob = script_get(vehicle, folder, name if isinstance(name, str) else "")
                    payload = {"ok": ok, "note": note}
                    if ok:
                        payload["name"] = name
                        payload["data"] = base64.b64encode(blob).decode("ascii")
                    code, ctype, body = json_body(payload, 200 if ok else 409)
                else:
                    raw = data.get("data")
                    if not isinstance(name, str) or not isinstance(raw, str):
                        self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                        return
                    try:
                        blob = base64.b64decode(raw, validate=True)
                    except (ValueError, TypeError):
                        self._send(400, "application/json", b'{"ok":false,"note":"arquivo invalido"}')
                        return
                    try:
                        index = int(data.get("index") or 1)
                        count = int(data.get("count") or 1)
                    except (TypeError, ValueError):
                        index, count = 1, 1
                    ok, note = script_put(vehicle, folder, name, blob, index=index, count=count)
                    code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/params/load":
                data = self._read_json(400000)
                text = data.get("text") if isinstance(data, dict) else None
                if not isinstance(text, str):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = start_load(vehicle, parse_parm(text))
                code, ctype, body = json_body({"ok": ok, "note": snapshot(link, vehicle)["note"]}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/params/write":
                data = self._read_json(400000)
                changes = data.get("changes") if isinstance(data, dict) else None
                if not isinstance(changes, dict):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = start_write(vehicle, changes)
                code, ctype, body = json_body({"ok": ok, "note": snapshot(link, vehicle)["note"]}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
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
                if not name or not PARAM_NAME.match(name) or value is None or not math.isfinite(value):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = set_param(link, vehicle, tx, enc, name, value)
            elif path == "/arm":
                if not isinstance(data.get("arm"), bool):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = set_arm(link, vehicle, tx, enc, data["arm"])
            elif path == "/log":
                action = data.get("action")
                if action not in ("start", "stop"):
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = set_recording(vehicle, action, data.get("name") or "")
            elif path == "/process":
                proc = PROCESS_BY_ID.get(data.get("id"))
                if proc is None:
                    self._send(400, "application/json", b'{"ok":false}')
                    return
                ok = run_process(link, vehicle, tx, enc, proc)
            elif path == "/params/refresh":
                ok = refresh_params(vehicle)
            elif path == "/params/store":
                ok = start_write(vehicle, data.get("changes") if isinstance(data, dict) else {})
            elif path == "/params/reset":
                ok = ask_reset(link, vehicle, tx, enc)
            elif path == "/params/reboot":
                ok = ask_reboot(link, vehicle, tx, enc)
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
    ap.add_argument("--csv", default=None,
                    help="inicia ja um CSV em Rover/tools/logs com base esc_log_ (vazio = data/hora)")
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
    vehicle.ftp = FtpClient(link, tx, enc)
    if args.csv:
        filename = log_filename(args.csv)
        if filename is None:
            print("Nome de CSV invalido: %s" % args.csv, file=sys.stderr)
            return 1
        ok, note = vehicle.recorder.start(filename)
        print(note, flush=True)
        if not ok:
            return 1
    else:
        print("CSV parado. Inicie pela pagina de logs.", flush=True)
    threading.Thread(target=reader, args=(link, vehicle, stop, tx, enc), daemon=True).start()

    try:
        httpd = LocalServer(("0.0.0.0", args.http), make_handler(link, vehicle, tx, enc))
    except OSError as exc:
        stop.set()
        vehicle.recorder.stop()
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
        vehicle.recorder.stop()
        try:
            link.sock.close()
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
