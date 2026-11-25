#!/usr/bin/env python3
"""Interface local do rover. O navegador nao fala MAVLink: este processo
escuta a UDP, chama o HereLink e serve a pagina na rede local.

A pagina que abre e a de Operação (atalhos e leituras). Armar e desarmar ficam
na barra de estado em todas as abas. As outras trocam modo, parametro ou
preparam um ensaio (SCR_USER1). Nenhum ensaio arma sozinho.
O CSV so abre quando a pagina Logs pede e grava o mesmo conteudo do log do
esc_monitor: telemetria, mensagens, parametros e mudancas de modo e de armamento.
A aba Telemetria mostra os graficos ao vivo. A aba Scripts lista, copia, envia
e exclui os .lua de /APM/scripts e /APM/scripts/modules pelo MAVFtp, como o
Mission Planner em Config, MAVFtp.
Também limpa /APM/STRG_BAK (backups de storage) sem listar os arquivos na tela
e formata o cartão SD (MAV_CMD_STORAGE_FORMAT), como o sd_format.py.
A aba Logs baixa e apaga os dataflash do cartão (LOG_REQUEST_* / LOG_ERASE).
A aba Missão baixa, edita rumos/ré e envia a missão do rover (mesmo protocolo
do mission_editor.py), sem abrir outra porta UDP.
O mapa pede as imagens por este processo (/maptile/...), que guarda uma cópia
em Rover/tools/map_cache. Sem a imagem exata, amplia uma cópia mais grossa ou
reduz uma mais detalhada já salva. Sem nenhuma cópia e sem rede, devolve uma grade.
O rastro do GPS fica em memória (/track) para a missão comparar o percurso
feito com o caminho previsto. Ao terminar a missão no AUTO, um relatório HTML
é gravado em Rover/tools/mission_reports.

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
import io
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
import urllib.request
from urllib.parse import parse_qs, urlparse

try:
    from PIL import Image
except ImportError:
    Image = None

import esc_monitor as mon
import mission_core as mission

MSG_HEARTBEAT = 0
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_REQUEST_LIST = 21
MSG_PARAM_SET = 23
MSG_SET_MODE = 11
MSG_COMMAND_LONG = 76
MSG_FILE_TRANSFER_PROTOCOL = 110
MSG_LOG_REQUEST_LIST = 117
MSG_LOG_ENTRY = 118
MSG_LOG_REQUEST_DATA = 119
MSG_LOG_DATA = 120
MSG_LOG_ERASE = 121
MSG_LOG_REQUEST_END = 122

CRC_EXTRA = {
    MSG_HEARTBEAT: 50,
    MSG_PARAM_REQUEST_READ: 214,
    MSG_PARAM_REQUEST_LIST: 159,
    MSG_PARAM_SET: 168,
    MSG_SET_MODE: 89,
    MSG_COMMAND_LONG: 152,
    MSG_FILE_TRANSFER_PROTOCOL: 84,
    MSG_LOG_REQUEST_LIST: 128,
    MSG_LOG_ENTRY: 56,
    MSG_LOG_REQUEST_DATA: 116,
    MSG_LOG_DATA: 134,
    MSG_LOG_ERASE: 237,
    MSG_LOG_REQUEST_END: 203,
}
mission.register_mission_messages(CRC_EXTRA)

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_PREFLIGHT_STORAGE = 245
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_STORAGE_FORMAT = 526
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8

# 255 is SYSID_MYGCS on this rover (the id Mission Planner uses). With the
# planner closed, commands from any other id are easy for the link to drop.
GCS_SYSID = 255
GCS_COMPID = 190
COMMAND_WAIT_S = 4.0
COMMAND_RETRY_S = 0.4
SD_FORMAT_WAIT_S = 60.0
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
GPS_FIELDS = ("GPI_lat", "GPI_lon", "GPI_hdg", "GPI_vn", "GPI_ve")
# GPS_RAW_INT: tipo de fix e satélites, o mesmo par que o Mission Planner desenha no HUD.
GPS_STATUS_FIELDS = ("GPS_fix", "GPS_sats")


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

    def log_request_list(self, sysid, compid, start=0, end=0xFFFF):
        return self.pack(MSG_LOG_REQUEST_LIST, struct.pack(
            "<HHBB", int(start) & 0xFFFF, int(end) & 0xFFFF, sysid, compid))

    def log_request_data(self, sysid, compid, log_id, ofs, count):
        return self.pack(MSG_LOG_REQUEST_DATA, struct.pack(
            "<IIHBB", int(ofs) & 0xFFFFFFFF, int(count) & 0xFFFFFFFF,
            int(log_id) & 0xFFFF, sysid, compid))

    def log_request_end(self, sysid, compid):
        return self.pack(MSG_LOG_REQUEST_END, struct.pack("<BB", sysid, compid))

    def log_erase(self, sysid, compid):
        return self.pack(MSG_LOG_ERASE, struct.pack("<BB", sysid, compid))


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
LOG_DATA_CHUNK = 90
LOG_LIST_TIMEOUT_S = 12.0
LOG_DATA_IDLE_S = 2.0
LOG_WINDOW_BYTES = 16 * 1024
LOG_WINDOW_RETRIES = 6
LOG_MAX_BYTES = 40 * 1024 * 1024
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


class LogError(Exception):
    pass


def parse_log_entry(payload):
    """LOG_ENTRY wire order: time_utc, size, id, num_logs, last_log_num."""
    if payload is None or len(payload) < 14:
        return None
    time_utc, size, log_id, num_logs, last_log_num = struct.unpack("<IIHHH", payload[:14])
    return {
        "id": int(log_id),
        "num_logs": int(num_logs),
        "last_log_num": int(last_log_num),
        "time_utc": int(time_utc),
        "size": int(size),
    }


def parse_log_data(payload):
    """LOG_DATA wire order: ofs, id, count, data[90]."""
    if payload is None or len(payload) < 7:
        return None
    ofs, log_id, count = struct.unpack("<IHB", payload[:7])
    count = min(int(count), LOG_DATA_CHUNK, max(0, len(payload) - 7))
    return {
        "id": int(log_id),
        "ofs": int(ofs),
        "count": count,
        "data": bytes(payload[7:7 + count]),
    }


class LogClient:
    """Dataflash list / download / erase over LOG_REQUEST_* (Mission Planner path)."""

    def __init__(self, link, tx, enc):
        self.link = link
        self.tx = tx
        self.enc = enc
        self.cv = threading.Condition()
        self.entries = collections.deque()
        self.chunks = collections.deque()
        self.op_lock = threading.Lock()
        self._sysid = 1
        self._compid = 1
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

    def push_entry(self, entry):
        if not entry:
            return
        with self.cv:
            self.entries.append(entry)
            self.cv.notify()

    def push_data(self, packet):
        if not packet:
            return
        with self.cv:
            self.chunks.append(packet)
            self.cv.notify()

    def begin_progress(self, action, name, total=0, phase=""):
        with self.progress_lock:
            self.progress = {
                "active": True,
                "action": action,
                "name": name or "",
                "folder": "dataflash",
                "phase": phase or "",
                "done": 0,
                "total": int(total) if total else 0,
                "index": 1,
                "count": 1,
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

    def _begin(self, vehicle):
        with vehicle.lock:
            if vehicle.sysid is None:
                raise LogError("sem rover")
            if vehicle.armed:
                raise LogError("desarme antes de mexer nos logs do cartão")
            if vehicle.jobs or vehicle.load is not None:
                raise LogError("espere o comando atual terminar")
            self._sysid = vehicle.sysid
            self._compid = vehicle.compid or 1

    def _clear_queues(self):
        with self.cv:
            self.entries.clear()
            self.chunks.clear()

    def _send(self, frame):
        with self.tx:
            self.link.send(frame)

    def _end_transfer(self):
        self._send(self.enc.log_request_end(self._sysid, self._compid))

    def list_logs(self):
        self._clear_queues()
        self.begin_progress("list", "dataflash", total=0, phase="listar")
        self._send(self.enc.log_request_list(self._sysid, self._compid, 0, 0xFFFF))
        found = {}
        num_logs = None
        last_log_num = None
        deadline = time.monotonic() + LOG_LIST_TIMEOUT_S
        try:
            while time.monotonic() < deadline:
                with self.cv:
                    while self.entries:
                        entry = self.entries.popleft()
                        num_logs = entry["num_logs"]
                        last_log_num = entry["last_log_num"]
                        if entry["id"] == 0 and num_logs == 0:
                            self.update_progress(done=0, total=0, phase="pronto")
                            return []
                        if entry["id"] >= 1:
                            found[entry["id"]] = {
                                "id": entry["id"],
                                "size": entry["size"],
                                "time_utc": entry["time_utc"],
                            }
                        # Board stops listing when it has sent last_log_num.
                        if last_log_num is not None and entry["id"] == last_log_num:
                            logs = [found[i] for i in sorted(found)]
                            self.update_progress(done=len(logs), total=len(logs), phase="pronto")
                            return logs
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self.cv.wait(min(0.5, remaining))
            if found:
                logs = [found[i] for i in sorted(found)]
                self.update_progress(done=len(logs), total=len(logs), phase="pronto")
                return logs
            raise LogError("sem lista de logs do rover")
        finally:
            self._end_transfer()

    def _collect_window(self, log_id, want_ofs, want_count, received):
        """Ask for [want_ofs, want_ofs+want_count) and fold arriving LOG_DATA into received."""
        self._clear_queues()
        self._send(self.enc.log_request_data(
            self._sysid, self._compid, log_id, want_ofs, want_count))
        end = want_ofs + want_count
        last_rx = time.monotonic()
        stall_deadline = time.monotonic() + max(12.0, want_count / 200.0)
        while time.monotonic() < stall_deadline:
            with self.cv:
                while self.chunks:
                    pkt = self.chunks.popleft()
                    if pkt["id"] != log_id or pkt["count"] <= 0:
                        continue
                    ofs = pkt["ofs"]
                    if ofs < want_ofs or ofs >= end:
                        continue
                    if ofs not in received:
                        received[ofs] = pkt["data"]
                    last_rx = time.monotonic()
                    # Short last packet ends this window on the board.
                    if pkt["count"] < LOG_DATA_CHUNK:
                        return
                covered = sum(
                    len(blob) for ofs, blob in received.items()
                    if want_ofs <= ofs < end
                )
                if covered >= want_count:
                    return
                if time.monotonic() - last_rx >= LOG_DATA_IDLE_S:
                    return
                self.cv.wait(0.2)

    def _missing_in(self, received, start, end):
        """Gaps inside [start, end) given chunk offsets already received."""
        gaps = []
        cursor = start
        for ofs in sorted(o for o in received if start <= o < end):
            if ofs > cursor:
                gaps.append((cursor, ofs - cursor))
            cursor = max(cursor, ofs + len(received[ofs]))
        if cursor < end:
            gaps.append((cursor, end - cursor))
        return gaps

    def download(self, log_id, size):
        if size <= 0:
            raise LogError("log vazio")
        if size > LOG_MAX_BYTES:
            raise LogError("log grande demais (%s)" % format_bytes_py(size))
        name = "LOG%03d.BIN" % int(log_id)
        self.begin_progress("get", name, total=size, phase="baixar")
        received = {}
        try:
            # Small windows so the HereLink can lose packets without forcing a
            # full-file retry (the board rejects a new REQUEST while busy).
            ofs = 0
            while ofs < size:
                window = min(LOG_WINDOW_BYTES, size - ofs)
                end = ofs + window
                for _try in range(LOG_WINDOW_RETRIES):
                    gaps = self._missing_in(received, ofs, end)
                    if not gaps:
                        break
                    for gap_ofs, gap_count in gaps:
                        self.update_progress(
                            done=sum(len(v) for v in received.values()),
                            total=size, phase="baixar")
                        self._collect_window(log_id, gap_ofs, gap_count, received)
                        self._end_transfer()
                        time.sleep(0.12)
                if self._missing_in(received, ofs, end):
                    have = sum(len(v) for v in received.values())
                    raise LogError("download incompleto em ofs %d (%d de %d bytes)" % (
                        ofs, have, size))
                ofs = end
                self.update_progress(
                    done=sum(len(v) for v in received.values()),
                    total=size, phase="baixar")
            have = sum(len(v) for v in received.values())
            if have < size:
                raise LogError("download incompleto (%d de %d bytes)" % (have, size))
            out = bytearray(size)
            for chunk_ofs, blob in received.items():
                out[chunk_ofs:chunk_ofs + len(blob)] = blob
            self.update_progress(done=size, total=size, phase="pronto")
            return name, bytes(out)
        finally:
            try:
                self._end_transfer()
            except Exception:
                pass

    def erase(self):
        self.begin_progress("erase", "dataflash", total=0, phase="apagar")
        self._clear_queues()
        self._send(self.enc.log_erase(self._sysid, self._compid))
        # Erase has no ACK; give the board a moment and stop any transfer state.
        time.sleep(1.0)
        self._end_transfer()
        self.update_progress(done=1, total=1, phase="pronto")


def format_bytes_py(n):
    n = int(n) or 0
    if n < 1024:
        return "%d B" % n
    if n < 1048576:
        return "%d kB" % round(n / 1024)
    return "%.1f MB" % (n / 1048576.0)


class MissionError(Exception):
    pass


class MissionTransfer:
    """Download/upload MISSION_* on the shared MAVLink link (FtpClient pattern)."""

    def __init__(self, link, tx, enc):
        self.link = link
        self.tx = tx
        self.enc = enc
        self.cv = threading.Condition()
        self.inbox = collections.deque()
        self.op_lock = threading.Lock()
        self._sysid = 1
        self._compid = 1
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

    def push(self, msgid, payload):
        with self.cv:
            self.inbox.append((msgid, payload))
            self.cv.notify()

    def begin_progress(self, action, total=0, phase=""):
        with self.progress_lock:
            self.progress = {
                "active": True,
                "action": action,
                "name": "missão",
                "folder": "",
                "phase": phase or "",
                "done": 0,
                "total": int(total) if total else 0,
                "index": 1,
                "count": 1,
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

    def _clear(self):
        with self.cv:
            self.inbox.clear()

    def _send(self, frame):
        with self.tx:
            self.link.send(frame)

    def _wait(self, timeout):
        deadline = time.monotonic() + timeout
        with self.cv:
            while True:
                if self.inbox:
                    return self.inbox.popleft()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.cv.wait(remaining)

    def _begin(self, vehicle):
        with vehicle.lock:
            if vehicle.sysid is None:
                raise MissionError("sem rover")
            if vehicle.jobs or vehicle.load is not None:
                raise MissionError("espere o pedido atual terminar")
            self._sysid = vehicle.sysid
            self._compid = vehicle.compid or 1
        ftp = getattr(vehicle, "ftp", None)
        if ftp and ftp.progress_snapshot().get("active"):
            raise MissionError("espere o FTP terminar")
        logs = getattr(vehicle, "logs", None)
        if logs and logs.progress_snapshot().get("active"):
            raise MissionError("espere o download de logs terminar")

    def download(self):
        self.begin_progress("download", total=0, phase="pedir lista")
        self._clear()
        count = None
        end = time.monotonic() + 20
        last_req = 0.0
        while count is None:
            if time.monotonic() > end:
                raise MissionError("sem resposta ao pedido da missão")
            if time.monotonic() - last_req > 1.5:
                last_req = time.monotonic()
                self._send(self.enc.pack(
                    mission.MSG_MISSION_REQUEST_LIST,
                    struct.pack("<BB", self._sysid, self._compid)))
            got = self._wait(0.2)
            if got is None:
                continue
            msgid, payload = got
            if msgid == mission.MSG_MISSION_COUNT:
                count = struct.unpack("<H", payload[:2])[0]
        if count > mission.MISSION_MAX_ITEMS:
            raise MissionError("missão grande demais (%d itens)" % count)
        self.update_progress(done=0, total=count, phase="baixando")
        items = []
        for seq in range(count):
            got_item = None
            tries = 0
            last_req = 0.0
            while got_item is None:
                if time.monotonic() - last_req > 1.0:
                    if tries >= 8:
                        raise MissionError("item %d não chegou" % seq)
                    tries += 1
                    last_req = time.monotonic()
                    self._send(self.enc.pack(
                        mission.MSG_MISSION_REQUEST_INT,
                        struct.pack("<HBB", seq, self._sysid, self._compid)))
                got = self._wait(0.25)
                if got is None:
                    continue
                msgid, payload = got
                if msgid == mission.MSG_MISSION_ITEM_INT:
                    it = mission.Item.from_payload(payload)
                    if it.seq == seq:
                        got_item = it
            items.append(got_item)
            self.update_progress(done=seq + 1, total=count, phase="baixando")
        self._send(self.enc.pack(
            mission.MSG_MISSION_ACK,
            struct.pack("<BBB", self._sysid, self._compid, 0)))
        self.update_progress(done=count, total=count, phase="pronto")
        return items

    def upload(self, items):
        for i, it in enumerate(items):
            it.seq = i
        n = len(items)
        if n < 1:
            raise MissionError("missão vazia")
        if n > mission.MISSION_MAX_ITEMS:
            raise MissionError("missão grande demais (%d itens)" % n)
        self.begin_progress("upload", total=n, phase="enviar contagem")
        self._clear()
        last_count = 0.0
        last_activity = time.monotonic()
        last_seq = -1
        sent = set()
        start = time.monotonic()
        while True:
            now = time.monotonic()
            if not sent and now - last_count > 1.5:
                if now - start > 15:
                    raise MissionError("o rover não pediu os itens")
                last_count = now
                self._send(self.enc.pack(
                    mission.MSG_MISSION_COUNT,
                    struct.pack("<HBB", n, self._sysid, self._compid)))
            if now - last_activity > 10:
                raise MissionError("envio parado no item %d" % (max(sent) + 1 if sent else 0))
            if last_seq == n - 1 and now - last_activity > 1.5 and now - last_count > 1.5:
                last_count = now
                self._send(self.enc.pack(
                    mission.MSG_MISSION_ITEM_INT,
                    items[last_seq].payload(self._sysid, self._compid)))
            got = self._wait(0.2)
            if got is None:
                continue
            msgid, payload = got
            if msgid in (mission.MSG_MISSION_REQUEST_INT, mission.MSG_MISSION_REQUEST):
                seq = struct.unpack("<H", payload[:2])[0]
                if seq < n and not (seq == last_seq and time.monotonic() - last_activity < 0.7):
                    self._send(self.enc.pack(
                        mission.MSG_MISSION_ITEM_INT,
                        items[seq].payload(self._sysid, self._compid)))
                    sent.add(seq)
                    last_seq = seq
                    last_activity = time.monotonic()
                    self.update_progress(done=len(sent), total=n, phase="enviando")
            elif msgid == mission.MSG_MISSION_ACK and sent:
                result = payload[2]
                if result == 0 and (n - 1) in sent:
                    self.update_progress(done=n, total=n, phase="pronto")
                    return
                if result in (0, 13):
                    continue
                raise MissionError("rover recusou a missão: %s" %
                                  mission.ACK_RESULT.get(result, result))


def mission_progress(vehicle):
    client = getattr(vehicle, "mission", None)
    if client is None:
        return {"active": False}
    return client.progress_snapshot()


def mission_download(vehicle):
    client = vehicle.mission
    with client.op_lock:
        try:
            client._begin(vehicle)
            items = client.download()
        except MissionError as exc:
            client.end_progress()
            note = str(exc)
            print("mission: " + note, flush=True)
            return False, note, None
        except Exception as exc:
            client.end_progress()
            note = "falha ao baixar: %s" % exc
            print("mission: " + note, flush=True)
            return False, note, None
    client.end_progress()
    note = "missão baixada: %d itens" % len(items)
    print("mission: " + note, flush=True)
    remember_plan(vehicle, items)
    with vehicle.lock:
        push_message(vehicle, 6, note)
    return True, note, items


def mission_upload(vehicle, items):
    with vehicle.lock:
        if vehicle.armed:
            note = "desarme antes de enviar a missão"
            push_message(vehicle, 4, note)
            return False, note
    client = vehicle.mission
    with client.op_lock:
        try:
            client._begin(vehicle)
            client.upload(items)
        except MissionError as exc:
            client.end_progress()
            note = str(exc)
            print("mission: " + note, flush=True)
            return False, note
        except Exception as exc:
            client.end_progress()
            note = "falha ao enviar: %s" % exc
            print("mission: " + note, flush=True)
            return False, note
    client.end_progress()
    note = "missão enviada e aceita (%d itens)" % len(items)
    print("mission: " + note, flush=True)
    remember_plan(vehicle, items)
    with vehicle.lock:
        push_message(vehicle, 6, note)
    return True, note


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


def sdlog_progress(vehicle):
    logs = getattr(vehicle, "logs", None)
    if logs is None:
        return {"active": False}
    return logs.progress_snapshot()


def sdlog_list(vehicle):
    logs = vehicle.logs
    with logs.op_lock:
        try:
            logs._begin(vehicle)
            items = logs.list_logs()
        except LogError as exc:
            logs.end_progress()
            note = str(exc)
            print("sdlog: " + note, flush=True)
            return {"ok": False, "logs": [], "note": note}
    logs.end_progress()
    note = ("nenhum log no cartão" if not items
            else ("%d log%s" % (len(items), "" if len(items) == 1 else "s")))
    print("sdlog: " + note, flush=True)
    return {"ok": True, "logs": items, "note": note}


def sdlog_get(vehicle, log_id, size):
    logs = vehicle.logs
    with logs.op_lock:
        try:
            logs._begin(vehicle)
            name, blob = logs.download(int(log_id), int(size))
        except LogError as exc:
            logs.end_progress()
            note = str(exc)
            print("sdlog: " + note, flush=True)
            return False, note, None, None
        except Exception as exc:
            logs.end_progress()
            note = "falha ao baixar: %s" % exc
            print("sdlog: " + note, flush=True)
            return False, note, None, None
    logs.end_progress()
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, name)
        with open(path, "wb") as handle:
            handle.write(blob)
        note = "baixou %s (%s) em logs/%s" % (name, format_bytes_py(len(blob)), name)
    except OSError as exc:
        note = "baixou %s (%s); nao gravou em disco: %s" % (
            name, format_bytes_py(len(blob)), exc)
    print("sdlog: " + note, flush=True)
    with vehicle.lock:
        push_message(vehicle, 6, note)
    return True, note, name, blob


def sdlog_erase(vehicle):
    logs = vehicle.logs
    with logs.op_lock:
        try:
            logs._begin(vehicle)
            logs.erase()
        except LogError as exc:
            logs.end_progress()
            note = str(exc)
            print("sdlog: " + note, flush=True)
            return False, note
    logs.end_progress()
    note = "logs do cartão apagados"
    print("sdlog: " + note, flush=True)
    with vehicle.lock:
        push_message(vehicle, 4, note)
    return True, note


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
        except ConnectionResetError:
            # Windows UDP: ICMP "port unreachable" after a send to a closed peer.
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
        self.track = collections.deque(maxlen=20000)  # (seq, lat, lon)
        self.track_seq = 0
        self.track_gen = 0
        self.track_last = None
        self.plan_items = []
        self.run_active = False
        self.run_from = 0
        self.run_t0 = 0.0
        self.report_pending = False
        self.reports = load_report_index()


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


def _track_meters(lat1, lon1, lat2, lon2):
    mid = math.radians((lat1 + lat2) * 0.5)
    dy = (lat2 - lat1) * 111320.0
    dx = (lon2 - lon1) * 111320.0 * math.cos(mid)
    return math.hypot(dx, dy)


def append_track(vehicle, lat, lon):
    """Keep a thinned GPS trail for the mission comparison. Drops points closer than 0.25 m."""
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return
    if abs(lat) < 1e-5 and abs(lon) < 1e-5:
        return
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return
    with vehicle.lock:
        last = vehicle.track_last
        if last is not None and _track_meters(last[0], last[1], lat, lon) < 0.25:
            return
        vehicle.track_seq += 1
        vehicle.track.append((vehicle.track_seq, round(lat, 7), round(lon, 7)))
        vehicle.track_last = (lat, lon)


def track_payload(vehicle, after):
    with vehicle.lock:
        items = list(vehicle.track)
        gen = vehicle.track_gen
        seq = vehicle.track_seq
    if not items:
        return {"gen": gen, "reset": True, "after": seq, "points": []}
    oldest = items[0][0]
    if after <= 0 or after + 1 < oldest:
        chosen = items
        reset = True
    else:
        chosen = [item for item in items if item[0] > after]
        reset = False
    return {
        "gen": gen,
        "reset": reset,
        "after": items[-1][0],
        "points": [[item[0], item[1], item[2]] for item in chosen],
    }


def clear_track(vehicle):
    with vehicle.lock:
        vehicle.track.clear()
        vehicle.track_last = None
        vehicle.track_gen += 1
        return vehicle.track_gen, vehicle.track_seq


REPORT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mission_reports")
REPORT_NAME = re.compile(r"^missao_\d{8}_\d{6}(?:_\d+)?\.html$")
REPORT_INDEX = os.path.join(REPORT_DIR, "index.json")


def remember_plan(vehicle, items):
    """Keep the mission last sent to or read from the rover. Caller may hold no lock."""
    rows = [it.to_dict() for it in items]
    with vehicle.lock:
        vehicle.plan_items = rows


def note_mission_mode(vehicle, mode):
    """Mark the GPS trail from the moment AUTO starts. Caller holds vehicle.lock."""
    if mode == MODE_ID["AUTO"]:
        vehicle.run_active = True
        vehicle.run_from = vehicle.track_seq
        vehicle.run_t0 = time.time()
        vehicle.report_pending = False


def note_mission_text(vehicle, text):
    """Ask for a report when the rover says the mission is over. Caller holds vehicle.lock."""
    low = (text or "").strip().lower()
    if "mission complete" not in low and "missão completa" not in low and "missao completa" not in low:
        return
    if not vehicle.run_active:
        return
    vehicle.run_active = False
    vehicle.report_pending = True


def _plan_points(rows):
    points = []
    for i, row in enumerate(rows or []):
        try:
            lat = float(row.get("lat") or 0)
            lon = float(row.get("lon") or 0)
            command = int(row.get("command") or 0)
        except (TypeError, ValueError):
            continue
        if not (lat or lon):
            continue
        if i != 0 and command not in mission.NAV_WITH_POSITION:
            continue
        points.append({
            "seq": int(row.get("seq", i)),
            "name": str(row.get("name") or "CMD_%d" % command),
            "lat": lat,
            "lon": lon,
        })
    return points


def _local_meters(lat, lon, origin_lat, origin_lon):
    mid = math.radians((lat + origin_lat) * 0.5)
    dy = (lat - origin_lat) * 111320.0
    dx = (lon - origin_lon) * 111320.0 * math.cos(mid)
    return dx, dy


def _dist_to_segment(lat, lon, a, b):
    ax, ay = 0.0, 0.0
    bx, by = _local_meters(b["lat"], b["lon"], a["lat"], a["lon"])
    px, py = _local_meters(lat, lon, a["lat"], a["lon"])
    span = bx * bx + by * by
    t = 0.0 if span < 1e-8 else (px * bx + py * by) / span
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    dx = px - t * bx
    dy = py - t * by
    return math.hypot(dx, dy), t


def mission_report_stats(plan_rows, track, radius):
    """Compare the driven GPS trail with the planned path. Distances are metres."""
    plan = _plan_points(plan_rows)
    pts = []
    for lat, lon in track or []:
        if math.isfinite(lat) and math.isfinite(lon) and (lat or lon):
            pts.append((float(lat), float(lon)))
    driven = 0.0
    for i in range(1, len(pts)):
        driven += _track_meters(pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1])
    planned = 0.0
    legs = []
    for i in range(1, len(plan)):
        dist = _track_meters(plan[i - 1]["lat"], plan[i - 1]["lon"], plan[i]["lat"], plan[i]["lon"])
        planned += dist
        legs.append({
            "from": plan[i - 1]["seq"],
            "to": plan[i]["seq"],
            "to_name": plan[i]["name"],
            "planned_m": dist,
            "sum": 0.0,
            "n": 0,
            "max": 0.0,
        })
    errors = []
    for lat, lon in pts:
        best_i = None
        best_d = None
        for i in range(1, len(plan)):
            dist, _t = _dist_to_segment(lat, lon, plan[i - 1], plan[i])
            if best_d is None or dist < best_d:
                best_d = dist
                best_i = i - 1
        if best_d is None:
            continue
        errors.append(best_d)
        leg = legs[best_i]
        leg["sum"] += best_d
        leg["n"] += 1
        if best_d > leg["max"]:
            leg["max"] = best_d
    reached = []
    limit = max(0.5, float(radius or 2.0))
    for point in plan:
        if point["seq"] == 0:
            continue
        nearest = None
        for lat, lon in pts:
            dist = _track_meters(lat, lon, point["lat"], point["lon"])
            if nearest is None or dist < nearest:
                nearest = dist
        reached.append({
            "seq": point["seq"],
            "name": point["name"],
            "nearest_m": nearest,
            "ok": nearest is not None and nearest <= limit,
        })
    for leg in legs:
        leg["mean"] = (leg["sum"] / leg["n"]) if leg["n"] else None
        del leg["sum"]
    return {
        "plan": plan,
        "track": pts,
        "planned_m": planned,
        "driven_m": driven,
        "mean_m": (sum(errors) / len(errors)) if errors else None,
        "max_m": max(errors) if errors else None,
        "samples": len(errors),
        "legs": legs,
        "reached": reached,
        "radius_m": limit,
    }


def _esc(text):
    return (str(text)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;"))


def _fmt_m(value):
    if value is None:
        return "—"
    return "%.2f m" % value


def _report_svg(stats):
    pts = [(p["lat"], p["lon"]) for p in stats["plan"]] + list(stats["track"])
    if len(pts) < 2:
        return "<p>Sem pontos suficientes para o desenho.</p>"
    lat0 = sum(p[0] for p in pts) / len(pts)
    lon0 = sum(p[1] for p in pts) / len(pts)
    xy = [_local_meters(lat, lon, lat0, lon0) for lat, lon in pts]
    xs = [p[0] for p in xy]
    ys = [p[1] for p in xy]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 1.0)
    width, height, pad = 640, 420, 28

    def proj(lat, lon):
        x, y = _local_meters(lat, lon, lat0, lon0)
        px = pad + (x - minx) / span * (width - 2 * pad)
        py = height - pad - (y - miny) / span * (height - 2 * pad)
        return px, py

    def poly(points, color, width_px):
        if len(points) < 2:
            return ""
        body = " ".join("%.1f,%.1f" % proj(lat, lon) for lat, lon in points)
        return '<polyline fill="none" stroke="%s" stroke-width="%s" points="%s"/>' % (color, width_px, body)

    marks = []
    for point in stats["plan"]:
        x, y = proj(point["lat"], point["lon"])
        marks.append('<circle cx="%.1f" cy="%.1f" r="4" fill="#073e73"/>' % (x, y))
        marks.append('<text x="%.1f" y="%.1f" font-size="12" fill="#073e73">%s</text>' % (
            x + 6, y - 6, _esc(point["seq"])))
    return (
        '<svg viewBox="0 0 %d %d" width="100%%" role="img" aria-label="Caminho previsto e percorrido">'
        '<rect width="100%%" height="100%%" fill="#f7f4ee"/>'
        '%s%s%s</svg>'
    ) % (width, height, poly([(p["lat"], p["lon"]) for p in stats["plan"]], "#8fbf7a", 8),
         poly(stats["track"], "#1f6feb", 3), "".join(marks))


def mission_report_html(when, stats):
    leg_rows = []
    for leg in stats["legs"]:
        leg_rows.append(
            "<tr><td>%d → %d %s</td><td>%s</td><td>%s</td><td>%s</td><td>%d</td></tr>" % (
                leg["from"], leg["to"], _esc(leg["to_name"]),
                _fmt_m(leg["planned_m"]), _fmt_m(leg["mean"]), _fmt_m(leg["max"]), leg["n"],
            )
        )
    reach_rows = []
    for row in stats["reached"]:
        reach_rows.append(
            "<tr><td>%d %s</td><td>%s</td><td>%s</td></tr>" % (
                row["seq"], _esc(row["name"]), _fmt_m(row["nearest_m"]),
                "sim" if row["ok"] else "não",
            )
        )
    plan_note = "" if stats["plan"] else (
        "<p>O painel não tinha a missão do rover. Envie ou baixe a missão antes do AUTO "
        "para comparar com o caminho previsto.</p>"
    )
    return """<!DOCTYPE html>
<html lang="pt">
<head>
<meta charset="utf-8">
<title>Relatório de missão %s</title>
<style>
  body { margin: 24px; font: 16px "Segoe UI", sans-serif; color: #1c1814; background: #f7f4ee; }
  h1 { font-size: 22px; margin: 0 0 8px; }
  p { margin: 8px 0; }
  table { border-collapse: collapse; width: 100%%; margin: 12px 0 24px; }
  th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #ddd4c8; }
  th { font-size: 13px; color: #5c564e; }
  .nums { display: flex; flex-wrap: wrap; gap: 16px; margin: 12px 0 20px; }
  .nums div { min-width: 140px; }
  .nums strong { display: block; font-size: 20px; }
</style>
</head>
<body>
<h1>Relatório de missão</h1>
<p>%s</p>
%s
<div class="nums">
  <div>Previsto<strong>%s</strong></div>
  <div>Percorrido<strong>%s</strong></div>
  <div>Erro médio<strong>%s</strong></div>
  <div>Erro máximo<strong>%s</strong></div>
</div>
%s
<h2>Trechos</h2>
<table>
<tr><th>Trecho</th><th>Previsto</th><th>Erro médio</th><th>Erro máximo</th><th>Amostras</th></tr>
%s
</table>
<h2>Pontos</h2>
<p>Um ponto conta como alcançado se o rover passou a até %.1f m dele.</p>
<table>
<tr><th>Ponto</th><th>Passagem mais perto</th><th>Alcançado</th></tr>
%s
</table>
</body>
</html>
""" % (
        _esc(when), _esc(when), plan_note,
        _fmt_m(stats["planned_m"]) if stats["plan"] else "—",
        _fmt_m(stats["driven_m"]),
        _fmt_m(stats["mean_m"]),
        _fmt_m(stats["max_m"]),
        _report_svg(stats),
        "".join(leg_rows) or "<tr><td colspan=\"5\">Sem trechos previstos.</td></tr>",
        stats["radius_m"],
        "".join(reach_rows) or "<tr><td colspan=\"3\">Sem pontos previstos.</td></tr>",
    )


def _report_public(row):
    return {
        "file": row["file"],
        "at": row["at"],
        "mean_m": row["mean_m"],
        "max_m": row["max_m"],
        "planned_m": row["planned_m"],
        "driven_m": row["driven_m"],
    }


def maybe_write_mission_report(vehicle):
    """Write the comparison file after Mission Complete. Safe to call often."""
    with vehicle.lock:
        if not vehicle.report_pending:
            return None
        vehicle.report_pending = False
        track = [(lat, lon) for seq, lat, lon in vehicle.track if seq >= vehicle.run_from]
        plan = list(vehicle.plan_items)
        started = vehicle.run_t0 or time.time()
        try:
            radius = float(vehicle.params.get("WP_RADIUS", 2.0))
        except (TypeError, ValueError):
            radius = 2.0
    try:
        stats = mission_report_stats(plan, track, radius)
        when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started))
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        os.makedirs(REPORT_DIR, exist_ok=True)
        name = "missao_%s.html" % stamp
        extra = 2
        while os.path.exists(os.path.join(REPORT_DIR, name)):
            name = "missao_%s_%d.html" % (stamp, extra)
            extra += 1
        path = os.path.join(REPORT_DIR, name)
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(mission_report_html(when, stats))
        row = {
            "file": name,
            "at": when,
            "mean_m": None if stats["mean_m"] is None else round(stats["mean_m"], 2),
            "max_m": None if stats["max_m"] is None else round(stats["max_m"], 2),
            "planned_m": round(stats["planned_m"], 1),
            "driven_m": round(stats["driven_m"], 1),
        }
    except Exception as exc:
        print("relatorio: %s" % exc, flush=True)
        with vehicle.lock:
            push_message(vehicle, 4, "Relatório da missão não foi salvo: %s" % exc)
        return None
    with vehicle.lock:
        vehicle.reports.insert(0, row)
        del vehicle.reports[20:]
        saved = list(vehicle.reports)
        note = "Relatório salvo: %s" % name
        push_message(vehicle, 6, note)
        vehicle.note = note
    try:
        save_report_index(saved)
    except OSError as exc:
        print("relatorio: indice %s" % exc, flush=True)
    print("relatorio: " + name, flush=True)
    return row


def load_report_index():
    try:
        with open(REPORT_INDEX, encoding="utf-8") as handle:
            rows = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return []
    out = []
    if not isinstance(rows, list):
        return out
    for row in rows:
        if isinstance(row, dict) and REPORT_NAME.match(str(row.get("file") or "")):
            out.append(row)
        if len(out) >= 20:
            break
    return out


def save_report_index(rows):
    os.makedirs(REPORT_DIR, exist_ok=True)
    tmp = REPORT_INDEX + ".part"
    with open(tmp, "w", encoding="utf-8", newline="\n") as handle:
        json.dump(rows[:20], handle)
    os.replace(tmp, REPORT_INDEX)


def report_list(vehicle):
    with vehicle.lock:
        return [_report_public(row) for row in vehicle.reports]


def report_file(name):
    if not REPORT_NAME.match(name or ""):
        return None
    path = os.path.join(REPORT_DIR, name)
    if not os.path.isfile(path):
        return None
    return path


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
            if msgid == MSG_LOG_ENTRY:
                vehicle.logs.push_entry(parse_log_entry(payload))
                continue
            if msgid == MSG_LOG_DATA:
                vehicle.logs.push_data(parse_log_data(payload))
                continue
            if msgid in (mission.MSG_MISSION_COUNT, mission.MSG_MISSION_ITEM_INT,
                         mission.MSG_MISSION_REQUEST, mission.MSG_MISSION_REQUEST_INT,
                         mission.MSG_MISSION_ACK):
                vehicle.mission.push(msgid, payload)
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
                        note_mission_mode(vehicle, msg["mode"])
                continue
            if msgid == mon.MSG_STATUSTEXT:
                # Not tied to the first heartbeat. Mission Planner lists every
                # STATUSTEXT from the Pixhawk, including long lines sent in pieces.
                for sev, text in take_statustext(vehicle, payload):
                    with vehicle.lock:
                        push_message(vehicle, sev, text)
                        vehicle.recorder.write("STATUSTEXT", int(sev), "", text)
                        note_mission_text(vehicle, text)
                    maybe_write_mission_report(vehicle)
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
                lat = lon = None
                for name, value in msg["values"]:
                    add_value(vehicle, name, value)
                    vehicle.recorder.write(name, value, boot)
                    if name == "GPI_lat":
                        lat = value
                    elif name == "GPI_lon":
                        lon = value
                if lat is not None and lon is not None:
                    append_track(vehicle, lat, lon)


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
        link.send(enc.command_long(
            sysid, compid, job["command"],
            job.get("p1", 0), job.get("p2", 0), job.get("p3", 0),
            job.get("p4", 0), job.get("p5", 0), job.get("p6", 0), job.get("p7", 0)))


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
            if job["kind"] == "wait":
                span = job["seconds"]
            else:
                span = job.get("seconds", COMMAND_WAIT_S)
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
        # Long tasks (SD format) ACK with IN_PROGRESS first; keep waiting for
        # ACCEPTED and do not re-send (a second request is TEMPORARILY_REJECTED).
        if job["kind"] == "command":
            ack = vehicle.last_ack
            if (ack and ack[0] == job["command"] and ack[2] >= job.get("opened", 0)
                    and ack[1] == 5):
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


def ask_sd_format(link, vehicle, tx, enc):
    """Wipe the FC microSD (scripts, logs, terrain). Params and mission stay.

    Same MAV_CMD_STORAGE_FORMAT path as Rover/tools/sd_format.py. Blocks until
    the board ACKs ACCEPTED, fails, or the wait times out.
    """
    with vehicle.lock:
        if _refuse_if_busy(vehicle, "formatar o cartão SD"):
            return False, vehicle.note
        if vehicle.jobs or vehicle.load is not None:
            vehicle.note = "espere o comando atual terminar"
            push_message(vehicle, 4, vehicle.note)
            return False, vehicle.note
        opened = time.monotonic()
    ok = enqueue(
        vehicle,
        [{
            "kind": "command",
            "command": MAV_CMD_STORAGE_FORMAT,
            "p1": 1.0,
            "p2": 1.0,
            "name": "formatar SD",
            "seconds": SD_FORMAT_WAIT_S,
        }],
        "cartão SD formatado — reinicie a placa e reenvie os scripts",
        remember=False,
        sev=4,
    )
    if not ok:
        with vehicle.lock:
            return False, vehicle.note

    deadline = time.monotonic() + SD_FORMAT_WAIT_S + 5.0
    while time.monotonic() < deadline:
        with vehicle.lock:
            pending = any(
                (j.get("kind") == "command" and j.get("command") == MAV_CMD_STORAGE_FORMAT)
                or j.get("kind") == "note"
                for j in vehicle.jobs
            )
            note = vehicle.note
            ack = vehicle.last_ack
        if not pending:
            success = bool(
                ack and ack[0] == MAV_CMD_STORAGE_FORMAT
                and ack[1] == 0 and ack[2] >= opened
            )
            return success, note
        time.sleep(0.2)

    with vehicle.lock:
        note = vehicle.note or "sem confirmação do rover"
        pending = any(
            j.get("kind") == "command" and j.get("command") == MAV_CMD_STORAGE_FORMAT
            for j in vehicle.jobs
        )
        if pending:
            vehicle.jobs.clear()
            vehicle.revert = None
            note = "Sem resposta ao pedir formatar SD."
            vehicle.note = note
            push_message(vehicle, 4, note)
    return False, note


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
        gps = {name: vehicle.latest[name] for name in GPS_FIELDS if name in vehicle.latest}
        for name in GPS_STATUS_FIELDS:
            if name in vehicle.latest:
                gps[name] = vehicle.latest[name]
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
        "gps": gps,
        "blocked_modes": blocked,
        "reset_inputs": reset_inputs,
        "param_epoch": param_epoch,
        "recording": recording,
        "reports": report_list(vehicle),
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
        "mission_commands": mission.command_catalog(),
    }


def json_body(payload, code=200):
    body = json.dumps(payload).encode("utf-8")
    return code, "application/json", body


# Ruas (OpenStreetMap, até o zoom 19, com cópia @2x) e satélite (Esri, até 22).
# Acima disso a página amplia a imagem guardada.
MAP_CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_cache")
MAP_NATIVE_ZOOM = 19
MAP_ZOOM_CAP = {"osm": 19, "sat": 22}
TILE_URLS = {
    "osm": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "sat": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
}
TILE_UA = "GRIN-rover-hmi/1.0 (local ground station; map tile cache)"
_tile_slots = threading.Semaphore(4)
_tile_lock_guard = threading.Lock()
_tile_locks = {}
_tile_fail_lock = threading.Lock()
_tile_fail_until = {}


def _png_chunk(tag, data):
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def _blank_png():
    # 1x1 transparente. O navegador recebe 200 e não registra falha de rede.
    raw = b"\x00\x00\x00\x00\x00"
    ihdr = struct.pack(">IIBBBBB", 1, 1, 8, 6, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", zlib.compress(raw))
        + _png_chunk(b"IEND", b"")
    )


BLANK_TILE = _blank_png()


def _grid_tile():
    """256px pale grid, so a missing tile is never a white hole."""
    if Image is None:
        return BLANK_TILE
    im = Image.new("RGB", (256, 256), (231, 239, 224))
    pix = im.load()
    line = (186, 198, 188)
    edge = (120, 140, 150)
    for i in range(0, 256, 32):
        for t in range(256):
            pix[i, t] = line
            pix[t, i] = line
    for t in range(256):
        pix[0, t] = edge
        pix[t, 0] = edge
    out = io.BytesIO()
    im.save(out, format="PNG", optimize=True)
    return out.getvalue()


GRID_TILE = _grid_tile()
_RESAMPLE = None
_RESAMPLE_FAST = None
if Image is not None:
    _pool = getattr(Image, "Resampling", Image)
    _RESAMPLE = getattr(_pool, "BILINEAR", Image.BILINEAR)
    _RESAMPLE_FAST = getattr(_pool, "NEAREST", Image.NEAREST)


def _image_type(data):
    if len(data) >= 8 and data.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if len(data) >= 3 and data.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    return None


def _tile_file_lock(key):
    with _tile_lock_guard:
        lock = _tile_locks.get(key)
        if lock is None:
            lock = threading.Lock()
            _tile_locks[key] = lock
        return lock


def map_tile_file(source, z, x, y, retina=False):
    cap = MAP_ZOOM_CAP.get(source)
    if cap is None or z < 0 or z > cap:
        return None
    n = 1 << z
    if x < 0 or y < 0 or x >= n or y >= n:
        return None
    name = "%d@2.img" % y if retina else "%d.img" % y
    return os.path.join(MAP_CACHE_DIR, source, str(z), str(x), name)


def _read_cached_tile(path):
    try:
        with open(path, "rb") as handle:
            data = handle.read()
    except OSError:
        return None
    if _image_type(data) is None:
        return None
    return data


def _upscale_from_cache(source, z, x, y):
    """Build this tile from a coarser one already on disk. None if nothing is saved."""
    if Image is None or z <= 0:
        return None
    for shift in range(1, 7):
        parent_z = z - shift
        if parent_z < 0:
            break
        parent_x = x >> shift
        parent_y = y >> shift
        path = map_tile_file(source, parent_z, parent_x, parent_y)
        if not path:
            continue
        data = _read_cached_tile(path)
        if not data:
            continue
        try:
            image = Image.open(io.BytesIO(data)).convert("RGBA")
        except Exception:
            continue
        span = 1 << shift
        width, height = image.size
        cell_w = width / float(span)
        cell_h = height / float(span)
        ox = x - (parent_x << shift)
        oy = y - (parent_y << shift)
        box = (ox * cell_w, oy * cell_h, (ox + 1) * cell_w, (oy + 1) * cell_h)
        sample = _RESAMPLE_FAST if shift > 2 else _RESAMPLE
        crop = image.crop(box).resize((256, 256), sample)
        out = io.BytesIO()
        crop.save(out, format="PNG")
        return out.getvalue()
    return None


def _downsample_from_cache(source, z, x, y, min_coverage=0.0):
    """Monta este tile com as cópias mais detalhadas já salvas. None se a cobertura não chega."""
    cap = MAP_ZOOM_CAP.get(source, MAP_NATIVE_ZOOM)
    if Image is None or z >= cap:
        return None
    for shift in range(1, 5):
        child_z = z + shift
        if child_z > cap:
            break
        span = 1 << shift
        cell = 256 // span
        total = span * span
        canvas = None
        found = 0
        sample = _RESAMPLE if shift <= 2 else _RESAMPLE_FAST
        for dy in range(span):
            for dx in range(span):
                path = map_tile_file(source, child_z, (x << shift) + dx, (y << shift) + dy)
                if not path:
                    continue
                data = _read_cached_tile(path)
                if not data:
                    continue
                try:
                    image = Image.open(io.BytesIO(data)).convert("RGBA")
                except Exception:
                    continue
                if canvas is None:
                    canvas = Image.new("RGBA", (256, 256), (231, 239, 224, 255))
                canvas.paste(image.resize((cell, cell), sample), (dx * cell, dy * cell))
                found += 1
        if not found or found / float(total) < min_coverage:
            continue
        out = io.BytesIO()
        canvas.save(out, format="PNG")
        return out.getvalue()
    return None


def _download_tile(source, z, x, y, retina=False):
    url = TILE_URLS[source].format(z=z, x=x, y=y)
    if retina and source == "osm" and url.endswith(".png"):
        url = url[:-4] + "@2x.png"
    req = urllib.request.Request(url, headers={
        "User-Agent": TILE_UA,
        "Accept": "image/png,image/jpeg;q=0.9,*/*;q=0.1",
    })
    try:
        with _tile_slots:
            with urllib.request.urlopen(req, timeout=12) as resp:
                data = resp.read(2_000_000)
    except Exception:
        return None
    if _image_type(data) is None:
        return None
    return data


def _cached_overview(source, z, x, y, min_coverage):
    scaled = _upscale_from_cache(source, z, x, y)
    if scaled:
        return scaled
    return _downsample_from_cache(source, z, x, y, min_coverage)


def _save_tile(dest, data):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    tmp = dest + ".part"
    with open(tmp, "wb") as handle:
        handle.write(data)
    os.replace(tmp, dest)


def map_tile_payload(path, only_cache=False, prefer_net=False, near=False, retina=False):
    """Devolve (bytes, content-type, hit|fetch|scale|empty). empty é uma grade, nunca erro."""
    match = re.fullmatch(r"/maptile/(osm|sat)/(\d+)/(\d+)/(\d+)\.png", path)
    if not match:
        return GRID_TILE, "image/png", "empty"
    source = match.group(1)
    z, x, y = int(match.group(2)), int(match.group(3)), int(match.group(4))
    dest = map_tile_file(source, z, x, y)
    if dest is None:
        return GRID_TILE, "image/png", "empty"
    if retina and source == "osm":
        sharp = map_tile_file(source, z, x, y, retina=True)
        sharp_data = _read_cached_tile(sharp) if sharp else None
        if sharp_data is not None:
            return sharp_data, _image_type(sharp_data), "hit"
        if sharp and not only_cache and not near:
            data = _download_tile(source, z, x, y, retina=True)
            if data:
                try:
                    _save_tile(sharp, data)
                except OSError:
                    return data, _image_type(data), "fetch"
                return data, _image_type(data), "fetch"
    cached = _read_cached_tile(dest)
    if cached is not None:
        return cached, _image_type(cached), "hit"
    if near:
        scaled = _cached_overview(source, z, x, y, 0.55)
        if scaled:
            return scaled, "image/png", "scale"
        return GRID_TILE, "image/png", "empty"
    if only_cache:
        return GRID_TILE, "image/png", "empty"
    key = (source, z, x, y)
    now = time.time()
    with _tile_fail_lock:
        if now < _tile_fail_until.get(key, 0):
            scaled = None if prefer_net else _cached_overview(source, z, x, y, 0.0)
            if scaled:
                return scaled, "image/png", "scale"
            return GRID_TILE, "image/png", "empty"
    with _tile_file_lock(key):
        cached = _read_cached_tile(dest)
        if cached is not None:
            return cached, _image_type(cached), "hit"
        data = _download_tile(source, z, x, y)
        if data is None:
            with _tile_fail_lock:
                _tile_fail_until[key] = time.time() + 20
            if not prefer_net:
                scaled = _cached_overview(source, z, x, y, 0.0)
                if scaled:
                    return scaled, "image/png", "scale"
            return GRID_TILE, "image/png", "empty"
        try:
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            tmp = dest + ".part"
            with open(tmp, "wb") as handle:
                handle.write(data)
            os.replace(tmp, dest)
        except OSError:
            return data, _image_type(data), "fetch"
        return data, _image_type(data), "fetch"


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

        def _send(self, code, ctype, body, extra=None, cache="no-store"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache)
            for key, value in extra or ():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                return

        def _map_tile(self, path, query):
            params = parse_qs(query)
            only = params.get("only", ["0"])[0] == "1"
            near = params.get("near", ["0"])[0] == "1"
            prefer_net = params.get("fetch", ["0"])[0] == "1"
            retina = params.get("retina", ["0"])[0] == "1"
            body, ctype, kind = map_tile_payload(
                path, only_cache=only, prefer_net=prefer_net, near=near, retina=retina)
            cache = "no-store" if kind in ("empty", "scale") else "public, max-age=2592000"
            self._send(200, ctype, body, (("X-Map-Tile", kind),), cache)

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
            parsed = urlparse(self.path)
            path = parsed.path
            if path.startswith("/maptile/"):
                self._map_tile(path, parsed.query)
                return
            if path == "/track":
                try:
                    after = int(parse_qs(parsed.query).get("after", ["0"])[0])
                except (TypeError, ValueError):
                    after = 0
                code, ctype, body = json_body(track_payload(vehicle, after))
                self._send(code, ctype, body)
                return
            if path.startswith("/mission/report/"):
                file_path = report_file(path[len("/mission/report/"):])
                if file_path is None:
                    self._send(404, "text/plain; charset=utf-8", "relatório não encontrado".encode("utf-8"))
                    return
                with open(file_path, "rb") as handle:
                    body = handle.read()
                self._send(200, "text/html; charset=utf-8", body)
                return
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
            if path == "/sdlogs":
                code, ctype, body = json_body(sdlog_list(vehicle))
                self._send(code, ctype, body)
                return
            if path == "/sdlogs/progress":
                code, ctype, body = json_body(sdlog_progress(vehicle))
                self._send(code, ctype, body)
                return
            if path == "/mission/progress":
                code, ctype, body = json_body(mission_progress(vehicle))
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
            if path == "/track/clear":
                self._read_json(8000)
                gen, seq = clear_track(vehicle)
                code, ctype, body = json_body({
                    "ok": True, "note": "rastro apagado", "gen": gen, "after": seq,
                })
                self._send(code, ctype, body)
                return
            if path == "/mission/download":
                self._read_json(8000)
                ok, note, items = mission_download(vehicle)
                payload = {"ok": ok, "note": note}
                if ok and items is not None:
                    payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/mission/new":
                data = self._read_json(8000)
                lat = lon = 0.0
                if isinstance(data, dict):
                    try:
                        lat = float(data.get("lat") or 0)
                        lon = float(data.get("lon") or 0)
                    except (TypeError, ValueError):
                        lat = lon = 0.0
                with vehicle.lock:
                    if (not lat and not lon) and "GPI_lat" in vehicle.latest and "GPI_lon" in vehicle.latest:
                        lat = float(vehicle.latest["GPI_lat"])
                        lon = float(vehicle.latest["GPI_lon"])
                items = mission.empty_mission(lat, lon)
                payload = {"ok": True, "note": "missão nova (home em %.6f, %.6f)" % (lat, lon)}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/mission/analyse":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                except ValueError as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                payload = {"ok": True, "note": ""}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/mission/upload":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                except ValueError as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                ok, note = mission_upload(vehicle, items)
                code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/mission/import":
                data = self._read_json(400000)
                text = data.get("text") if isinstance(data, dict) else None
                if not isinstance(text, str):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.parse_waypoints_text(text)
                except ValueError as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                payload = {"ok": True, "note": "arquivo importado: %d itens" % len(items)}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/mission/export":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                except ValueError as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                text = mission.format_waypoints_text(items)
                self._send(200, "text/plain; charset=utf-8", text.encode("utf-8"), (
                    ("Content-Disposition", 'attachment; filename="mission.waypoints"'),
                ))
                return
            if path == "/mission/rebuild":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                    seq = int(data.get("seq"))
                    want = bool(data.get("reverse"))
                    items, keep, flipped = mission.apply_reverse_toggle(items, seq, want)
                except (TypeError, ValueError) as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                note = "trecho até o WP %d agora %s" % (keep, "de RÉ" if want else "de frente")
                if flipped is not None:
                    note += "; alinhamento do WP anterior virou %.0f graus" % flipped
                payload = {"ok": True, "note": note, "selected": keep}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/mission/heading":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                    seq = int(data.get("seq"))
                    if seq <= 0 or seq >= len(items) or items[seq].command != mission.CMD_WAYPOINT:
                        raise ValueError("só WAYPOINT tem rumo do próximo trecho")
                    brg = mission.leg_bearing(items, seq)
                    if brg is None:
                        raise ValueError("não há waypoint depois deste")
                    items[seq].z = round(mission.front_heading(items, seq, brg), 1)
                except (TypeError, ValueError) as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                payload = {"ok": True, "note": "rumo do próximo trecho: %.0f graus" % items[seq].z,
                           "selected": seq}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/mission/fix":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                except ValueError as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                n = 0
                info = mission.analyse(items)
                for i, it in enumerate(items):
                    if i and it.command == mission.CMD_WAYPOINT and not mission.is_pass(it.z) and (
                            it.z > 360 or it.z < -1 or info[i][2]):
                        it.z = -1.0
                        n += 1
                note = ("%d waypoints com Alt de altitude ou conflito viraram 'passa direto'" % n
                        if n else "nenhum waypoint suspeito para corrigir")
                payload = {"ok": True, "note": note}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/mission/all-next":
                data = self._read_json(400000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    items = mission.items_from_dicts(data.get("items"))
                except ValueError as exc:
                    code, ctype, body = json_body({"ok": False, "note": str(exc)}, 400)
                    self._send(code, ctype, body)
                    return
                n = 0
                for i, it in enumerate(items):
                    if i and it.command == mission.CMD_WAYPOINT and not mission.is_pass(it.z):
                        brg = mission.leg_bearing(items, i)
                        if brg is not None:
                            it.z = round(mission.front_heading(items, i, brg), 1)
                            n += 1
                note = "%d alinhamentos agora apontam para o próximo trecho" % n
                payload = {"ok": True, "note": note}
                payload.update(mission.items_payload(items))
                code, ctype, body = json_body(payload)
                self._send(code, ctype, body)
                return
            if path == "/storage/clear-bak":
                self._read_json(8000)
                ok, note = storage_bak_clear(vehicle)
                code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/scripts/format":
                self._read_json(8000)
                ok, note = ask_sd_format(link, vehicle, tx, enc)
                code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/sdlogs/erase":
                self._read_json(8000)
                ok, note = sdlog_erase(vehicle)
                code, ctype, body = json_body({"ok": ok, "note": note}, 200 if ok else 409)
                self._send(code, ctype, body)
                return
            if path == "/sdlogs/get":
                data = self._read_json(8000)
                if not isinstance(data, dict):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                try:
                    log_id = int(data.get("id"))
                    size = int(data.get("size"))
                except (TypeError, ValueError):
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                if log_id < 1 or size < 0:
                    self._send(400, "application/json", b'{"ok":false,"note":"pedido invalido"}')
                    return
                ok, note, name, blob = sdlog_get(vehicle, log_id, size)
                if not ok or blob is None:
                    code, ctype, body = json_body({"ok": False, "note": note}, 409)
                    self._send(code, ctype, body)
                    return
                filename = name or ("LOG%03d.BIN" % log_id)
                self._send(200, "application/octet-stream", blob, (
                    ("Content-Disposition", 'attachment; filename="%s"' % filename),
                ))
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
    vehicle.logs = LogClient(link, tx, enc)
    vehicle.mission = MissionTransfer(link, tx, enc)
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
