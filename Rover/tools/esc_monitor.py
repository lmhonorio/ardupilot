#!/usr/bin/env python3
"""
Monitor ao vivo dos motores CAN (esc_can.lua) pelo HereLink.

Recebe por MAVLink os valores que o esc_can.lua publica com ESC_GCS_HZ > 0:
  ESCn_I     corrente medida (A)
  ESCn_ICMD  corrente comandada (A)
  ESCn_RPM   rpm da roda
  ESC_TGT_L / ESC_TGT_R  rpm alvo de cada lado
e as mensagens STATUSTEXT (por exemplo "ESC: ESC 2 lost ...").

Mostra um grafico ao vivo e grava tudo em CSV. O MAVLink e decodificado aqui
mesmo (sem pymavlink); o grafico usa matplotlib.

Exemplos:
  python esc_monitor.py                              # escuta UDP 14550
  python esc_monitor.py --conn tcp:192.168.43.1:5760 # TCP no controle HereLink
  python esc_monitor.py --conn udp:0.0.0.0:14551     # porta encaminhada pelo Mission Planner
  python esc_monitor.py --no-plot                    # so console + CSV
"""

import argparse
import collections
import csv
import datetime
import socket
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Minimal MAVLink v1/v2 decoder for the three messages used here
# ---------------------------------------------------------------------------
MSG_HEARTBEAT = 0
MSG_NAMED_VALUE_FLOAT = 251
MSG_STATUSTEXT = 253

CRC_EXTRA = {MSG_HEARTBEAT: 50, MSG_NAMED_VALUE_FLOAT: 170, MSG_STATUSTEXT: 83}
# Full payload length; MAVLink 2 trims trailing zero bytes, so pad back to this
PAYLOAD_LEN = {MSG_HEARTBEAT: 9, MSG_NAMED_VALUE_FLOAT: 18, MSG_STATUSTEXT: 51}

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_MODE_FLAG_SAFETY_ARMED = 0x80

GCS_SYSID = 254   # not 255, so it does not clash with Mission Planner
GCS_COMPID = 190


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


def encode_heartbeat(seq):
    payload = struct.pack("<IBBBBB", 0, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 3)
    header = bytes([len(payload), seq & 0xFF, GCS_SYSID, GCS_COMPID, MSG_HEARTBEAT])
    crc = x25_crc(header + payload + bytes([CRC_EXTRA[MSG_HEARTBEAT]]))
    return b"\xfe" + header + payload + struct.pack("<H", crc)


def decode(msgid, payload):
    if msgid == MSG_NAMED_VALUE_FLOAT:
        boot_ms, value, name = struct.unpack("<If10s", payload[:18])
        return {"boot_ms": boot_ms, "value": value,
                "name": name.split(b"\0", 1)[0].decode("ascii", "replace")}
    if msgid == MSG_STATUSTEXT:
        return {"severity": payload[0],
                "text": payload[1:51].split(b"\0", 1)[0].decode("ascii", "replace")}
    if msgid == MSG_HEARTBEAT:
        _, mav_type, autopilot, base_mode, _, _ = struct.unpack("<IBBBBB", payload[:9])
        return {"type": mav_type, "autopilot": autopilot,
                "armed": bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)}
    return None


# ---------------------------------------------------------------------------
# Transport: UDP listen, UDP out, or TCP client
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, conn):
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
                raise OSError("%s; a porta %d ja esta em uso (Mission Planner?). Encaminhe pelo MP "
                              "(Ctrl+F > MAVLink > UDP Outbound) para outra porta e use --conn udp:0.0.0.0:<porta>"
                              % (exc, self.addr[1]))
        elif kind == "udpout":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.peer = self.addr
        elif kind == "tcp":
            self.sock = socket.create_connection(self.addr, timeout=5)
        else:
            raise ValueError("tipo de conexao desconhecido: %s" % kind)
        self.sock.settimeout(0.5)

    def recv(self):
        try:
            if self.kind == "tcp":
                data = self.sock.recv(4096)
                if not data:
                    raise ConnectionError("conexao TCP fechada")
                return data
            data, addr = self.sock.recvfrom(4096)
            if self.kind == "udp":
                self.peer = addr   # answer the sender so it keeps streaming to us
            return data
        except socket.timeout:
            return b""

    def send(self, data):
        try:
            if self.kind == "tcp":
                self.sock.sendall(data)
            elif self.peer is not None:
                self.sock.sendto(data, self.peer)
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
        self.last_rx = None
        self.t0 = time.monotonic()

    def now(self):
        return time.monotonic() - self.t0

    def add_value(self, name, value):
        t = self.now()
        with self.lock:
            d = self.series[name]
            d.append((t, value))
            while d and d[0][0] < t - self.window_s:
                d.popleft()
            self.last_rx = t

    def add_event(self, severity, text):
        with self.lock:
            self.events.append((self.now(), severity, text))

    def snapshot(self):
        with self.lock:
            return ({k: list(v) for k, v in self.series.items()},
                    list(self.events), self.armed, self.last_rx)


SEVERITY = ["EMERG", "ALERT", "CRIT", "ERRO", "AVISO", "NOTICE", "INFO", "DEBUG"]


def reader(link, store, csv_writer, csv_file, stop):
    parser = MavlinkParser()
    seq = 0
    last_hb = 0.0
    vehicle_sys = None
    while not stop.is_set():
        now = time.monotonic()
        if now - last_hb >= 1.0:
            last_hb = now
            link.send(encode_heartbeat(seq))
            seq += 1
        try:
            data = link.recv()
        except (OSError, ConnectionError) as exc:
            print("Erro de conexao: %s" % exc, file=sys.stderr)
            stop.set()
            break
        if not data:
            continue
        for msgid, sysid, compid, payload in parser.feed(data):
            if sysid == GCS_SYSID:
                continue
            msg = decode(msgid, payload)
            wall = datetime.datetime.now().isoformat(timespec="milliseconds")
            if msgid == MSG_HEARTBEAT:
                if msg["autopilot"] != MAV_AUTOPILOT_INVALID and msg["type"] != MAV_TYPE_GCS:
                    vehicle_sys = sysid
                    if store.armed != msg["armed"]:
                        store.armed = msg["armed"]
                        print("[%s] %s" % (wall[11:], "ARMADO" if msg["armed"] else "DESARMADO"))
                continue
            if vehicle_sys is not None and sysid != vehicle_sys:
                continue
            if msgid == MSG_NAMED_VALUE_FLOAT:
                if not msg["name"].startswith("ESC"):
                    continue
                store.add_value(msg["name"], msg["value"])
                csv_writer.writerow([wall, "%.3f" % store.now(), msg["boot_ms"], msg["name"],
                                     "%.4f" % msg["value"], ""])
            elif msgid == MSG_STATUSTEXT:
                sev = SEVERITY[msg["severity"]] if msg["severity"] < len(SEVERITY) else str(msg["severity"])
                store.add_event(msg["severity"], msg["text"])
                csv_writer.writerow([wall, "%.3f" % store.now(), "", "STATUSTEXT", msg["severity"], msg["text"]])
                print("[%s] %-6s %s" % (wall[11:], sev, msg["text"]))
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


def run_console(store, stop, duration):
    end = time.monotonic() + duration if duration else None
    while not stop.is_set():
        if end and time.monotonic() >= end:
            break
        time.sleep(1.0)
        series, _, armed, last_rx = store.snapshot()
        if last_rx is None:
            print("Aguardando dados ESC... (ESC_GCS_HZ > 0 e ESC_CAN = 1?)")
            continue
        parts = []
        for m in motor_ids(series):
            def last(name):
                d = series.get(name)
                return d[-1][1] if d else float("nan")
            parts.append("M%d %5.1fA/%5.1fA %6.0frpm" % (
                m, last("ESC%d_I" % m), last("ESC%d_ICMD" % m), last("ESC%d_RPM" % m)))
        print(" | ".join(parts) + ("   [armado]" if armed else ""))
    stop.set()


def run_plot(store, stop, window_s):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
    fig, (ax_i, ax_r) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    fig.canvas.manager.set_window_title("ESC CAN monitor")
    status = fig.text(0.01, 0.01, "", fontsize=9, family="monospace")
    lines = {}

    def line(ax, key, **kw):
        if key not in lines:
            lines[key], = ax.plot([], [], **kw)
        return lines[key]

    ax_i.set_ylabel("Corrente (A)")
    ax_i.set_title("solida = medida, tracejada = comandada")
    ax_r.set_ylabel("Roda (rpm)")
    ax_r.set_xlabel("tempo (s)")
    for ax in (ax_i, ax_r):
        ax.grid(True, alpha=0.3)
    event_marks = []

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
                    ("ESC%d_RPM" % m, ax_r, "-", "M%d" % m)):
                d = series.get(key, [])
                ln = line(ax, key, color=c, linestyle=style, label=label, linewidth=1.2)
                ln.set_data([p[0] for p in d], [p[1] for p in d])
        for key, style in (("ESC_TGT_L", ":"), ("ESC_TGT_R", "-.")):
            d = series.get(key, [])
            ln = line(ax_r, key, color="black", linestyle=style, label=key[-5:], linewidth=1)
            ln.set_data([p[0] for p in d], [p[1] for p in d])
        for mark in event_marks:
            mark.remove()
        event_marks.clear()
        for t, sev, text in events:
            if t >= now - window_s and ("lost" in text or sev <= 3):
                event_marks.append(ax_i.axvline(t, color="red", alpha=0.5))
                event_marks.append(ax_r.axvline(t, color="red", alpha=0.5))
        for ax in (ax_i, ax_r):
            ax.set_xlim(max(0, now - window_s), max(window_s, now))
            ax.relim()
            ax.autoscale_view(scalex=False)
            handles, _ = ax.get_legend_handles_labels()
            if handles and ax.get_legend() is None:
                ax.legend(loc="upper left", fontsize=8, ncol=4)
            elif ax.get_legend() is not None and len(handles) != len(ax.get_legend().get_texts()):
                ax.legend(loc="upper left", fontsize=8, ncol=4)
        last_text = events[-1][2] if events else ""
        state = "sem dados" if last_rx is None else ("ARMADO" if armed else "desarmado")
        status.set_text("%s | ultima msg: %s" % (state, last_text[:80]))
        return []

    anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    fig._esc_anim = anim  # keep a reference so it is not garbage collected
    plt.tight_layout(rect=(0, 0.03, 1, 1))
    plt.show()
    stop.set()


def main():
    ap = argparse.ArgumentParser(description="Monitor ao vivo das correntes dos motores CAN via MAVLink")
    ap.add_argument("--conn", default="udp:0.0.0.0:14550",
                    help="udp:HOST:PORTA (escuta), udpout:HOST:PORTA ou tcp:HOST:PORTA (padrao: udp:0.0.0.0:14550)")
    ap.add_argument("--csv", default=None, help="arquivo CSV (padrao: esc_log_DATA.csv na pasta atual)")
    ap.add_argument("--window", type=float, default=20.0, help="janela do grafico em segundos")
    ap.add_argument("--no-plot", action="store_true", help="sem grafico, so console e CSV")
    ap.add_argument("--duration", type=float, default=0, help="encerrar apos N segundos (0 = sem limite)")
    args = ap.parse_args()

    csv_path = args.csv or datetime.datetime.now().strftime("esc_log_%Y%m%d_%H%M%S.csv")
    try:
        link = Link(args.conn)
    except (OSError, ValueError) as exc:
        sys.exit("Nao foi possivel abrir %s: %s" % (args.conn, exc))
    print("Conectado em %s; gravando %s" % (args.conn, csv_path))

    store = Store(args.window)
    stop = threading.Event()
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["hora_pc", "t_s", "boot_ms", "nome", "valor", "texto"])
        th = threading.Thread(target=reader, args=(link, store, writer, f, stop), daemon=True)
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
    print("CSV salvo em %s" % csv_path)


if __name__ == "__main__":
    main()
