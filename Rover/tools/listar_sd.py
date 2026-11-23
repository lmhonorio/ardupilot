#!/usr/bin/env python3
"""Lista o cartão SD da FC via MAVFtp (mesmo caminho HereLink do mode_panel).

Fecha Mission Planner / esc_monitor / mode_panel antes: a porta UDP 14550
só aceita um processo.

  py -3 listar_sd.py
  py -3 listar_sd.py --virtual          # também entra em @SYS, @ROMFS, etc.
  py -3 listar_sd.py --conn udp:0.0.0.0:14550
  py -3 listar_sd.py --kick none
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

import mode_panel as mp

ROOTS = ("/", "/APM")


def wait_vehicle(vehicle, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with vehicle.lock:
            if vehicle.sysid is not None and (time.monotonic() - vehicle.last_hb) < 3.0:
                return vehicle.sysid, vehicle.compid or 1
        time.sleep(0.1)
    return None, None


def fmt_size(n):
    if n is None:
        return ""
    if n < 1024:
        return "%d B" % n
    if n < 1024 * 1024:
        return "%.1f KB" % (n / 1024.0)
    return "%.2f MB" % (n / (1024.0 * 1024.0))


def walk(ftp, vehicle, path, depth, into_virtual, stats, seen):
    key = path.rstrip("/") or "/"
    if key in seen:
        return
    seen.add(key)

    def list_once():
        return ftp.list_dir(path if path != "/" else "/")

    try:
        entries = ftp._call(vehicle, list_once)
    except mp.FtpError as exc:
        print("%s[ERRO] %s — %s" % ("  " * depth, path, exc))
        stats["errors"] += 1
        return

    dirs = []
    files = []
    for kind, name, size in entries:
        if not name or name in (".", ".."):
            continue
        if kind == "D":
            dirs.append(name)
        elif kind == "F":
            files.append((name, size or 0))

    dirs.sort(key=str.lower)
    files.sort(key=lambda item: item[0].lower())

    for name in dirs:
        child = "/" + name if path == "/" else path.rstrip("/") + "/" + name
        virtual = name.startswith("@")
        tag = " [virtual]" if virtual else ""
        print("%s[DIR] %s%s" % ("  " * depth, child, tag))
        stats["dirs"] += 1
        if virtual and not into_virtual:
            continue
        walk(ftp, vehicle, child, depth + 1, into_virtual, stats, seen)

    for name, size in files:
        child = "/" + name if path == "/" else path.rstrip("/") + "/" + name
        print("%s      %s  (%s)" % ("  " * depth, child, fmt_size(size)))
        stats["files"] += 1
        stats["bytes"] += size


def main(argv=None):
    ap = argparse.ArgumentParser(description="Lista pastas/arquivos do cartão via MAVFtp")
    ap.add_argument("--conn", default="udp:0.0.0.0:14550",
                    help="udp:HOST:PORTA (padrao udp:0.0.0.0:14550)")
    ap.add_argument("--kick", default=mp.HERELINK_KICK,
                    help="heartbeat HereLink HOST:PORTA (padrao %s; 'none' desliga)" % mp.HERELINK_KICK)
    ap.add_argument("--timeout", type=float, default=25.0,
                    help="segundos esperando heartbeat do rover")
    ap.add_argument("--virtual", action="store_true",
                    help="também lista conteúdo de pastas @SYS/@ROMFS/...")
    ap.add_argument("--root", action="append", default=None,
                    help="caminho inicial (pode repetir). Padrao: / e /APM")
    args = ap.parse_args(argv)

    try:
        link = mp.Link(args.conn, args.kick)
    except (OSError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1

    vehicle = mp.Vehicle()
    stop = threading.Event()
    tx = threading.Lock()
    enc = mp.Encoder()
    vehicle.ftp = mp.FtpClient(link, tx, enc)
    threading.Thread(target=mp.reader, args=(link, vehicle, stop, tx, enc), daemon=True).start()

    print("Aguardando rover em %s ..." % args.conn, flush=True)
    sysid, compid = wait_vehicle(vehicle, args.timeout)
    if sysid is None:
        print("Sem heartbeat. PC no AndroidAP? Mission Planner fechado?", file=sys.stderr)
        stop.set()
        link.sock.close()
        return 1

    print("Rover sysid=%s compid=%s" % (sysid, compid), flush=True)
    roots = args.root or list(ROOTS)
    stats = {"dirs": 0, "files": 0, "bytes": 0, "errors": 0}
    seen = set()

    try:
        vehicle.ftp._call(vehicle, vehicle.ftp.reset)
        for root in roots:
            print("\n=== %s ===" % root, flush=True)
            walk(vehicle.ftp, vehicle, root, 0, args.virtual, stats, seen)
    except mp.FtpError as exc:
        print("Falha MAVFtp: %s" % exc, file=sys.stderr)
        stop.set()
        link.sock.close()
        return 1
    finally:
        stop.set()
        try:
            link.sock.close()
        except OSError:
            pass

    print("\nResumo: %d pastas, %d arquivos, %s, %d erros"
          % (stats["dirs"], stats["files"], fmt_size(stats["bytes"]), stats["errors"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
