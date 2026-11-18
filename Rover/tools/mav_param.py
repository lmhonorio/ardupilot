"""Read or set ArduPilot parameters through the HereLink (no pymavlink needed).

Usage (close esc_monitor.py first: both listen on UDP 14550):
  python mav_param.py WP_SPEED ESC_RPM            # read
  python mav_param.py WP_SPEED=1.0 ESC_IMAX=15    # set (saved by ArduPilot) and confirm

Sends heartbeats to the HereLink at 192.168.43.1:14552 so it starts streaming,
then PARAM_REQUEST_READ (20) / PARAM_SET (23) and waits for PARAM_VALUE (22).
"""
import socket
import struct
import sys
import time

KICK = ("192.168.43.1", 14552)
TIMEOUT_S = 25


def x25(data, crc=0xFFFF):
    for b in data:
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


_seq = 0


def pack(msgid, payload, crc_extra):
    """MAVLink v1 frame from GCS sysid 255, compid 190."""
    global _seq
    hdr = bytes([len(payload), _seq & 0xFF, 255, 190, msgid])
    _seq += 1
    crc = x25(bytes([crc_extra]), x25(hdr + payload))
    return b"\xfe" + hdr + payload + struct.pack("<H", crc)


def frames(d):
    """Yield (msgid, payload) for v1 and v2 frames in one datagram."""
    i = 0
    while i < len(d):
        if d[i] == 0xFE and i + 8 <= len(d):
            n = d[i + 1]
            yield d[i + 5], d[i + 6:i + 6 + n]
            i += 8 + n
        elif d[i] == 0xFD and i + 12 <= len(d):
            n = d[i + 1]
            mid = int.from_bytes(d[i + 7:i + 10], "little")
            yield mid, d[i + 10:i + 10 + n]
            i += 12 + n + (13 if d[i + 2] & 1 else 0)
        else:
            i += 1


def main(args):
    sets = {}
    names = []
    for a in args:
        if "=" in a:
            k, v = a.split("=", 1)
            sets[k] = float(v)
            names.append(k)
        else:
            names.append(a)
    if not names:
        print(__doc__)
        return 1

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 14550))
    s.settimeout(0.2)
    hb = pack(0, struct.pack("<IBBBBB", 0, 6, 8, 0, 0, 3), 50)
    got = {}
    peer = None
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < TIMEOUT_S and len(got) < len(names):
        if time.time() - last > 1.0:
            last = time.time()
            s.sendto(hb, KICK)
            for n in names:
                if n in got:
                    continue
                if n in sets:
                    p = struct.pack("<fBB16sB", sets[n], 1, 1, n.encode(), 9)
                    s.sendto(pack(23, p, 168), peer or KICK)
                else:
                    p = struct.pack("<hBB16s", -1, 1, 1, n.encode())
                    s.sendto(pack(20, p, 214), peer or KICK)
        try:
            d, a = s.recvfrom(4096)
        except socket.timeout:
            continue
        peer = peer or a
        for mid, pl in frames(d):
            if mid != 22:
                continue
            pl = pl + bytes(max(0, 25 - len(pl)))
            v, _cnt, _idx, pid, _ty = struct.unpack("<fHH16sB", pl[:25])
            name = pid.split(b"\0")[0].decode(errors="replace")
            if name in names and name not in got:
                if name in sets and abs(v - sets[name]) > 1e-4:
                    continue  # old value echoed before the set took effect
                got[name] = v
                print(f"{name} = {v:g}", flush=True)
    missing = [n for n in names if n not in got]
    if missing:
        print("sem resposta:", " ".join(missing))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
