#!/usr/bin/env python3
"""Regras e codec da missao do rover (sem UI).

Campo Alt (z) de NAV_WAYPOINT: -1 passa direto; qualquer outro valor e rumo
de alinhamento da frente (0 = norte, sentido horario). Trechos de re usam
DO_SET_REVERSE; rebuild_reverse monta um DO 1 antes de cada waypoint de re.
"""

from __future__ import annotations

import math
import struct

import esc_monitor as em

MSG_MISSION_REQUEST = 40
MSG_MISSION_REQUEST_LIST = 43
MSG_MISSION_COUNT = 44
MSG_MISSION_ACK = 47
MSG_MISSION_REQUEST_INT = 51
MSG_MISSION_ITEM_INT = 73

MISSION_MSG_TABLE = (
    (MSG_MISSION_REQUEST, 230, 4),
    (MSG_MISSION_REQUEST_LIST, 132, 2),
    (MSG_MISSION_COUNT, 221, 4),
    (MSG_MISSION_ACK, 153, 3),
    (MSG_MISSION_REQUEST_INT, 196, 4),
    (MSG_MISSION_ITEM_INT, 38, 37),
)

CMD_WAYPOINT = 16
CMD_LOITER_UNLIM = 17
CMD_LOITER_TIME = 19
CMD_RTL = 20
CMD_DELAY = 93
CMD_COND_DELAY = 112
CMD_COND_DIST = 114
CMD_DO_JUMP = 177
CMD_DO_CHANGE_SPEED = 178
CMD_DO_SET_HOME = 179
CMD_DO_SET_RELAY = 181
CMD_DO_SET_SERVO = 183
CMD_DO_SET_REVERSE = 194
CMD_DO_REPEAT_RELAY = 182
CMD_DO_REPEAT_SERVO = 184
CMD_DO_AUX_FUNCTION = 218
CMD_GUIDED_ENABLE = 92
FRAME_GLOBAL = 0
FRAME_GLOBAL_RELATIVE_ALT = 3

CMD_NAMES = {
    16: "WAYPOINT", 17: "LOITER_UNLIM", 19: "LOITER_TIME", 20: "RTL", 21: "LAND", 22: "TAKEOFF",
    82: "SPLINE_WP", 92: "GUIDED_ENABLE", 93: "DELAY", 112: "COND_DELAY", 114: "COND_DIST",
    115: "COND_YAW", 177: "DO_JUMP", 178: "DO_CHANGE_SPEED", 179: "DO_SET_HOME", 181: "DO_SET_RELAY",
    182: "DO_REPEAT_RELAY", 183: "DO_SET_SERVO", 184: "DO_REPEAT_SERVO", 194: "DO_SET_REVERSE",
    201: "DO_SET_ROI", 218: "DO_AUX_FUNCTION", 222: "DO_GUIDED_LIMITS",
}
NAV_WITH_POSITION = (16, 17, 19, 21, 22, 82)
ACK_RESULT = {
    0: "aceita", 1: "erro", 2: "frame nao suportado", 3: "comando nao suportado", 4: "sem espaco",
    5: "invalida", 13: "sequencia errada", 15: "negada", 14: "cancelada",
}
MISSION_MAX_ITEMS = 500

# Rover Flight Plan command palette (Mission Planner subset for Rover).
# params: list of {key, label, hint} where key is p1..p4 or z.
ROVER_COMMANDS = (
    {"id": 16, "name": "WAYPOINT", "kind": "nav", "has_pos": True,
     "params": [
         {"key": "p1", "label": "Delay", "hint": "s"},
         # Neste rover o mixer le Alt (z) como rumo de alinhamento, nao altitude.
         {"key": "z", "label": "Rumo", "hint": "-1 passa direto; senao graus (0=N)"},
     ]},
    {"id": 17, "name": "LOITER_UNLIM", "kind": "nav", "has_pos": True,
     "params": [{"key": "z", "label": "z", "hint": "ignorado no solo"}]},
    {"id": 19, "name": "LOITER_TIME", "kind": "nav", "has_pos": True,
     "params": [
         {"key": "p1", "label": "Tempo", "hint": "s"},
         {"key": "z", "label": "z", "hint": "ignorado no solo"},
     ]},
    {"id": 20, "name": "RTL", "kind": "nav", "has_pos": False, "params": []},
    {"id": 93, "name": "DELAY", "kind": "nav", "has_pos": False,
     "params": [{"key": "p1", "label": "Tempo", "hint": "s"}]},
    {"id": 92, "name": "GUIDED_ENABLE", "kind": "nav", "has_pos": False,
     "params": [{"key": "p1", "label": "On", "hint": "0/1"}]},
    {"id": 112, "name": "COND_DELAY", "kind": "condition", "has_pos": False,
     "params": [{"key": "p1", "label": "Tempo", "hint": "s"}]},
    {"id": 114, "name": "COND_DIST", "kind": "condition", "has_pos": False,
     "params": [{"key": "p1", "label": "Distancia", "hint": "m"}]},
    {"id": 177, "name": "DO_JUMP", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "WP#", "hint": "seq destino"},
         {"key": "p2", "label": "Repetir", "hint": "-1 = infinito"},
     ]},
    {"id": 178, "name": "DO_CHANGE_SPEED", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "Tipo", "hint": "0=ar 1=solo 2=subir"},
         {"key": "p2", "label": "Velocidade", "hint": "m/s (-1=nao muda)"},
         {"key": "p3", "label": "Throttle", "hint": "% (-1=nao muda)"},
     ]},
    {"id": 194, "name": "DO_SET_REVERSE", "kind": "do", "has_pos": False,
     "params": [{"key": "p1", "label": "Re", "hint": "0 frente / 1 re"}]},
    {"id": 179, "name": "DO_SET_HOME", "kind": "do", "has_pos": True,
     "params": [{"key": "p1", "label": "Atual", "hint": "1=posicao atual"}]},
    {"id": 183, "name": "DO_SET_SERVO", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "Servo", "hint": "canal"},
         {"key": "p2", "label": "PWM", "hint": "us"},
     ]},
    {"id": 181, "name": "DO_SET_RELAY", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "Relay", "hint": "n"},
         {"key": "p2", "label": "Estado", "hint": "0/1"},
     ]},
    {"id": 184, "name": "DO_REPEAT_SERVO", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "Servo", "hint": "canal"},
         {"key": "p2", "label": "PWM", "hint": "us"},
         {"key": "p3", "label": "Repetir", "hint": "vezes"},
         {"key": "p4", "label": "Periodo", "hint": "s"},
     ]},
    {"id": 182, "name": "DO_REPEAT_RELAY", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "Relay", "hint": "n"},
         {"key": "p2", "label": "Repetir", "hint": "vezes"},
         {"key": "p3", "label": "Periodo", "hint": "s"},
     ]},
    {"id": 218, "name": "DO_AUX_FUNCTION", "kind": "do", "has_pos": False,
     "params": [
         {"key": "p1", "label": "Funcao", "hint": "aux id"},
         {"key": "p2", "label": "Switch", "hint": "0/1/2"},
     ]},
)
ROVER_COMMAND_BY_ID = {c["id"]: c for c in ROVER_COMMANDS}


def register_mission_messages(crc_extra=None, payload_len=None):
    """Register mission msgid CRC/length on esc_monitor and optional encoder tables."""
    for mid, crc, length in MISSION_MSG_TABLE:
        em.CRC_EXTRA[mid] = crc
        em.PAYLOAD_LEN[mid] = length
        if crc_extra is not None:
            crc_extra[mid] = crc
        if payload_len is not None:
            payload_len[mid] = length


register_mission_messages()


class Item:
    __slots__ = ("seq", "frame", "command", "current", "autocontinue", "p1", "p2", "p3", "p4", "lat", "lon", "z")

    def __init__(self, seq, frame, command, current, autocontinue, p1, p2, p3, p4, lat, lon, z):
        self.seq, self.frame, self.command, self.current, self.autocontinue = seq, frame, command, current, autocontinue
        self.p1, self.p2, self.p3, self.p4 = p1, p2, p3, p4
        self.lat, self.lon, self.z = lat, lon, z

    @classmethod
    def from_payload(cls, p):
        p1, p2, p3, p4, x, y, z, seq, cmd, _ts, _tc, frame, cur, auto = struct.unpack("<4fiifHHBBBBB", p[:37])
        return cls(seq, frame, cmd, cur, auto, p1, p2, p3, p4, x * 1e-7, y * 1e-7, z)

    def payload(self, sysid, compid):
        return struct.pack("<4fiifHHBBBBB", self.p1, self.p2, self.p3, self.p4,
                           int(round(self.lat * 1e7)), int(round(self.lon * 1e7)), self.z,
                           self.seq, self.command, sysid, compid, self.frame, self.current, self.autocontinue)

    def copy(self):
        return Item(*[getattr(self, s) for s in Item.__slots__])

    def to_dict(self):
        return {
            "seq": int(self.seq),
            "frame": int(self.frame),
            "command": int(self.command),
            "current": int(self.current),
            "autocontinue": int(self.autocontinue),
            "p1": float(self.p1),
            "p2": float(self.p2),
            "p3": float(self.p3),
            "p4": float(self.p4),
            "lat": float(self.lat),
            "lon": float(self.lon),
            "z": float(self.z),
            "name": CMD_NAMES.get(int(self.command), "CMD_%d" % int(self.command)),
        }


def item_from_dict(data):
    if not isinstance(data, dict):
        raise ValueError("item invalido")
    try:
        seq = int(data["seq"])
        frame = int(data["frame"])
        command = int(data["command"])
        current = int(data.get("current", 0))
        autocontinue = int(data.get("autocontinue", 1))
        vals = [float(data[k]) for k in ("p1", "p2", "p3", "p4", "lat", "lon", "z")]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("item incompleto") from exc
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("item com numero invalido")
    return Item(seq, frame, command, current, autocontinue, *vals)


def items_from_dicts(raw):
    if not isinstance(raw, list):
        raise ValueError("lista de itens invalida")
    if len(raw) > MISSION_MAX_ITEMS:
        raise ValueError("missao grande demais (%d itens)" % len(raw))
    items = [item_from_dict(row) for row in raw]
    for i, it in enumerate(items):
        it.seq = i
    return items


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def mission_stats(items):
    """Total path length and per-leg distances between positional nav items."""
    pts = [(i, it) for i, it in enumerate(items)
           if (i == 0 or it.command in NAV_WITH_POSITION) and (it.lat or it.lon)]
    legs = []
    total = 0.0
    for k in range(1, len(pts)):
        i0, a = pts[k - 1]
        i1, b = pts[k]
        dist = haversine_m(a.lat, a.lon, b.lat, b.lon)
        total += dist
        legs.append({"from": i0, "to": i1, "m": round(dist, 2)})
    return {"total_m": round(total, 1), "legs": legs, "waypoints": max(0, len(pts) - (1 if pts else 0))}


def command_catalog():
    return [dict(cmd) for cmd in ROVER_COMMANDS]


def make_item(command, lat=0.0, lon=0.0, z=-1.0, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
              frame=None, current=0, autocontinue=1):
    spec = ROVER_COMMAND_BY_ID.get(int(command))
    if frame is None:
        frame = FRAME_GLOBAL if int(command) == CMD_WAYPOINT and current else FRAME_GLOBAL_RELATIVE_ALT
    if spec and not spec.get("has_pos"):
        lat = lon = 0.0
    return Item(0, int(frame), int(command), int(current), int(autocontinue),
                float(p1), float(p2), float(p3), float(p4), float(lat), float(lon), float(z))


def empty_mission(lat=0.0, lon=0.0):
    home = make_item(CMD_WAYPOINT, lat=lat, lon=lon, z=0.0, current=1, frame=FRAME_GLOBAL)
    home.seq = 0
    return [home]


def renumber(items):
    for i, it in enumerate(items):
        it.seq = i
    return items


def insert_item(items, index, item):
    if index < 1:
        index = len(items)
    index = max(1, min(int(index), len(items)))
    items = list(items)
    items.insert(index, item)
    return renumber(items)


def delete_item(items, seq):
    if seq <= 0 or seq >= len(items):
        raise ValueError("nao da para apagar o home")
    items = [it for i, it in enumerate(items) if i != seq]
    return renumber(items)


def move_item_latlon(items, seq, lat, lon):
    if seq < 0 or seq >= len(items):
        raise ValueError("item invalido")
    it = items[seq]
    if seq > 0 and it.command not in NAV_WITH_POSITION and it.command != CMD_DO_SET_HOME:
        raise ValueError("este comando nao tem posicao")
    it.lat, it.lon = float(lat), float(lon)
    return items


def items_payload(items):
    info = analyse(items)
    flags = reverse_flags(items)
    rows = []
    for i, it in enumerate(items):
        row = it.to_dict()
        direction, effect, warn = info[i]
        row["direction"] = direction
        row["effect"] = effect
        row["warn"] = warn
        row["reverse"] = bool(flags[i])
        row["has_pos"] = bool(i == 0 or it.command in NAV_WITH_POSITION or it.command == CMD_DO_SET_HOME)
        spec = ROVER_COMMAND_BY_ID.get(int(it.command))
        row["kind"] = spec["kind"] if spec else ("nav" if it.command < 100 else "do")
        rows.append(row)
    stats = mission_stats(items)
    return {
        "items": rows,
        "count": len(rows),
        "warns": sum(1 for row in rows if row["warn"]),
        "stats": stats,
    }


def is_pass(z):
    return z == -1


def analyse(items):
    """Per item: (direction 'frente'/'re', effect text, warning text or '')."""
    out = []
    reverse = False
    for idx, it in enumerate(items):
        warn = ""
        if idx == 0:
            out.append(("", "home", ""))
            continue
        if it.command == CMD_DO_SET_REVERSE:
            reverse = it.p1 >= 0.5
            out.append(("re" if reverse else "frente", "liga a re" if reverse else "volta de frente", ""))
            continue
        if it.command == CMD_WAYPOINT:
            if is_pass(it.z):
                effect = "passa direto"
            else:
                effect = "para e alinha %.0f graus" % (it.z % 360)
                if it.z > 360 or it.z < -1:
                    warn = "Alt %.1f vira rumo %.0f graus (era altitude?)" % (it.z, it.z % 360)
            if reverse and not warn:
                prev_do = items[idx - 1].command == CMD_DO_SET_REVERSE and items[idx - 1].p1 >= 0.5
                if not prev_do:
                    warn = "re sem DO_SET_REVERSE logo antes: marque 'Chegar de re' de novo"
        else:
            effect = ""
        out.append(("re" if reverse else "frente", effect, warn))
    return out


def reverse_flags(items):
    """Per item: True when the leg that ends at it is driven in reverse."""
    return [d == "re" for d, _e, _w in analyse(items)]


def rebuild_reverse(items, rev_wp):
    """Drop every DO_SET_REVERSE and put back one DO_SET_REVERSE 1 before each
    nav item in rev_wp (a set of items) and a 0 where it goes forward again."""
    out = [items[0]]
    reverse = False
    for it in items[1:]:
        if it.command == CMD_DO_SET_REVERSE:
            continue
        if it.command < 100:
            want = it in rev_wp
            if want or reverse != want:
                out.append(Item(0, it.frame, CMD_DO_SET_REVERSE, 0, 1, 1.0 if want else 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            reverse = want
        out.append(it)
    for i, it in enumerate(out):
        it.seq = i
    return out


def next_nav(items, i):
    for j in range(i + 1, len(items)):
        if items[j].command < 100:
            return j
    return None


def leg_bearing(items, i):
    """Bearing (deg, 0 = north) from waypoint i to the next nav item with a position."""
    a = items[i]
    for b in items[i + 1:]:
        if b.command in NAV_WITH_POSITION and (b.lat or b.lon):
            dn = b.lat - a.lat
            de = (b.lon - a.lon) * math.cos(math.radians(a.lat))
            if abs(dn) + abs(de) < 1e-9:
                continue
            return math.degrees(math.atan2(de, dn)) % 360
    return None


def front_heading(items, i, brg):
    """Heading of the rover front for the leg after i."""
    nxt = next_nav(items, i)
    if nxt is not None and reverse_flags(items)[nxt]:
        return (brg + 180) % 360
    return brg


def apply_reverse_toggle(items, seq, want):
    """Toggle reverse for the nav item at seq; may flip previous WP alignment."""
    if seq <= 0 or seq >= len(items):
        raise ValueError("waypoint invalido")
    cur = items[seq]
    if cur.command != CMD_WAYPOINT:
        raise ValueError("so WAYPOINT tem trecho de re")
    flags = reverse_flags(items)
    rev = {it for it, f in zip(items, flags) if f and it.command < 100}
    if want:
        rev.add(cur)
    else:
        rev.discard(cur)
    prev = None
    for j in range(seq - 1, 0, -1):
        if items[j].command < 100:
            prev = items[j]
            break
    flipped = None
    if prev is not None and prev.command == CMD_WAYPOINT and not is_pass(prev.z) and flags[seq] != want:
        prev.z = round((prev.z + 180) % 360, 1)
        flipped = prev.z
    items = rebuild_reverse(items, rev)
    return items, items.index(cur), flipped


def parse_waypoints_text(text):
    if not isinstance(text, str):
        raise ValueError("texto invalido")
    lines = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError("nao e um arquivo .waypoints do Mission Planner")
    items = []
    for line in lines[1:]:
        c = line.strip().split("\t")
        if len(c) < 12:
            continue
        items.append(Item(int(c[0]), int(c[2]), int(c[3]), int(c[1]), int(float(c[11])),
                          float(c[4]), float(c[5]), float(c[6]), float(c[7]),
                          float(c[8]), float(c[9]), float(c[10])))
    if not items:
        raise ValueError("arquivo sem itens")
    if len(items) > MISSION_MAX_ITEMS:
        raise ValueError("missao grande demais (%d itens)" % len(items))
    for i, it in enumerate(items):
        it.seq = i
    return items


def format_waypoints_text(items):
    lines = ["QGC WPL 110"]
    for i, it in enumerate(items):
        lines.append("\t".join(
            [str(i), str(it.current), str(it.frame), str(it.command)]
            + ["%.8f" % v for v in (it.p1, it.p2, it.p3, it.p4, it.lat, it.lon)]
            + ["%.6f" % it.z, str(it.autocontinue)]
        ))
    return "\r\n".join(lines) + "\r\n"


def load_waypoints(path):
    with open(path, encoding="utf-8") as f:
        return parse_waypoints_text(f.read())


def save_waypoints(path, items):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(format_waypoints_text(items))
