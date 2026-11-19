#!/usr/bin/env python3
"""
Editor dos rumos de alinhamento da missao do rover (campo Alt dos waypoints).

  python mission_editor.py

1. "Baixar do rover" le a missao atual pela HereLink (UDP 14550; feche o
   esc_monitor.py e o mav_param.py antes, eles usam a mesma porta).
2. Clique num waypoint e escolha o rumo na bussola (0 = norte, sentido horario)
   ou marque "Passa direto" (Alt = -1). Enter no campo de graus tambem aplica.
3. "Enviar ao rover" grava a missao de volta. Tambem abre e salva .waypoints
   (formato do Mission Planner).

Regra do mixer (rover-controlallocation.lua): ao chegar num WAYPOINT, Alt = -1
passa direto; qualquer outro valor para o rover e gira ate esse rumo (frente).
O alinhamento troca o modo (AUTO -> STEERING -> AUTO) e isso DESLIGA a re
(DO_SET_REVERSE): o waypoint antes de um DO_SET_REVERSE e os waypoints de um
trecho de re precisam de Alt = -1. A lista marca em vermelho o que quebra essa regra.
"""

import math
import os
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esc_monitor as em  # noqa: E402  (MAVLink parser, link and encoder)

# ---------------------------------------------------------------------------
# Mission protocol messages (added to the monitor's decoder tables)
# ---------------------------------------------------------------------------
MSG_MISSION_REQUEST = 40
MSG_MISSION_REQUEST_LIST = 43
MSG_MISSION_COUNT = 44
MSG_MISSION_ACK = 47
MSG_MISSION_REQUEST_INT = 51
MSG_MISSION_ITEM_INT = 73
for _mid, _crc, _len in ((MSG_MISSION_REQUEST, 230, 4), (MSG_MISSION_REQUEST_LIST, 132, 2),
                         (MSG_MISSION_COUNT, 221, 4), (MSG_MISSION_ACK, 153, 3),
                         (MSG_MISSION_REQUEST_INT, 196, 4), (MSG_MISSION_ITEM_INT, 38, 37)):
    em.CRC_EXTRA[_mid] = _crc
    em.PAYLOAD_LEN[_mid] = _len

CMD_WAYPOINT = 16
CMD_DO_SET_REVERSE = 194
CMD_NAMES = {16: "WAYPOINT", 17: "LOITER_UNLIM", 19: "LOITER_TIME", 20: "RTL", 21: "LAND", 22: "TAKEOFF",
             82: "SPLINE_WP", 92: "GUIDED_ENABLE", 93: "DELAY", 112: "COND_DELAY", 114: "COND_DIST",
             115: "COND_YAW", 177: "DO_JUMP", 178: "DO_CHANGE_SPEED", 179: "DO_SET_HOME", 181: "DO_SET_RELAY",
             183: "DO_SET_SERVO", 194: "DO_SET_REVERSE", 201: "DO_SET_ROI", 222: "DO_GUIDED_LIMITS"}
NAV_WITH_POSITION = (16, 17, 19, 21, 22, 82)
ACK_RESULT = {0: "aceita", 1: "erro", 2: "frame nao suportado", 3: "comando nao suportado", 4: "sem espaco",
              5: "invalida", 13: "sequencia errada", 15: "negada", 14: "cancelada"}


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


# ---------------------------------------------------------------------------
# Talking to the rover
# ---------------------------------------------------------------------------
class MissionClient:
    def __init__(self, conn, kick=em.HERELINK_KICK, log=print):
        self.link = em.Link(conn, kick)
        self.parser = em.MavlinkParser()
        self.enc = em.Encoder()
        self.sysid = None
        self.compid = 1
        self.log = log
        self.last_hb = 0.0

    def close(self):
        try:
            self.link.sock.close()
        except OSError:
            pass

    def _pump(self):
        """Heartbeat + one receive; returns decoded (msgid, payload) of the vehicle."""
        now = time.monotonic()
        if now - self.last_hb >= 1.0:
            self.last_hb = now
            self.link.send(self.enc.heartbeat())
        out = []
        data = self.link.recv()
        if not data:
            return out
        for msgid, sysid, compid, payload in self.parser.feed(data):
            if sysid == em.GCS_SYSID:
                continue
            if msgid == em.MSG_HEARTBEAT:
                hb = em.decode(msgid, payload)
                if hb["autopilot"] != em.MAV_AUTOPILOT_INVALID and hb["type"] != em.MAV_TYPE_GCS and self.sysid is None:
                    self.sysid, self.compid = sysid, compid
                continue
            if self.sysid is not None and sysid == self.sysid:
                out.append((msgid, payload))
        return out

    def wait_vehicle(self, timeout=15):
        end = time.monotonic() + timeout
        while self.sysid is None:
            if time.monotonic() > end:
                raise TimeoutError("rover nao respondeu (Wi-Fi AndroidAP, rede Privada, monitor fechado?)")
            self._pump()

    def download(self):
        self.wait_vehicle()
        count = None
        end = time.monotonic() + 20
        last_req = 0.0
        while count is None:
            if time.monotonic() > end:
                raise TimeoutError("sem resposta ao pedido da missao")
            if time.monotonic() - last_req > 1.5:
                last_req = time.monotonic()
                self.link.send(self.enc.pack(MSG_MISSION_REQUEST_LIST, struct.pack("<BB", self.sysid, self.compid)))
            for msgid, p in self._pump():
                if msgid == MSG_MISSION_COUNT:
                    count = struct.unpack("<H", p[:2])[0]
        self.log("Missao com %d itens (0 = home)" % count)
        items = []
        for seq in range(count):
            got = None
            tries = 0
            last_req = 0.0
            while got is None:
                if time.monotonic() - last_req > 1.0:
                    if tries >= 8:
                        raise TimeoutError("item %d nao chegou" % seq)
                    tries += 1
                    last_req = time.monotonic()
                    self.link.send(self.enc.pack(MSG_MISSION_REQUEST_INT,
                                                 struct.pack("<HBB", seq, self.sysid, self.compid)))
                for msgid, p in self._pump():
                    if msgid == MSG_MISSION_ITEM_INT:
                        it = Item.from_payload(p)
                        if it.seq == seq:
                            got = it
            items.append(got)
            self.log("Baixando item %d/%d" % (seq + 1, count))
        self.link.send(self.enc.pack(MSG_MISSION_ACK, struct.pack("<BBB", self.sysid, self.compid, 0)))
        return items

    def upload(self, items):
        self.wait_vehicle()
        for i, it in enumerate(items):
            it.seq = i
        n = len(items)
        last_count = 0.0
        last_activity = time.monotonic()
        last_seq = -1
        sent = set()
        start = time.monotonic()
        while True:
            now = time.monotonic()
            if not sent and now - last_count > 1.5:
                if now - start > 15:
                    raise TimeoutError("o rover nao pediu os itens")
                last_count = now
                self.link.send(self.enc.pack(MSG_MISSION_COUNT, struct.pack("<HBB", n, self.sysid, self.compid)))
            if now - last_activity > 10:
                raise TimeoutError("envio parado no item %d" % (max(sent) + 1 if sent else 0))
            # ArduPilot repeats its own request when an item is lost; only the final
            # ack can go missing for good, so after the last item resend it until acked
            if last_seq == n - 1 and now - last_activity > 1.5 and now - last_count > 1.5:
                last_count = now
                last = last_seq
                self.link.send(self.enc.pack(MSG_MISSION_ITEM_INT, items[last].payload(self.sysid, self.compid)))
            for msgid, p in self._pump():
                if msgid in (MSG_MISSION_REQUEST_INT, MSG_MISSION_REQUEST):
                    seq = struct.unpack("<H", p[:2])[0]
                    # The HereLink delivers each request twice; answering both puts a
                    # second copy of the item out of order (the rover then acks
                    # "invalid sequence"). Answer a repeated request only after a pause.
                    if seq < n and not (seq == last_seq and time.monotonic() - last_activity < 0.7):
                        self.link.send(self.enc.pack(MSG_MISSION_ITEM_INT, items[seq].payload(self.sysid, self.compid)))
                        sent.add(seq)
                        last_seq = seq
                        last_activity = time.monotonic()
                        self.log("Enviando item %d/%d" % (seq + 1, n))
                elif msgid == MSG_MISSION_ACK and sent:
                    result = p[2]
                    if result == 0 and (n - 1) in sent:
                        return
                    if result in (0, 13):
                        # 13 = item out of sequence: ArduPilot only reports it and keeps
                        # asking for the item it expects, so the upload goes on
                        continue
                    raise RuntimeError("rover recusou a missao: %s" % ACK_RESULT.get(result, result))


# ---------------------------------------------------------------------------
# Mission Planner .waypoints files
# ---------------------------------------------------------------------------
def load_waypoints(path):
    items = []
    with open(path, encoding="utf-8") as f:
        head = f.readline()
        if not head.startswith("QGC WPL"):
            raise ValueError("nao e um arquivo .waypoints do Mission Planner")
        for line in f:
            c = line.strip().split("\t")
            if len(c) < 12:
                continue
            items.append(Item(int(c[0]), int(c[2]), int(c[3]), int(c[1]), int(float(c[11])),
                              float(c[4]), float(c[5]), float(c[6]), float(c[7]),
                              float(c[8]), float(c[9]), float(c[10])))
    return items


def save_waypoints(path, items):
    with open(path, "w", encoding="utf-8", newline="\r\n") as f:
        f.write("QGC WPL 110\n")
        for i, it in enumerate(items):
            f.write("\t".join([str(i), str(it.current), str(it.frame), str(it.command)]
                              + ["%.8f" % v for v in (it.p1, it.p2, it.p3, it.p4, it.lat, it.lon)]
                              + ["%.6f" % it.z, str(it.autocontinue)]) + "\n")


# ---------------------------------------------------------------------------
# Mixer rules
# ---------------------------------------------------------------------------
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
                if reverse:
                    warn = "alinhamento dentro da re: a re desliga"
                else:
                    # a DO_SET_REVERSE(1) after this waypoint, before the next nav command
                    for nxt in items[idx + 1:]:
                        if nxt.command == CMD_DO_SET_REVERSE:
                            if nxt.p1 >= 0.5:
                                warn = "alinha antes de DO_SET_REVERSE: a re nao liga"
                            break
                        if nxt.command < 100:   # next NAV command
                            break
                if not warn and (it.z > 360 or it.z < -1):
                    warn = "Alt %.1f vira rumo %.0f graus (era altitude?)" % (it.z, it.z % 360)
        else:
            effect = ""
        out.append(("re" if reverse else "frente", effect, warn))
    return out


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class Compass(tk.Canvas):
    """Heading dial: 0 = north (up), clockwise. Click or drag to choose."""

    def __init__(self, master, on_pick, size=280):
        super().__init__(master, width=size, height=size, bg="white", highlightthickness=0)
        self.size = size
        self.on_pick = on_pick
        self.angle = None
        self.enabled = False
        self.bind("<Button-1>", self._click)
        self.bind("<B1-Motion>", self._click)
        self.draw()

    def _click(self, ev):
        if not self.enabled:
            return
        c = self.size / 2
        ang = math.degrees(math.atan2(ev.x - c, c - ev.y)) % 360
        step = 1 if (ev.state & 0x0001) else 5   # shift = 1 degree, otherwise 5
        ang = (round(ang / step) * step) % 360
        self.on_pick(ang)

    def set(self, angle, enabled):
        self.angle, self.enabled = angle, enabled
        self.draw()

    def draw(self):
        self.delete("all")
        s, c, r = self.size, self.size / 2, self.size / 2 - 52
        fg = "#222" if self.enabled else "#bbb"
        self.create_oval(c - r, c - r, c + r, c + r, outline=fg, width=2)
        for d in range(0, 360, 15):
            a = math.radians(d)
            r0 = r - (12 if d % 90 == 0 else 7 if d % 45 == 0 else 4)
            self.create_line(c + r0 * math.sin(a), c - r0 * math.cos(a), c + r * math.sin(a), c - r * math.cos(a),
                             fill=fg, width=2 if d % 45 == 0 else 1)
        for d, lab, anchor in ((0, "N 0", "s"), (90, "L 90", "w"), (180, "S 180", "n"), (270, "O 270", "e")):
            a = math.radians(d)
            self.create_text(c + (r + 4) * math.sin(a), c - (r + 4) * math.cos(a), text=lab, fill=fg,
                             anchor=anchor, font=("Segoe UI", 9, "bold"))
        if self.enabled and self.angle is not None:
            a = math.radians(self.angle)
            tipx, tipy = c + (r - 14) * math.sin(a), c - (r - 14) * math.cos(a)
            self.create_line(c, c, tipx, tipy, fill="#d62728", width=4, arrow=tk.LAST, arrowshape=(14, 16, 6))
            self.create_text(6, 4, text="%.0f graus" % self.angle, anchor="nw", fill="#d62728", font=("Segoe UI", 11, "bold"))
        elif self.enabled:
            self.create_text(c, c, text="passa direto\n(Alt = -1)", fill="#555", justify="center")
        else:
            self.create_text(c, c, text="selecione um\nWAYPOINT", fill="#999", justify="center")


class App:
    def __init__(self, root):
        self.root = root
        root.title("Rover - editor de rumos da missao")
        self.items = []
        self.busy = False
        self.dirty = False

        top = ttk.Frame(root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Conexao:").pack(side="left")
        self.conn = tk.StringVar(value="udp:0.0.0.0:14550")
        ttk.Entry(top, textvariable=self.conn, width=22).pack(side="left", padx=4)
        self.btns = []
        for text, cmd in (("Baixar do rover", self.download), ("Enviar ao rover", self.upload),
                          ("Abrir .waypoints", self.open_file), ("Salvar .waypoints", self.save_file)):
            b = ttk.Button(top, text=text, command=cmd)
            b.pack(side="left", padx=3)
            self.btns.append(b)
        self.status = tk.StringVar(value="Feche o esc_monitor.py antes de baixar/enviar (mesma porta UDP).")
        ttk.Label(root, textvariable=self.status, padding=(8, 0)).pack(fill="x")

        body = ttk.Frame(root, padding=6)
        body.pack(fill="both", expand=True)
        cols = ("n", "cmd", "p1", "alt", "sentido", "efeito", "alerta")
        self.tree = ttk.Treeview(body, columns=cols, show="headings", height=22, selectmode="browse")
        for col, w, txt in (("n", 40, "#"), ("cmd", 150, "Comando"), ("p1", 45, "P1"), ("alt", 70, "Alt"),
                            ("sentido", 70, "Sentido"), ("efeito", 190, "Efeito"), ("alerta", 390, "Alerta")):
            self.tree.heading(col, text=txt)
            self.tree.column(col, width=w, anchor="w")
        self.tree.tag_configure("warn", foreground="#c00000")
        self.tree.tag_configure("re", background="#fff1e0")
        self.tree.tag_configure("align", font=("Segoe UI", 9, "bold"))
        self.tree.pack(side="left", fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self.on_select)

        right = ttk.Frame(body, padding=(8, 0))
        right.pack(side="left", fill="y")
        self.sel_label = tk.StringVar(value="")
        ttk.Label(right, textvariable=self.sel_label, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.compass = Compass(right, self.pick_angle)
        self.compass.pack(pady=4)
        ttk.Label(right, text="Clique na bussola (passo 5; Shift = 1 grau)", foreground="#666").pack()
        row = ttk.Frame(right)
        row.pack(pady=4)
        ttk.Label(row, text="Rumo (graus):").pack(side="left")
        self.angle_var = tk.StringVar()
        self.angle_entry = ttk.Entry(row, textvariable=self.angle_var, width=7)
        self.angle_entry.pack(side="left", padx=4)
        self.angle_entry.bind("<Return>", lambda e: self.apply_entry())
        ttk.Button(row, text="Aplicar", command=self.apply_entry).pack(side="left")
        self.pass_var = tk.BooleanVar()
        self.pass_chk = ttk.Checkbutton(right, text="Passa direto (Alt = -1)", variable=self.pass_var,
                                        command=self.toggle_pass)
        self.pass_chk.pack(anchor="w")
        ttk.Button(right, text="Rumo do proximo trecho", command=self.next_leg).pack(fill="x", pady=(8, 2))
        ttk.Button(right, text="Todos os alinhamentos -> proximo trecho", command=self.all_next_leg).pack(fill="x", pady=2)
        ttk.Button(right, text="Todos sem rumo -> passa direto", command=self.all_pass).pack(fill="x", pady=2)
        ttk.Label(right, text="Mapa (norte para cima)").pack(anchor="w", pady=(8, 0))
        self.map = tk.Canvas(right, width=300, height=260, bg="#fafafa", highlightthickness=1,
                             highlightbackground="#ccc")
        self.map.pack()
        ttk.Label(right, text="verde = frente, laranja = re, seta vermelha = alinhamento",
                  foreground="#666").pack(anchor="w")
        self.set_enabled_editor(False)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---- helpers
    def selected(self):
        sel = self.tree.selection()
        return int(sel[0]) if sel else None

    def set_enabled_editor(self, on):
        state = "normal" if on else "disabled"
        self.angle_entry.configure(state=state)
        self.pass_chk.configure(state=state)

    def refresh(self, keep=None):
        self.tree.delete(*self.tree.get_children())
        info = analyse(self.items)
        for i, (it, (direction, effect, warn)) in enumerate(zip(self.items, info)):
            tags = []
            if warn:
                tags.append("warn")
            if direction == "re":
                tags.append("re")
            if it.command == CMD_WAYPOINT and not is_pass(it.z) and i > 0:
                tags.append("align")
            name = CMD_NAMES.get(it.command, str(it.command))
            p1 = ("%g" % it.p1) if it.command == CMD_DO_SET_REVERSE else ""
            alt = "%g" % it.z if it.command in NAV_WITH_POSITION or i == 0 else ""
            self.tree.insert("", "end", iid=str(i), values=(i, name, p1, alt, direction, effect, warn), tags=tags)
        if keep is not None and keep < len(self.items):
            self.tree.selection_set(str(keep))
            self.tree.see(str(keep))
        self.draw_map()
        nwarn = sum(1 for x in info if x[2])
        self.root.title("Rover - editor de rumos da missao%s%s" % (" *" if self.dirty else "",
                                                                     "  (%d alertas)" % nwarn if nwarn else ""))

    def on_select(self, _=None):
        i = self.selected()
        if i is None or i == 0 or self.items[i].command != CMD_WAYPOINT:
            self.compass.set(None, False)
            self.set_enabled_editor(False)
            self.sel_label.set("" if i is None else "Item %d: so WAYPOINT tem rumo" % i)
            self.draw_map()
            return
        it = self.items[i]
        self.set_enabled_editor(True)
        self.sel_label.set("WAYPOINT %d" % i)
        if is_pass(it.z):
            self.pass_var.set(True)
            self.angle_var.set("")
            self.compass.set(None, True)
        else:
            self.pass_var.set(False)
            self.angle_var.set("%.0f" % (it.z % 360))
            self.compass.set(it.z % 360, True)
        self.draw_map()

    def set_z(self, z):
        i = self.selected()
        if i is None or i == 0 or self.items[i].command != CMD_WAYPOINT:
            return
        self.items[i].z = z
        self.dirty = True
        self.refresh(keep=i)
        self.on_select()

    def pick_angle(self, ang):
        self.set_z(float(ang))

    def apply_entry(self):
        txt = self.angle_var.get().strip().replace(",", ".")
        try:
            ang = float(txt) % 360
        except ValueError:
            messagebox.showerror("Rumo", "Digite um numero de 0 a 359")
            return
        self.set_z(ang)

    def toggle_pass(self):
        if self.pass_var.get():
            self.set_z(-1.0)
        else:
            self.set_z(0.0)

    def leg_bearing(self, i):
        """Bearing (deg, 0 = north) from waypoint i to the next nav item with a position."""
        a = self.items[i]
        for b in self.items[i + 1:]:
            if b.command in NAV_WITH_POSITION and (b.lat or b.lon):
                dn = b.lat - a.lat
                de = (b.lon - a.lon) * math.cos(math.radians(a.lat))
                if abs(dn) + abs(de) < 1e-9:
                    continue
                return math.degrees(math.atan2(de, dn)) % 360
        return None

    def next_leg(self):
        i = self.selected()
        if i is None or i == 0 or self.items[i].command != CMD_WAYPOINT:
            return
        brg = self.leg_bearing(i)
        if brg is None:
            messagebox.showinfo("Rumo", "Nao ha waypoint depois deste")
            return
        self.set_z(round(brg, 1))

    def all_next_leg(self):
        n = 0
        for i, it in enumerate(self.items):
            if i and it.command == CMD_WAYPOINT and not is_pass(it.z):
                brg = self.leg_bearing(i)
                if brg is not None:
                    it.z = round(brg, 1)
                    n += 1
        if n:
            self.dirty = True
        self.refresh(keep=self.selected())
        self.on_select()
        self.status.set("%d alinhamentos agora apontam para o proximo trecho" % n)

    def all_pass(self):
        n = 0
        info = analyse(self.items)
        for i, it in enumerate(self.items):
            if i and it.command == CMD_WAYPOINT and not is_pass(it.z) and (it.z > 360 or it.z < -1 or info[i][2]):
                it.z = -1.0
                n += 1
        if n:
            self.dirty = True
        self.refresh(keep=self.selected())
        self.status.set("%d waypoints com Alt de altitude ou em conflito com a re viraram 'passa direto'" % n)

    # ---- map
    def draw_map(self):
        cv = self.map
        cv.delete("all")
        w, h = int(cv["width"]), int(cv["height"])
        pts = [(i, it) for i, it in enumerate(self.items)
               if i > 0 and it.command in NAV_WITH_POSITION and (it.lat or it.lon)]
        if not pts:
            cv.create_text(w / 2, h / 2, text="sem waypoints", fill="#999")
            return
        lat0 = pts[0][1].lat
        k = 111320.0
        xy = {i: ((it.lon - pts[0][1].lon) * k * math.cos(math.radians(lat0)), (it.lat - lat0) * k) for i, it in pts}
        xs = [p[0] for p in xy.values()]
        ys = [p[1] for p in xy.values()]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0)
        sc = (min(w, h) - 50) / span
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2

        def px(i):
            x, y = xy[i]
            return w / 2 + (x - cx) * sc, h / 2 - (y - cy) * sc

        info = analyse(self.items)
        prev = None
        for i, it in pts:
            if prev is not None:
                color = "#e67e22" if info[i][0] == "re" else "#2ca02c"
                (x0, y0), (x1, y1) = px(prev), px(i)
                cv.create_line(x0, y0, x1, y1, fill=color, width=3, arrow=tk.LAST, arrowshape=(10, 12, 4),
                               dash=(6, 3) if info[i][0] == "re" else None)
            prev = i
        sel = self.selected()
        for i, it in pts:
            x, y = px(i)
            r = 7 if i == sel else 5
            fill = "#c00000" if info[i][2] else ("#1f77b4" if i != sel else "#000")
            cv.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline="white")
            cv.create_text(x + 9, y - 9, text=str(i), font=("Segoe UI", 8))
            if it.command == CMD_WAYPOINT and not is_pass(it.z):
                a = math.radians(it.z % 360)
                cv.create_line(x, y, x + 24 * math.sin(a), y - 24 * math.cos(a), fill="#d62728", width=2,
                               arrow=tk.LAST)
        # scale bar
        meters = 10 ** math.floor(math.log10(span / 2)) if span > 2 else 1
        cv.create_line(10, h - 12, 10 + meters * sc, h - 12, width=2)
        cv.create_text(12 + meters * sc, h - 12, text=" %g m" % meters, anchor="w", font=("Segoe UI", 8))
        cv.create_text(w - 14, 14, text="N", font=("Segoe UI", 10, "bold"))
        cv.create_line(w - 14, 40, w - 14, 24, arrow=tk.LAST, width=2)

    # ---- actions
    def run_bg(self, title, fn, done):
        if self.busy:
            return
        self.busy = True
        for b in self.btns:
            b.configure(state="disabled")

        def log(msg):
            self.root.after(0, self.status.set, msg)

        def work():
            client = None
            try:
                client = MissionClient(self.conn.get(), log=log)
                result = fn(client)
                self.root.after(0, done, result)
            except Exception as exc:   # report any failure in the window
                self.root.after(0, messagebox.showerror, title, str(exc))
                log("%s: %s" % (title, exc))
            finally:
                if client:
                    client.close()
                self.root.after(0, self.end_busy)

        threading.Thread(target=work, daemon=True).start()

    def end_busy(self):
        self.busy = False
        for b in self.btns:
            b.configure(state="normal")

    def download(self):
        if self.dirty and not messagebox.askyesno("Baixar", "Descartar as alteracoes nao enviadas?"):
            return

        def done(items):
            self.items = items
            self.dirty = False
            self.refresh()
            self.status.set("Missao baixada: %d itens" % len(items))

        self.run_bg("Baixar", lambda c: c.download(), done)

    def upload(self):
        if len(self.items) < 2:
            messagebox.showinfo("Enviar", "Nenhuma missao carregada")
            return
        nwarn = sum(1 for x in analyse(self.items) if x[2])
        msg = "Enviar %d itens ao rover? A missao atual do rover sera substituida." % len(self.items)
        if nwarn:
            msg += "\n\nATENCAO: %d alertas na lista (vermelho)." % nwarn
        if not messagebox.askyesno("Enviar", msg):
            return
        items = [Item(*[getattr(it, s) for s in Item.__slots__]) for it in self.items]

        def done(_):
            self.dirty = False
            self.refresh(keep=self.selected())
            self.status.set("Missao enviada e aceita pelo rover (%d itens). Baixe de novo para conferir." % len(items))

        self.run_bg("Enviar", lambda c: c.upload(items), done)

    def open_file(self):
        path = filedialog.askopenfilename(filetypes=[("Mission Planner", "*.waypoints *.txt"), ("Todos", "*.*")])
        if not path:
            return
        try:
            self.items = load_waypoints(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Abrir", str(exc))
            return
        self.dirty = True
        self.refresh()
        self.status.set("Aberto %s (%d itens)" % (os.path.basename(path), len(self.items)))

    def save_file(self):
        if not self.items:
            return
        path = filedialog.asksaveasfilename(defaultextension=".waypoints",
                                            filetypes=[("Mission Planner", "*.waypoints")])
        if path:
            save_waypoints(path, self.items)
            self.status.set("Salvo em %s" % path)

    def on_close(self):
        if self.dirty and not messagebox.askyesno("Sair", "Sair sem enviar as alteracoes ao rover?"):
            return
        self.root.destroy()


def main():
    if sys.platform == "win32":
        try:   # sharp text on scaled Windows displays
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
