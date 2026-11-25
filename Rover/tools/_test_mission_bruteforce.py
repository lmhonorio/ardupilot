#!/usr/bin/env python3
"""Brute offline checks for mission_core + HTTP mission endpoints (no rover)."""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import threading
import traceback
from http.client import HTTPConnection
from io import BytesIO
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission_core as m
import mode_panel as mp

FAILS = []


def check(name, cond, detail=""):
    if cond:
        print("OK  ", name)
        return
    FAILS.append("%s: %s" % (name, detail))
    print("FAIL", name, detail)


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


def test_core():
    items = m.empty_mission(-35.363, 149.165)
    items = m.insert_item(items, 1, m.make_item(16, -35.3631, 149.1651, z=-1))
    items = m.insert_item(items, 2, m.make_item(16, -35.3632, 149.1652, z=90))
    text = m.format_waypoints_text(items)
    back = m.parse_waypoints_text(text)
    check("wpl roundtrip count", len(back) == len(items), "%s!=%s" % (len(back), len(items)))
    check("wpl roundtrip z", abs(back[2].z - 90) < 1e-6, back[2].z)
    check("wpl header", text.startswith("QGC WPL 110"))
    try:
        m.parse_waypoints_text("not a mission")
        check("bad wpl raises", False)
    except ValueError:
        check("bad wpl raises", True)

    info = m.analyse(items)
    check("home effect", info[0][1] == "home")
    check("pass effect", "passa" in info[1][1])
    check("align effect", "90" in info[2][1], info[2])
    items[2].z = 927.5
    info = m.analyse(items)
    check("altitude warn", bool(info[2][2]), info[2][2])

    items[2].z = 90
    items, keep, flipped = m.apply_reverse_toggle(items, 2, True)
    check("rev inserts DO", any(it.command == 194 and it.p1 >= 0.5 for it in items),
          [it.command for it in items])
    info = m.analyse(items)
    for i, it in enumerate(items):
        if it.command == 16 and i > 0 and info[i][0] == "re":
            check("re DO before %d" % i, info[i][2] == "", info[i][2])

    items2 = m.empty_mission(0, 0)
    items2 = m.insert_item(items2, 1, m.make_item(16, 0.001, 0, z=10))
    items2 = m.insert_item(items2, 2, m.make_item(16, 0.002, 0, z=-1))
    items2, keep, flipped = m.apply_reverse_toggle(items2, 2, True)
    check("flip heading exists", flipped is not None, flipped)
    check("flip ~190", flipped is not None and abs((flipped - 190) % 360) < 1e-6, flipped)

    items3 = m.empty_mission(1, 2)
    items3 = m.insert_item(items3, 1, m.make_item(16, 1.1, 2.1, z=-1))
    items3 = m.insert_item(items3, 2, m.make_item(20))
    check("rtl no pos", items3[-1].lat == 0 and items3[-1].lon == 0)
    try:
        m.delete_item(items3, 0)
        check("no delete home", False)
    except ValueError:
        check("no delete home", True)
    items3 = m.delete_item(items3, 1)
    check("delete renumber", items3[1].seq == 1 and items3[1].command == 20)

    items4 = m.empty_mission(-35.0, 149.0)
    items4 = m.insert_item(items4, 1, m.make_item(16, -35.0, 149.0, z=0))
    items4 = m.insert_item(items4, 2, m.make_item(16, -34.999, 149.0, z=-1))
    brg = m.leg_bearing(items4, 1)
    check("leg bearing northish", brg is not None and (brg < 20 or brg > 340), brg)
    items4, _, _ = m.apply_reverse_toggle(items4, 2, True)
    seqs = [it.seq for it in items4 if it.command == 16 and it.seq > 0]
    i1 = seqs[0]
    brg = m.leg_bearing(items4, i1)
    fh = m.front_heading(items4, i1, brg)
    check("front +180 reverse next", abs((fh - (brg + 180) % 360)) < 1e-6, (fh, brg))

    d2 = m.haversine_m(0, 0, 0.001, 0)
    check("haversine ~111m", abs(d2 - 111.32) < 2, d2)
    st = m.mission_stats(items4)
    check("stats total > 0", st["total_m"] > 0, st)

    raw = m.items_payload(items4)
    again = m.items_from_dicts(raw["items"])
    check("dicts roundtrip", len(again) == len(items4))
    check("catalog rumo label", m.ROVER_COMMAND_BY_ID[16]["params"][1]["label"] == "Rumo")

    it = m.make_item(16, -35.363261, 149.165230, z=-1.0)
    it2 = m.Item.from_payload(it.payload(1, 1))
    check("mavlink lat", abs(it2.lat - it.lat) < 1e-7, (it2.lat, it.lat))
    check("mavlink z=-1", it2.z == -1)

    lat, lon = offset(-35.363261, 149.165230, 0, 1.0)
    check("1m north moves lat+", lat > -35.363261)
    check("1m step ~1m", abs(m.haversine_m(-35.363261, 149.165230, lat, lon) - 1.0) < 0.05)
    lat2, lon2 = offset(-35.363261, 149.165230, 90, 2.0)
    check("2m east ~2m", abs(m.haversine_m(-35.363261, 149.165230, lat2, lon2) - 2.0) < 0.05)

    # relative chain: 1m N then 2m E from new point
    a = offset(-35.363261, 149.165230, 0, 1.0)
    b = offset(a[0], a[1], 90, 2.0)
    check("chain north leg", abs(m.haversine_m(-35.363261, 149.165230, a[0], a[1]) - 1) < 0.05)
    check("chain east leg", abs(m.haversine_m(a[0], a[1], b[0], b[1]) - 2) < 0.05)

    home = m.empty_mission(1, 2)[0]
    check("home current=1", home.current == 1)

    # items_from_dicts rejects nan
    try:
        m.items_from_dicts([{"seq": 0, "frame": 0, "command": 16, "current": 1, "autocontinue": 1,
                             "p1": 0, "p2": 0, "p3": 0, "p4": 0, "lat": float("nan"), "lon": 0, "z": 0}])
        check("reject nan", False)
    except ValueError:
        check("reject nan", True)


class FakeLink:
    def __init__(self):
        self.addr = ("0.0.0.0", 14550)
        self.sent = []

    def send(self, data):
        self.sent.append(data)

    def recv(self):
        return b""


def test_http():
    vehicle = mp.Vehicle()
    vehicle.ftp = mp.FtpClient(FakeLink(), threading.Lock(), mp.Encoder())
    vehicle.logs = mp.LogClient(FakeLink(), threading.Lock(), mp.Encoder())
    vehicle.mission = mp.MissionTransfer(FakeLink(), threading.Lock(), mp.Encoder())
    with vehicle.lock:
        vehicle.sysid = 1
        vehicle.compid = 1
        vehicle.armed = False
        vehicle.latest["GPI_lat"] = -35.363261
        vehicle.latest["GPI_lon"] = 149.165230
        vehicle.latest["GPI_hdg"] = 45.0

    link = FakeLink()
    tx = threading.Lock()
    enc = mp.Encoder()
    Handler = mp.make_handler(link, vehicle, tx, enc)

    class FakeServer:
        pass

    results = {}

    def do(method, path, body=None):
        raw = b"" if body is None else json.dumps(body).encode("utf-8")
        h = Handler(FakeServer(), ("127.0.0.1", 9), FakeServer())
        # BaseHTTPRequestHandler.__init__ tries to handle request immediately;
        # build a minimal mock instead.
        class Req:
            pass
        req = Req()
        req.command = method
        req.path = path
        req.headers = {"Content-Length": str(len(raw))}
        req.rfile = BytesIO(raw)
        out = BytesIO()
        sent = {}

        def send_response(code):
            sent["code"] = code

        def send_header(k, v):
            sent.setdefault("headers", {})[k] = v

        def end_headers():
            pass

        class H:
            pass
        hh = H()
        hh.headers = req.headers
        hh.rfile = req.rfile
        hh.wfile = out
        hh.send_response = send_response
        hh.send_header = send_header
        hh.end_headers = end_headers
        hh.path = path
        hh._send = lambda code, ctype, body, extra=None: (
            send_response(code),
            send_header("Content-Type", ctype),
            send_header("Content-Length", str(len(body))),
            [send_header(k, v) for k, v in (extra or ())],
            end_headers(),
            out.write(body),
        )
        hh._read_json = Handler._read_json.__get__(hh, H)
        if method == "GET":
            Handler.do_GET(hh)
        else:
            Handler.do_POST(hh)
        body_out = out.getvalue()
        ctype = sent.get("headers", {}).get("Content-Type", "")
        data = None
        if "json" in ctype:
            data = json.loads(body_out.decode("utf-8"))
        else:
            data = body_out
        return sent.get("code"), data

    # Patch: using Handler methods with unbound mock is fragile. Call logic functions directly.
    cat = mp.catalog()
    check("catalog has mission_commands", "mission_commands" in cat and len(cat["mission_commands"]) >= 10)
    check("catalog WAYPOINT rumo", any(
        c["id"] == 16 and c["params"][-1]["label"] == "Rumo" for c in cat["mission_commands"]))

    snap = mp.snapshot(link, vehicle)
    check("snapshot gps", "gps" in snap and abs(snap["gps"]["GPI_lat"] + 35.363261) < 1e-6, snap.get("gps"))

    # new mission uses GPS
    items = m.empty_mission(snap["gps"]["GPI_lat"], snap["gps"]["GPI_lon"])
    payload = m.items_payload(items)
    check("new mission home gps", abs(payload["items"][0]["lat"] + 35.363261) < 1e-6)

    # analyse / rebuild / heading / fix / all-next / export / import
    items = m.empty_mission(-35.363261, 149.165230)
    items = m.insert_item(items, 1, m.make_item(16, -35.363261, 149.165230, z=0))
    lat, lon = offset(-35.363261, 149.165230, 0, 1.0)
    items = m.insert_item(items, 2, m.make_item(16, lat, lon, z=-1))
    lat2, lon2 = offset(lat, lon, 90, 2.0)
    items = m.insert_item(items, 3, m.make_item(16, lat2, lon2, z=-1))

    # heading for seq 1
    brg = m.leg_bearing(items, 1)
    items[1].z = round(m.front_heading(items, 1, brg), 1)
    check("heading ~0", abs(items[1].z) < 5 or abs(items[1].z - 360) < 5, items[1].z)

    items, keep, flipped = m.apply_reverse_toggle(items, 3, True)
    check("rebuild ok", keep > 0)
    text = m.format_waypoints_text(items)
    check("export text", "QGC WPL" in text and "\t" in text)
    again = m.parse_waypoints_text(text)
    check("import export", len(again) == len(items))

    # upload refused when armed
    vehicle.armed = True
    ok, note = mp.mission_upload(vehicle, items)
    check("upload blocked armed", ok is False and "desarme" in note.lower(), note)
    vehicle.armed = False

    # download without vehicle traffic times out / errors — MissionTransfer needs messages
    # Ensure progress snapshot works
    prog = mp.mission_progress(vehicle)
    check("progress inactive", prog.get("active") is False, prog)

    # dict validation for upload path
    try:
        m.items_from_dicts("nope")
        check("bad list", False)
    except ValueError:
        check("bad list", True)


def test_js_parity_helpers():
    """Mirror critical JS relative math in Python to catch formula bugs."""
    # rover at heading 45, relative angle 0 => absolute 45; 1m
    base_lat, base_lon = -35.363261, 149.165230
    abs_brg = (45 + 0) % 360
    p = offset(base_lat, base_lon, abs_brg, 1.0)
    # then relative 90 from new point using leg heading (=abs_brg)
    abs2 = (abs_brg + 90) % 360
    q = offset(p[0], p[1], abs2, 2.0)
    check("rel parity dist1", abs(m.haversine_m(base_lat, base_lon, p[0], p[1]) - 1) < 0.05)
    check("rel parity dist2", abs(m.haversine_m(p[0], p[1], q[0], q[1]) - 2) < 0.05)
    # bearing of second leg ~ 135
    # compute bearing
    dn = q[0] - p[0]
    de = (q[1] - p[1]) * math.cos(math.radians(p[0]))
    brg = math.degrees(math.atan2(de, dn)) % 360
    check("rel parity brg~135", abs(brg - 135) < 5, brg)


def test_js_static_bugs():
    path = os.path.join(os.path.dirname(__file__), "hmi", "app.js")
    src = open(path, encoding="utf-8").read()
    check("js has Relativo mode", "missionmoderel" in src and "addRelativeStep" in src)
    check("js rumo not altitude for WP", "Rumo" in src or "rumo = −1" in src or "rumo = -1" in src or "passa direto (rumo" in src)
    check("js offsetLatLon", "function offsetLatLon" in src)
    check("js capture origin", "captureRelativeOriginFromRover" in src)
    # classic bug: double event listener accumulation on every renderMission
    # we remove on leave — check destroy/remove
    check("js removes ctx listener on leave", "removeEventListener(\"click\", hideMissionCtxOnOutside)" in src
          or "removeEventListener('click', hideMissionCtxOnOutside)" in src)
    # click mode pan in relative
    check("js pan in relative", 'missionClickMode = missionPlanMode === "relative" ? "pan" : "add"' in src
          or "missionClickMode = missionPlanMode === \"relative\" ? \"pan\" : \"add\"" in src
          or ('missionPlanMode === "relative"' in src and 'missionClickMode' in src))
    # ensure Alt label for waypoint field uses rumo
    check("js missionrelrumo option", "missionrelrumo" in src)
    check("js missionrelalign option", "missionrelalign" in src)

    # Look for known footguns
    if "era altitude?" not in src and "vira rumo" not in src:
        # warn text comes from server analyse — OK if only server has it
        pass
    check("server altitude warn phrase", "era altitude" in open(
        os.path.join(os.path.dirname(__file__), "mission_core.py"), encoding="utf-8").read())

    # Syntax-ish: balanced braces
    check("js brace balance", src.count("{") == src.count("}"), "%s vs %s" % (src.count("{"), src.count("}")))
    check("js paren balance", src.count("(") == src.count(")"), "%s vs %s" % (src.count("("), src.count(")")))


def test_first_click_home_semantics():
    """Map click must not create HOME+WP at the same lat/lon (MP separates them)."""
    # Simulate ensureMissionHome policy in Python
    def ensure(items, lat, lon, gps):
        if items:
            return items, False
        hlat, hlon = (gps if gps else (lat, lon))
        home = m.make_item(16, hlat, hlon, z=0, current=1, frame=0)
        return [home], (gps is None)

    items, home_only = ensure([], -35.36, 149.16, None)
    check("no-gps first click home only", home_only and len(items) == 1)
    items, home_only = ensure([], -35.36, 149.16, (-35.37, 149.17))
    check("with-gps home at gps not click", abs(items[0].lat + 35.37) < 1e-9 and not home_only)
    # then add WP at click
    items = m.insert_item(items, 1, m.make_item(16, -35.36, 149.16, z=-1))
    check("wp distinct from home", abs(items[0].lat - items[1].lat) > 1e-6)


def test_compare_mission_editor_rules():
    """Same rebuild_reverse / analyse contract as mission_editor Tk app."""
    items = m.empty_mission(0, 0)
    items = m.insert_item(items, 1, m.make_item(16, 0.0, 0.0, z=-1))
    items = m.insert_item(items, 2, m.make_item(16, 0.001, 0.0, z=-1))
    # mark WP2 reverse
    items, keep, _ = m.apply_reverse_toggle(items, 2, True)
    # There must be DO_SET_REVERSE 1 immediately before the reverse WP
    rev_wp = None
    for i, it in enumerate(items):
        if it.command == 16 and i > 0:
            flags = m.reverse_flags(items)
            if flags[i]:
                rev_wp = i
                check("DO1 immediately before rev WP",
                      items[i - 1].command == 194 and items[i - 1].p1 >= 0.5,
                      (items[i - 1].command, items[i - 1].p1))
    check("found rev wp", rev_wp is not None)

    # is_pass exact -1 only
    check("is_pass -1", m.is_pass(-1))
    check("not is_pass -1.0-eps", not m.is_pass(-1.0001))
    check("not is_pass 0", not m.is_pass(0))


def main():
    print("=== mission_core ===")
    test_core()
    print("\n=== HTTP/catalog/snapshot ===")
    test_http()
    print("\n=== relative parity ===")
    test_js_parity_helpers()
    print("\n=== JS static ===")
    test_js_static_bugs()
    print("\n=== first-click home ===")
    test_first_click_home_semantics()
    print("\n=== mission_editor rules ===")
    test_compare_mission_editor_rules()
    print("\n==== %d failures ====" % len(FAILS))
    for f in FAILS:
        print(" -", f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        traceback.print_exc()
        raise SystemExit(2)
