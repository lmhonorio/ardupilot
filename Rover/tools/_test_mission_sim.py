#!/usr/bin/env python3
"""Sobe fake rover + mode_panel e testa missao de ponta a ponta via HTTP."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
HTTP = 8766
MAV_PORT = 14551
FAKE_PORT = 14581
BASE = "http://127.0.0.1:%d" % HTTP

FAILS = []


def check(name, cond, detail=""):
    if cond:
        print("OK  ", name)
    else:
        FAILS.append("%s: %s" % (name, detail))
        print("FAIL", name, detail)


def http_json(method, path, body=None, timeout=30):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        BASE + path,
        data=data,
        method=method,
        headers={"Content-Type": "application/json"} if data is not None else {},
    )
    with urllib.request.urlopen(req, timeout=timeout) as res:
        raw = res.read()
        ctype = res.headers.get("Content-Type", "")
        if "json" in ctype:
            return res.status, json.loads(raw.decode("utf-8"))
        return res.status, raw


def wait_seen(deadline):
    while time.monotonic() < deadline:
        try:
            _, st = http_json("GET", "/state")
            if st.get("seen") and st.get("gps", {}).get("GPI_lat") is not None:
                return st
        except Exception:
            pass
        time.sleep(0.3)
    return None


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


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def main():
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    panel = subprocess.Popen(
        [PY, os.path.join(ROOT, "mode_panel.py"),
         "--conn", "udp:0.0.0.0:%d" % MAV_PORT,
         "--kick", "none",
         "--http", str(HTTP)],
        cwd=ROOT, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    # Wait until HTTP is up, then start the fake rover (avoids Win UDP reset).
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        try:
            http_json("GET", "/state")
            break
        except Exception:
            time.sleep(0.2)
    else:
        print("mode_panel nao subiu na porta %d" % HTTP)
        panel.terminate()
        return 2

    fake = subprocess.Popen(
        [PY, os.path.join(ROOT, "fake_rover_mission.py"),
         "--bind", "127.0.0.1:%d" % FAKE_PORT,
         "--gcs", "127.0.0.1:%d" % MAV_PORT],
        cwd=ROOT, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    try:
        print("aguardando rover simulado…", flush=True)
        st = wait_seen(time.monotonic() + 25)
        check("rover visto", st is not None, st)
        if not st:
            return 1
        check("gps presente", "GPI_lat" in st.get("gps", {}), st.get("gps"))
        check("desarmado", st.get("armed") is False, st.get("armed"))

        # catalog
        _, cat = http_json("GET", "/catalog")
        check("mission_commands", len(cat.get("mission_commands", [])) >= 10)

        # download seed mission
        code, dl = http_json("POST", "/mission/download", {})
        check("download ok", code == 200 and dl.get("ok"), dl)
        check("download 3 itens", dl.get("count") == 3, dl.get("count"))
        items = dl["items"]
        check("home current", items[0]["current"] == 1)
        check("wp1 pass", items[1]["z"] == -1)
        check("wp2 rumo 90", abs(items[2]["z"] - 90) < 1e-6, items[2]["z"])
        d01 = haversine(items[0]["lat"], items[0]["lon"], items[1]["lat"], items[1]["lon"])
        d12 = haversine(items[1]["lat"], items[1]["lon"], items[2]["lat"], items[2]["lon"])
        check("seed leg1 ~1m", abs(d01 - 1.0) < 0.08, d01)
        check("seed leg2 ~2m", abs(d12 - 2.0) < 0.08, d12)

        # relative step: +1m at 45° from home (rover hdg)
        lat0, lon0 = items[0]["lat"], items[0]["lon"]
        lat_n, lon_n = offset(lat0, lon0, 45.0, 1.0)
        new_items = [dict(x) for x in items]
        new_items.append({
            "seq": 3, "frame": 3, "command": 16, "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": lat_n, "lon": lon_n, "z": -1,
        })
        # set previous WP rumo to 45
        new_items[2]["z"] = 45.0

        code, an = http_json("POST", "/mission/analyse", {"items": new_items})
        check("analyse ok", code == 200 and an.get("ok"), an)
        check("analyse count 4", an.get("count") == 4, an.get("count"))

        code, rb = http_json("POST", "/mission/rebuild", {"items": an["items"], "seq": 3, "reverse": True})
        check("rebuild reverse ok", code == 200 and rb.get("ok"), rb)
        check("rebuild has DO", any(it["command"] == 194 for it in rb["items"]), 
              [it["command"] for it in rb["items"]])

        # upload rebuilt mission
        code, up = http_json("POST", "/mission/upload", {"items": rb["items"]}, timeout=40)
        check("upload ok", code == 200 and up.get("ok"), up)

        # download again and compare count
        code, dl2 = http_json("POST", "/mission/download", {}, timeout=40)
        check("redownload ok", code == 200 and dl2.get("ok"), dl2)
        check("redownload count", dl2.get("count") == rb.get("count"), (dl2.get("count"), rb.get("count")))
        # reverse WP still has DO before it
        warns = [it for it in dl2["items"] if it.get("warn")]
        check("sem alerta re sem DO", len(warns) == 0, warns)

        # heading next leg
        code, hd = http_json("POST", "/mission/heading", {"items": dl2["items"], "seq": 1})
        check("heading ok", code == 200 and hd.get("ok"), hd)

        # export / import
        req = urllib.request.Request(
            BASE + "/mission/export",
            data=json.dumps({"items": dl2["items"]}).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=20) as res:
            text = res.read().decode("utf-8")
        check("export QGC", text.startswith("QGC WPL"))
        code, imp = http_json("POST", "/mission/import", {"text": text})
        check("import ok", code == 200 and imp.get("ok") and imp.get("count") == dl2.get("count"), imp)

        # new mission from GPS
        code, nw = http_json("POST", "/mission/new", {})
        check("new ok", code == 200 and nw.get("ok") and nw.get("count") == 1, nw)
        check("new home near gps", abs(nw["items"][0]["lat"] - st["gps"]["GPI_lat"]) < 1e-5)

        # fix suspicious alt
        bad = [dict(x) for x in dl["items"]]
        bad[2]["z"] = 927.5
        code, fx = http_json("POST", "/mission/fix", {"items": bad})
        check("fix ok", code == 200 and fx["items"][2]["z"] == -1, fx["items"][2]["z"])

        print("\n==== %d failures ====" % len(FAILS))
        for f in FAILS:
            print(" -", f)
        return 1 if FAILS else 0
    finally:
        for proc in (panel, fake):
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        # show tails on failure
        if FAILS:
            for name, proc in (("panel", panel), ("fake", fake)):
                try:
                    out = proc.stdout.read().decode("utf-8", "replace") if proc.stdout else ""
                except Exception:
                    out = ""
                if out:
                    print("\n--- %s log (tail) ---" % name)
                    print("\n".join(out.splitlines()[-40:]))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except urllib.error.URLError as exc:
        print("HTTP falhou:", exc)
        raise SystemExit(2)
