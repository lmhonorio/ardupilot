#!/usr/bin/env python3
"""Own only these two ROS nodes and terminate both if either exits."""
import os, signal, subprocess, sys, time
from pathlib import Path
here=Path(__file__).resolve().parent
children=[]
def stop(signum,frame):
    for p in children:
        if p.poll() is None: p.terminate()
    raise SystemExit(0)
signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
try:
    for name in ('processor.py','mavlink_bridge.py'):
        children.append(subprocess.Popen([sys.executable,str(here/name)]))
    while all(p.poll() is None for p in children): time.sleep(.5)
    print('Um componente encerrou; interrompendo o modulo',flush=True)
    sys.exit(1)
finally:
    for p in children:
        if p.poll() is None: p.terminate()
    for p in children:
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: p.kill();p.wait()
