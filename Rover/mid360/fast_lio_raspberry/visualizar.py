#!/usr/bin/env python3
"""Runs only the SSH result receiver and RViz, independently of the Pi estimator."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sem-rviz', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    run = root/'run'
    run.mkdir(exist_ok=True)
    lock = (run/'lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Visualizacao deste teste ja esta aberta.')
    (run/'pid').write_text(str(os.getpid()))
    children = []
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    commands = [('ponte', [sys.executable, str(root/'receber_resultados.py')])]
    if not args.sem_rviz:
        commands.append(('rviz', ['rviz2', '-d', str(root/'fastlio.rviz'), '--ros-args', '-p', 'use_sim_time:=false']))
    result = 0
    try:
        for name, cmd in commands:
            log = (run/(name+'.log')).open('w')
            child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append((name, child, log))
            print(name, 'PID', child.pid, flush=True)
        print('RViz no PC: dominio 23, frame camera_init. Ctrl+C fecha somente a visualizacao.', flush=True)
        while not stopping:
            for name, child, _ in children:
                if child.poll() is not None:
                    print(name, 'encerrou:', child.returncode, flush=True)
                    result = child.returncode or 0
                    stopping = True
                    break
            time.sleep(.2)
    finally:
        for _, child, _ in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for _, child, log in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
            log.close()
        (run/'pid').unlink(missing_ok=True)
        print('Visualizacao encerrada. FAST-LIO continua na Raspberry; parar.sh encerra o teste completo.')
    return result


if __name__ == '__main__':
    raise SystemExit(main())
