#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
remote=${ROVER_REMOTE:-rover@192.168.0.167}
[[ "$remote" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || exit 2
/usr/bin/python3 - "$here" <<'PY'
import os,signal,sys
from pathlib import Path
root=Path(sys.argv[1]);f=root/'run/pid'
if f.exists():
    pid=int(f.read_text());cmd=Path('/proc')/str(pid)/'cmdline'
    if cmd.exists() and str(root/'visualizar.py').encode() in cmd.read_bytes():
        os.kill(pid,signal.SIGINT)
PY
ssh "$remote" 'cd ~/mid360/fast_lio_raspberry && docker compose down'
"$here/../control.sh" down
echo 'Teste fisico encerrado: RViz/ponte no PC, FAST-LIO/adaptador/driver na Raspberry.'
