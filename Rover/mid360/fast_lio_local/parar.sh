#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
/usr/bin/python3 - "$here" <<'PY'
import os,signal,sys
from pathlib import Path
root=Path(sys.argv[1]);p=root/'run/pid'
if not p.exists():raise SystemExit('Teste local nao esta rodando.')
pid=int(p.read_text());cmd=Path('/proc')/str(pid)/'cmdline'
if not cmd.exists() or str(root/'executar.py').encode() not in cmd.read_bytes():raise SystemExit('PID nao corresponde ao teste; nenhum sinal enviado.')
os.kill(pid,signal.SIGINT);print('Encerramento solicitado somente ao teste FAST-LIO local.')
PY
