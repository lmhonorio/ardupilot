#!/usr/bin/env python3
"""Small container health check; detailed operational state stays on ROS status."""
import json
from pathlib import Path
import sys
import time
try:
    run=Path(Path('/data/latest.txt').read_text().strip())
    if not str(run).startswith('/data/runs/'):raise ValueError('Diretorio de execucao invalido')
    log=run/'manager/events.jsonl'
    with log.open('rb') as f:
        offset=max(0,log.stat().st_size-65536);f.seek(offset)
        lines=f.read().decode().splitlines()
        if offset:lines=lines[1:]
    status=None
    for line in reversed(lines):
        try:row=json.loads(line)
        except ValueError:continue
        if row.get('kind')=='status':status=row;break
    if status is None or time.time()-status['wall']>5:raise ValueError('Status ausente/desatualizado')
    print(status['state'],status.get('last_error'))
    sys.exit(0 if status['fresh'] and status['state']!='FALHA' and status['inside'] is not None else 1)
except (OSError,ValueError,KeyError) as exc:
    print(str(exc));sys.exit(1)
