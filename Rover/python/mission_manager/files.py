"""Strict mission import and MAVLink readback comparison; no vehicle side effects."""
import json
import math
from pathlib import Path
import struct

from mission_manager.mission import validate


def load_mission(path):
    path = Path(path).expanduser().resolve(strict=True)
    if path.stat().st_size > 2_000_000:
        raise ValueError('Arquivo de missao maior que 2 MB')
    text = path.read_text(encoding='utf-8-sig')
    if text.lstrip().startswith('{'):
        data = json.loads(text)
        if data.get('fileType') != 'Plan':
            raise ValueError('JSON deve ser um arquivo QGroundControl .plan')
        plan = data['mission']
        home = plan['plannedHomePosition']
        items = [dict(seq=0, command=16, frame=0, x=home[0], y=home[1], z=home[2],
                      autocontinue=1, param1=0, param2=0, param3=0, param4=0)]
        for seq, item in enumerate(plan['items'], 1):
            if item.get('type') != 'SimpleItem':
                raise ValueError('Itens complexos/survey nao suportados; exporte waypoints simples')
            params = item['params']
            if len(params) != 7:
                raise ValueError('Item .plan deve ter sete parametros')
            items.append(dict(seq=seq, command=item['command'], frame=item['frame'],
                              autocontinue=int(item.get('autoContinue', True)),
                              **dict(zip(('param1','param2','param3','param4','x','y','z'), params))))
    else:
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        if not lines or lines[0] != 'QGC WPL 110':
            raise ValueError('Use QGC WPL 110 (.waypoints) ou QGroundControl .plan')
        items = []
        for line in lines[1:]:
            f = line.split()
            if len(f) != 12:
                raise ValueError('Linha WPL deve conter 12 campos')
            items.append(dict(seq=int(f[0]), frame=int(f[2]), command=int(f[3]),
                              autocontinue=int(f[11]),
                              **dict(zip(('param1','param2','param3','param4','x','y','z'), map(float, f[4:11])))))
    for item in items:
        for key in ('seq','command','frame','autocontinue'):
            if not isinstance(item[key], int):
                raise ValueError('Campo inteiro invalido: ' + key)
        for key in ('param1','param2','param3','param4','x','y','z'):
            value = item[key]
            if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Item %s: %s deve ser numero finito (nao null/NaN)' % (item['seq'], key))
        if item['frame'] not in (0,2,3,5,6):
            raise ValueError('Frame nao suportado: %s' % item['frame'])
        if item['autocontinue'] != 1:
            raise ValueError('Todos os itens devem ter autocontinue=1')
    validate(items)
    return items


def verify_readback(expected, actual):
    """Ignore mutable HOME, accept only protocol precision/frame aliases."""
    if len(expected) != len(actual):
        raise ValueError('Readback: numero de itens diferente')
    frame = lambda f: {5:0, 6:3, 2:0}.get(f, f)
    f32 = lambda x: struct.unpack('<f', struct.pack('<f', x))[0]
    for a, b in zip(expected[1:], actual[1:]):
        for key in ('seq','command','autocontinue'):
            if a[key] != b[key]:
                raise ValueError('Readback: item %s campo %s diferente' % (a['seq'],key))
        if frame(a['frame']) != frame(b['frame']):
            raise ValueError('Readback: frame diferente no item %s' % a['seq'])
        for key in ('param1','param2','param3','param4','z'):
            if not math.isclose(f32(a[key]), b[key], rel_tol=1e-6, abs_tol=1e-6):
                raise ValueError('Readback: item %s campo %s diferente' % (a['seq'],key))
        for key in ('x','y'):
            if abs(round(a[key]*1e7)/1e7-b[key]) > 1.1e-7:
                raise ValueError('Readback: coordenada diferente no item %s' % a['seq'])
    validate(actual)
