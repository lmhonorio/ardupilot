"""Bind an application docking request to an exact downloaded mission."""
import hashlib
import json
import math
from common.docking_stations import distance_and_bearing

def canonical(items):
    # HOME changes when arming, destination IDs/current flags are transport state.
    keys=('seq','command','frame','param1','param2','param3','param4','x','y','z','autocontinue')
    return [{k:round(float(i.get(k,0)),7) for k in keys} for i in items if i['seq']>0]

def fingerprint(items):
    return hashlib.sha256(json.dumps(canonical(items),sort_keys=True,allow_nan=False).encode()).hexdigest()

def validate(items):
    if len(items)<2 or [i['seq'] for i in items]!=list(range(len(items))):
        raise ValueError('Missao vazia ou sequencia invalida')
    if items[1]['command']!=16:
        raise ValueError('Nova missao deve iniciar com waypoint (seq 1)')
    # Initial version deliberately excludes jumping/scripted/actuator missions.
    # No ambiguous loops or physical relay requests can be interpreted as docking.
    if any(i['command'] not in (16,17,19,93,178) for i in items[1:]):
        raise ValueError('Missao contem comando nao validado pelo Mission Manager')
    fingerprint(items)
    for i in items[1:]:
        if i['command']==16 and (i['frame'] not in (0,3,5,6) or not -90<=i['x']<=90 or not -180<=i['y']<=180):
            raise ValueError('Waypoint GPS invalido')
    return True

def bind(items, station):
    validate(items)
    item=items[-1]
    if item['command']!=16:
        raise ValueError('Docking associado exige ultimo item waypoint, sem atuadores')
    distance,_=distance_and_bearing((item['x'],item['y']),(station['latitude'],station['longitude']))
    if not .7<distance<8:
        raise ValueError('Ultimo waypoint deve estar fora, a menos de 8 m do centro')
    # Point must be outside the entrance, on its approach side.
    _, bearing=distance_and_bearing((item['x'],item['y']),(station['latitude'],station['longitude']))
    if abs((bearing-station['rumo_entrada_graus']+180)%360-180)>45:
        raise ValueError('Ultimo waypoint fora do cone frontal da estacao')
    return dict(version=1,sha256=fingerprint(items),seq=item['seq'],station=station['nome'])

def requested(binding, items, reached, active, mode, distance):
    return bool(binding and mode==10 and distance<8 and reached==binding['seq']
                and active==binding['seq'] and binding['sha256']==fingerprint(items))
