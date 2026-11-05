"""Espera GUIDED e proximidade da estacao do cenario SEParnaiba (SITL)."""
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ['MAVLINK20']='1'
from pymavlink import mavutil

# Centro (x=5 m, y=-2 m) do container no gravel_plane, a partir de SEParnaiba.
DOCK_LAT=-3.123167966
DOCK_LON=-41.764450017
MAX_DISTANCE=8.0
FRESH_SECONDS=3.0


def distance_m(lat, lon, dock_lat=DOCK_LAT, dock_lon=DOCK_LON):
    north=math.radians(lat-dock_lat)*6378137.0
    east=math.radians(lon-dock_lon)*6378137.0*math.cos(math.radians(dock_lat))
    return math.hypot(north,east)


def trigger_state(latest, seen, now, blocked, stations=None):
    """Exige telemetria atual do alvo, fix 3D, SITL, armado e modo GUIDED."""
    for kind in ('HEARTBEAT','GLOBAL_POSITION_INT','GPS_RAW_INT','SIMSTATE'):
        if kind not in latest or now-seen.get(kind,0)>FRESH_SECONDS:
            return False, None, 'aguardando '+kind
    hb=latest['HEARTBEAT']; pos=latest['GLOBAL_POSITION_INT']; gps=latest['GPS_RAW_INT']
    if hb.type != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        return False, None, 'alvo nao e um rover'
    if not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return False, None, 'rover desarmado'
    if hb.custom_mode != 15:
        return False, None, 'aguardando GUIDED'
    if blocked:
        return False, None, 'aguardando sair de GUIDED antes de nova tentativa'
    if gps.fix_type<3 or not (pos.lat or pos.lon):
        return False, None, 'aguardando GPS 3D'
    if not stations:
        return False, None, 'sem estacoes cadastradas'
    station=min(stations,key=lambda s:distance_m(pos.lat/1e7,pos.lon/1e7,s['latitude'],s['longitude']))
    distance=distance_m(pos.lat/1e7,pos.lon/1e7,station['latitude'],station['longitude'])
    if distance>=MAX_DISTANCE:
        return False, distance, 'fora dos 8 m'
    return True, distance, station



def load_stations(path):
    data=json.loads(Path(path).read_text())
    stations=data.get('estacoes')
    if not isinstance(stations,list) or not stations:
        raise ValueError('Cadastre ao menos uma estacao em '+str(path))
    for station in stations:
        lat=float(station['latitude']); lon=float(station['longitude'])
        offset=float(station['distancia_alvo_parede_m'])
        if not (math.isfinite(lat) and -90<=lat<=90 and math.isfinite(lon) and -180<=lon<=180
                and math.isfinite(offset) and 0<offset<5 and station['qr'] and station['nome']):
            raise ValueError('Cadastro de estacao invalido')
        station.update(latitude=lat,longitude=lon,distancia_alvo_parede_m=offset)
    return stations


def monitorar(args):
    """Observa apenas o SITL; uma tentativa por entrada em GUIDED."""
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    stations=load_stations(args.estacoes)
    blocked=False
    print('Aguardando GUIDED e menos de 8 m do centro do container (GPS SEParnaiba).',flush=True)
    print('Mantenha o rover parado ao ativar GUIDED; o QR sera procurado.',flush=True)
    try:
        while True:
            try:
                conn=mavutil.mavlink_connection(args.mavproxy,source_system=244,source_component=191,
                                                 dialect='ardupilotmega')
            except OSError as exc:
                print('MAVProxy indisponivel (%s); nova tentativa em 2 s.'%exc,flush=True)
                time.sleep(2)
                continue
            latest={}; seen={}; report=0.; heartbeat=0.; request=0.
            disconnected=False
            try:
                while True:
                    now=time.monotonic()
                    if now-heartbeat>=1:
                        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
                        heartbeat=now
                    if now-request>=2:
                        conn.mav.command_long_send(args.sysid,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                            0,mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE,500000,0,0,0,0,0)
                        request=now
                    msg=conn.recv_match(blocking=True,timeout=.2)
                    if msg and msg.get_srcSystem()==args.sysid and msg.get_srcComponent()==1:
                        kind=msg.get_type()
                        if kind in ('HEARTBEAT','GLOBAL_POSITION_INT','GPS_RAW_INT','SIMSTATE'):
                            latest[kind]=msg; seen[kind]=time.monotonic()
                            if kind=='HEARTBEAT' and msg.custom_mode!=15:
                                blocked=False
                    now=time.monotonic()
                    ready,distance,reason=trigger_state(latest,seen,now,blocked,stations)
                    if ready:
                        print('GUIDED e estacao %s a %.2f m; verificando QR.'%(reason['nome'],distance),flush=True)
                        blocked=True
                        break
                    if now-report>=10:
                        print('Espera: %s%s'%(reason,(' (%.2f m)'%distance) if distance is not None else ''),flush=True)
                        report=now
            except OSError as exc:
                print('Falha de comunicacao (%s); retomando espera.'%exc,flush=True)
                disconnected=True
            finally:
                conn.close()
            if disconnected:
                time.sleep(2)
                continue
            # Libera a porta UDP antes do controlador visual abrir sua conexao.
            cmd=[sys.executable,str(Path(__file__).with_name('entrar_container.py')),
                 '--uma-vez','--apenas-guided','--mavproxy',args.mavproxy,'--sysid',str(args.sysid),
                 '--velocidade',str(args.velocidade),'--timeout',str(args.timeout),
                 '--qr',reason['qr'],'--offset-parede',str(reason['distancia_alvo_parede_m']),
                 '--latitude-estacao',str(reason['latitude']),'--longitude-estacao',str(reason['longitude'])]
            try:
                child=subprocess.Popen(cmd)
            except OSError as exc:
                print('Falha ao iniciar controlador: %s'%exc,flush=True)
                time.sleep(2)
                continue
            try:
                result=child.wait()
            except BaseException:
                if child.poll() is None:
                    child.terminate()
                    child.wait()
                raise
            print('Tentativa encerrada (codigo %d); retornando a espera.'%result,flush=True)
            if result:
                print('Corrija o motivo e saia de GUIDED antes de tentar novamente.',flush=True)
    except KeyboardInterrupt:
        print('Espera de docking encerrada.',flush=True)
        return 0
