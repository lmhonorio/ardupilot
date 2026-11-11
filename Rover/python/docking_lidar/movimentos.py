"""Controle experimental no Gazebo por geometria MID360, sem camera/QR."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
from common.docking_stations import DEFAULT_STATIONS, load_stations, select_station, nearest_station

base=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(base/'missions'))
sys.path.insert(0,str(base/'docking'))
from sensores import Sensores
from referencia import recuo_lio
from docking_lidar.alinhamento import VisualAlignment, deve_frear
from controle_saida import validar_inicio, Giro180, alinhamento
from aguardar_docking import distance_m
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from mavproxy_link import Link

MASK=1|2|4|64|128|256|1024


def setpoint(link,linear,angular):
    link.conn.mav.set_position_target_local_ned_send(0,link.sysid,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
        MASK,0,0,0,linear,0,0,0,0,0,0,angular)


def drain(link,duration=.05):
    until=time.monotonic()+duration
    while time.monotonic()<until:link.recv(.002)


def check_guided(link):
    h=link.latest.get('HEARTBEAT')
    if not h or time.monotonic()-link.last_seen.get('HEARTBEAT',0)>2:
        raise RuntimeError('Telemetria indisponivel')
    if h.custom_mode!=15 or not h.base_mode&128:
        raise RuntimeError('GUIDED ou armamento interrompido')


def parar(link,timeout=20):
    link.mode('HOLD');start=time.monotonic();stable=None
    while time.monotonic()-start<timeout:
        link.recv(.1);now=time.monotonic();v=link.latest.get('VFR_HUD');att=link.latest.get('ATTITUDE')
        if (v and att and now-link.last_seen.get('VFR_HUD',0)<2 and now-link.last_seen.get('ATTITUDE',0)<2
                and v.groundspeed<.05 and abs(att.yawspeed)<.05):
            if stable is None:stable=now
            if now-stable>1:return
        else:stable=None
    raise RuntimeError('Parada nao confirmada')


def estacao(path,name):
    return select_station(load_stations(path),name)


def pose_confirmada(sensor,link,station,seconds=15):
    end=time.monotonic()+seconds;samples=[]
    while time.monotonic()<end:
        drain(link)
        try:
            pose=sensor.pose(link,station)
            if samples and pose['stamp']<=samples[-1]['stamp']:
                time.sleep(.05);continue
            samples.append(pose)
            samples=samples[-3:]
            if len(samples)==3:
                a=np.array([[p['forward'],p['right'],p['normal_yaw']] for p in samples])
                if np.max(np.ptp(a,axis=0)[:2])<.08 and np.ptp(a[:,2])<math.radians(5):return pose
        except ValueError as e:
            samples=[];last=str(e)
        time.sleep(.1)
    raise RuntimeError('Nao foi confirmada a geometria do container: '+locals().get('last','instavel'))


def movimento(tipo):
    p=argparse.ArgumentParser(description='Docking SITL com MID360: '+tipo)
    p.add_argument('--mavproxy',default='udpin:127.0.0.1:14551')
    p.add_argument('--sysid',type=int,default=1)
    p.add_argument('--estacoes',default=str(DEFAULT_STATIONS),help='Cadastro TXT de eixos das estacoes')
    p.add_argument('--estacao',default='container_gravel_plane' if tipo=='saida' else None,
                   help='Nome da estacao; entrada sem esta opcao seleciona a mais proxima')
    p.add_argument('--cloud',default='/rover_slam_3d/local_cloud',
                   help='XYZ recente compensado pelo FAST-LIO; inicie slam_mid360_3d.py')
    p.add_argument('--odom',default='/fast_lio/odometry',
                   help='Odometria local continua camera_init -> body, sem saltos de fechamento de lacos')
    p.add_argument('--diagnostico',action='store_true',help='Verifica MID360 e /clock, sem conectar ao MAVProxy ou comandar o rover')
    p.add_argument('--velocidade',type=float,default=.06)
    p.add_argument('--timeout',type=float,default=300)
    a=p.parse_args()
    if not a.mavproxy.startswith('udpin:127.0.0.1:') or not 1<=a.sysid<=255 or not 0<a.velocidade<=.10 or not 0<a.timeout<=600:
        p.error('Use UDP local, velocidade entre 0 e 0.10 m/s e timeout ate 600 s')
    try:
        stations=load_stations(a.estacoes)
        if a.estacao:stations=[select_station(stations,a.estacao)]
    except (OSError,ValueError) as exc:p.error(str(exc))
    station=stations[0] if a.estacao else None
    ros.init_node('docking_lidar_'+tipo)
    root=base/'run/docking_lidar'/(tipo+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));root.mkdir(parents=True)
    log=(root/'pose_lidar.jsonl').open('w')
    sensor=None;link=None;engaged=False;changed=False;overridden=False;success=False;saved={}
    signal.signal(signal.SIGTERM,lambda sig,frame:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        sensor=Sensores(a.cloud,a.odom)
        if a.diagnostico:
            msg=sensor.wait_cloud()
            print('MID360 OK (recepcao; geometria do container nao avaliada): topico=%s tipo=%s frame=%s stamp=%.3f' %
                  (a.cloud,type(msg).__name__,msg.header.frame_id,ros.seconds(msg.header.stamp)),flush=True)
            deadline=time.monotonic()+5
            while True:
                try:
                    position,yaw=sensor.lio()
                    print('Odometria ROS 2 OK em %s: posicao=%s yaw=%.3f' % (a.odom,position,yaw),flush=True)
                    break
                except ValueError:
                    if time.monotonic()>deadline:raise
                    time.sleep(.05)
            return 0
        link=Link(a.mavproxy,a.sysid,root,timeout=5,source_system=242)
        hb=link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1,timeout=15)
        if hb.type!=mavutil.mavlink.MAV_TYPE_GROUND_ROVER:raise RuntimeError('Alvo nao e Rover')
        link.conn.mav.command_long_send(a.sysid,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
          0,mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE,500000,0,0,0,0,0)
        link.wait(['SIMSTATE','SIM_STATE'],timeout=10)
        if tipo=='entrada':
            print('Aguardando GUIDED, rover armado e estacao a menos de 8 m.',flush=True)
            while not ros.is_shutdown():
                drain(link,.15);h=link.latest.get('HEARTBEAT');pos=link.latest.get('GLOBAL_POSITION_INT')
                fresh=(all(time.monotonic()-link.last_seen.get(k,0)<3
                           for k in ('HEARTBEAT','GLOBAL_POSITION_INT','GPS_RAW_INT','SIMSTATE')))
                gps=link.latest.get('GPS_RAW_INT')
                if fresh and gps and gps.fix_type>=3 and h and h.custom_mode==15 and h.base_mode&128 and pos and (pos.lat or pos.lon):
                    station=nearest_station(stations,pos.lat/1e7,pos.lon/1e7)
                    if distance_m(pos.lat/1e7,pos.lon/1e7,station['latitude'],station['longitude'])<8:break
                time.sleep(.1)
            if ros.is_shutdown():raise RuntimeError('Espera de docking encerrada')
            print('Estacao selecionada: '+station['nome'],flush=True)
        else:
            if hb.custom_mode!=4:raise RuntimeError('Inicie saida com rover parado em HOLD')
        # Check sensor delivery before taking over the vehicle's mode.
        sensor.wait_cloud(poll=lambda:drain(link))
        if tipo=='entrada':check_guided(link)
        # Return to HOLD for preflight; no throttle or yaw until geometry is confirmed.
        parar(link)
        initial=pose_confirmada(sensor,link,station)
        print('Estacao detectada pelo MID360:',initial,flush=True)
        if tipo=='entrada' and alinhamento(initial)[0]<.30:
            if (abs(initial['forward'])>.08 or abs(initial['right'])>.10 or
                    abs(initial['normal_yaw'])>math.radians(8)):
                raise RuntimeError('Rover ja proximo ao interior, fora da tolerancia; sem manobras')
            hb=link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1,timeout=5)
            if hb.base_mode&128:link.disarm()
            print('Rover ja esta no centro; HOLD e desarmado.',flush=True)
            success=True
            return 0
        if tipo=='saida':
            try:validar_inicio(initial)
            except ValueError:
                along,lateral,heading=alinhamento(initial)
                raise ValueError('SAIDA recusada: centro ainda a %.2f m, lateral %.2f m, rumo %.1f graus. '
                                 'A saida exige rover dentro (ate 0.30 m do centro); '
                                 'para aproximar/entrar use entrar_lidar.py.' %
                                 (along,lateral,math.degrees(heading)))
            # Geometry can disappear outside: require sound LiDAR/IMU odometry before starting.
            anchor,yaw0=sensor.lio();time.sleep(.4);anchor2,yawcheck=sensor.lio()
            if np.linalg.norm(anchor2-anchor)>.12 or abs((yawcheck-yaw0+math.pi)%(2*math.pi)-math.pi)>math.radians(5):
                raise RuntimeError('FAST-LIO instavel com rover parado')
            anchor=anchor2;yaw0=yawcheck;previous=anchor.copy()
        for key in ('OA_TYPE','AVOID_ENABLE'):saved[key]=link.param(key)
        (root/'parametros_anteriores.json').write_text(json.dumps(saved,indent=2)+'\n')
        if tipo=='saida':
            changed=True;link.param('OA_TYPE',0);link.param('AVOID_ENABLE',int(saved['AVOID_ENABLE']) & ~2)
        h=link.wait('HEARTBEAT',lambda m:m.get_srcComponent()==1,timeout=5)
        if not h.base_mode&128:
            if tipo=='entrada':raise RuntimeError('Rover desarmado; entrada cancelada')
            link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            link.wait('HEARTBEAT',lambda x:x.get_srcComponent()==1 and x.base_mode&128,timeout=10)
        engaged=True;link.mode('GUIDED');end=time.monotonic()+a.timeout;last_report=0
        controller=VisualAlignment(a.velocidade)
        if tipo=='entrada':
            while time.monotonic()<end and not ros.is_shutdown():
                drain(link);check_guided(link)
                try:pose=sensor.pose(link,station)
                except ValueError as e:setpoint(link,0,0);raise RuntimeError('Pose LiDAR perdida: '+str(e))
                log.write(json.dumps({'time':time.monotonic(),'pose':pose})+'\n');log.flush()
                hud=link.latest.get('VFR_HUD');speed=hud.groundspeed if hud and time.monotonic()-link.last_seen.get('VFR_HUD',0)<2 else None
                if controller.phase=='entrada' and deve_frear(pose,speed):break
                phase,linear,angular,done=controller.step(pose)
                if phase=='entrada' and not changed:
                    setpoint(link,0,0);changed=True
                    link.param('OA_TYPE',0);link.param('AVOID_ENABLE',int(saved['AVOID_ENABLE']) & ~2)
                if done:break
                setpoint(link,linear,angular)
                if time.monotonic()-last_report>2:print('MID360',phase,pose,flush=True);last_report=time.monotonic()
                time.sleep(.1)
            else:raise RuntimeError('Tempo de entrada esgotado')
            setpoint(link,0,0);parar(link);pose=pose_confirmada(sensor,link,station)
            if abs(pose['forward'])>.08 or abs(pose['right'])>.10 or abs(pose['normal_yaw'])>math.radians(8):
                raise RuntimeError('Parado fora da tolerancia; rover permanece armado: '+str(pose))
        else:
            print('Recuo por FAST-LIO: 2 m; eixo obtido nas paredes MID360.',flush=True)
            while time.monotonic()<end and not ros.is_shutdown():
                drain(link);check_guided(link);position,yaw=sensor.lio()
                if np.linalg.norm(position-previous)>.3:raise RuntimeError('Salto na odometria FAST-LIO')
                previous=position.copy()
                back,side,err,angular=recuo_lio(anchor,position,yaw0,yaw)
                if abs(side)>.18 or abs(err)>math.radians(18) or back<-.15:raise RuntimeError('Recuo desalinhado')
                if back>=1.96:break
                linear=-min(a.velocidade,max(.015,(2-back)*.15))
                setpoint(link,linear,angular)
                if time.monotonic()-last_report>2:print('FAST-LIO recuo %.2f/2m lateral %.2f'%(back,side),flush=True);last_report=time.monotonic()
                time.sleep(.1)
            else:raise RuntimeError('Tempo de recuo esgotado')
            setpoint(link,0,0);parar(link);position,yaw=sensor.lio()
            back,side,err,_=recuo_lio(anchor,position,yaw0,yaw)
            if not 1.85<=back<=2.15:raise RuntimeError('Recuo apos parada fora da tolerancia: %.2f'%back)
            if abs(side)>.18 or abs(err)>math.radians(18):raise RuntimeError('Saida desalinhada; giro cancelado')
            att=link.wait('ATTITUDE',timeout=5);turn=Giro180(att.yaw);link.mode('GUIDED');end=time.monotonic()+a.timeout;stable=None
            while time.monotonic()<end and not ros.is_shutdown():
                drain(link);check_guided(link);att=link.latest.get('ATTITUDE');now=time.monotonic()
                if not att or now-link.last_seen.get('ATTITUDE',0)>1:setpoint(link,0,0);continue
                rate,done=turn.step(att.yaw);setpoint(link,0,rate)
                if done and abs(att.yawspeed)<.05:
                    if stable is None:stable=now
                    if now-stable>=1:break
                else:stable=None
                time.sleep(.05)
            else:raise RuntimeError('Tempo de giro esgotado')
            setpoint(link,0,0);parar(link)
            att=link.wait('ATTITUDE',timeout=5);turn.step(att.yaw)
            if abs(math.pi-turn.progresso)>math.radians(6):raise RuntimeError('Giro fora da tolerancia')
        link.disarm();success=True;engaged=False
        print('Docking LiDAR concluido; HOLD e rover desarmado.',flush=True)
    except (Exception,KeyboardInterrupt) as e:
        print('Docking LiDAR interrompido:',e,flush=True)
    finally:
        if link and engaged:
            try:
                h=link.latest.get('HEARTBEAT')
                if h and time.monotonic()-link.last_seen.get('HEARTBEAT',0)<2 and h.custom_mode not in (4,15):overridden=True
                if not overridden:link.mode('HOLD');print('HOLD confirmado',flush=True)
            except Exception as e:print('Nao foi possivel confirmar HOLD:',e,flush=True)
        if link and changed:
            for k,v in saved.items():
                try:link.param(k,v);print('Restaurado %s=%s'%(k,v),flush=True)
                except Exception as e:print('Erro restaurando %s: %s; backup em %s'%(k,e,root),flush=True)
        if sensor:sensor.close()
        if link:link.log.close();link.conn.close()
        log.close()
        ros.signal_shutdown('docking lidar encerrado')
    return 0 if success else 1
