#!/usr/bin/env python3.8
"""SITL: recua 2 m usando o QR e gira 180 graus pelo MAVProxy."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetJointProperties

from aguardar_docking import load_stations
from controle_saida import validar_inicio, recuo, Giro180, alinhamento
from relative_pose import solve

os.environ['MAVLINK20']='1'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'missions'))
from pymavlink import mavutil
from mavproxy_link import Link

MASK=1|2|4|64|128|256|1024


def setpoint(link, speed, yaw):
    link.conn.mav.set_position_target_local_ned_send(0,link.sysid,1,
        mavutil.mavlink.MAV_FRAME_BODY_NED,MASK,0,0,0,speed,0,0,0,0,0,0,yaw)


def aguardar_parada(link, timeout=20):
    start=time.monotonic(); stable=None
    while time.monotonic()-start<timeout:
        link.recv(.1); now=time.monotonic()
        hud=link.latest.get('VFR_HUD'); att=link.latest.get('ATTITUDE')
        if (hud and att and now-link.last_seen.get('VFR_HUD',0)<2 and
                now-link.last_seen.get('ATTITUDE',0)<2 and hud.groundspeed<.05 and abs(att.yawspeed)<.05):
            if stable is None: stable=now
            if now-stable>=1: return
        else: stable=None
    raise RuntimeError('Nao foi possivel confirmar a parada')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mavproxy',default='udpin:127.0.0.1:14551')
    p.add_argument('--sysid',type=int,default=1)
    p.add_argument('--estacoes',default=str(Path(__file__).with_name('estacoes.json')))
    p.add_argument('--estacao',default='container_gravel_plane')
    p.add_argument('--velocidade',type=float,default=.08)
    p.add_argument('--timeout',type=float,default=180)
    a=p.parse_args()
    if not a.mavproxy.startswith('udpin:127.0.0.1:') or not 1<=a.sysid<=255 or not 0<a.velocidade<=.12 or not 0<a.timeout<=600:
        p.error('Use UDP local, sysid valido, velocidade ate 0,12 m/s e timeout ate 600 s')
    matches=[s for s in load_stations(a.estacoes) if s['nome']==a.estacao]
    if len(matches)!=1: p.error('Estacao ausente ou duplicada no cadastro')
    station=matches[0]
    rospy.init_node('saida_visual_docking')
    ns='/rover_argo_N1/Instance1/camera'
    bridge=CvBridge(); detector=cv2.QRCodeDetector(); lock=threading.Lock()
    state={'frame':None,'info':None,'wall':0.,'stamp':None}
    def image_cb(msg):
        try: frame=bridge.imgmsg_to_cv2(msg,'bgr8')
        except Exception as e: rospy.logwarn_throttle(3,'Camera: %s',e);return
        with lock:state.update(frame=frame,wall=time.monotonic(),stamp=msg.header.stamp)
    def info_cb(msg):
        with lock:state['info']=msg
    subs=[rospy.Subscriber(ns+'/image_raw',Image,image_cb,queue_size=1,buff_size=32*1024*1024),
          rospy.Subscriber(ns+'/camera_info',CameraInfo,info_cb,queue_size=1)]
    pan=rospy.Publisher(ns+'/pan_cmd',Float64,queue_size=1)
    tilt=rospy.Publisher(ns+'/tilt_cmd',Float64,queue_size=1)
    def joint(name):
        v=rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)('rover_argo_N1::'+name)
        if not v.success or not v.position:raise RuntimeError('Sem posicao das juntas da camera')
        return v.position[0]
    def pose():
        with lock:frame,info,stamp,wall=state['frame'],state['info'],state['stamp'],state['wall']
        if frame is None or info is None or stamp is None or time.monotonic()-wall>.7:
            raise RuntimeError('Camera ou CameraInfo indisponivel/desatualizada')
        age=(rospy.Time.now()-stamp).to_sec()
        if age<-.05 or age>.5:raise RuntimeError('Timestamp da camera desatualizado')
        actual_pan=joint('pt_support_s23_pan_joint'); actual_tilt=joint('pt_support_s23_tilt_joint')
        if abs(actual_pan)>.06 or not -.65<=actual_tilt<=.05:raise RuntimeError('Camera fora da calibracao')
        text,corners,_=detector.detectAndDecode(frame)
        if text!=station['qr'] or corners is None:raise RuntimeError('QR cadastrado nao esta visivel')
        return solve(corners,info.K,info.D,tilt_rad=actual_tilt,
                     wall_offset=station['distancia_alvo_parede_m'])
    root=Path(__file__).resolve().parents[1]/'run/docking'/('saida_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    root.mkdir(parents=True,exist_ok=True)
    try:link=Link(a.mavproxy,a.sysid,root,timeout=5,source_system=245)
    except OSError as e:
        print('Porta MAVProxy indisponivel: %s. Encerre o monitor de entrada que usa a mesma porta.'%e,flush=True)
        return 1
    saved={};changed=False;engaged=False;complete=False;overridden=False
    signal.signal(signal.SIGTERM,lambda sig,frame:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        hb=link.wait('HEARTBEAT',lambda x:x.get_srcComponent()==1,timeout=15)
        if hb.type!=mavutil.mavlink.MAV_TYPE_GROUND_ROVER or mavutil.mode_string_v10(hb)!='HOLD':
            raise RuntimeError('Inicie com Rover em HOLD e parado')
        link.conn.mav.command_long_send(a.sysid,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE,500000,0,0,0,0,0)
        link.wait(['SIMSTATE','SIM_STATE'],timeout=10)
        aguardar_parada(link)
        rospy.wait_for_service('/gazebo/get_joint_properties',timeout=5)
        until=time.monotonic()+8
        while time.monotonic()<until:
            pan.publish(0.);tilt.publish(-.5);link.recv(.05)
            if abs(joint('pt_support_s23_pan_joint'))<.04 and abs(joint('pt_support_s23_tilt_joint')+.5)<.06:break
        else:raise RuntimeError('Camera nao atingiu a orientacao inicial')
        camera_ready=time.monotonic()
        until=camera_ready+8
        while True:
            try:
                with lock:image_wall=state['wall']
                if image_wall<=camera_ready:raise RuntimeError('Aguardando imagem apos mover camera')
                initial_pose=pose();break
            except (RuntimeError, ValueError, cv2.error):
                if time.monotonic()>until:raise RuntimeError('Nao foi possivel confirmar o QR da estacao')
                link.recv(.05)
        inicio=validar_inicio(initial_pose)
        print('QR e pose dentro da estacao confirmados:',initial_pose,flush=True)
        for name in ('OA_TYPE','AVOID_ENABLE'):saved[name]=link.param(name)
        (root/'parametros_anteriores.json').write_text(json.dumps(saved,indent=2)+'\n')
        changed=True
        link.param('OA_TYPE',0);link.param('AVOID_ENABLE',int(saved['AVOID_ENABLE']) & ~2)
        if not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1 and h.base_mode&128,timeout=15)
        engaged=True;link.mode('GUIDED')
        print('GUIDED: recuando 2 m alinhado ao QR.',flush=True)
        until=time.monotonic()+a.timeout;last_report=0
        while time.monotonic()<until and not rospy.is_shutdown():
            end=time.monotonic()+.05
            while time.monotonic()<end:link.recv(.002)
            h=link.latest.get('HEARTBEAT')
            if h and time.monotonic()-link.last_seen.get('HEARTBEAT',0)<2 and h.custom_mode!=15:
                overridden=True
            if not h or time.monotonic()-link.last_seen.get('HEARTBEAT',0)>2 or h.custom_mode!=15 or not h.base_mode&128:
                raise RuntimeError('GUIDED, armamento ou telemetria interrompidos')
            try:current=pose()
            except (RuntimeError,ValueError,cv2.error) as e:
                setpoint(link,0,0)
                raise RuntimeError('Recuo interrompido: '+str(e))
            speed,rate,distance,done=recuo(current,inicio,a.velocidade)
            desired_tilt=max(-.55,min(-.10,-math.atan2(.22,max(.2,current['marker_forward']-.07))))
            tilt.publish(desired_tilt)
            if done:break
            setpoint(link,speed,rate)
            if time.monotonic()-last_report>=2:
                print('Recuo visual: %.2f / 2,00 m'%distance,flush=True);last_report=time.monotonic()
            time.sleep(.1)
        else:raise RuntimeError('Tempo de recuo esgotado')
        setpoint(link,0,0);link.mode('HOLD');aguardar_parada(link)
        stopped_pose=pose();distance=alinhamento(stopped_pose)[0]-inicio
        if not 1.85<=distance<=2.15:
            raise RuntimeError('Distancia de recuo apos parada fora da tolerancia: %.2f m'%distance)
        if abs(alinhamento(stopped_pose)[1])>.18 or abs(alinhamento(stopped_pose)[2])>math.radians(18):
            raise RuntimeError('Rover saiu desalinhado; giro cancelado')
        print('Recuo confirmado: %.2f m; iniciando giro de 180 graus.'%distance,flush=True)
        att=link.wait('ATTITUDE',timeout=5);turn=Giro180(att.yaw)
        link.mode('GUIDED');until=time.monotonic()+a.timeout; stable=None
        while time.monotonic()<until and not rospy.is_shutdown():
            msg=link.recv(.1);now=time.monotonic()
            h=link.latest.get('HEARTBEAT');att=link.latest.get('ATTITUDE')
            if h and now-link.last_seen.get('HEARTBEAT',0)<2 and h.custom_mode!=15:
                overridden=True
            if not h or now-link.last_seen.get('HEARTBEAT',0)>2 or h.custom_mode!=15 or not h.base_mode&128:
                raise RuntimeError('GUIDED ou telemetria interrompidos no giro')
            if not att or now-link.last_seen.get('ATTITUDE',0)>1:
                setpoint(link,0,0);continue
            yawrate,done=turn.step(att.yaw)
            setpoint(link,0,yawrate)
            if done and abs(att.yawspeed)<.05:
                if stable is None:stable=now
                if now-stable>=1:break
            else:stable=None
        else:raise RuntimeError('Tempo do giro esgotado')
        setpoint(link,0,0);link.mode('HOLD');aguardar_parada(link)
        att=link.wait('ATTITUDE',timeout=5);turn.step(att.yaw)
        if abs(math.pi-turn.progresso)>math.radians(6):raise RuntimeError('Giro final fora da tolerancia')
        link.disarm();complete=True;engaged=False
        print('Saida concluida: HOLD, giro %.1f graus e rover desarmado.'%math.degrees(turn.progresso),flush=True)
    except (Exception,KeyboardInterrupt) as e:
        print('Saida interrompida:',e,flush=True)
    finally:
        if engaged and not overridden:
            try:
                link.mode('HOLD');print('HOLD confirmado',flush=True)
            except Exception as e: print('Nao foi possivel confirmar HOLD:',e,flush=True)
        if changed:
            for name,value in saved.items():
                try:link.param(name,value);print('Restaurado %s=%s'%(name,value),flush=True)
                except Exception as e:print('Falha restaurando %s: %s; consulte %s'%(name,e,root/'parametros_anteriores.json'),flush=True)
        for sub in subs:sub.unregister()
        link.log.close();link.conn.close()
    return 0 if complete else 1

if __name__=='__main__':sys.exit(main())
