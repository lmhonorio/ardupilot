#!/usr/bin/env python3.8
"""Docking visual experimental para a simulacao: QR -> MAVProxy -> GUIDED."""
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
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetJointProperties
from relative_pose import solve
from visual_alignment import VisualAlignment, deve_frear, distancia_frenagem
from busca_qr import yaw_rate_para_estacao

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'missions'))
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from mavproxy_link import Link

EXPECTED='ROVER_DOCKING_STATION_01'
TILT=-.30

class QRPerdido(RuntimeError):
    pass


def taxa_busca(exc, link, now, latitude, longitude):
    """Giro e permitido apenas se a camera atual nao reconheceu o QR."""
    if not isinstance(exc, QRPerdido):
        return None
    return yaw_rate_para_estacao(link.latest.get('GLOBAL_POSITION_INT'),
        link.latest.get('GPS_RAW_INT'),now,
        link.last_seen.get('GLOBAL_POSITION_INT',0),link.last_seen.get('GPS_RAW_INT',0),
        latitude,longitude)


def clamp(x,lo,hi): return min(max(x,lo),hi)


def concluir_desarmando(link, verificar_pose=None):
    """Confirma HOLD e parada antes de solicitar e confirmar o desarmamento."""
    link.mode('HOLD')
    started=time.monotonic(); stable=None
    while time.monotonic()-started<15:
        link.recv(.2)
        now=time.monotonic()
        v=link.latest.get('VFR_HUD'); attitude=link.latest.get('ATTITUDE')
        fresh=(link.last_seen.get('VFR_HUD',0)>=started and link.last_seen.get('ATTITUDE',0)>=started
               and now-link.last_seen.get('VFR_HUD',0)<2 and now-link.last_seen.get('ATTITUDE',0)<2)
        hb=link.latest.get('HEARTBEAT')
        if hb is not None and (now-link.last_seen.get('HEARTBEAT',0)>3 or
                               hb.custom_mode!=link.conn.mode_mapping()['HOLD'] or not hb.base_mode&128):
            raise RuntimeError('Modo ou armamento mudou durante a confirmacao da parada')
        stopped=(fresh and v.groundspeed<.05 and abs(attitude.yawspeed)<.05)
        if stopped:
            if stable is None: stable=now
            if now-stable>=1.0: break
        else: stable=None
    else:
        raise RuntimeError('HOLD confirmado, mas a parada nao foi confirmada; rover permanece armado')
    if verificar_pose is not None:
        pose=verificar_pose()
        if (abs(pose['forward'])>.06 or abs(pose['right'])>.08 or
                abs(pose['normal_yaw'])>math.radians(6)):
            raise RuntimeError('Pose visual saiu da tolerancia apos parar; rover permanece armado: '+str(pose))
        print('Pose final: frente=%.3f m lateral=%.3f m angulo=%.1f graus'%
              (pose['forward'],pose['right'],math.degrees(pose['normal_yaw'])),flush=True)
    link.disarm()
    print('Rover parado e desarmado confirmado.',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mavproxy',default='udpin:127.0.0.1:14551')
    p.add_argument('--sysid',type=int,default=1)
    p.add_argument('--velocidade',type=float,default=.08)
    p.add_argument('--timeout',type=float,default=300.)
    p.add_argument('--estacoes',default=str(Path(__file__).with_name('estacoes.json')),
                   help='Cadastro JSON de estacoes')
    p.add_argument('--qr',default=EXPECTED,help=argparse.SUPPRESS)
    p.add_argument('--offset-parede',type=float,default=.5335,help=argparse.SUPPRESS)
    p.add_argument('--latitude-estacao',type=float,help=argparse.SUPPRESS)
    p.add_argument('--longitude-estacao',type=float,help=argparse.SUPPRESS)
    p.add_argument('--uma-vez',action='store_true',help='Executa uma entrada imediata, sem aguardar ativacao')
    p.add_argument('--apenas-guided',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    if not args.mavproxy.startswith('udpin:127.0.0.1:') or not 1 <= args.sysid <= 255 or not 0 < args.velocidade <= .12 or not 0 < args.timeout <= 600:
        p.error('Somente UDP local, velocidade 0..0.12 m/s e timeout ate 600 s')
    if not args.uma_vez:
        from aguardar_docking import monitorar
        return monitorar(args)
    if (args.latitude_estacao is None) != (args.longitude_estacao is None):
        p.error('Informe latitude e longitude da estacao juntas')
    if args.latitude_estacao is None or args.longitude_estacao is None:
        from aguardar_docking import load_stations
        matching=[s for s in load_stations(args.estacoes) if s['qr']==args.qr]
        if len(matching)!=1:
            p.error('Informe uma estacao com QR unico no cadastro')
        args.latitude_estacao=matching[0]['latitude']
        args.longitude_estacao=matching[0]['longitude']
    if not (math.isfinite(args.latitude_estacao) and -90<=args.latitude_estacao<=90 and
            math.isfinite(args.longitude_estacao) and -180<=args.longitude_estacao<=180):
        p.error('Coordenadas da estacao invalidas')
    rospy.init_node('dock_visual_guided')
    namespace='/rover_argo_N1/Instance1/camera'
    image_bridge=CvBridge(); detector=cv2.QRCodeDetector(); lock=threading.Lock()
    state={'frame':None,'info':None,'wall':0.,'stamp':None}
    def image_cb(msg):
        try: frame=image_bridge.imgmsg_to_cv2(msg,'bgr8')
        except Exception as exc: rospy.logwarn_throttle(3,'Erro camera: %s',exc);return
        with lock: state.update(frame=frame,wall=time.monotonic(),stamp=msg.header.stamp)
    def info_cb(msg):
        with lock: state['info']=msg
    subs=[rospy.Subscriber(namespace+'/image_raw',Image,image_cb,queue_size=1,buff_size=32*1024*1024),
          rospy.Subscriber(namespace+'/camera_info',CameraInfo,info_cb,queue_size=1)]
    pan=rospy.Publisher(namespace+'/pan_cmd',Float64,queue_size=1)
    tilt=rospy.Publisher(namespace+'/tilt_cmd',Float64,queue_size=1)
    root=Path(__file__).resolve().parents[1]/'run/docking'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True,exist_ok=True)
    link=Link(args.mavproxy,args.sysid,root,timeout=5,source_system=244)
    saved={}; changed=False; engaged=False; complete=False; last_pose=None
    pose_log=(root/'pose_visual.jsonl').open('w')
    signal.signal(signal.SIGTERM,lambda sig,frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    def frame_pose():
        with lock: frame,info,stamp,wall=state['frame'],state['info'],state['stamp'],state['wall']
        if frame is None or info is None or stamp is None or time.monotonic()-wall>.7:
            raise RuntimeError('Imagem ou CameraInfo indisponivel/desatualizada')
        age=(rospy.Time.now()-stamp).to_sec()
        if age<-.05 or age>.5:raise RuntimeError('Timestamp da imagem invalido/desatualizado')
        actual_pan=joint('pt_support_s23_pan_joint')
        actual_tilt=joint('pt_support_s23_tilt_joint')
        if abs(actual_pan)>.06 or not -.65 <= actual_tilt <= .05:
            raise RuntimeError('Orientacao da camera fora da calibracao')
        text,corners,_=detector.detectAndDecode(frame)
        if text!=args.qr or corners is None: raise QRPerdido('QR da estacao perdido')
        return solve(corners,info.K,info.D,tilt_rad=actual_tilt,wall_offset=args.offset_parede)
    def joint(name):
        result=rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)('rover_argo_N1::'+name)
        if not result.success or not result.position: raise RuntimeError('Sem feedback do pan/tilt Gazebo')
        return result.position[0]
    try:
        hb=link.wait('HEARTBEAT',lambda x:x.get_srcComponent()==1,timeout=15)
        if hb.type!=mavutil.mavlink.MAV_TYPE_GROUND_ROVER or not (hb.base_mode&128):
            raise RuntimeError('Rover deve estar armado e parado antes do docking')
        # Only use the companion in a known SITL, not a physical rover.
        link.conn.mav.command_long_send(args.sysid,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE,500000,0,0,0,0,0)
        link.wait(['SIMSTATE','SIM_STATE'],timeout=8)
        v=link.wait('VFR_HUD',timeout=8)
        if v.groundspeed>.1:raise RuntimeError('Aguarde o rover parar antes do docking')
        if args.apenas_guided:
            hb=link.wait('HEARTBEAT',lambda x:x.get_srcComponent()==1,timeout=5)
        modo=mavutil.mode_string_v10(hb)
        if args.apenas_guided and modo != 'GUIDED':
            raise RuntimeError('A ativacao foi cancelada: rover saiu de GUIDED')
        if modo not in ('HOLD','AUTO','GUIDED'):
            raise RuntimeError('Inicie na frente do QR, com rover em HOLD, AUTO ou GUIDED')
        link.mode('HOLD')
        rospy.wait_for_service('/gazebo/get_joint_properties',timeout=5)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            pan.publish(0.);tilt.publish(TILT);link.recv(.05)
            if abs(joint('pt_support_s23_pan_joint'))<.04 and abs(joint('pt_support_s23_tilt_joint')-TILT)<.06:break
            time.sleep(.15)
        else: raise RuntimeError('Pan/tilt nao atingiu a posicao calibrada')
        # Discard frames captured while the camera was still moving.
        aligned_at=time.monotonic()
        while time.monotonic()-aligned_at < 3:
            with lock: image_wall=state['wall']
            if image_wall > aligned_at: break
            link.recv(.05)
        else: raise RuntimeError('Sem imagem nova apos posicionar a camera')
        for name in ('OA_TYPE','AVOID_ENABLE'):
            saved[name]=link.param(name)
        # Persist a recovery record before any parameter is changed.
        (root/'parametros_anteriores.txt').write_text('\n'.join('%s %s'%(k,v) for k,v in saved.items())+'\n')
        # A mode change can succeed even if its acknowledgement is delayed.
        # From this point onward always attempt HOLD in the cleanup path.
        engaged=True
        link.mode('GUIDED')
        print('GUIDED ativo; aproximando do eixo visual do container.',flush=True)
        controller=VisualAlignment(args.velocidade)
        end=time.monotonic()+args.timeout;last_report=0; stable=None; qr_lost_at=None
        while not rospy.is_shutdown():
            # Consume pending traffic through the latest autopilot heartbeat;
            # otherwise a busy MAVProxy stream can make a live link look stale.
            drain_until=time.monotonic()+.05
            while time.monotonic()<drain_until:
                link.recv(.002)
            heartbeat=link.latest.get('HEARTBEAT')
            if heartbeat is None or time.monotonic()-link.last_seen.get('HEARTBEAT',0)>2:
                raise RuntimeError('Telemetria perdida')
            if heartbeat.custom_mode != link.conn.mode_mapping()['GUIDED'] or not heartbeat.base_mode&128:
                raise RuntimeError('GUIDED/armamento interrompido')
            try:
                pose=frame_pose()
            except (RuntimeError, ValueError, cv2.error) as exc:
                if controller.phase=='entrada' and last_pose is not None and last_pose['forward']<=.30:
                    raise RuntimeError('QR perdido perto do alvo; HOLD sem novas manobras')
                # Nunca avance sem o QR. Gire somente com imagem e GPS atuais.
                now=time.monotonic()
                rate=taxa_busca(exc,link,now,args.latitude_estacao,args.longitude_estacao)
                link.conn.mav.set_position_target_local_ned_send(0,args.sysid,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
                    1|2|4|64|128|256|1024,0,0,0,0,0,0,0,0,0,0,rate or 0.)
                stable=None
                if qr_lost_at is None: qr_lost_at=now
                if now-last_report>5:
                    print('Aguardando QR: %s; %s'%(exc,
                          'girando para estacao (%.3f rad/s)'%rate if rate is not None else 'sem giro: dados insuficientes'),flush=True)
                    last_report=now
                # O prazo da entrada nao corre enquanto procuramos o QR.
                end=now+args.timeout
                time.sleep(.10)
                continue
            if qr_lost_at is not None:
                print('QR reencontrado; retomando alinhamento visual.',flush=True)
                qr_lost_at=None
            if time.monotonic()>=end: raise RuntimeError('Tempo de docking esgotado')
            last_pose=pose
            hud=link.latest.get('VFR_HUD')
            speed_measured=(hud.groundspeed if hud is not None and
                            time.monotonic()-link.last_seen.get('VFR_HUD',0)<2 else None)
            pose_log.write(json.dumps({'monotonic':time.monotonic(),'phase':controller.phase,
                                       'pose':pose,'groundspeed':speed_measured})+'\n')
            pose_log.flush()
            if controller.phase=='entrada' and deve_frear(pose,speed_measured):
                print('Regiao final: HOLD imediato (frente=%.3f m margem=%.3f m).'%
                      (pose['forward'],distancia_frenagem(speed_measured)),flush=True)
                concluir_desarmando(link,frame_pose)
                complete=True
                engaged=False
                break
            if pose['forward'] < -.015:
                raise RuntimeError('Rover ultrapassou o alvo estimado')
            # Keep the elevated marker in view as the camera approaches it.
            # The estimated camera height in this model is approximately 0.33 m;
            # the marker is at 0.55 m above the ground.
            desired_tilt=clamp(-math.atan2(.22,max(.2,pose['marker_forward']-.07)),-.55,-.10)
            tilt.publish(desired_tilt)
            phase,speed,yawrate,at_target=controller.step(pose)
            if phase=='entrada' and not changed:
                # Pare antes de desabilitar o desvio para a passagem estreita.
                link.conn.mav.set_position_target_local_ned_send(0,args.sysid,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
                    1|2|4|64|128|256|1024,0,0,0,0,0,0,0,0,0,0,0)
                changed=True
                link.param('OA_TYPE',0)
                link.param('AVOID_ENABLE',int(saved['AVOID_ENABLE']) & ~2)
                print('Alinhado para entrada; desvio desabilitado.',flush=True)
            target=np.array([pose['forward'],pose['right'],pose['down']])
            distance=float(np.linalg.norm(target))
            # LANDING_TARGET describes the desired dock point in BODY_FRD.
            link.conn.mav.landing_target_send(int(rospy.Time.now().to_nsec()//1000),0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,0,0,distance,0,0,
                float(target[0]),float(target[1]),float(target[2]),[1,0,0,0],
                mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,1)
            if at_target:
                print('Alvo visual atingido; solicitando HOLD.',flush=True)
                concluir_desarmando(link,frame_pose)
                complete=True
                engaged=False
                break
            # BODY_NED, velocity and yaw rate; AP controls throttle and steering.
            mask=1|2|4|64|128|256|1024
            link.conn.mav.set_position_target_local_ned_send(0,args.sysid,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
                mask,0,0,0,speed,0,0,0,0,0,0,yawrate)
            if time.monotonic()-last_report>1:
                print('%s: frente=%.2fm direita=%.2fm erro_reprojecao=%.1fpx vel=%.2f'%
                    (phase,pose['forward'],pose['right'],pose['reprojection_px'],speed),flush=True)
                last_report=time.monotonic()
            time.sleep(.10)
        if not complete: raise RuntimeError('Docking encerrado antes da confirmacao da parada')
    except (Exception,KeyboardInterrupt) as exc:
        print('Docking interrompido:',exc,flush=True)
    finally:
        if engaged:
            try: link.mode('HOLD');print('HOLD confirmado',flush=True)
            except Exception as exc: print('ERRO: nao foi possivel confirmar HOLD:',exc,flush=True)
        if changed:
            for name,value in saved.items():
                try: link.param(name,value);print('Restaurado %s=%s'%(name,value),flush=True)
                except Exception as exc:print('ERRO restaurando %s: %s; consulte %s'%(name,exc,root/'parametros_anteriores.txt'),flush=True)
        for sub in subs:sub.unregister()
        pose_log.close()
        link.log.close();link.conn.close()
    return 0 if complete else 1

if __name__=='__main__':sys.exit(main())
