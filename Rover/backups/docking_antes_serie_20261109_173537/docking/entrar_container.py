#!/usr/bin/env python3
"""Docking visual experimental para a simulacao: QR -> MAVProxy -> GUIDED."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
from common.qr_detector import QRDetector
from common.docking_camera import DockingCamera, add_camera_arguments, diagnose
from std_msgs.msg import Float64
from relative_pose import solve
from pose_tracker import PoseTracker, PosePending
from visual_alignment import VisualAlignment, deve_frear, distancia_frenagem
from busca_qr import yaw_rate_para_estacao, direcao_estacao, AproximacaoBusca

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'missions'))
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from mavproxy_link import Link

EXPECTED='ROVER_DOCKING_STATION_01'
TILT=-.30

class QRPerdido(RuntimeError):
    def __init__(self, message, candidate_bearing=None):
        super().__init__(message)
        self.candidate_bearing=candidate_bearing


def taxa_busca(exc, link, now, latitude, longitude):
    """Giro e permitido apenas se a camera atual nao reconheceu o QR."""
    if not isinstance(exc, QRPerdido):
        return None
    rate=yaw_rate_para_estacao(link.latest.get('GLOBAL_POSITION_INT'),
        link.latest.get('GPS_RAW_INT'),now,
        link.last_seen.get('GLOBAL_POSITION_INT',0),link.last_seen.get('GPS_RAW_INT',0),
        latitude,longitude)
    # GPS points at the registered center; the visible marker may still be a
    # few degrees off axis. Center it before allowing the short forward step.
    bearing=exc.candidate_bearing
    if rate==0. and bearing is not None and math.isfinite(bearing) and abs(bearing)>=math.radians(5):
        return min(.15,max(-.15,.6*bearing))
    return rate


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
    p.add_argument('--offset-parede',type=float,help=argparse.SUPPRESS)
    p.add_argument('--latitude-estacao',type=float,help=argparse.SUPPRESS)
    p.add_argument('--longitude-estacao',type=float,help=argparse.SUPPRESS)
    p.add_argument('--uma-vez',action='store_true',help='Executa uma entrada imediata, sem aguardar ativacao')
    p.add_argument('--apenas-guided',action='store_true',help=argparse.SUPPRESS)
    add_camera_arguments(p)
    args=p.parse_args()
    if args.diagnostico:
        return diagnose(args.namespace)
    if not args.mavproxy.startswith('udpin:127.0.0.1:') or not 1 <= args.sysid <= 255 or not 0 < args.velocidade <= .12 or not 0 < args.timeout <= 600:
        p.error('Somente UDP local, velocidade 0..0.12 m/s e timeout ate 600 s')
    if not args.uma_vez:
        from aguardar_docking import monitorar
        return monitorar(args)
    if (args.latitude_estacao is None) != (args.longitude_estacao is None):
        p.error('Informe latitude e longitude da estacao juntas')
    if args.latitude_estacao is None or args.offset_parede is None:
        from aguardar_docking import load_stations
        matching=[s for s in load_stations(args.estacoes) if s['qr']==args.qr]
        if len(matching)!=1:
            p.error('Informe uma estacao com QR unico no cadastro')
        if args.latitude_estacao is None:
            args.latitude_estacao=matching[0]['latitude']
            args.longitude_estacao=matching[0]['longitude']
        if args.offset_parede is None:
            args.offset_parede=matching[0]['distancia_alvo_parede_m']
    if not math.isfinite(args.offset_parede) or not 0<args.offset_parede<5:
        p.error('Distancia do alvo a parede invalida')
    if not (math.isfinite(args.latitude_estacao) and -90<=args.latitude_estacao<=90 and
            math.isfinite(args.longitude_estacao) and -180<=args.longitude_estacao<=180):
        p.error('Coordenadas da estacao invalidas')
    ros.init_node('dock_visual_guided')
    camera=DockingCamera(args.namespace)
    joints=ros.JointFeedback(camera.namespace+'/joint_states')
    namespace=camera.namespace+'/camera'
    detector=QRDetector()
    pan=ros.Publisher(namespace+'/pan_cmd',Float64,queue_size=1)
    tilt=ros.Publisher(namespace+'/tilt_cmd',Float64,queue_size=1)
    root=Path(__file__).resolve().parents[1]/'run/docking'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True,exist_ok=True)
    link=Link(args.mavproxy,args.sysid,root,timeout=5,source_system=244)
    saved={}; changed=False; engaged=False; complete=False; last_pose=None
    pose_log=(root/'pose_visual.jsonl').open('w')
    control_log=(root/'controle.jsonl').open('w')
    tracker=PoseTracker(args.offset_parede)
    def event(**data):
        control_log.write(json.dumps(dict(monotonic=time.monotonic(),**data))+'\n')
        control_log.flush()
    signal.signal(signal.SIGTERM,lambda sig,frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    def frame_pose():
        received_at=time.monotonic()
        frame,info,stamp=camera.snapshot()
        actual_pan=joint('pt_support_s23_pan_joint')
        actual_tilt=joint('pt_support_s23_tilt_joint')
        if abs(actual_pan)>.06 or not -.65 <= actual_tilt <= .05:
            raise RuntimeError('Orientacao da camera fora da calibracao')
        text,corners,_=detector.detectAndDecode(frame)
        if time.monotonic()-received_at>.7 or not -.05<=ros.age(stamp)<=.5:
            raise RuntimeError('Imagem expirou durante a leitura do QR')
        if text!=args.qr or corners is None:
            detail=('Codigo lido nao cadastrado: '+text) if text else detector.status
            bearing=None
            if not text and corners is not None and np.isfinite(corners).all():
                bearing=math.atan2(float(corners.reshape(4,2)[:,0].mean())-info.k[2],info.k[0])
            raise QRPerdido(detail,bearing)
        raw=solve(corners,info.k,info.d,tilt_rad=actual_tilt,wall_offset=args.offset_parede)
        att=link.latest.get('ATTITUDE')
        if att is None or time.monotonic()-link.last_seen.get('ATTITUDE',0)>.7:
            raise PosePending('Atitude recente indisponivel para validar pose')
        # Account for the image age while turning; camera and MAVLink use the
        # same simulated clock rate. Bound latency before applying the estimate.
        yaw=att.yaw-att.yawspeed*max(0.,ros.age(stamp))
        event(kind='measurement',raw=raw,stamp=ros.seconds(stamp),yaw=yaw)
        return tracker.update(raw,ros.seconds(stamp),yaw)
    def joint(name):
        return joints.position(name)
    def final_pose():
        deadline=time.monotonic()+10
        confirmed=[]
        while time.monotonic()<deadline:
            link.recv(.05)
            try:p=frame_pose()
            except (PosePending,QRPerdido):continue
            if abs(p['forward'])>.06 or abs(p['right'])>.08 or abs(p['normal_yaw'])>math.radians(6):
                confirmed=[]
                continue
            confirmed.append(p)
            if len(confirmed)>=3 and confirmed[-1]['stamp']-confirmed[0]['stamp']>=.4:
                return p
        raise RuntimeError('Pose final nao confirmada em imagens recentes; permanece em HOLD')
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
        joints.wait(timeout=5)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            pan.publish(0.);tilt.publish(TILT);link.recv(.05)
            if abs(joint('pt_support_s23_pan_joint'))<.04 and abs(joint('pt_support_s23_tilt_joint')-TILT)<.06:break
            time.sleep(.15)
        else: raise RuntimeError('Pan/tilt nao atingiu a posicao calibrada')
        # Discard frames captured while the camera was still moving.
        aligned_at=time.monotonic()
        while time.monotonic()-aligned_at < 3:
            with camera.lock: image_wall=camera.wall
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
        search_approach=AproximacaoBusca()
        end=time.monotonic()+args.timeout;last_report=0; stable=None; qr_lost_at=None
        while not ros.is_shutdown():
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
                if not isinstance(exc,PosePending) and controller.phase=='entrada' and last_pose is not None and last_pose['forward']<=.30:
                    raise RuntimeError('QR perdido perto do alvo; HOLD sem novas manobras')
                # A short advance may recover a small QR; never use an undecoded
                # candidate as the final docking pose or advance inside the box.
                now=time.monotonic()
                rate=taxa_busca(exc,link,now,args.latitude_estacao,args.longitude_estacao)
                position=link.latest.get('GLOBAL_POSITION_INT')
                direction=direcao_estacao(position,link.latest.get('GPS_RAW_INT'),now,
                    link.last_seen.get('GLOBAL_POSITION_INT',0),link.last_seen.get('GPS_RAW_INT',0),
                    args.latitude_estacao,args.longitude_estacao)
                hud=link.latest.get('VFR_HUD');att=link.latest.get('ATTITUDE')
                motion_ok=(hud is not None and att is not None
                    and now-link.last_seen.get('VFR_HUD',0)<1
                    and now-link.last_seen.get('ATTITUDE',0)<1
                    and 0<=hud.groundspeed<.10 and abs(att.yawspeed)<.05)
                speed,search_reason=search_approach.step(now,direction,position,
                    exc.candidate_bearing if isinstance(exc,QRPerdido) else None,motion_ok,
                    allowed=controller.phase!='entrada' and not changed)
                if isinstance(exc,PosePending):
                    speed=0.;rate=0.;search_reason='parado para revalidar pose'
                    if controller.phase=='alinhar':controller.stable_since=None
                    if qr_lost_at is not None and now-qr_lost_at>10:
                        raise RuntimeError('Pose visual nao estabilizou em 10 s: '+str(exc))
                link.conn.mav.set_position_target_local_ned_send(0,args.sysid,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
                    1|2|4|64|128|256|1024,0,0,0,speed,0,0,0,0,0,0,rate or 0.)
                event(kind='waiting',phase=controller.phase,reason=str(exc),decision=search_reason,speed=speed,yawrate=rate or 0.)
                stable=None
                if qr_lost_at is None: qr_lost_at=now
                if now-last_report>5:
                    error_text=('erro=%.1f graus; '%math.degrees(direction[0])) if direction else ''
                    print('Aguardando QR: %s; %s%s; giro=%.3f rad/s'%(exc,error_text,
                          search_reason,rate or 0.),flush=True)
                    last_report=now
                # O prazo da entrada nao corre enquanto procuramos o QR.
                end=now+args.timeout
                time.sleep(.10)
                continue
            if qr_lost_at is not None:
                search_approach.step(time.monotonic(),None,None,None,False,allowed=False)
                print('Pose QR validada; retomando alinhamento visual.',flush=True)
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
                concluir_desarmando(link,final_pose)
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
            phase,speed,yawrate,at_target=controller.step(pose,now=pose['stamp'])
            lateral=pose['forward']*math.sin(pose['normal_yaw'])+pose['right']*math.cos(pose['normal_yaw'])
            event(kind='command',phase=phase,speed=speed,yawrate=yawrate,lateral=lateral,
                  heading_deg=math.degrees(pose['normal_yaw']),pose=pose)
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
            link.conn.mav.landing_target_send(int(ros.nanoseconds(ros.now())//1000),0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,0,0,distance,0,0,
                float(target[0]),float(target[1]),float(target[2]),[1,0,0,0],
                mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,1)
            if at_target:
                print('Alvo visual atingido; solicitando HOLD.',flush=True)
                concluir_desarmando(link,final_pose)
                complete=True
                engaged=False
                break
            # BODY_NED, velocity and yaw rate; AP controls throttle and steering.
            mask=1|2|4|64|128|256|1024
            link.conn.mav.set_position_target_local_ned_send(0,args.sysid,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
                mask,0,0,0,speed,0,0,0,0,0,0,yawrate)
            if time.monotonic()-last_report>1:
                print('%s: frente=%.2fm direita=%.2fm lateral_eixo=%.3fm angulo=%.1fg erro_reprojecao=%.1fpx vel=%.2f'%
                    (phase,pose['forward'],pose['right'],lateral,math.degrees(pose['normal_yaw']),pose['reprojection_px'],speed),flush=True)
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
        camera.close();joints.close()
        pose_log.close()
        control_log.close()
        link.log.close();link.conn.close()
    return 0 if complete else 1

if __name__=='__main__':
    try:
        sys.exit(main())
    finally:
        ros.signal_shutdown('docking encerrado')
