#!/usr/bin/env python3
"""Explicit integration trial. --execute is required; saves mission backup.

Manager and Gazebo/SITL must already be running. Motion from mission manager,
independent Gazebo truth only measures results and never enters the estimator.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
from common.docking_stations import load_stations
from missions.mavproxy_link import Link
from mission_manager.mission import fingerprint
from std_msgs.msg import String
from std_srvs.srv import Trigger
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--cycles',type=int,default=2)
    p.add_argument('--dock-first',action='store_true',help='Primeira rodada pede entrada direta; seguintes executam missao')
    p.add_argument('--battery-rejection',action='store_true',help='Exige manager fora da estacao em ESPERA com SOC simulado abaixo do minimo; verifica recusa sem movimento')
    a=p.parse_args()
    if not a.execute:p.error('Use --execute para executar movimentos no SITL')
    ros.init_node('mm_validation')
    root=Path(__file__).resolve().parents[1]/'run/mission_manager_validation'/('series_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    root.mkdir(parents=True);print('RESULTS',root,flush=True)
    history=[];state={};truth={};results=[]
    def receive(msg):
        value=json.loads(msg.data);state.clear();state.update(value)
        history.append(dict(wall=time.monotonic(),**value))
    sub=ros.Subscriber('/mission_manager/status',String,receive,queue_size=10)
    node=Node()
    def ground(msg):
        for m in msg.pose:
            if m.name=='rover_argo_N1':
                q=m.orientation;truth.update(x=m.position.x,y=m.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),wall=time.monotonic())
    node.subscribe(Pose_V,'/world/gravel_plane/pose/info',ground)
    def until(predicate, seconds=45):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if state.get('state')=='FALHA' and not state.get('busy'):raise RuntimeError('Mission Manager em FALHA: '+json.dumps(state))
            if predicate():return
            time.sleep(.1)
        raise TimeoutError('Estado esperado nao chegou: '+json.dumps(state))
    def service(name, expected=True):
        c=ros.node().create_client(Trigger,'/mission_manager/'+name)
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Servico ausente '+name)
        future=c.call_async(Trigger.Request());deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline:time.sleep(.05)
        if not future.done() or future.result().success!=expected:raise RuntimeError('Resposta inesperada '+name+': '+(future.result().message if future.done() else 'timeout'))
        ros.node().destroy_client(c)
        # Service acceptance precedes the worker and the next status sample.
        if expected:time.sleep(3)
    link=None;backup=None
    try:
        until(lambda:state.get('state') in ('ESPERA','ESTACIONADO','RECARGA_CONCLUIDA') and not state.get('busy') and state.get('fresh'),60)
        link=Link('udpin:127.0.0.1:14557',1,root,timeout=5,source_system=247)
        link.wait('HEARTBEAT',timeout=10);link.wait('SIMSTATE',timeout=10)
        backup=link.download_mission();(root/'mission_backup.json').write_text(json.dumps(backup,indent=2))
        if a.battery_rejection:
            from mission_manager.mission import validate
            validate(backup)
            b=state['battery']
            if state['state']!='ESPERA' or state.get('inside') is not False or b['source'] not in ('simulated','sitl_battery_status') or b['operational_soc']>=b['minimum']:
                raise RuntimeError('Teste exige ESPERA fora da estacao e SOC simulado insuficiente; dentro inicia recarga automatica')
            before=dict(truth)
            service('start')
            until(lambda:not state['busy'] and not state['pending'])
            time.sleep(3)
            drift=math.hypot(truth['x']-before['x'],truth['y']-before['y'])
            passed=state['state']=='ESPERA' and state['mode']==4 and not state['armed'] and drift<.02
            results.append(dict(test='insufficient_battery_no_exit',passed=passed,drift_m=drift,battery=state['battery']))
            if not passed:raise RuntimeError('Bateria insuficiente autorizou saida')
            return 0
        st=load_stations()[0];pt=st['ponto_4m']
        items=[dict(seq=i,command=16,frame=3,lat=pt['latitude'],lon=pt['longitude'],altitude=0.) for i in range(2)]
        link.upload_mission(items);readback=link.verify_mission(items)
        lock_before=link.param('DOCK_LOCK')
        link.param('DOCK_LOCK',1)
        accepted=link.command(176,1,10,required=False)
        hb=link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1)
        results.append(dict(test='firmware_auto_interlock',passed=not accepted and hb.custom_mode==4))
        if accepted or hb.custom_mode==10:raise RuntimeError('AUTO interlock failed')
        link.param('DOCK_LOCK',lock_before)
        service('bind');until(lambda:(state.get('binding') or {}).get('sha256')==fingerprint(readback) and not state.get('busy'))
        for cycle in range(a.cycles):
            start=time.monotonic()
            if cycle==0 and a.dock_first:
                link.command(400,1);link.wait('HEARTBEAT',lambda h:h.base_mode&128)
                service('dock')
                until(lambda:state.get('state') in ('DOCKING_SOLICITADO','ALINHAMENTO','APROXIMACAO'),60)
            else:
                service('start')
                until(lambda:state.get('state') in ('MISSAO','SAIDA','PREPARAR_MISSAO'),60)
            if state.get('busy'):
                service('start',expected=False)
                results.append(dict(test='duplicate_start_rejected',cycle=cycle+1,passed=True))
            until(lambda:state.get('state')=='RECARGA_CONCLUIDA',1200)
            if time.monotonic()-truth.get('wall',0)>3:raise RuntimeError('Sem ground truth')
            e=dict(longitudinal=truth['x']-5,lateral=truth['y']+2,heading_deg=math.degrees(truth['yaw']))
            trace=[r for r in history if r['wall']>=start]
            charges=[r for r in trace if r['state']=='RECARGA']
            # A status from the previous parked cycle can arrive just after
            # start. Pair completion only with THIS cycle's charging interval.
            complete=[r for r in trace if r['state']=='RECARGA_CONCLUIDA' and charges and r['wall']>=charges[0]['wall']]
            duration=complete[0]['wall']-charges[0]['wall'] if charges and complete else None
            passed=(abs(e['longitudinal'])<.08 and abs(e['lateral'])<.08 and abs(e['heading_deg'])<6 and
                    state['mode']==4 and not state['armed'] and state['battery']['simulated_soc']==100 and
                    duration is not None and 14.0<duration<16.)
            row=dict(test='cycle',cycle=cycle+1,passed=passed,errors=e,charging_wall_s=duration,
                     elapsed_wall=time.monotonic()-start,states=list(dict.fromkeys(r['state'] for r in trace)))
            results.append(row);print(json.dumps(row),flush=True)
            if not passed:raise RuntimeError('Cycle geometry/state criteria failed')
            # Merely setting GUIDED after charging must not cause any exit.
            link.mode('GUIDED');before=dict(truth);time.sleep(3)
            stationary=math.hypot(truth['x']-before['x'],truth['y']-before['y'])<.02 and not state['pending']
            results.append(dict(test='manual_guided_no_exit',cycle=cycle+1,passed=stationary))
            link.mode('HOLD')
            if not stationary:raise RuntimeError('Unexpected exit on manual GUIDED')
    except Exception as exc:
        results.append(dict(test='execution',passed=False,reason=str(exc)));print(str(exc),flush=True)
    finally:
        (root/'results.json').write_text(json.dumps(results,indent=2))
        (root/'status_history.json').write_text(json.dumps(history,indent=2))
        if link:link.close()
        node.unsubscribe('/world/gravel_plane/pose/info')
        sub.unregister();ros.signal_shutdown('validation complete')
    return 0 if results and all(r['passed'] for r in results) else 1

if __name__=='__main__':sys.exit(main())
