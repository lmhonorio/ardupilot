#!/usr/bin/env python3
"""Observe a requested simulation cycle; all control requests go through ROS services."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from common import ros2_runtime as ros
from std_msgs.msg import String
from std_srvs.srv import Trigger
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

p=argparse.ArgumentParser();p.add_argument('mission');p.add_argument('--output',required=True);a=p.parse_args()
root=Path(a.output);root.mkdir(parents=True,exist_ok=True)
ros.init_node('companion_mission_validation',anonymous=True);state={};truth={};history=[];phases=[]
def status(m):
    value=json.loads(m.data);state.clear();state.update(value);history.append(value)
sub=ros.Subscriber('/mission_manager/status',String,status,queue_size=5)
gz=Node()
def pose(msg):
    for q in msg.pose:
        if q.name=='rover_argo_N1':
            t=q.orientation
            truth.update(x=q.position.x,y=q.position.y,yaw=math.atan2(2*(t.w*t.z+t.x*t.y),1-2*(t.y*t.y+t.z*t.z)),wall=time.monotonic())
gz.subscribe(Pose_V,'/world/gravel_plane/pose/info',pose)
result={};started=False
try:
    end=time.monotonic()+60
    while time.monotonic()<end:
        if state.get('state')=='FALHA':raise RuntimeError(state.get('last_error'))
        if state.get('state') in ('ESPERA','RECARGA_CONCLUIDA') and not state.get('busy'):break
        time.sleep(.2)
    else:raise RuntimeError('Gerenciador nao ficou pronto')
    if state.get('mode')!=4 or state.get('armed'):raise RuntimeError('Inicio de teste exige HOLD desarmado')
    print('Estado inicial:',state['state'],flush=True)
    cmd=[sys.executable,str(Path(__file__).parents[1]/'mission_remote.py'),a.mission]
    subprocess.run(cmd,check=True,timeout=45);started=True
    begin=time.monotonic();last=None;seen_mission=False;seen_charge=False
    while time.monotonic()-begin<900:
        s=dict(state);phase=s.get('state')
        if phase!=last:phases.append(phase);print(phase,s.get('last_error'),flush=True);last=phase
        if phase=='FALHA':raise RuntimeError(s.get('last_error'))
        if (s.get('last_request') or {}).get('result')=='rejeitado':raise RuntimeError(s['last_request']['message'])
        if phase=='MISSAO':seen_mission=True
        if seen_mission and phase=='RECARGA':seen_charge=True
        if seen_charge and phase=='RECARGA_CONCLUIDA':
            if time.monotonic()-truth.get('wall',0)>3:raise RuntimeError('Sem referencia independente do Gazebo')
            errors={'longitudinal_m':truth['x']-5,'lateral_m':truth['y']+2,'heading_deg':math.degrees(truth['yaw'])}
            passed=(s['mode']==4 and not s['armed'] and s['inside'] and s['battery']['operational_soc']==100
                    and abs(errors['longitudinal_m'])<.08 and abs(errors['lateral_m'])<.08 and abs(errors['heading_deg'])<6)
            result={'passed':passed,'elapsed_s':time.monotonic()-begin,'errors':errors,'status':s};break
        time.sleep(.2)
    else:raise RuntimeError('Timeout do ciclo')
except Exception as exc:
    result={'passed':False,'error':str(exc),'status':dict(state)}
    if started and state.get('state')!='FALHA':
        client=ros.node().create_client(Trigger,'/mission_manager/cancel')
        if client.wait_for_service(timeout_sec=2):
            f=client.call_async(Trigger.Request());end=time.monotonic()+5
            while not f.done() and time.monotonic()<end:time.sleep(.1)
finally:
    (root/'result.json').write_text(json.dumps(dict(result=result,phases=phases),indent=2))
    (root/'status.json').write_text(json.dumps(history,indent=2))
    print(json.dumps(result),flush=True)
    gz.unsubscribe('/world/gravel_plane/pose/info');sub.unregister();ros.signal_shutdown('validation finished')
sys.exit(0 if result.get('passed') else 1)
