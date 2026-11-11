#!/usr/bin/env python3
"""Explicit SITL/Gazebo docking trials; never run by unittest discovery.

Requires --execute. Resets only the simulated rover pose while disarmed,
commands docking via MAVProxy, and measures independent Gazebo ground truth.
"""
import argparse
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'python/missions'))
from mavproxy_link import Link
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

CASES={
    'aligned':(3.2,-2.,0.),
    'left_offset':(2.,-1.5,20.),
    'right_offset':(2.,-2.5,-20.),
    'yaw_left':(2.5,-2.1,25.),
    'yaw_right':(2.5,-1.9,-25.),
    'repeat_aligned':(3.2,-2.,0.),
}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--case',choices=CASES,action='append')
    p.add_argument('--speed',type=float,default=.25)
    a=p.parse_args()
    if not a.execute:p.error('Use --execute para autorizar os ensaios no SITL')
    lock=(ROOT/'ros2/run/docking-controller.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    root=ROOT/'python/run/docking_validation'/datetime.now().strftime('%Y%m%d_%H%M%S')
    root.mkdir(parents=True);print('RESULTADOS',root,flush=True)
    guard=threading.Lock();state={'pose':None,'wall':0.,'trace':[]}
    node=Node()
    def receive(msg):
        for model in msg.pose:
            if model.name!='rover_argo_N1':continue
            q=model.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            row=dict(t=time.monotonic(),x=model.position.x,y=model.position.y,
                     z=model.position.z,yaw=yaw)
            with guard:
                state['pose']=row;state['wall']=time.monotonic()
                if not state['trace'] or row['t']-state['trace'][-1]['t']>.1:state['trace'].append(row)
    node.subscribe(Pose_V,'/world/gravel_plane/pose/info',receive)
    def truth():
        with guard:
            if state['pose'] is None or time.monotonic()-state['wall']>2:
                raise RuntimeError('Sem ground truth recente do Gazebo')
            return dict(state['pose'])
    def connect(path):
        path.mkdir(parents=True,exist_ok=True)
        link=Link('udpin:127.0.0.1:14551',1,path,source_system=244)
        hb=link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1,timeout=10)
        if hb.type!=10:raise RuntimeError('Alvo nao e Rover')
        link.conn.mav.command_long_send(1,1,511,0,164,500000,0,0,0,0,0)
        link.wait('SIMSTATE',timeout=5)
        return link
    def stop_child(child):
        child.terminate()
        try:child.wait(timeout=12)
        except subprocess.TimeoutExpired:child.kill();child.wait()
    results=[];child=None;link=None
    try:
        for name in a.case or CASES:
            x,y,deg=CASES[name];folder=root/name;folder.mkdir()
            print('INICIO',name,CASES[name],flush=True)
            link=connect(folder/'prepare')
            link.mode('HOLD');link.wait_stationary(timeout=20)
            if link.latest['HEARTBEAT'].base_mode&128:link.disarm()
            before=link.download_mission()
            parameters_before={name:link.param(name) for name in
                ('OA_TYPE','AVOID_ENABLE','ATC_STR_RAT_FF','ATC_STR_RAT_P','ATC_STR_RAT_I')}
            angle=math.radians(deg)
            req='name: "rover_argo_N1", position: {x: %s, y: %s, z: .1513}, orientation: {z: %s, w: %s}'%(x,y,math.sin(angle/2),math.cos(angle/2))
            subprocess.run(['gz','service','-s','/world/gravel_plane/set_pose','--reqtype','gz.msgs.Pose',
                '--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],check=True,timeout=8,capture_output=True)
            # Instant Gazebo yaw changes are not observed by the gyro. Let EKF
            # recover before the test; never feed ground truth to the controller.
            until=time.monotonic()+180;stable=None
            while time.monotonic()<until:
                link.recv(.1)
                att=link.latest.get('ATTITUDE');sim=link.latest.get('SIMSTATE')
                error=math.inf if not att or not sim else abs((att.yaw-sim.yaw+math.pi)%(2*math.pi)-math.pi)
                if error<math.radians(2):
                    if stable is None:stable=time.monotonic()
                    if time.monotonic()-stable>3:break
                else:stable=None
            else:raise RuntimeError('EKF nao estabilizou apos reposicionamento; nao iniciando ensaio')
            # A position teleport also induces a transient EKF velocity. Check
            # it after the reset, not only before resetting the Gazebo pose.
            link.wait_stationary(timeout=45)
            current=truth()
            if math.hypot(current['x']-x,current['y']-y)>.03:raise RuntimeError('Reset do modelo nao confirmado')
            link.command(400,1)
            link.wait('HEARTBEAT',lambda h:h.base_mode&128,timeout=10)
            link.wait_stationary(timeout=15)
            link.close();link=None
            with guard:state['trace']=[]
            start=time.monotonic()
            with (folder/'console.log').open('w') as log:
                child=subprocess.Popen([sys.executable,str(ROOT/'python/docking/entrar_container.py'),
                    '--uma-vez','--timeout','180','--velocidade',str(a.speed)],stdout=log,stderr=subprocess.STDOUT)
                reason=''
                while child.poll() is None:
                    time.sleep(.3);current=truth()
                    if time.monotonic()-start>240:reason='tempo total excedido'
                    # Conservative body envelope, checked independently of visual pose.
                    c,s=math.cos(current['yaw']),math.sin(current['yaw'])
                    for dx in (-.30,.30):
                        for dy in (-.27,.27):
                            px=current['x']+c*dx-s*dy;py=current['y']+s*dx+c*dy
                            if 4.50<px<5.55 and (abs(py+2)>.48 or px>5.49):
                                reason='envelope do rover ultrapassou limite interno'
                    if reason:stop_child(child);break
                code=child.wait();child=None
            elapsed=time.monotonic()-start
            link=connect(folder/'verify')
            hb=link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1,timeout=5)
            after=link.download_mission()
            parameters_after={name:link.param(name) for name in parameters_before}
            (folder/'mission_before.json').write_text(json.dumps(before,indent=2)+'\n')
            (folder/'mission_after.json').write_text(json.dumps(after,indent=2)+'\n')
            current=truth()
            errors=dict(longitudinal=current['x']-5,lateral=current['y']+2,heading_deg=math.degrees(current['yaw']))
            passed=(code==0 and not reason and hb.custom_mode==4 and not hb.base_mode&128
                    and abs(errors['longitudinal'])<=.08 and abs(errors['lateral'])<=.08
                    and abs(errors['heading_deg'])<=6)
            # Ignore link destination fields when comparing a downloaded mission.
            # ArduPilot updates HOME (seq 0) normally when arming after reset.
            # The test must preserve every user mission item, seq >= 1.
            clean=lambda items:[{k:v for k,v in i.items() if k not in ('target_system','target_component')}
                                for i in items if i['seq']!=0]
            unchanged=clean(before)==clean(after)
            restored=all(abs(parameters_before[k]-parameters_after[k])<1e-5 for k in parameters_before)
            passed=passed and unchanged and restored
            result=dict(case=name,start=CASES[name],speed=a.speed,elapsed_wall=elapsed,exit_code=code,
                        reason=reason,final=current,errors=errors,hold=hb.custom_mode==4,
                        disarmed=not bool(hb.base_mode&128),mission_unchanged=unchanged,passed=passed,
                        parameters_restored=restored,parameters_before=parameters_before,
                        parameters_after=parameters_after)
            results.append(result)
            (root/'results.json').write_text(json.dumps(results,indent=2)+'\n')
            with guard:(folder/'ground_truth.json').write_text(json.dumps(state['trace'])+'\n')
            print('RESULTADO',json.dumps(result),flush=True)
            if hb.base_mode&128 or hb.custom_mode!=4:
                link.mode('HOLD');link.wait_stationary(timeout=20)
                if link.latest['HEARTBEAT'].base_mode&128:link.disarm()
            link.close();link=None
            if not passed:break
    finally:
        if child is not None and child.poll() is None:stop_child(child)
        if link is not None:link.close()
    return 0 if results and all(r['passed'] for r in results) and len(results)==len(a.case or CASES) else 1

if __name__=='__main__':sys.exit(main())
