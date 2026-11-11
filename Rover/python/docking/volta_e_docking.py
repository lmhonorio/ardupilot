#!/usr/bin/env python3
"""Simulacao: missao AUTO de volta e posterior entrada visual GUIDED."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
os.environ['MAVLINK20']='1'
from pymavlink import mavutil

PYTHON_DIR=Path(__file__).resolve().parents[1]
ROVER=PYTHON_DIR.parent
sys.path.insert(0,str(PYTHON_DIR))
from common.docking_stations import DEFAULT_STATIONS, load_stations, select_station
sys.path.insert(0,str(PYTHON_DIR/'missions'))
from mavproxy_link import Link, json_safe


def read_wpl(path):
    lines=path.read_text().splitlines()
    if not lines or lines[0].strip()!='QGC WPL 110': raise ValueError('Arquivo de missao invalido')
    items=[]
    for line in lines[1:]:
        f=line.split()
        if len(f)!=12: raise ValueError('Linha WPL invalida')
        items.append(dict(seq=int(f[0]),frame=int(f[2]),command=int(f[3]),param1=float(f[4]),
                          lat=float(f[8]),lon=float(f[9]),altitude=float(f[10])))
    if len(items)<2 or any(item['seq']!=i for i,item in enumerate(items)): raise ValueError('Sequencia WPL invalida')
    return items


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mavproxy',default='udpin:127.0.0.1:14551')
    p.add_argument('--sysid',type=int,default=1)
    p.add_argument('--timeout',type=float,default=300)
    p.add_argument('--namespace',default='/rover_argo_N1/Instance1',help='Namespace ROS 2 da camera e juntas')
    p.add_argument('--estacoes',default=str(DEFAULT_STATIONS))
    a=p.parse_args()
    if not a.mavproxy.startswith('udpin:127.0.0.1:') or not 1<=a.sysid<=255 or a.timeout<=0: p.error('Endpoint local, sysid e timeout invalidos')
    # This predefined mission approaches this station specifically. Validate
    # its registration before any mission upload, even with multiple stations.
    try:station=select_station(load_stations(a.estacoes),'container_gravel_plane')
    except (OSError,ValueError) as exc:p.error(str(exc))
    root=PYTHON_DIR/'run/docking'/('volta_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    root.mkdir(parents=True,exist_ok=True)
    items=read_wpl(ROVER/'simulacoes/docking/volta_docking.waypoints')
    link=Link(a.mavproxy,a.sysid,root,timeout=8,source_system=243)
    started=False; closed=False
    signal.signal(signal.SIGTERM,lambda sig,frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        hb=link.wait('HEARTBEAT',lambda m:m.get_srcComponent()==1,timeout=20)
        if hb.type!=mavutil.mavlink.MAV_TYPE_GROUND_ROVER or hb.base_mode&128:
            raise RuntimeError('Inicie com Rover desarmado; missao existente nao alterada')
        link.conn.mav.command_long_send(a.sysid,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE,500000,0,0,0,0,0)
        link.wait(['SIMSTATE','SIM_STATE'],timeout=10)
        old=link.download_mission()
        (root/'missao_anterior.json').write_text(json.dumps(json_safe(old),indent=2,allow_nan=False))
        print('Backup da missao:',root/'missao_anterior.json',flush=True)
        link.upload_mission(items);link.verify_mission(items)
        print('Missao de volta carregada; verifique no QGroundControl.',flush=True)
        link.mode('HOLD')
        link.command(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,1,0)
        link.wait_stationary(timeout=60)
        link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
        link.wait('HEARTBEAT',lambda m:m.get_srcComponent()==1 and m.base_mode&128,timeout=15)
        started=True
        link.command(mavutil.mavlink.MAV_CMD_MISSION_START,1,0)
        link.wait('HEARTBEAT',lambda m:m.get_srcComponent()==1 and m.custom_mode==10,timeout=15)
        print('AUTO: executando volta e aproximacao...',flush=True)
        deadline=time.monotonic()+a.timeout; report=0;complete=False
        while time.monotonic()<deadline:
            m=link.recv(.2)
            if time.monotonic()>=report:
                v=link.latest.get('VFR_HUD');n=link.latest.get('NAV_CONTROLLER_OUTPUT');cur=link.latest.get('MISSION_CURRENT')
                print('waypoint=%s distancia=%s velocidade=%s'%(getattr(cur,'seq',None),getattr(n,'wp_dist',None),round(v.groundspeed,2) if v else None),flush=True)
                report=time.monotonic()+5
            if m and m.get_type()=='MISSION_CURRENT' and getattr(m,'mission_state',0)==5 and m.seq>=len(items)-1:
                complete=True;break
            if m and m.get_type()=='STATUSTEXT' and 'Mission Complete' in str(m.text) and link.seq>=len(items)-1:
                complete=True;break
            if m and m.get_type()=='HEARTBEAT' and m.get_srcComponent()==1 and (not m.base_mode&128 or m.custom_mode not in (10,4)):
                raise RuntimeError('Missao interrompida por mudanca de modo ou desarmamento')
        if not complete: raise RuntimeError('Missao nao foi concluida no prazo')
        link.mode('HOLD'); print('Aproximacao concluida; aguardando parar.',flush=True)
        link.wait_stationary(timeout=45)
        link.close();closed=True
        print('Iniciando entrada visual GUIDED...',flush=True)
        result=subprocess.call([sys.executable,str(PYTHON_DIR/'docking/entrar_container.py'),
                                '--estacoes',a.estacoes,'--estacao',station['nome'],
                                '--uma-vez','--namespace',a.namespace,'--sysid',str(a.sysid),'--mavproxy',a.mavproxy])
        if result: raise RuntimeError('Entrada visual interrompida; veja log acima')
        print('Sequencia AUTO e entrada visual concluida.',flush=True)
        return 0
    except (Exception,KeyboardInterrupt) as e:
        print('Sequencia interrompida:',e,flush=True)
        if started and not closed:
            try:link.mode('HOLD');print('HOLD confirmado',flush=True)
            except Exception as err:print('Nao foi possivel confirmar HOLD:',err,flush=True)
        return 1
    finally:
        if not closed:
            link.log.close();link.conn.close()

if __name__=='__main__':sys.exit(main())
