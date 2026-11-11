#!/usr/bin/env python3
"""ROS 2 Mission Manager: one MAVProxy owner, LiDAR docking, optional QR, SITL charge."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
os.environ['MAVLINK20']='1'
from common import ros2_runtime as ros
from common.docking_stations import DEFAULT_STATIONS, load_stations, select_station, nearest_station, distance_and_bearing
from missions.mavproxy_link import Link
from mission_manager.policy import Machine, State, Battery
from mission_manager.geometry import Envelope
from mission_manager.mission import validate, bind, fingerprint, requested
from mission_manager.perception import Perception
from mission_manager.motion import Motion
from mission_manager.files import load_mission, verify_readback
from mission_manager.charging import SITLCharger
from mission_manager.clock_sync import BootClock

class RequestRejected(ValueError):
    """Input rejection before any upload or movement; preserve current state."""

class ManagedLink(Link):
    component_filter=1  # MAVROS/camera heartbeats must not replace autopilot state.
    def send_item(self, item, integer=True):
        # This manager imports full mission items, including speed/hold params.
        self.conn.mav.mission_item_int_send(
            self.sysid,self.compid,item['seq'],{0:5,3:6}.get(item['frame'],item['frame']),
            item['command'],int(item['seq']==0),item['autocontinue'],
            *(item.get('param%d'%n,0) for n in range(1,5)),
            int(round(item['x']*1e7)),int(round(item['y']*1e7)),item['z'])

    def recv(self, timeout=.2):
        if getattr(self,'owner',None):
            self.owner.check_cancel()
            self.owner.clock_sync.request(ros.seconds(ros.now()),time.monotonic(),self.conn.mav.timesync_send)
        msg=super().recv(timeout)
        if msg and msg.get_srcComponent()==1 and getattr(self,'owner',None):self.owner.observe(msg)
        return msg

class Manager:
    def __init__(self, args):
        self.args=args;self.stations=load_stations(args.estacoes)
        # Restrict the docking destination, not the physical occupancy check:
        # a rover parked in another registered station still needs a safe exit.
        self.station=select_station(self.stations,args.station) if args.station else self.stations[0]
        self.cancel=threading.Event();self.shutting=False
        self.ignore_cancel=False;self.busy=False;self.guard=threading.Lock();self.jobs=queue.Queue(maxsize=1)
        self.root=Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parents[1]/'run/mission_manager'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.root.mkdir(parents=True,exist_ok=True)
        self.events=(self.root/'events.jsonl').open('a')
        self.event_guard=threading.Lock();self.last_snapshot=0.
        sources=list(Path(__file__).parent.glob('*.py'))
        rover=Path(__file__).resolve().parents[2]
        sources += [rover/'estacoes_recarga.txt',rover/'python/docking/visual_alignment.py',
                    rover/'python/docking_lidar/sensores.py',rover/'python/docking_lidar/geometria.py',
                    rover/'python/missions/mavproxy_link.py',rover/'ros2/launch/simulation.launch.py',
                    rover.parent/'build/sitl/bin/ardurover']
        (self.root/'manifest.json').write_text(json.dumps(dict(arguments=vars(args),
            sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.exists()}),indent=2))
        self.machine=Machine(self.event)
        self.battery=Battery(args.simulate_charge,args.initial_soc,args.minimum_soc,args.discharge_rate,native=args.simulate_charge)
        self.charger=SITLCharger(self)
        self.envelope=Envelope(**json.loads(Path(args.envelope).read_text())) if args.envelope else Envelope()
        for station in self.stations:self.envelope.validate(station)
        self.items=[];self.binding=None;self.reached=None;self.bound_at=0.;self.reached_at=0.
        self.last_error=None;self.mission_file=None;self.last_request=None
        self.clock_sync=BootClock()
        self.link=ManagedLink(args.mavproxy,args.sysid,self.root,timeout=args.mode_timeout,source_system=244)
        self.link.owner=self
        self.perception=None;self.last_publish=0.;self.last_mission=0.;self.pending_hash=None
        self.recovery_file=Path(args.recovery);self.recovery_file.parent.mkdir(parents=True,exist_ok=True)
        self.pub=ros.Publisher('/mission_manager/status',__import__('std_msgs.msg',fromlist=['String']).String,queue_size=1,latch=True)
        from std_srvs.srv import Trigger
        self.services=[]
        for name in ('start','dock','bind','reconcile','cancel'):
            def callback(req,res,action=name):
                ok,message=self.enqueue(action);res.success=ok;res.message=message;return res
            self.services.append(ros.node().create_service(Trigger,'/mission_manager/'+name,callback))
        from rover_mission_interfaces.srv import StartMission
        def start_file(req,res):
            res.success,res.message=self.enqueue('start_file',req.mission_file)
            return res
        self.services.append(ros.node().create_service(StartMission,'/mission_manager/start_file',start_file))
        # The ROS thread keeps publishing status even during bounded MAVLink waits.
        self.timer=ros.node().create_timer(.5,self.publish)

    def event(self, **row):
        row=dict(wall=time.time(),monotonic=time.monotonic(),**row)
        with self.event_guard:
            if self.events.closed:return
            self.events.write(json.dumps(row,allow_nan=False,default=str)+'\n');self.events.flush()
        if row.get('kind') in ('transition','failure','request','exit_confirmed'):
            print(json.dumps(row,ensure_ascii=False),flush=True)

    def status(self):
        hb=self.link.latest.get('HEARTBEAT');pos=self.link.latest.get('GLOBAL_POSITION_INT')
        return dict(state=self.machine.state.value,inside=self.machine.inside,pending=self.machine.pending,
            busy=self.busy,mode=hb.custom_mode if hb else None,armed=bool(hb and hb.base_mode&128),
            fresh=self.fresh(),station=self.station['nome'],seq=self.link.seq,reached=self.reached,
            position=[pos.lat/1e7,pos.lon/1e7] if pos else None,battery=self.battery.status(time.monotonic()),
            mission_hash=fingerprint(self.items) if self.items else None,binding=self.binding,
            mission_file=self.mission_file,last_error=self.last_error,last_request=self.last_request,
            clock_sync=self.clock_sync.status(),
            lidar_error=self.perception.error if self.perception else 'inicializando',
            lidar_diagnostic=self.perception.diagnostic if self.perception else {},log_dir=str(self.root))

    def publish(self):
        if time.monotonic()-self.last_publish<.2:return
        self.last_publish=time.monotonic()
        status=self.status()
        self.pub.publish(json.dumps(status,default=str))
        if time.monotonic()-self.last_snapshot>=1.:
            self.last_snapshot=time.monotonic();self.event(kind='status',**status)

    def observe(self, msg):
        kind=msg.get_type()
        if kind=='TIMESYNC':
            if self.clock_sync.response(msg.tc1,msg.ts1,ros.seconds(ros.now()),time.monotonic()):
                self.event(kind='clock_sync',**self.clock_sync.status())
        if kind=='BATTERY_STATUS' and msg.id==0:self.battery.observe(msg.battery_remaining,time.monotonic())
        elif kind=='MISSION_ITEM_REACHED':self.reached=msg.seq;self.reached_at=time.monotonic()

    def fresh(self):
        return all(time.monotonic()-self.link.last_seen.get(k,0)<3 for k in ('HEARTBEAT','GLOBAL_POSITION_INT','ATTITUDE','VFR_HUD','SIMSTATE'))

    def check_cancel(self):
        if self.cancel.is_set() and not self.ignore_cancel:raise RuntimeError('Operacao cancelada pelo operador')

    def motion_state(self):
        att=self.link.latest.get('ATTITUDE')
        if (any(time.monotonic()-self.link.last_seen.get(k,0)>3 for k in ('VFR_HUD','ATTITUDE'))
                or att is None):
            raise RuntimeError('Velocidade/atitude desatualizada')
        try:age=self.clock_sync.age(att.time_boot_ms*.001,ros.seconds(ros.now()),time.monotonic())
        except ValueError as exc:raise RuntimeError('Velocidade/atitude desatualizada: '+str(exc)) from exc
        if abs(age)>.5:raise RuntimeError('Velocidade/atitude desatualizada: idade corrigida %.3f s'%age)
        return self.link.latest['VFR_HUD'].groundspeed,self.link.latest['ATTITUDE'].yawspeed

    def wait_start_stopped(self):
        """Read-only preflight: wait for fresh motion telemetry, never use stale data."""
        deadline=time.monotonic()+getattr(self.args,'mode_timeout',5.)
        reported=False
        while True:
            if self.link.latest['HEARTBEAT'].custom_mode!=4:
                raise RequestRejected('Solicite inicio com rover parado em HOLD')
            try:
                speed,yaw_rate=self.motion_state()
            except RuntimeError as exc:
                if not str(exc).startswith('Velocidade/atitude desatualizada'):raise
                if not reported:
                    self.event(kind='start_wait',reason=str(exc));reported=True
                if time.monotonic()>=deadline:
                    raise RequestRejected('Inicio recusado antes do upload: velocidade/atitude nao ficaram recentes; aguarde telemetria e tente novamente') from exc
                self.link.recv(.05)
                continue
            if not all(math.isfinite(v) for v in (speed,yaw_rate)):
                raise RequestRejected('Inicio recusado: velocidade/atitude nao finitas')
            if abs(speed)>.05 or abs(yaw_rate)>.05:
                raise RequestRejected('Aguarde parada completa antes de enviar a missao')
            return

    def enqueue(self, action, mission_file=None):
        if action=='cancel':self.cancel.set();return True,'Parada solicitada; acompanhe HOLD/status'
        with self.guard:
            if self.busy or not self.jobs.empty():return False,'Operacao ja em andamento'
            if self.machine.state==State.FAULT and action!='reconcile':
                return False,'Falha do gerenciador: %s; use reconcile apos corrigir a causa'%getattr(self,'last_error',None)
            if action in ('start','start_file'):
                if self.machine.pending:return False,'Missao ja pendente'
                if self.machine.state not in (State.WAIT,State.PARKED,State.CHARGING,State.CHARGED):
                    return False,'Inicio exige rover parado ou em recarga; estado '+self.machine.state.value
                if action=='start_file' and (not mission_file or not Path(mission_file).is_absolute()):
                    return False,'Informe mission_file como caminho absoluto no computador do manager'
            if action=='dock' and self.machine.state not in (State.WAIT,State.MISSION):return False,'Estado nao permite entrada'
            if action=='reconcile' and self.machine.state not in (State.WAIT,State.FAULT,State.PARKED):return False,'Cancele a operacao antes de reconciliar'
            if action=='bind' and self.machine.state not in (State.WAIT,State.PARKED,State.CHARGED):return False,'Vincule a missao com o rover parado, fora de uma operacao'
            self.jobs.put_nowait((action,mission_file) if action=='start_file' else action)
        return True,'Solicitacao aceita; resultado em /mission_manager/status e events.jsonl'

    def save_recovery(self, saved):
        tmp=self.recovery_file.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(sysid=self.args.sysid,params=saved,station=self.station['nome']),indent=2))
        tmp.replace(self.recovery_file)

    def clear_recovery(self):
        self.recovery_file.unlink(missing_ok=True)

    def recover_params(self):
        if self.recovery_file.exists():
            record=json.loads(self.recovery_file.read_text())
            if record['sysid']!=self.args.sysid:raise RuntimeError('Recovery de outro sysid')
            for k,v in record['params'].items():self.link.param(k,v)
            self.clear_recovery()

    def select_nearest(self, name=None):
        pos=self.link.latest.get('GLOBAL_POSITION_INT');gps=self.link.latest.get('GPS_RAW_INT')
        if not pos or not gps or gps.fix_type<3 or time.monotonic()-self.link.last_seen.get('GPS_RAW_INT',0)>3:
            raise RuntimeError('GPS 3D recente necessario para associacao')
        self.station=select_station(self.stations,name) if name else nearest_station(self.stations,pos.lat/1e7,pos.lon/1e7)
        self.perception.select(self.station)
        return distance_and_bearing((pos.lat/1e7,pos.lon/1e7),(self.station['latitude'],self.station['longitude']))[0]

    def reconcile(self):
        self.ignore_cancel=True
        try:
            self.cancel.clear();self.machine.pending=False;self.pending_hash=None
            if self.perception is None:
                self.link.param('DOCK_LOCK')
                self.link.command(511,164,200000)
                self.link.wait('SIMSTATE',timeout=10)
                self.perception=Perception(self.link,self.args.namespace,self.args.cloud,self.args.odom,self.args.no_qr)
            hb=self.link.latest['HEARTBEAT']
            if hb.custom_mode not in (4,15) and hb.base_mode&128:
                raise RuntimeError('Para reconciliar, pare o rover em HOLD')
            motion=Motion(self);motion.stop();self.recover_params()
            distance=self.select_nearest()
            far=max(3.,math.hypot(*self.station['dimensoes_internas'][:2])/2+
                    math.hypot(self.envelope.length,self.envelope.width)/2+1.)
            if distance>far:
                self.machine.confirmed_outside();self.link.param('DOCK_LOCK',0)
                self.machine.move(State.WAIT,'GPS fresco confirma longe da estacao')
            else:
                self.link.param('DOCK_LOCK',1)
                pose=motion.pose()
                if self.envelope.final(pose,self.station):
                    self.link.disarm();self.machine.confirmed_parked()
                elif self.envelope.outside(pose,self.station,rotation=True):
                    self.machine.confirmed_outside();self.link.param('DOCK_LOCK',0)
                    self.machine.move(State.WAIT,'LiDAR confirma envelope fora da estacao')
                else:raise RuntimeError('Rover parcialmente dentro; recuperacao manual em GUIDED/HOLD necessaria')
            self.items=self.link.download_mission()
            self.last_error=None
            self.begin_charge_if_parked()
        finally:self.ignore_cancel=False

    def begin_charge_if_parked(self):
        if self.args.simulate_charge and self.machine.state==State.PARKED:
            hb=self.link.latest['HEARTBEAT']
            if hb.custom_mode!=4 or hb.base_mode&128:
                raise RuntimeError('Recarga exige HOLD e desarmamento confirmado')
            if self.battery.native:self.charger.prepare()
            self.battery.begin_charge(time.monotonic(),sitl=self.fresh(),parked=self.machine.inside is True)
            self.machine.move(State.CHARGING,'Recarga SITL: 15 segundos, confirmacao pelo ArduPilot')

    def destination_binding(self, items):
        validate(items)
        destination=nearest_station(self.stations,items[-1]['x'],items[-1]['y'])
        if self.args.station and destination['nome']!=self.args.station:
            raise RuntimeError('Destino da missao difere de --station')
        return bind(items,destination)

    def store_binding(self, items, binding):
        self.binding=binding;self.items=items;self.reached=None;self.bound_at=time.monotonic()
        (self.root/'mission_binding.json').write_text(json.dumps(self.binding,indent=2))
        self.event(kind='binding',binding=self.binding)

    def bind_current(self):
        if self.link.latest['HEARTBEAT'].custom_mode!=4:raise RuntimeError('Vincule a missao em HOLD')
        items=self.link.download_mission();validate(items)
        # Bind the destination, not the station where the rover happens to be.
        # Keep self.station as the physical station for a pending exit.
        self.store_binding(items,self.destination_binding(items))

    def dock(self, station_name=None):
        if self.machine.state not in (State.WAIT,State.MISSION):raise RuntimeError('Estado nao permite entrada')
        if self.select_nearest(station_name or self.args.station)>=8:raise RuntimeError('Estacao fora dos 8 m')
        if not self.link.latest['HEARTBEAT'].base_mode&128:raise RuntimeError('Entrada exige rover armado')
        self.machine.move(State.REQUESTED,'Solicitacao docking reconhecida')
        self.link.param('DOCK_LOCK',1)
        self.link.mode('GUIDED')
        motion=Motion(self)
        try:
            motion.dock()
        finally:
            # Restoring avoidance while still moving could change the stop behavior.
            self.ignore_cancel=True
            try:motion.stop();motion.restore()
            finally:self.ignore_cancel=False
        self.begin_charge_if_parked()

    def request_start(self, mission_file=None):
        # Reject invalid requests without aborting an ongoing simulated charge.
        try:
            if self.machine.state not in (State.WAIT,State.PARKED,State.CHARGING,State.CHARGED):
                raise ValueError('Estado nao permite iniciar missao')
            if self.machine.pending:raise ValueError('Missao ja pendente')
            if self.machine.inside is None:raise ValueError('Localizacao desconhecida; reconciliar')
            if not self.fresh():raise ValueError('Telemetria desatualizada; aguarde comunicacao')
            if self.link.latest['HEARTBEAT'].custom_mode!=4:
                raise ValueError('Solicite inicio com rover parado em HOLD')
            if not self.battery.sufficient(time.monotonic()) and self.machine.state!=State.CHARGING:
                raise ValueError('Bateria insuficiente ou indisponivel')
            items=load_mission(mission_file) if mission_file else self.link.download_mission()
            binding=self.destination_binding(items)
        except (ValueError,RuntimeError,OSError,KeyError,TypeError) as exc:
            raise RequestRejected(str(exc)) from exc
        self.wait_start_stopped()
        if mission_file:
            backup=self.link.download_mission()
            name='mission_backup_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json'
            (self.root/name).write_text(json.dumps(backup,indent=2))
            self.link.upload_mission(items)
            received=self.link.download_mission()
            verify_readback(items,received)
            items=received;binding=self.destination_binding(items)
            self.mission_file=str(Path(mission_file).resolve())
            self.event(kind='mission_uploaded',file=self.mission_file,backup=name,items=len(items))
        self.store_binding(items,binding)
        self.last_error=None
        ok,reason=self.machine.request_start(True,self.battery.sufficient(time.monotonic()))
        self.event(kind='request',action='start',accepted=ok,reason=reason)
        if not ok:return
        self.items=items;self.pending_hash=fingerprint(items)
        if self.machine.state!=State.CHARGING:self.start_pending()

    def start_pending(self):
        if not self.machine.pending:return
        if not self.battery.sufficient(time.monotonic()):
            self.machine.pending=False;self.event(kind='request',accepted=False,reason='Bateria insuficiente');return
        items=self.link.download_mission();validate(items)
        if fingerprint(items)!=self.pending_hash:raise RuntimeError('Missao alterada depois da solicitacao; solicite novamente')
        self.machine.move(State.PREPARE,'Missao e bateria validadas')
        motion=Motion(self)
        if self.machine.inside:
            self.link.param('DOCK_LOCK',1)
            self.link.mode('GUIDED')
            self.machine.move(State.EXIT,'GUIDED confirmado e inicio de missao autorizado')
            try:motion.exit()
            finally:
                self.ignore_cancel=True
                try:motion.stop();motion.restore()
                finally:self.ignore_cancel=False
            self.machine.move(State.PREPARE,'Envelope totalmente fora; saida confirmada')
        if self.machine.inside is not False:raise RuntimeError('Localizacao externa nao confirmada')
        # A new upload while exiting must never run under the old authorization.
        check=self.link.download_mission()
        if fingerprint(check)!=self.pending_hash:raise RuntimeError('Missao mudou durante a saida')
        if not self.battery.sufficient(time.monotonic()):raise RuntimeError('Bateria insuficiente apos saida')
        self.link.mode('HOLD')
        restart=self.link.param('MIS_RESTART')
        if restart not in (0.,1.):raise RuntimeError('MIS_RESTART inesperado')
        # Both MIS_RESTART settings start at 1, never resume an old current item.
        self.link.conn.mav.mission_set_current_send(self.link.sysid,1,1)
        self.link.wait('MISSION_CURRENT',lambda m:m.seq==1)
        self.reached=None;self.reached_at=0.
        self.link.param('DOCK_LOCK',0)
        self.link.command(400,1)
        self.link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1 and h.base_mode&128)
        self.link.mode('AUTO')  # mode_auto.cpp calls mission.start_or_resume().
        self.items=check;self.machine.pending=False;self.pending_hash=None
        self.machine.move(State.MISSION,'AUTO confirmado, inicio seq 1; sem segundo MISSION_START')
        self.battery.last=time.monotonic()  # Exit time is not AUTO mission time.

    def fail(self, exc):
        self.last_error=str(exc)
        self.event(kind='failure',reason=str(exc))
        self.machine.pending=False;self.pending_hash=None;self.battery.started=None
        self.machine.move(State.FAULT,str(exc));self.ignore_cancel=True
        try:
            motion=Motion(self);motion.stop()
            self.recover_params()
        except Exception as stop_exc:self.event(kind='stop_failure',reason=str(stop_exc))
        finally:self.ignore_cancel=False;self.cancel.clear()

    def run(self):
        self.busy=True
        try:
            hb=self.link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1,timeout=15)
            if hb.type!=10:raise RuntimeError('Alvo nao e Rover')
            # A slow simulator stretches the default 1 Hz heartbeat in wall
            # time. Request bounded supervision streams instead of relaxing
            # watchdogs or treating stale telemetry as live.
            for msg_id in (0,24,30,33,74,164,147):self.link.command(511,msg_id,100000)
            self.link.wait('SIMSTATE',timeout=10)
            # Refuse an unpatched firmware instead of claiming a reactive guard.
            self.link.param('DOCK_LOCK')
            self.perception=Perception(self.link,self.args.namespace,self.args.cloud,self.args.odom,self.args.no_qr)
            end=time.monotonic()+15
            while (not self.fresh() or self.clock_sync.offset is None) and time.monotonic()<end:self.link.recv(.1)
            if not self.fresh():raise RuntimeError('Telemetria inicial incompleta')
            if self.clock_sync.offset is None:raise RuntimeError('Sincronizacao TIMESYNC inicial incompleta')
            self.link.wait('GPS_RAW_INT',lambda m:m.fix_type>=3,timeout=60)
            self.link.wait('BATTERY_STATUS',lambda m:m.id==0 and m.battery_remaining>=0,timeout=10)
            self.reconcile()
        except Exception as exc:
            self.fail(exc)
        finally:self.busy=False
        while not self.shutting and not ros.is_shutdown():
            try:
                self.link.recv(.02)
                now=time.monotonic()
                if self.cancel.is_set():raise RuntimeError('Cancelado')
                if not self.fresh() and self.machine.state!=State.FAULT:raise RuntimeError('Comunicacao perdida')
                charge_done=self.battery.tick(now,self.machine.state==State.MISSION)
                if self.machine.state==State.CHARGING and self.battery.native:
                    self.charger.sync(self.battery.soc,now,final=charge_done)
                if charge_done:
                    self.machine.move(State.CHARGED,'ArduPilot confirmou carga de 100%; aguardando solicitacao de missao')
                if self.machine.state in (State.PARKED,State.CHARGING,State.CHARGED):
                    hb=self.link.latest.get('HEARTBEAT')
                    if hb and (hb.custom_mode==10 or hb.base_mode&128):raise RuntimeError('Rover estacionado armado ou AUTO indevido')
                if self.machine.pending and self.machine.state==State.CHARGED:
                    self.busy=True;self.start_pending();self.busy=False
                with self.guard:
                    try:action=self.jobs.get_nowait();self.busy=True
                    except queue.Empty:action=None
                if action:
                    name,path=action if isinstance(action,tuple) else (action,None)
                    self.last_request=dict(action=name,file=path,result='executando')
                    try:
                        if name=='start_file':self.request_start(path)
                        else:{'start':self.request_start,'dock':self.dock,'bind':self.bind_current,'reconcile':self.reconcile}[name]()
                        self.last_request.update(result='concluido',message='Solicitacao processada; acompanhe state')
                    except RequestRejected as exc:
                        self.last_error=str(exc)
                        self.last_request.update(result='rejeitado',message=str(exc))
                        self.event(kind='request',action=name,accepted=False,reason=str(exc))
                    except Exception as exc:
                        self.last_request.update(result='falha',message=str(exc));raise
                    finally:self.busy=False
                hb=self.link.latest.get('HEARTBEAT')
                if hb and hb.custom_mode==10 and self.machine.state==State.WAIT and self.machine.inside is False:
                    self.items=self.link.download_mission();self.machine.move(State.MISSION,'AUTO externo observado')
                    self.battery.last=time.monotonic()
                if self.machine.state==State.MISSION:
                    if hb.custom_mode!=10:self.machine.move(State.WAIT,'Operador saiu de AUTO')
                    elif now-self.last_mission>3.:
                        self.items=self.link.download_mission();self.last_mission=now
                        distance=self.select_nearest(self.binding['station'] if self.binding else None)
                        if self.reached_at>self.bound_at and requested(self.binding,self.items,self.reached,self.link.seq,hb.custom_mode,distance):
                            self.busy=True;self.dock(self.binding['station']);self.busy=False
                self.publish()
            except Exception as exc:self.busy=False;self.fail(exc)
        self.ignore_cancel=True
        try:
            if self.machine.state not in (State.WAIT,State.FAULT,State.PARKED,State.CHARGED):Motion(self).stop()
        finally:
            if self.perception:self.perception.close()
            self.link.close();self.timer.cancel()
            with self.event_guard:self.events.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mavproxy',default='udpin:127.0.0.1:14551');p.add_argument('--sysid',type=int,default=1)
    p.add_argument('--estacoes',default=str(DEFAULT_STATIONS));p.add_argument('--station')
    p.add_argument('--namespace',default='/rover_argo_N1/Instance1')
    p.add_argument('--cloud',default='/rover_slam_3d/local_cloud');p.add_argument('--odom',default='/fast_lio/odometry')
    charge=p.add_mutually_exclusive_group()
    charge.add_argument('--simulate-charge',dest='simulate_charge',action='store_true',help='Padrao: recarga operacional SITL de 15 s')
    charge.add_argument('--no-simulate-charge',dest='simulate_charge',action='store_false')
    p.set_defaults(simulate_charge=True)
    p.add_argument('--initial-soc',type=float,default=50.)
    p.add_argument('--minimum-soc',type=float,default=30.);p.add_argument('--discharge-rate',type=float,default=.02)
    p.add_argument('--speed',type=float,default=.15);p.add_argument('--no-qr',action='store_true')
    p.add_argument('--mode-timeout',type=float,default=5.);p.add_argument('--docking-timeout',type=float,default=240.)
    p.add_argument('--phase-timeout',type=float,default=120.);p.add_argument('--exit-timeout',type=float,default=180.)
    p.add_argument('--pose-timeout',type=float,default=30.);p.add_argument('--sensor-recovery',type=float,default=10.)
    p.add_argument('--envelope');p.add_argument('--output-dir')
    p.add_argument('--recovery',default=str(Path(__file__).resolve().parents[2]/'ros2/run/mission_manager_recovery.json'))
    a=p.parse_args()
    if not a.mavproxy.startswith('udpin:127.0.0.1:') or not 1<=a.sysid<=255 or not 0<a.speed<=.25 or not all(math.isfinite(v) and 0<v<=600 for v in (a.mode_timeout,a.docking_timeout,a.phase_timeout,a.exit_timeout,a.pose_timeout,a.sensor_recovery)):
        p.error('Parametros de comunicacao/velocidade/timeout invalidos')
    lock=None
    if os.environ.get('ROVER_DOCKING_LOCKED')!='1':
        path=Path(__file__).resolve().parents[2]/'ros2/run/docking-controller.lock';path.parent.mkdir(parents=True,exist_ok=True)
        lock=path.open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ros.init_node('mission_manager')
    manager=Manager(a)
    def stop(sig,frame):manager.shutting=True;manager.cancel.set()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:manager.run()
    finally:ros.signal_shutdown('Mission Manager encerrado')

if __name__=='__main__':main()
