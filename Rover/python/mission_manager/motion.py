"""Reusable bounded docking operations; no missions or background MAVLink owner."""
import math
import time
from common import ros2_runtime as ros
from common.ros_cloud import cloud_xyz
from docking.visual_alignment import VisualAlignment, deve_frear
from docking.controle_saida import alinhamento
from docking_lidar.referencia import recuo_lio
from mission_manager.geometry import obstacle_in_sweep, reposition_route, acquisition_heading
from mission_manager.policy import State
from geometria import pontos_3d
from referencia import prior_estacao

STEERING={'ATC_STR_RAT_FF':.7,'ATC_STR_RAT_P':.7,'ATC_STR_RAT_I':0.}
EXIT_DISTANCE=3.0  # displacement from the parked starting position, not the doorway

class Motion:
    def __init__(self, owner):
        self.o=owner;self.link=owner.link;self.envelope=owner.envelope
        self.saved={};self.phase_wall=0.

    def check(self, guided=True):
        self.o.check_cancel()
        if not self.o.fresh():raise RuntimeError('Telemetria indisponivel')
        hb=self.link.latest['HEARTBEAT']
        if guided and (hb.custom_mode!=15 or not hb.base_mode&128):
            raise RuntimeError('GUIDED ou armamento interrompido')

    def command(self, speed, yaw):
        self.link.conn.mav.set_position_target_local_ned_send(0,self.link.sysid,1,8,
            1|2|4|64|128|256|1024,0,0,0,speed,0,0,0,0,0,0,yaw)

    def drain(self, seconds=.05):
        until=time.monotonic()+seconds
        while time.monotonic()<until:self.link.recv(.002)
        self.o.publish()

    def pose(self, timeout=None, final=False):
        deadline=time.monotonic()+(self.o.args.pose_timeout if timeout is None else timeout);good=[];last='sem nuvem'
        while time.monotonic()<deadline:
            self.drain();self.check(False)
            try:
                p,points=self.o.perception.snapshot()
                if good and p['stamp']<=good[-1]['stamp']:continue
                if final and not self.envelope.final(p,self.o.station):
                    good=[];last='fora das tolerancias finais';continue
                good.append(p);good=good[-5:]
                if len(good)>=3 and good[-1]['stamp']-good[0]['stamp']>=.4-1e-6:
                    if max(x['forward'] for x in good)-min(x['forward'] for x in good)<.06 and max(x['right'] for x in good)-min(x['right'] for x in good)<.06:
                        return p
            except ValueError as exc:last=str(exc);good=[]
            time.sleep(.02)
        raise RuntimeError('Confirmacao geometrica falhou: '+last)

    def stop(self, disarm=False):
        self.command(0,0);self.link.mode('HOLD')
        deadline=time.monotonic()+15;stable=None
        while time.monotonic()<deadline:
            self.drain();self.check(False)
            try:speed,yaw=self.o.motion_state()
            except RuntimeError as exc:
                if not str(exc).startswith('Velocidade/atitude desatualizada'):raise
                # HOLD is already confirmed. Keep zero commands and wait for
                # fresh samples within the existing stop timeout.
                stable=None;self.command(0,0);continue
            if speed<.04 and abs(yaw)<.03:
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>=.7:break
            else:stable=None
        else:raise RuntimeError('Parada nao confirmada')
        if disarm:self.link.disarm()

    def tune(self):
        for key in ('OA_TYPE','AVOID_ENABLE',*STEERING):self.saved[key]=self.link.param(key)
        self.o.save_recovery(self.saved)
        for key,value in STEERING.items():self.link.param(key,value)

    def allow_inside(self):
        self.link.param('OA_TYPE',0)
        self.link.param('AVOID_ENABLE',int(self.saved['AVOID_ENABLE'])&~2)

    def restore(self):
        for key,value in self.saved.items():self.link.param(key,value)
        self.o.clear_recovery()

    def reposition(self, initial):
        route=reposition_route(initial,self.o.station,self.envelope)
        self.o.event(kind='reposition_route',points=route)
        orient=VisualAlignment(self.o.args.speed,pivot_speed_tolerance=.10)
        deadline=time.monotonic()+self.o.args.phase_timeout
        for gx,gy in route:
            orient.turning=True
            while time.monotonic()<deadline:
                self.drain();self.check()
                raw,points=self.o.perception.snapshot()
                along,lat,h=alinhamento(raw)
                dx,dy=gx+along,gy+lat
                distance=math.hypot(dx,dy)
                if distance<.08:self.command(0,0);break
                error=(math.atan2(dy,dx)-h+math.pi)%(2*math.pi)-math.pi
                speed,rate=self.o.motion_state();raw.update(groundspeed=speed,yawrate=rate)
                ready,w=orient._orient(error,raw,raw['stamp'])
                v=min(.12,self.o.args.speed,max(.035,distance*.25)) if ready else 0.
                if obstacle_in_sweep(points,v,w,self.envelope):raise RuntimeError('Reposicionamento bloqueado por obstaculo')
                self.command(v,w)
            else:raise RuntimeError('Timeout de reposicionamento externo')
        self.stop();return self.pose()

    def acquire(self):
        try:return self.pose(timeout=3.)
        except RuntimeError as exc:
            self.o.event(kind='acquisition',reason=str(exc))
        self.link.mode('GUIDED')
        deadline=time.monotonic()+self.o.args.phase_timeout
        stable=None
        while time.monotonic()<deadline:
            self.drain();self.check()
            gps=self.link.latest.get('GPS_RAW_INT');pos=self.link.latest['GLOBAL_POSITION_INT']
            if not gps or gps.fix_type<3 or time.monotonic()-self.link.last_seen.get('GPS_RAW_INT',0)>3 or pos.hdg==65535:
                raise RuntimeError('GPS/rumo indisponivel para orientar a aquisicao')
            prior=prior_estacao(pos.lat/1e7,pos.lon/1e7,pos.hdg/100.,self.o.station)
            error=acquisition_heading(prior,self.o.station,self.envelope)
            points=self.o.perception.sweep()
            if len(points)<100:raise RuntimeError('Varredura insuficiente para verificar giro externo')
            speed,rate=self.o.motion_state()
            w=max(-.20,min(.20,.8*error-.35*rate)) if speed<.10 else 0.
            if obstacle_in_sweep(points,0,w,self.envelope):raise RuntimeError('Aquisicao bloqueada por obstaculo no giro')
            if abs(error)<math.radians(4) and abs(rate)<.03:
                w=0.
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>.5:break
            else:stable=None
            self.command(0,w)
        else:raise RuntimeError('Timeout de orientacao para aquisicao LiDAR')
        self.stop()
        # GPS never authorizes forward motion. Require stable measured walls.
        return self.pose()

    def dock(self):
        station=self.o.station;self.envelope.validate(station)
        self.stop();self.tune();initial=self.acquire()
        along,lat,h=alinhamento(initial)
        if self.envelope.final(initial,station):
            self.link.disarm();self.o.machine.confirmed_parked();return
        if math.hypot(along,lat)>=8:raise RuntimeError('Estacao fora dos 8 m')
        self.link.mode('GUIDED')
        self.o.machine.move(State.ALIGN,'Reposicionar no eixo externo usando geometria LiDAR')
        if along<=.75 or abs(lat)>3.:
            self.reposition(initial)
            self.link.mode('GUIDED')
        controller=VisualAlignment(self.o.args.speed,staging_distance=max(1.2,station['dimensoes_internas'][0]/2+self.envelope.length+.1),allow_rear_target=True,pivot_speed_tolerance=.10)
        deadline=time.monotonic()+self.o.args.docking_timeout
        lost=None;previous=None;last_phase=None;phase_start=time.monotonic();inside_enabled=False
        try:
            while time.monotonic()<deadline:
                self.drain();self.check()
                try:
                    raw,points=self.o.perception.snapshot()
                    pose=self.envelope.target(raw)
                except ValueError as exc:
                    self.command(0,0)
                    if lost is None:lost=time.monotonic()
                    if time.monotonic()-lost>self.o.args.sensor_recovery:raise RuntimeError('Posicao LiDAR perdida: '+str(exc))
                    continue
                lost=None
                if previous and pose['stamp']==previous['stamp']:
                    # Do not integrate the control/filter twice on the same cloud.
                    continue
                if previous:
                    dt=pose['stamp']-previous['stamp']
                    if dt<=0:
                        self.command(0,0)
                        continue
                    # Compare displacement in STATION axes: a pivot rotates
                    # the body-relative vector even with no translation.
                    pa,pl,_=alinhamento(previous);a,l,_=alinhamento(pose)
                    if math.hypot(a-pa,l-pl)>.1+1.5*dt:
                        self.o.event(kind='pose_jump',previous=previous,current=pose,dt=dt)
                        raise RuntimeError('Salto de pose LiDAR; HOLD para reassociar')
                previous=pose
                speed,rate=self.o.motion_state();pose.update(groundspeed=speed,yawrate=rate)
                if controller.phase=='entrada' and deve_frear(pose,speed):
                    self.stop();self.pose(final=True);self.link.disarm()
                    self.o.machine.confirmed_parked();return
                phase,v,w,done=controller.step(pose,now=pose['stamp'])
                if phase!=last_phase:phase_start=time.monotonic();last_phase=phase
                if time.monotonic()-phase_start>self.o.args.phase_timeout:raise RuntimeError('Timeout da fase '+phase)
                desired=State.APPROACH if phase=='entrada' else State.ALIGN
                self.o.machine.move(desired,'Controlador: '+phase)
                if phase=='entrada' and not inside_enabled:
                    self.command(0,0);self.allow_inside();inside_enabled=True
                if not self.envelope.corridor(raw,station):raise RuntimeError('Envelope fora do corredor de entrada')
                if obstacle_in_sweep(points,v,w,self.envelope):raise RuntimeError('Obstaculo na faixa de movimento')
                if done:
                    self.stop();self.pose(final=True);self.link.disarm();self.o.machine.confirmed_parked();return
                self.command(v,w)
                self.o.event(kind='control',phase=phase,speed=v,yawrate=w,pose=pose)
            raise RuntimeError('Timeout total do docking')
        finally:
            self.command(0,0)
            # Caller handles HOLD on errors. Restore only after no longer moving.

    def exit_observation(self, guided=True):
        """Zero motion during transient delivery lag; never use stale odometry."""
        deadline=time.monotonic()+self.o.args.sensor_recovery
        reported=False
        while True:
            self.drain();self.check(guided)
            try:
                position,yaw=self.o.perception.sensor.lio()
                points=self.o.perception.sweep()
                return position,yaw,points
            except ValueError as exc:
                self.command(0,0)
                # Clock resets, invalid covariance and missing geometry are not
                # ordinary latency: those require explicit reconciliation.
                if str(exc) not in ('Timestamp FAST-LIO invalido',
                                    'Odometria FAST-LIO ausente ou desatualizada',
                                    'Varredura de protecao desatualizada'):
                    raise
                if not reported:
                    self.o.event(kind='exit_sensor_wait',reason=str(exc));reported=True
                if time.monotonic()>=deadline:
                    raise RuntimeError('Saida parada: sensores nao recuperaram: '+str(exc)) from exc

    def exit(self):
        if not self.o.machine.pending:raise RuntimeError('Saida sem solicitacao de missao')
        self.link.mode('GUIDED')
        initial=self.pose()
        if not self.envelope.final(initial,self.o.station):raise RuntimeError('Rover nao confirma docking para saida')
        self.tune();self.allow_inside()
        # Require a fresh odometry + protective scan before arming.
        self.exit_observation(guided=False)
        self.link.command(400,1)
        self.link.wait('HEARTBEAT',lambda h:h.get_srcComponent()==1 and h.base_mode&128)
        self.check()
        origin,yaw0,_=self.exit_observation();end=time.monotonic()+self.o.args.exit_timeout;last_back=0.
        initial_along,initial_side,initial_heading=alinhamento(initial)
        previous=initial;lost=None
        while time.monotonic()<end:
            self.drain();self.check()
            position,yaw,points=self.exit_observation();back,side,error,turn=recuo_lio(origin,position,yaw0,yaw)
            if abs(side)>.12 or abs(error)>math.radians(12) or back<-.1 or back-last_back>.2:
                raise RuntimeError('Saida perdeu alinhamento/odometria')
            last_back=back
            # Station-relative walls measure the requested distance directly.
            # LIO remains an independent alignment/direction check; its accumulated
            # translation can drift even with fresh timestamps and low covariance.
            try:measured,_=self.o.perception.snapshot()
            except ValueError as exc:
                self.command(0,0)
                if lost is None:lost=time.monotonic()
                if time.monotonic()-lost>self.o.args.sensor_recovery:
                    raise RuntimeError('Geometria perdida durante a saida: '+str(exc)) from exc
                continue
            lost=None
            dt=measured['stamp']-previous['stamp']
            if dt<=0:
                self.command(0,0);continue
            along,lateral,heading=alinhamento(measured)
            pa,pl,_=alinhamento(previous)
            if math.hypot(along-pa,lateral-pl)>.1+1.5*dt:
                raise RuntimeError('Salto de geometria durante a saida')
            previous=measured
            geometric_back=along-initial_along
            if (geometric_back<-.1 or abs(lateral-initial_side)>.12
                    or abs(heading-initial_heading)>math.radians(12)):
                raise RuntimeError('Saida perdeu alinhamento com as paredes')
            self.o.event(kind='exit_reverse',back=back,geometric_back=geometric_back,
                         side=side,heading_error=error,target=EXIT_DISTANCE)
            if geometric_back>=EXIT_DISTANCE-.04:break
            v=-min(.12,max(.03,(EXIT_DISTANCE-geometric_back)*.25))
            if obstacle_in_sweep(points,v,turn,self.envelope):raise RuntimeError('Obstaculo atras do rover')
            self.command(v,turn)
        else:raise RuntimeError('Timeout de recuo')
        self.stop()
        pose=self.pose()
        if not self.envelope.outside(pose,self.o.station,rotation=True):raise RuntimeError('Rover ainda intersecta a estacao')
        # A LiDAR geometric observation confirms clearance independently of LIO.
        final_along,final_side,final_heading=alinhamento(pose)
        geometric_back=final_along-initial_along
        final,yaw,_=self.exit_observation(guided=False)
        back,side,error,_=recuo_lio(origin,final,yaw0,yaw)
        clearance=alinhamento(pose)[0]-math.hypot(self.envelope.length,self.envelope.width)/2
        self.o.event(kind='exit_check',strategy='reverse_only',target=EXIT_DISTANCE,
                     back=back,geometric_back=geometric_back,side=side,heading_error=error,clearance=clearance)
        if (back<=.1 or abs(geometric_back-EXIT_DISTANCE)>.15
                or abs(side)>.12 or abs(error)>math.radians(12)
                or abs(final_side-initial_side)>.12
                or abs(final_heading-initial_heading)>math.radians(12)):
            raise RuntimeError('Recuo de 3 m/alinhamento final nao confirmado pelo LiDAR e FAST-LIO')
        # No pivot here. The manager restores parameters and starts AUTO only
        # after this stopped, independently checked outside pose is confirmed.
        self.o.machine.confirmed_outside()
        self.o.event(kind='exit_confirmed',strategy='reverse_only',back=back,
                     geometric_back=geometric_back,side=side,clearance=clearance)
