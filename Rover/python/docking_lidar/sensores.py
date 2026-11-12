"""Assinatura MID360 e FAST-LIO; sem camera e sem mudanca de missao."""
import math
import sys
import threading
import time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros

from common.ros_cloud import cloud_xyz,subscribe_cloud
from geometria import estimar
from referencia import prior_estacao

class Sensores:
    def __init__(self,topic,odom_topic='/fast_lio/odometry',wall_timeout=1.):
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        self.lock=threading.Lock();self.cloud=None;self.odom=None
        self.wall_timeout=wall_timeout
        self.cloud_wall=0.;self.odom_wall=0.
        self.topic=topic;self.clock_value=None;self.clock_wall=0.
        self.clock_reset=False
        self.use_sim_time=ros.get_param('/use_sim_time',False)
        self.csub=ros.Subscriber('/clock',Clock,self.clock_callback,queue_size=1) if self.use_sim_time else None
        self.sub=subscribe_cloud(ros,topic,self.cloud_callback,15)
        self.osub=ros.Subscriber(odom_topic,Odometry,self.odom_callback,queue_size=1)
    def clock_callback(self,m):
        with self.lock:
            value=ros.seconds(m.clock)
            if self.clock_value is not None and value < self.clock_value:
                self.cloud=self.odom=None
                self.clock_reset=True
            if value!=self.clock_value:self.clock_wall=time.monotonic()
            self.clock_value=value
    def cloud_callback(self,m):
        with self.lock:self.cloud=m;self.cloud_wall=time.monotonic()
    def odom_callback(self,m):
        with self.lock:self.odom=m;self.odom_wall=time.monotonic()
    def close(self):
        self.sub.unregister();self.osub.unregister()
        if self.csub:self.csub.unregister()
    def fresh_cloud(self):
        """Check delivery AND simulator time; an advertised topic is not a sensor."""
        now=time.monotonic()
        with self.lock:
            msg,wall=self.cloud,self.cloud_wall
            clock_wall=self.clock_wall
        if getattr(self,'clock_reset',False):
            raise ValueError('Relogio /clock reiniciado; reinicie a rotina de docking para descartar a referencia anterior')
        if self.use_sim_time and (not clock_wall or now-clock_wall>1.):
            raise ValueError('Relogio /clock sem avancar: Gazebo pausado, encerrado ou sem conexao. '
                             'Reinicie o Gazebo Harmonic se encerrou; confira ros2 topic hz /clock')
        if msg is None or now-wall>getattr(self,'wall_timeout',1.):
            raise ValueError('MID360 sem mensagens recentes em %s (conexoes: %d). '
                             'Confira o Gazebo Harmonic e ros2 topic hz %s; '
                             'um topico listado nao garante publicacao' %
                             (self.topic,self.sub.get_num_connections(),self.topic))
        age=ros.age(msg.header.stamp)
        if age<-.05 or age>1.:raise ValueError('Timestamp da nuvem invalido (idade %.3f s)'%age)
        if msg.header.frame_id not in ('mid360_sensor','body'):
            raise ValueError('Frame MID360 inesperado: '+msg.header.frame_id)
        return msg
    def wait_cloud(self,seconds=10,poll=lambda:None):
        """Wall-clock timeout continues even if Gazebo stops publishing /clock."""
        deadline=time.monotonic()+seconds;last='ROS encerrado'
        while not ros.is_shutdown() and time.monotonic()<deadline:
            poll()
            try:return self.fresh_cloud()
            except ValueError as e:last=str(e)
            time.sleep(.05)
        raise ValueError(last)
    def pose(self,link,station,msg=None,max_points=6000,executor=None):
        now=time.monotonic();pos=link.latest.get('GLOBAL_POSITION_INT');gps=link.latest.get('GPS_RAW_INT')
        if (not pos or not gps or now-link.last_seen.get('GLOBAL_POSITION_INT',0)>3 or
            now-link.last_seen.get('GPS_RAW_INT',0)>3 or gps.fix_type<3 or pos.hdg==65535):
            raise ValueError('GPS/rumo atual indisponivel para associar a estacao')
        msg=self.fresh_cloud() if msg is None else msg
        prior=prior_estacao(pos.lat/1e7,pos.lon/1e7,pos.hdg/100.,station)
        if math.hypot(prior[0],prior[1])>=8.5:raise ValueError('Estacao fora da regiao de associacao')
        dimensions=station.get('dimensoes_internas',[1.,1.,1.])
        base_height=station.get('altura_base_rover',.15)
        if executor is None:
            result=estimar(cloud_xyz(msg),prior,corrected=True,
                           dimensions=dimensions,base_height=base_height,max_points=max_points)
        else:
            from mission_manager.geometry_worker import fit_and_sweep
            result,points=executor.submit(fit_and_sweep,cloud_xyz(msg),prior,
                                         dimensions,base_height,max_points).result(timeout=5.)
            result['_sweep_points']=points
        result['stamp']=ros.seconds(msg.header.stamp)
        return result
    def lio(self):
        with self.lock:m,wall=self.odom,self.odom_wall
        if getattr(self,'clock_reset',False):
            raise ValueError('Relogio /clock reiniciado; reinicie a rotina de docking')
        if not m or time.monotonic()-wall>getattr(self,'wall_timeout',1.):raise ValueError('Odometria FAST-LIO ausente ou desatualizada')
        age=ros.age(m.header.stamp)
        if age<-.05 or age>1.:raise ValueError('Timestamp FAST-LIO invalido')
        if m.header.frame_id != 'camera_init' or m.child_frame_id != 'body':
            raise ValueError('Odometria deve usar camera_init -> body; nao substitua por mapa global com saltos de loop closure')
        p=m.pose.pose.position;q=m.pose.pose.orientation
        v=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w],float)
        cov=np.asarray(m.pose.covariance,dtype=float).reshape(6,6)
        if not np.isfinite(v).all() or np.max(np.abs(v[:3]))>10000 or not np.isfinite(cov).all():
            raise ValueError('FAST-LIO inconsistente: posicao/covariancia invalida')
        diagonal=np.diag(cov)[:3]
        if np.any(diagonal<0) or np.any(diagonal>.25) or not np.any(diagonal>0):
            raise ValueError('FAST-LIO inconsistente: covariancia de posicao excessiva')
        if not .95 <= np.linalg.norm(v[3:]) <= 1.05:raise ValueError('Orientacao FAST-LIO invalida')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return np.array([p.x,p.y,p.z],float),yaw
