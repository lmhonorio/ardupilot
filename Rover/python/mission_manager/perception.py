"""Asynchronous LiDAR fitting and optional calibrated QR observations."""
import math
from pathlib import Path
import sys
import threading
import time
import numpy as np
from common import ros2_runtime as ros
from common.ros_cloud import cloud_xyz
from common.docking_camera import DockingCamera
from common.qr_detector import QRDetector
from docking.relative_pose import solve
from docking.qr_refinement import QRRefiner
from mission_manager.geometry import fuse
# Retain compatibility with the existing standalone LiDAR scripts.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking_lidar'))
from sensores import Sensores
from geometria import pontos_3d

class Perception:
    def __init__(self, link, namespace, cloud, odom, no_qr=False):
        self.link=link
        self.sensor=Sensores(cloud,odom,wall_timeout=3.)
        self.raw_sensor=Sensores(namespace+'/livox/lidar',odom,wall_timeout=3.)
        self.station=None;self.lock=threading.Lock();self.done=threading.Event()
        self.lidar=None;self.qr=None;self.points=None
        self.error='Aguardando geometria';self.wall=0.;self.threads=[]
        self.diagnostic={};self.last_dump=0.
        self.camera=None;self.joints=None
        if not no_qr:
            import cv2
            cv2.setNumThreads(1)
            from std_msgs.msg import Float64
            self.camera=DockingCamera(namespace)
            self.joints=ros.JointFeedback(namespace+'/joint_states')
            self.pan=ros.Publisher(namespace+'/camera/pan_cmd',Float64,queue_size=1)
            self.tilt=ros.Publisher(namespace+'/camera/tilt_cmd',Float64,queue_size=1)
            self.detector=QRDetector()
        for method in ([self._lidar,self._visual] if self.camera else [self._lidar]):
            t=threading.Thread(target=method,daemon=True);t.start();self.threads.append(t)

    def select(self, station):
        with self.lock:
            if self.station and station['nome']==self.station['nome']:return
            self.station=station;self.lidar=self.qr=None;self.wall=0.

    def _lidar(self):
        last_stamp=None
        while not self.done.wait(.08):
            with self.lock:station=self.station
            if station is None:continue
            try:
                msg=self.raw_sensor.fresh_cloud()
                stamp=ros.seconds(msg.header.stamp)
                if stamp==last_stamp:continue
                last_stamp=stamp
                # Prefer the current scan: no SLAM accumulation latency. The
                # registered local window is a quality-gated fallback when a
                # single scan lacks enough independent container surfaces.
                try:
                    result=self.raw_sensor.pose(self.link,station,msg,max_points=1200)
                    result['cloud_source']=self.raw_sensor.topic
                except ValueError:
                    local=self.sensor.fresh_cloud()
                    result=self.sensor.pose(self.link,station,local,max_points=1200)
                    result['cloud_source']=self.sensor.topic
                points=pontos_3d(cloud_xyz(msg),corrected=True)
                with self.lock:
                    if station is self.station:
                        if self.lidar is not None and result['stamp']<=self.lidar['stamp']:
                            raise ValueError('Ajuste LiDAR fora de ordem; aguardando uma observacao nova')
                        self.lidar=result;self.points=points;self.wall=time.monotonic();self.error=''
            except Exception as exc:
                with self.lock:self.error=str(exc)
                if time.monotonic()-self.last_dump>15:
                    self.last_dump=time.monotonic()
                    try:
                        msg=self.sensor.fresh_cloud();xyz=cloud_xyz(msg)
                        pos=self.link.latest['GLOBAL_POSITION_INT']
                        diagnostic=dict(reason=str(exc),stamp=ros.seconds(msg.header.stamp),points=len(xyz),
                                        gps=[pos.lat/1e7,pos.lon/1e7,pos.hdg/100],station=station['nome'])
                        with self.lock:self.diagnostic=diagnostic
                        if hasattr(self.link,'owner'):
                            np.savez_compressed(self.link.owner.root/'failed_cloud.npz',xyz=xyz,
                                gps=diagnostic['gps'],stamp=diagnostic['stamp'])
                            self.link.owner.event(kind='lidar_diagnostic',**diagnostic)
                    except Exception:pass

    def _visual(self):
        code=None;refiner=None;last_stamp=None
        while not self.done.wait(.08):
            with self.lock:station=self.station;lp=self.lidar
            if station is None:continue
            try:
                if code!=station['qr']:code=station['qr'];refiner=QRRefiner(code)
                if lp is not None:
                    self.pan.publish(max(-1.2,min(1.2,math.atan2(-lp['right'],max(.1,lp['forward']+.5)))))
                    self.tilt.publish(max(-.55,min(-.1,-math.atan2(.22,max(.2,lp['forward']+.43)))))
                else:self.pan.publish(0.);self.tilt.publish(-.3)
                frame,info,stamp=self.camera.snapshot()
                image_time=ros.seconds(stamp)
                if image_time==last_stamp:continue
                last_stamp=image_time
                pan=self.joints.position('pt_support_s23_pan_joint',stamp)
                tilt=self.joints.position('pt_support_s23_tilt_joint',stamp)
                if abs(pan)>1.35 or not -.65<=tilt<=.05:raise ValueError('Camera fora de calibracao')
                text,corners,_=self.detector.detectAndDecode(frame)
                if text!=code or corners is None:raise ValueError('QR opcional nao identificado')
                corners,_=refiner.refine(frame,corners)
                qr=solve(corners,info.k,info.d,tilt_rad=tilt,pan_rad=pan,
                         wall_offset=station['distancia_alvo_parede_m'],
                         heading_prior=lp['normal_yaw'] if lp else None)
                hud=self.link.latest.get('VFR_HUD');att=self.link.latest.get('ATTITUDE');now=time.monotonic()
                qr.update(stamp=ros.seconds(stamp),identity_ok=True,
                    stationary=bool(hud and att and now-self.link.last_seen.get('ATTITUDE',0)<.5
                                    and now-self.link.last_seen.get('VFR_HUD',0)<.5
                                    and hud.groundspeed<.035 and abs(att.yawspeed)<.025))
                if ros.age(stamp)>.5:raise ValueError('QR expirou')
                with self.lock:
                    if station is self.station:self.qr=qr
            except Exception:
                with self.lock:self.qr=None

    def snapshot(self):
        with self.lock:lp,qr,wall,reason,points=self.lidar,self.qr,self.wall,self.error,self.points
        if lp is None or reason or time.monotonic()-wall>3.:raise ValueError('LiDAR: '+reason)
        return fuse(lp,qr,ros.seconds(ros.now())), points

    def sweep(self):
        msg=self.raw_sensor.fresh_cloud()
        if not -.05<=ros.age(msg.header.stamp)<=.6:
            raise ValueError('Varredura de protecao desatualizada')
        points=pontos_3d(cloud_xyz(msg),corrected=True)
        if len(points)<35:raise ValueError('Varredura de protecao vazia/insuficiente')
        return points

    def close(self):
        self.done.set()
        for t in self.threads:t.join(timeout=3)
        self.sensor.close()
        self.raw_sensor.close()
        if self.camera:self.camera.close();self.joints.close()
