"""Common station frame, vehicle envelope and conservative sensor fusion."""
from dataclasses import dataclass
import math
import numpy as np
from docking.controle_saida import alinhamento

@dataclass
class Envelope:
    length: float = .60
    width: float = .54
    height: float = .65
    margin: float = .06
    # Base link to desired body center, station axes. Zero for current rover.
    base_to_center_x: float = 0.
    base_to_center_y: float = 0.

    def validate(self, station):
        dims = station['dimensoes_internas']
        values=(self.length,self.width,self.height,self.margin,self.base_to_center_x,self.base_to_center_y)
        if not all(math.isfinite(v) for v in values) or min(values[:4])<=0:
            raise ValueError('Envelope invalido')
        if any(a+2*self.margin>b for a,b in zip(values[:3],dims)):
            raise ValueError('Rover/envelope nao cabe na estacao')
        if abs(self.base_to_center_x)+self.length/2+self.margin>dims[0]/2 or abs(self.base_to_center_y)+self.width/2+self.margin>dims[1]/2:
            raise ValueError('Deslocamento do alvo nao cabe na estacao')

    def target(self, pose):
        p=dict(pose); h=p['normal_yaw']
        p['forward']-=self.base_to_center_x*math.cos(h)+self.base_to_center_y*math.sin(h)
        p['right']+=self.base_to_center_x*math.sin(h)-self.base_to_center_y*math.cos(h)
        return p

    def final(self, pose, station):
        along,lat,h=alinhamento(self.target(pose))
        return abs(along)<=.07 and abs(lat)<=.07 and abs(h)<=math.radians(5) and self.fits(pose,station)

    def fits(self, pose, station):
        along,lat,h=alinhamento(pose)
        half_l=abs(math.cos(h))*self.length/2+abs(math.sin(h))*self.width/2
        half_w=abs(math.sin(h))*self.length/2+abs(math.cos(h))*self.width/2
        return abs(along)+half_l+self.margin<=station['dimensoes_internas'][0]/2 and abs(lat)+half_w+self.margin<=station['dimensoes_internas'][1]/2

    def outside(self, pose, station, rotation=False):
        along,lat,h=alinhamento(pose)
        radius=math.hypot(self.length,self.width)/2 if rotation else abs(math.cos(h))*self.length/2+abs(math.sin(h))*self.width/2
        return along-radius>station['dimensoes_internas'][0]/2+self.margin

    def corridor(self, pose, station):
        along,lat,h=alinhamento(pose)
        if along>station['dimensoes_internas'][0]/2+self.length/2+self.margin:
            return True
        half_w=abs(math.sin(h))*self.length/2+abs(math.cos(h))*self.width/2
        return abs(lat)+half_w+self.margin<station['dimensoes_internas'][1]/2


def fuse(lidar, qr, now_stamp):
    """Both poses are station center in base FRD, same yaw sign, seconds ROS.

    LiDAR is mandatory. QR only adds a capped innovation after identity/time/
    residual checks. Never blend poses from different epochs while turning.
    """
    p=dict(lidar)
    if not all(math.isfinite(p[k]) for k in ('forward','right','normal_yaw','stamp','rms')):
        raise ValueError('LiDAR nao finito')
    if not -.05<=now_stamp-p['stamp']<=.6 or p['rms']>.035 or p.get('n',0)<35:
        raise ValueError('LiDAR desatualizado ou sem qualidade: idade=%.3fs rms=%.4fm n=%d' % (now_stamp-p['stamp'],p['rms'],p.get('n',0)))
    p['fusion']='lidar_only'
    if qr is None:return p
    if not all(math.isfinite(qr.get(k,float('nan'))) for k in ('forward','right','normal_yaw','stamp','reprojection_px')):
        p['fusion']='qr_invalid';return p
    # Runtime aligns timestamp by a stationary gate; otherwise only diagnosis.
    delta=math.hypot(qr['forward']-p['forward'],qr['right']-p['right'])
    dh=(qr['normal_yaw']-p['normal_yaw']+math.pi)%(2*math.pi)-math.pi
    if not qr.get('identity_ok') or abs(qr['stamp']-p['stamp'])>.12 or qr['reprojection_px']>1. or not qr.get('stationary',False):
        p['fusion']='qr_not_trusted';return p
    if delta>.15 or abs(dh)>math.radians(8):
        p['fusion']='qr_divergent';return p
    weight=min(.15, .15*(1.-min(1.,qr['reprojection_px'])))
    for k in ('forward','right'):
        p[k]+=max(-.015,min(.015,weight*(qr[k]-p[k])))
    p['normal_yaw']+=max(-math.radians(.5),min(math.radians(.5),weight*dh))
    p['fusion']='lidar_qr'
    return p


def obstacle_in_sweep(points, speed, yawrate, envelope, pose=None, station=None):
    """Short swept box incl. braking distance; input XYZ in base FLU.

    Reject floor/self; station surfaces remain real obstacles, no blanket mask.
    """
    xyz=np.asarray(points).reshape(-1,3)
    xyz=xyz[np.isfinite(xyz).all(axis=1)]
    xyz=xyz[(xyz[:,2]>.08)&(xyz[:,2]<envelope.height+.08)]
    # Current body is excluded with only 2 cm sensor/self-return tolerance.
    xyz=xyz[(np.abs(xyz[:,0])>envelope.length/2+.02)|(np.abs(xyz[:,1])>envelope.width/2+.02)]
    if not len(xyz):return False
    if abs(yawrate)>.02:
        radius=math.hypot(envelope.length,envelope.width)/2+.035
        return np.count_nonzero(np.linalg.norm(xyz[:,:2],axis=1)<radius)>=4
    if abs(speed)<.001:return False
    reach=envelope.length/2+.06+abs(speed)*.35+speed**2/(2*.3)
    x=xyz[:,0]*(1 if speed>0 else -1)
    return np.count_nonzero((x>0)&(x<reach)&(np.abs(xyz[:,1])<envelope.width/2+.04))>=4


def reposition_route(pose, station, envelope):
    """Outside-only route around the inflated station, in station XY (FRD).

    Never plans a segment through the box. The live LiDAR sweep remains a
    separate veto for other scene obstacles; this is not a global map planner.
    """
    along,lat,_=alinhamento(pose)
    x,y=-along,-lat
    radius=math.hypot(envelope.length,envelope.width)/2+envelope.margin+.20
    hx=station['dimensoes_internas'][0]/2+radius
    hy=station['dimensoes_internas'][1]/2+radius
    stage=max(1.3,hx+.2)
    if abs(x)<hx and abs(y)<hy:
        raise ValueError('Sem margem externa para reposicionamento automatico')
    if x<=-hx:return [(-stage,0.)]
    lane=math.copysign(max(hy,abs(y)),y if abs(y)>.01 else 1.)
    route=[]
    if abs(y)<hy:route.append((x,lane))
    route.extend([(-stage,lane),(-stage,0.)])
    return route


def acquisition_heading(prior, station, envelope):
    """GPS is allowed only to orient outside; never to complete docking.

    Return BODY_NED yaw error. A generous external margin protects the station
    before LiDAR association, and the live raw sweep independently vetoes turn.
    prior is FLU forward/left/station yaw from prior_estacao.
    """
    f,left,h=prior
    if not all(math.isfinite(v) for v in prior):raise ValueError('Referencia GPS invalida')
    along=f*math.cos(h)+left*math.sin(h)
    lateral=-f*math.sin(h)+left*math.cos(h)
    minimum=max(2.,station['dimensoes_internas'][0]/2+math.hypot(envelope.length,envelope.width)/2+.75)
    if along<minimum or abs(lateral)>3 or math.hypot(f,left)>=8:
        raise ValueError('Sem margem frontal para aquisicao da estacao por giro')
    return math.atan2(-left,f)
