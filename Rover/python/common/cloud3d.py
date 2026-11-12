"""3D geometry shared by the simulated MID360, map and docking. No ROS imports."""
import math
import numpy as np


def corrected_mid360(xyz, min_range=.35, max_range=50.):
    """Gazebo Livox rays begin at 0.10 m: returns omit that ray-start offset."""
    xyz=np.asarray(xyz,dtype=float).reshape(-1,3)
    d=np.linalg.norm(xyz,axis=1)
    valid=np.isfinite(xyz).all(axis=1)&(d>.001)
    xyz=xyz[valid];d=d[valid]
    xyz=xyz*(1+.10/d[:,None]);d=d+.10
    return xyz[(d>=min_range)&(d<=max_range)]


def rotation_xyzw(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or not .95<=np.linalg.norm(q)<=1.05:
        raise ValueError('Quaternion LiDAR invalido')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def checked_pose(message):
    p=message.pose.pose.position;q=message.pose.pose.orientation
    position=np.array([p.x,p.y,p.z],float)
    cov=np.asarray(message.pose.covariance,dtype=float).reshape(6,6)
    if (not np.isfinite(position).all() or np.max(np.abs(position))>10000 or
            not np.isfinite(cov).all() or np.any(np.diag(cov)<0) or
            np.any(np.diag(cov)[:3]>.25) or not np.any(np.diag(cov)[:3]>0) or
            np.any(np.diag(cov)[3:]>math.radians(15)**2)):
        raise ValueError('FAST-LIO: posicao/covariancia inconsistente')
    return position,rotation_xyzw([q.x,q.y,q.z,q.w])


def voxel_points(xyz,size):
    xyz=np.asarray(xyz,dtype=float).reshape(-1,3)
    xyz=xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):return xyz
    keys=np.floor(xyz/size).astype(np.int64)
    # Compare packed integer rows, avoiding slow structured-field comparisons.
    # Equality is unchanged, including negative coordinates. Sorting original
    # indices retains the same first point per voxel and the same point order.
    rows=np.ascontiguousarray(keys).view(np.dtype((np.void,3*keys.dtype.itemsize))).ravel()
    _,indices=np.unique(rows,return_index=True)
    return xyz[np.sort(indices)]


class LocalCloud:
    """Short rolling world clouds, transformed to the SAME current IMU frame.

    Never concatenate clouds in moving sensor frames. Reset on time reversal,
    teleport, invalid frame or invalid covariance before publishing to docking.
    """
    def __init__(self,window=1.5,voxel=.025):
        from collections import deque
        self.window=window;self.voxel=voxel;self.history=deque();self.previous=None

    def add(self,stamp,world,position,rotation):
        if not math.isfinite(stamp) or stamp<=0:raise ValueError('Timestamp invalido')
        if self.previous is not None:
            t,p,r=self.previous;dt=stamp-t
            angle=math.acos(np.clip((np.trace(r.T@rotation)-1)/2,-1,1))
            if dt<=0 or dt>1. or np.linalg.norm(position-p)>.03+2*dt or angle>.05+2*dt:
                self.clear()
                raise ValueError('Descontinuidade FAST-LIO: buffer 3D limpo')
        self.previous=(stamp,position.copy(),rotation.copy())
        self.history.append((stamp,np.asarray(world,dtype=float)))
        while self.history and self.history[0][0]<stamp-self.window:self.history.popleft()
        points=np.concatenate([x for _,x in self.history])
        local=(points-position)@rotation
        local=local[np.isfinite(local).all(axis=1)&(np.linalg.norm(local,axis=1)<10)]
        return voxel_points(local,self.voxel)

    def clear(self):
        self.history.clear();self.previous=None

    def invalidate_window(self):
        # Expire points while retaining the time/pose discontinuity guard.
        self.history.clear()
