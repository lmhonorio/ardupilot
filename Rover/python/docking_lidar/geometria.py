"""Fit the finite 3D surfaces of the open 1 m container in rover FLU axes.

The rover runs on level ground: yaw is estimated, roll/pitch are assumed small.
Height is measured if roof/floor is visible, otherwise it retains the known
base height. XY/yaw always require back AND a side, never a ground plane alone.
"""
import math
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.cloud3d import corrected_mid360,voxel_points

OFFSET=np.array([-.003,0.,.434])
SIM_MIN_RANGE=.10


def pontos_3d(xyz,corrected=False):
    xyz=np.asarray(xyz,dtype=float).reshape(-1,3)
    if not corrected:xyz=corrected_mid360(xyz,min_range=.2,max_range=10.)
    xyz=xyz[np.isfinite(xyz).all(axis=1)]+OFFSET
    return voxel_points(xyz[(np.linalg.norm(xyz[:,:2],axis=1)>.2)&
                            (np.linalg.norm(xyz,axis=1)<10)],.015)


def pontos_paredes(xyz):
    """Compatibility diagnostic; docking uses full XYZ via pontos_3d."""
    return pontos_3d(xyz)


def local(points,pose):
    x,y,z,theta=pose;c=math.cos(theta);s=math.sin(theta)
    d=points-np.array([x,y,z])
    return np.column_stack((c*d[:,0]+s*d[:,1],-s*d[:,0]+c*d[:,1],d[:,2]))


def superficies(q,dimensions=(1.,1.,1.)):
    """Distances to five finite rectangles: back, left, right, roof, floor."""
    a,b,c=np.asarray(dimensions)/2
    x,y,z=q.T
    ex=np.maximum(np.abs(x)-a,0);ey=np.maximum(np.abs(y)-b,0);ez=np.maximum(np.abs(z)-c,0)
    return np.column_stack((np.sqrt((x-a)**2+ey**2+ez**2),
        np.sqrt((y-b)**2+ex**2+ez**2),np.sqrt((y+b)**2+ex**2+ez**2),
        np.sqrt((z-c)**2+ex**2+ey**2),np.sqrt((z+c)**2+ex**2+ey**2)))


def estimar(xyz,prior,corrected=False,dimensions=(1.,1.,1.),base_height=.15,max_points=6000):
    prior=np.asarray(prior,dtype=float);dims=np.asarray(dimensions,dtype=float)
    if prior.shape!=(3,) or not np.isfinite(prior).all() or dims.shape!=(3,) or not np.isfinite(dims).all() or np.any(dims<=0):
        raise ValueError('Referencia/dimensoes invalidas')
    points=pontos_3d(xyz,corrected)
    seed=np.array([prior[0],prior[1],dims[2]/2-base_height,prior[2]])
    points=points[np.min(superficies(local(points,seed),dims),axis=1)<.35]
    if len(points)<35:
        raise ValueError('Nuvem 3D insuficiente perto do container: %d pontos; aguarde acumulacao registrada FAST-LIO'%len(points))
    if not 200<=max_points<=6000:raise ValueError('Limite de pontos de ajuste invalido')
    if len(points)>max_points:points=points[::int(math.ceil(len(points)/max_points))]
    margin=np.array([.45,.45,.20,math.radians(25)])
    def residual(p):
        return np.r_[np.min(superficies(local(points,p),dims),axis=1),.05*(p[2]-seed[2])]
    # Robust bounded Gauss-Newton using NumPy, compatible with both Python 3.8
    # installations here (system SciPy is incompatible with user NumPy).
    current=seed.copy()
    def cost(r):return np.sum(2*.025**2*(np.sqrt(1+(r/.025)**2)-1))
    for _ in range(70):
        r=residual(current);w=1/np.sqrt(1+(r/.025)**2)
        jac=np.column_stack([(residual(current+np.eye(4)[j]*1e-5)-r)/1e-5 for j in range(4)])
        step=np.linalg.solve(jac.T@(w[:,None]*jac)+np.eye(4)*1e-5,-jac.T@(w*r))
        accepted=False
        for factor in (1.,.5,.25,.125,.0625):
            trial=np.clip(current+factor*step,seed-margin,seed+margin)
            if cost(residual(trial))<cost(r):
                current=trial;accepted=True;break
        if not accepted or np.linalg.norm(step)<1e-5:break
    q=local(points,current);dist=superficies(q,dims);which=np.argmin(dist,axis=1)
    best=np.min(dist,axis=1);inside=best<.05
    counts=[];coverage=[];vertical=[]
    for i in range(5):
        group=q[inside&(which==i)];counts.append(len(group))
        coverage.append(float(np.ptp(group[:,1 if i==0 else 0])) if len(group)>1 else 0.)
        vertical.append(float(np.ptp(group[:,2])) if len(group)>1 else 0.)
    strong=[counts[i]>=12 and coverage[i]>=.20 and vertical[i]>=.08 for i in range(3)]
    if not strong[0] or not (strong[1] or strong[2]):
        raise ValueError('Geometria 3D insuficiente: fundo e lateral com extensao horizontal/vertical; contagens=%s'%counts)
    err=float(np.sqrt(np.mean(best[inside]**2)))
    if err>.035 or np.count_nonzero(inside)<35:
        raise ValueError('Ajuste 3D inconsistente')
    # Reject fits stuck at the association boundary instead of trusting the seed.
    if np.any(np.abs(current[[0,1,3]]-seed[[0,1,3]])>=.98*margin[[0,1,3]]):
        raise ValueError('Ajuste 3D no limite da associacao GPS/rumo')
    return {'forward':float(current[0]),'right':float(-current[1]),'up':float(current[2]),
            'normal_yaw':float(current[3]),'rms':err,'counts':counts,
            'coverage':coverage,'vertical_coverage':vertical,'n':int(np.count_nonzero(inside)),
            'height_observed':bool(counts[3]>=12 or counts[4]>=12),'source':'container_surfaces_3d'}
