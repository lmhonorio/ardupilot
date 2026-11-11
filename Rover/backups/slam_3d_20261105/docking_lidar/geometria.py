"""Ajuste da geometria em U do contêiner ao MID360, em metros e eixos FLU."""
import math
import numpy as np

OFFSET=np.array([-.003,0.,.434])
SIM_MIN_RANGE=.10  # plugin Livox retorna distancia a partir do inicio do raio


def pontos_paredes(xyz):
    xyz=np.asarray(xyz,dtype=float).reshape(-1,3)
    distance=np.linalg.norm(xyz,axis=1)
    valid=np.isfinite(xyz).all(axis=1)&(distance>.001)
    xyz=xyz[valid];distance=distance[valid]
    xyz=xyz*(1+SIM_MIN_RANGE/distance[:,None])+OFFSET
    r=np.hypot(xyz[:,0],xyz[:,1])
    ok=np.isfinite(xyz).all(axis=1)&(xyz[:,2]>.18)&(xyz[:,2]<.78)&(r>.20)&(r<9.)
    return xyz[ok,:2]


def local(points,pose):
    x,y,theta=pose;c=math.cos(theta);s=math.sin(theta)
    d=points-np.array([x,y]);return np.column_stack((c*d[:,0]+s*d[:,1],-s*d[:,0]+c*d[:,1]))


def segmentos(q):
    x=q[:,0];y=q[:,1]
    return np.column_stack((np.hypot(x-.5,np.maximum(np.abs(y)-.5,0)),
        np.hypot(y-.5,np.maximum(np.abs(x)-.5,0)),
        np.hypot(y+.5,np.maximum(np.abs(x)-.5,0))))


def estimar(xyz,prior):
    """Requer observacoes em duas paredes; prior e somente para selecionar candidato."""
    pts=pontos_paredes(xyz)
    if len(pts)<30: raise ValueError('Nuvem sem pontos de paredes suficientes')
    prior=np.asarray(prior,dtype=float)
    if prior.shape!=(3,) or not np.isfinite(prior).all():raise ValueError('Referencia inicial invalida')
    mask=np.min(segmentos(local(pts,prior)),axis=1)<.35
    pts=pts[mask]
    if len(pts)<30:raise ValueError('Poucos pontos perto do contorno da estacao')
    if len(pts)>1200:pts=pts[::int(math.ceil(len(pts)/1200))]
    def residual(p):
        return np.min(segmentos(local(pts,p)),axis=1)
    low=prior-np.array([.45,.45,math.radians(25)])
    high=prior+np.array([.45,.45,math.radians(25)])
    current=prior.copy()
    def cost(r): return float(np.sum(2*.035**2*(np.sqrt(1+(r/.035)**2)-1)))
    for _ in range(50):
        r=residual(current);w=1/np.sqrt(1+(r/.035)**2)
        jac=np.column_stack([(residual(current+np.eye(3)[j]*1e-5)-r)/1e-5 for j in range(3)])
        step=np.linalg.solve(jac.T@(w[:,None]*jac)+np.eye(3)*1e-5,-jac.T@(w*r))
        old=cost(r); accepted=False
        for factor in (1,.5,.25,.125,.0625):
            trial=np.clip(current+factor*step,low,high)
            if cost(residual(trial))<old:
                current=trial;accepted=True;break
        if not accepted or np.linalg.norm(step)<1e-5: break
    q=local(pts,current);d=segmentos(q);which=np.argmin(d,axis=1);best=np.min(d,axis=1)
    inside=best<.065
    counts=[int(np.count_nonzero(inside&(which==i))) for i in range(3)]
    # Duas superficies e extensão vertical de pontos são necessárias para pose observável.
    coverage=[]
    for i in range(3):
        group=q[inside&(which==i)]
        span=float(np.ptp(group[:,1 if i==0 else 0])) if len(group)>=8 else 0.
        coverage.append(span)
    strong=[counts[i]>=12 and coverage[i]>=.20 for i in range(3)]
    # Lados paralelos sozinhos nao definem a distancia ate o fundo.
    if not strong[0] or not (strong[1] or strong[2]):
        raise ValueError('Geometria insuficiente: fundo e uma lateral devem estar visiveis')
    if np.count_nonzero(inside)<35:raise ValueError('Poucos pontos ajustados a estacao')
    err=float(np.sqrt(np.mean(best[inside]**2)))
    if err>.04:raise ValueError('Ajuste geometrico inconsistente')
    return {'forward':float(current[0]),'right':float(-current[1]),
            'normal_yaw':float(current[2]),'rms':err,'counts':counts,
            'coverage':coverage,'n':int(np.count_nonzero(inside))}
