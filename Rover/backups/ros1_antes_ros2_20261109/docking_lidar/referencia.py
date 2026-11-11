"""Semente de associacao: localizacao final e ajustada a nuvem MID360."""
import math


def prior_estacao(lat, lon, heading_deg, station):
    north=math.radians(station['latitude']-lat)*6378137.
    de=(station['longitude']-lon+180)%360-180
    east=math.radians(de)*6378137.*math.cos(math.radians((lat+station['latitude'])/2))
    h=math.radians(heading_deg)
    forward=north*math.cos(h)+east*math.sin(h)
    left=north*math.sin(h)-east*math.cos(h)
    angle=(math.radians(heading_deg-station['azimute_entrada_graus'])+math.pi)%(2*math.pi)-math.pi
    return [forward,left,angle]


def recuo_lio(origem, atual, yaw0, yaw):
    """Deslocamento FAST-LIO FLU para recuo e comando de giro BODY_NED."""
    dx,dy=atual[0]-origem[0],atual[1]-origem[1]
    back=-(dx*math.cos(yaw0)+dy*math.sin(yaw0))
    side=-dx*math.sin(yaw0)+dy*math.cos(yaw0)
    error=(yaw-yaw0+math.pi)%(2*math.pi)-math.pi
    if not all(math.isfinite(v) for v in (back,side,error)):
        raise ValueError('Odometria nao finita')
    angular=max(-.12,min(.12,-side*.35+error*.6))
    return back,side,error,angular
