"""Giro em direcao a uma estacao cadastrada, sem deslocamento linear."""
import math


def yaw_rate_para_estacao(position, gps, now, pos_seen, gps_seen, latitude, longitude):
    """Retorna taxa NED (positiva para direita) ou None se dados nao confiaveis."""
    if position is None or gps is None or now-pos_seen>3 or now-gps_seen>3:
        return None
    if gps.fix_type<3 or not (position.lat or position.lon) or position.hdg==65535:
        return None
    lat=position.lat/1e7; lon=position.lon/1e7
    if not all(math.isfinite(v) for v in (lat,lon,latitude,longitude)):
        return None
    north=math.radians(latitude-lat)*6378137.0
    delta_lon=(longitude-lon+180.0)%360.0-180.0
    east=math.radians(delta_lon)*6378137.0*math.cos(math.radians((latitude+lat)/2))
    # Rumo GPS para um ponto quase coincidente e dominado pelo erro de posicao.
    if math.hypot(north,east)<1.0:
        return None
    target=math.atan2(east,north)
    heading=math.radians(position.hdg/100.0)
    error=(target-heading+math.pi)%(2*math.pi)-math.pi
    if abs(error)<math.radians(5): return 0.0
    return min(.15,max(-.15,error*.6))
