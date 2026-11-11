"""Busca da estacao e aproximacao limitada para recuperar a leitura do QR."""
import math


def direcao_estacao(position, gps, now, pos_seen, gps_seen, latitude, longitude):
    """Erro angular NED e distancia ao centro cadastrado, ou None."""
    if position is None or gps is None or now-pos_seen>3 or now-gps_seen>3:
        return None
    if gps.fix_type<3 or not (position.lat or position.lon) or not 0<=position.hdg<=36000:
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
    return error, math.hypot(north,east)


def yaw_rate_para_estacao(position, gps, now, pos_seen, gps_seen, latitude, longitude):
    """Retorna taxa NED (positiva para direita) ou None se dados nao confiaveis."""
    direction=direcao_estacao(position,gps,now,pos_seen,gps_seen,latitude,longitude)
    if direction is None:return None
    error,_=direction
    if abs(error)<math.radians(5): return 0.0
    return min(.15,max(-.15,error*.6))


class AproximacaoBusca:
    """A single bounded search advance per docking attempt, never inside the box.

    Reacquiring or losing the QR does not replenish the distance/time budget.
    The full visual controller still requires the decoded station identifier.
    """
    SPEED=.04
    MAX_DISTANCE=.40
    MAX_TIME=12.
    MIN_STATION_DISTANCE=2.
    ALIGN_TIME=.8

    def __init__(self):
        self.aligned_since=None
        self.origin=None
        self.last_time=None
        self.moving=False
        self.elapsed=0.
        self.exhausted=False

    def step(self, now, direction, position, candidate_bearing, motion_ok, allowed=True):
        if self.last_time is not None and self.moving:
            self.elapsed+=max(0.,now-self.last_time)
        self.last_time=now
        self.moving=False
        reason=None
        if self.origin is not None and position is not None:
            lat,lon=self.origin
            north=math.radians(position.lat/1e7-lat)*6378137.
            east=math.radians(position.lon/1e7-lon)*6378137.*math.cos(math.radians(lat))
            if math.hypot(north,east)>=self.MAX_DISTANCE:self.exhausted=True
        if self.elapsed>=self.MAX_TIME:self.exhausted=True
        if self.exhausted:reason='limite de aproximacao sem leitura atingido (0,40 m / 12 s)'
        elif not allowed:reason='aproximacao de busca desabilitada nesta fase'
        elif candidate_bearing is None:reason='sem candidato QR atual para aproximacao'
        elif direction is None or position is None:reason='GPS/rumo indisponivel para aproximacao'
        elif direction[1]<=self.MIN_STATION_DISTANCE:reason='a menos de 2 m do centro; exige QR decodificado'
        elif not math.isfinite(candidate_bearing) or abs(candidate_bearing)>=math.radians(5):
            reason='candidato QR fora do eixo de 5 graus'
        elif abs(direction[0])>=math.radians(5):reason='alinhando rumo: erro deve ser menor que 5 graus'
        elif not motion_ok:reason='aguardando velocidade e giro recentes e estaveis'
        if reason:
            self.aligned_since=None
            return 0.,reason
        if self.aligned_since is None:self.aligned_since=now
        if now-self.aligned_since<self.ALIGN_TIME:
            return 0.,'confirmando alinhamento por 0,8 s'
        if self.origin is None:self.origin=(position.lat/1e7,position.lon/1e7)
        self.moving=True
        return self.SPEED,'aproximando a 0,04 m/s para decodificar QR (limite 0,40 m)'
