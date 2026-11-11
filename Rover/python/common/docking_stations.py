"""Shared GPS approach-axis registry for QR and LiDAR docking (no ROS I/O)."""
import argparse
import csv
import math
from pathlib import Path
import re

DEFAULT_STATIONS = Path(__file__).resolve().parents[2] / 'estacoes_recarga.txt'
EARTH_RADIUS = 6378137.0  # Same local GPS scale as the SEParnaiba simulation.


def distance_and_bearing(start, end):
    """Spherical distance and forward bearing, degrees clockwise from north."""
    a, b = map(math.radians, start)
    c, d = map(math.radians, end)
    delta = (d-b+math.pi) % (2*math.pi)-math.pi
    h = math.sin((c-a)/2)**2 + math.cos(a)*math.cos(c)*math.sin(delta/2)**2
    distance = 2*EARTH_RADIUS*math.asin(math.sqrt(min(1., max(0., h))))
    heading = math.degrees(math.atan2(math.sin(delta)*math.cos(c),
        math.cos(a)*math.sin(c)-math.sin(a)*math.cos(c)*math.cos(delta))) % 360
    return distance, heading


def destination(point, heading, distance):
    lat, lon = map(math.radians, point)
    angle = math.radians(heading)
    arc = distance/EARTH_RADIUS
    new_lat = math.asin(math.sin(lat)*math.cos(arc) +
                        math.cos(lat)*math.sin(arc)*math.cos(angle))
    new_lon = lon + math.atan2(math.sin(angle)*math.sin(arc)*math.cos(lat),
                               math.cos(arc)-math.sin(lat)*math.sin(new_lat))
    return math.degrees(new_lat), (math.degrees(new_lon)+180) % 360-180


def load_stations(path=DEFAULT_STATIONS):
    """NAME=lat4,lon4,lat2,lon2,QR,length,width,height,target_to_qr.

    The points are 4 m and 2 m OUTSIDE the entrance, not outside the center.
    They determine the entry direction. The target is the interior midpoint.
    """
    path = Path(path).expanduser()
    stations = []
    names, codes = set(), set()
    for number, line in enumerate(path.read_text(encoding='utf-8-sig').splitlines(), 1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        try:
            name, values = line.split('=', 1)
            name = name.strip()
            if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
                raise ValueError('nome deve conter apenas letras, numeros, _ ou -')
            fields = next(csv.reader([values], skipinitialspace=True, strict=True))
            if len(fields) != 9:
                raise ValueError('esperados 9 campos: lat4,lon4,lat2,lon2,QR,comprimento,largura,altura,alvo_ate_qr')
            coordinates = [float(v) for v in fields[:4]]
            dimensions = [float(v) for v in fields[5:8]]
            offset = float(fields[8])
            if not all(math.isfinite(v) for v in coordinates+dimensions+[offset]):
                raise ValueError('valores numericos devem ser finitos')
            far, near = tuple(coordinates[:2]), tuple(coordinates[2:])
            for lat, lon in (far, near):
                if not (-90 < lat < 90 and -180 <= lon <= 180):
                    raise ValueError('latitude/longitude fora dos limites')
            qr = fields[4]
            if not qr.strip() or any(ord(c) < 32 for c in qr):
                raise ValueError('codigo QR vazio ou com caracteres de controle')
            if name in names or qr in codes:
                raise ValueError('nome e codigo QR devem ser unicos no cadastro')
            if min(dimensions) <= 0 or not 0 < offset <= dimensions[0]:
                raise ValueError('dimensoes devem ser positivas; alvo_ate_qr deve estar dentro do comprimento')
            distance, heading = distance_and_bearing(far, near)
            if abs(distance-2.) > .10:
                raise ValueError('pontos de 4 m e 2 m devem distar 2 m (tolerancia 0,10 m); medido %.3f m' % distance)
            # Use the forward tangent at P2 (important across longitude wrap).
            _, backward = distance_and_bearing(near, far)
            heading_at_near = (backward+180.) % 360
            entrance = destination(near, heading_at_near, 2.)
            center = destination(near, heading_at_near, 2.+dimensions[0]/2.)
            stations.append(dict(nome=name, latitude=center[0], longitude=center[1], qr=qr,
                ponto_4m=dict(latitude=far[0], longitude=far[1]),
                ponto_2m=dict(latitude=near[0], longitude=near[1]),
                entrada=dict(latitude=entrance[0], longitude=entrance[1]),
                rumo_entrada_graus=heading, azimute_entrada_graus=heading,
                dimensoes_internas=dimensions, distancia_alvo_parede_m=offset))
            names.add(name)
            codes.add(qr)
        except (ValueError, csv.Error) as exc:
            raise ValueError('%s:%d: %s' % (path, number, exc)) from exc
    if not stations:
        raise ValueError('Cadastre ao menos uma estacao em '+str(path))
    return stations


def select_station(stations, name):
    matches = [s for s in stations if s['nome'] == name]
    if len(matches) != 1:
        raise ValueError('Estacao ausente ou duplicada: '+str(name))
    return matches[0]


def nearest_station(stations, latitude, longitude):
    if not stations:
        raise ValueError('Sem estacoes cadastradas')
    return min(stations, key=lambda s: distance_and_bearing(
        (latitude, longitude), (s['latitude'], s['longitude']))[0])


def visual_station(stations, name=None, qr=None):
    choices = [select_station(stations, name)] if name else stations
    if qr is not None:
        choices = [s for s in choices if s['qr'] == qr]
    if len(choices) != 1:
        raise ValueError('Selecione --estacao NOME para uma estacao unica com o QR correspondente')
    return choices[0]


def main():
    parser = argparse.ArgumentParser(description='Conferir cadastro de estacoes, sem conectar ao rover')
    parser.add_argument('--arquivo', default=str(DEFAULT_STATIONS))
    args = parser.parse_args()
    try:
        stations = load_stations(args.arquivo)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    for station in stations:
        print('%s: rumo=%.3f graus; centro=%.12f,%.12f; QR=%s' %
              (station['nome'], station['rumo_entrada_graus'], station['latitude'],
               station['longitude'], station['qr']))


if __name__ == '__main__':
    main()
