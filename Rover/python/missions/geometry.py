"""Mission geometry in local north/east metres. No vehicle connection."""
import math
import json

EARTH_RADIUS = 6378137.0


def gps_offset(lat, lon, north, east):
    return (lat + math.degrees(north / EARTH_RADIUS),
            lon + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat)))))


def local_position(lat, lon, origin):
    return (math.radians(lat - origin[0]) * EARTH_RADIUS,
            math.radians(lon - origin[1]) * EARTH_RADIUS * math.cos(math.radians(origin[0])))


def make_plan(radius, origin, center=(0.0, 0.0), points=36, speed=0.6):
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('radius deve ser positivo e finito')
    if points < 12 or points > 360:
        raise ValueError('circle-points deve estar entre 12 e 360')
    if not all(math.isfinite(x) for x in (*origin, *center, speed)) or speed <= 0:
        raise ValueError('Coordenadas e velocidade devem ser finitas; velocidade positiva')
    if abs(origin[0]) >= 85 or abs(origin[1]) > 180:
        raise ValueError('Origem GPS invalida ou muito proxima dos polos')
    items = []

    def add(north, east, phase, command=16, hold=0):
        lat, lon = gps_offset(*origin, north, east)
        items.append(dict(seq=len(items), command=command, frame=3,
                          north=north, east=east, lat=lat, lon=lon,
                          altitude=0, phase=phase, param1=hold))

    add(0, 0, 'home')
    cn, ce = center
    for i in range(points + 1):
        angle = 2 * math.pi * i / points
        add(cn + radius * math.cos(angle), ce + radius * math.sin(angle),
            'approach' if i == 0 else 'circle')
    for i, (n, e) in enumerate([(1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1)]):
        add(cn + radius * n, ce + radius * e, 'connector' if i == 0 else 'square')
    add(items[-1]['north'], items[-1]['east'], 'finish', command=19, hold=2)
    length=sum(math.hypot(b['north']-a['north'],b['east']-a['east'])
               for a,b in zip(items,items[1:]))
    return dict(radius_m=radius, square_side_m=2 * radius, circle_points=points,
                origin_lat=origin[0], origin_lon=origin[1], center_north=cn,
                center_east=ce, speed_m_s=speed, nominal_length_m=length,
                nominal_time_s=length/speed+2, items=items)


def write_wpl(path, items):
    lines = ['QGC WPL 110']
    for item in items:
        lines.append('%d\t%d\t%d\t%d\t%.6f\t0\t0\t0\t%.9f\t%.9f\t%.3f\t1' %
                     (item['seq'], int(item['seq'] == 0), item.get('frame', 3),
                      item['command'], item.get('param1', 0), item['lat'],
                      item['lon'], item.get('altitude', 0)))
    path.write_text('\n'.join(lines) + '\n')


def write_qgc_plan(path, plan):
    """Native QGC plan, with home metadata instead of an extra home waypoint."""
    items=[]
    for item in plan['items'][1:]:
        items.append(dict(type='SimpleItem',doJumpId=item['seq'],autoContinue=True,
                          command=item['command'],frame=item['frame'],Altitude=0,
                          AltitudeMode=1,AMSLAltAboveTerrain=None,
                          params=[item.get('param1',0),0,0,0,item['lat'],item['lon'],0]))
    data=dict(fileType='Plan',version=1,groundStation='QGroundControl',
              geoFence=dict(circles=[],polygons=[],version=2),
              rallyPoints=dict(points=[],version=2),
              mission=dict(version=2,firmwareType=3,vehicleType=10,
                           cruiseSpeed=plan['speed_m_s'],hoverSpeed=plan['speed_m_s'],
                           globalPlanAltitudeMode=1,items=items,
                           plannedHomePosition=[plan['origin_lat'],plan['origin_lon'],
                                                plan.get('origin_alt_m',0)]))
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
