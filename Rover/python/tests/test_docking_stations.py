"""Station axes must preserve physical doorway distances and controller frames."""
import csv
import io
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.docking_stations import (DEFAULT_STATIONS, load_stations, destination,
    distance_and_bearing, nearest_station, select_station, visual_station)


class StationsTests(unittest.TestCase):
    def row(self, name='dock', qr='QR', center=(-3.12,-41.76), heading=90., depth=1.):
        p4=destination(center,(heading+180)%360,4+depth/2)
        p2=destination(center,(heading+180)%360,2+depth/2)
        stream=io.StringIO()
        csv.writer(stream).writerow([*p4,*p2,qr,depth,1.,1.,depth/2-.001])
        return name+'='+stream.getvalue()

    def load(self, text):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'stations.txt';p.write_text(text)
            return load_stations(p)

    def test_existing_station_keeps_center_qr_and_wall_offset(self):
        s=load_stations()[0]
        self.assertEqual(DEFAULT_STATIONS.name,'estacoes_recarga.txt')
        self.assertEqual(s['qr'],'ROVER_DOCKING_STATION_01')
        d,_=distance_and_bearing((s['latitude'],s['longitude']),(-3.123167966,-41.764450017))
        self.assertLess(d,.001)
        self.assertAlmostEqual(s['rumo_entrada_graus'],90,delta=.001)
        self.assertEqual(s['distancia_alvo_parede_m'],.499)

    def test_points_are_four_and_two_metres_from_door_not_center(self):
        s=load_stations()[0]
        door=tuple(s['entrada'].values());center=(s['latitude'],s['longitude'])
        for key,dist in [('ponto_4m',4),('ponto_2m',2)]:
            p=tuple(s[key].values())
            self.assertAlmostEqual(distance_and_bearing(p,door)[0],dist,delta=.001)
            self.assertAlmostEqual(distance_and_bearing(p,center)[0],dist+.5,delta=.001)

    def test_all_cardinal_axes_and_different_depths(self):
        for heading in (0.,90.,180.,270.):
            for depth in (1.,2.):
                s=self.load(self.row(heading=heading,depth=depth))[0]
                d,_=distance_and_bearing((s['latitude'],s['longitude']),(-3.12,-41.76))
                self.assertLess(d,.001)
                error=(s['rumo_entrada_graus']-heading+180)%360-180
                self.assertLess(abs(error),.001)
                self.assertEqual(s['azimute_entrada_graus'],s['rumo_entrada_graus'])
                self.assertEqual(s['dimensoes_internas'],[depth,1.,1.])

    def test_dateline_crossing(self):
        s=self.load(self.row(center=(10.,-179.99999)))[0]
        self.assertLess(distance_and_bearing((s['latitude'],s['longitude']),(10.,-179.99999))[0],.001)

    def test_comments_blank_lines_bom_and_csv_qr(self):
        code=' station=A, port#2 '
        stations=self.load('\ufeff# comentario\n\n'+self.row(qr=code)+'\n')
        self.assertEqual(stations[0]['qr'],code)

    def test_multiple_stations_and_selection(self):
        stations=self.load(self.row('first','A')+self.row('second','B',center=(-3.13,-41.77)))
        self.assertEqual(nearest_station(stations,-3.13,-41.77)['nome'],'second')
        self.assertEqual(select_station(stations,'first')['qr'],'A')
        self.assertEqual(visual_station(stations,name='second')['qr'],'B')
        self.assertEqual(visual_station(stations,qr='A')['nome'],'first')
        with self.assertRaises(ValueError):visual_station(stations)
        with self.assertRaises(ValueError):visual_station(stations,name='first',qr='B')
        with self.assertRaises(ValueError):select_station(stations,'missing')

    def test_duplicate_name_or_qr_rejected(self):
        for second in (self.row('dock','OTHER'),self.row('other','QR')):
            with self.assertRaisesRegex(ValueError,':2:.*unicos'):
                self.load(self.row()+second)

    def test_wrong_axis_spacing_and_coincident_points_rejected(self):
        for spacing in (0.,1.,3.):
            near=(-3.12,-41.76);far=destination(near,270,spacing)
            row='bad='+','.join(map(str,[*far,*near,'QR',1,1,1,.499]))
            with self.assertRaisesRegex(ValueError,'distar 2 m'):self.load(row)

    def test_invalid_numbers_coordinates_dimensions_and_format_rejected(self):
        fields=next(csv.reader([self.row().split('=',1)[1]]))
        for index,value in ((0,'nan'),(0,'91'),(1,'181'),(5,'0'),(6,'-1'),(7,'inf'),(8,'2'),(4,'')):
            bad=list(fields);bad[index]=value
            with self.assertRaises(ValueError,msg=(index,value)):
                self.load('bad='+','.join(bad))
        for text in ('# somente comentario','no equals','bad=1,2,3','bad name='+','.join(fields)):
            with self.assertRaises(ValueError):self.load(text)

    def test_reversing_points_changes_entry_direction(self):
        fields=next(csv.reader([self.row().split('=',1)[1]]))
        fields[:4]=fields[2:4]+fields[:2]
        s=self.load('reverse='+','.join(fields))[0]
        self.assertAlmostEqual(s['rumo_entrada_graus'],270,delta=.001)

    def test_qr_and_lidar_use_the_same_station(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking_lidar'))
        from docking_lidar.movimentos import estacao
        from docking_lidar.referencia import prior_estacao
        from docking.aguardar_docking import load_stations as qr_load
        qr=qr_load(DEFAULT_STATIONS)[0]
        lidar=estacao(DEFAULT_STATIONS,qr['nome'])
        self.assertEqual(qr,lidar)
        prior=prior_estacao(lidar['latitude'],lidar['longitude'],90.,lidar)
        self.assertAlmostEqual(prior[0],0.,places=6)
        self.assertAlmostEqual(prior[1],0.,places=6)
        self.assertAlmostEqual(prior[2],0.,places=5)
