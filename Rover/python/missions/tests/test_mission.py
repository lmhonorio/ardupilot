import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import make_plan, local_position, write_qgc_plan
from tempfile import TemporaryDirectory
from analysis import propose, summarize, improves, stage_improves, TUNED
from mavproxy_link import json_safe, Link
from types import SimpleNamespace
from run import mission_finished, parse_args
from unittest.mock import patch
from contextlib import redirect_stderr
from io import StringIO


class NavigationArgumentTests(unittest.TestCase):
    def test_invalid_navigation_limits_are_rejected(self):
        for options in [['--wp-radius','nan'],['--wp-radius','0'],
                        ['--wp-radius','3'],['--pivot-angle','10'],
                        ['--pivot-rate','inf'],['--pivot-rate','0'],
                        ['--pivot-rate','61']]:
            with self.subTest(options=options), patch.object(sys,'argv',['run.py','--radius','3']+options):
                with redirect_stderr(StringIO()), self.assertRaises(SystemExit) as raised:
                    parse_args()
                self.assertEqual(raised.exception.code,2)

    def test_continuous_turns_can_be_selected_explicitly(self):
        with patch.object(sys,'argv',['run.py','--radius','3','--pivot-angle','0','--wp-radius','0.15']):
            args=parse_args()
        self.assertEqual(args.pivot_angle,0)
        self.assertEqual(args.wp_radius,.15)


class GeometryTests(unittest.TestCase):
    def test_qgc_plan_is_rover_and_does_not_duplicate_home(self):
        plan=make_plan(3,(-3.12315,-41.764495))
        with TemporaryDirectory() as directory:
            path=Path(directory)/'mission.plan'
            write_qgc_plan(path,plan)
            qgc=json.loads(path.read_text())
        self.assertEqual(qgc['fileType'],'Plan')
        self.assertEqual(qgc['mission']['vehicleType'],10)
        self.assertEqual(qgc['mission']['firmwareType'],3)
        self.assertEqual(len(qgc['mission']['items']),len(plan['items'])-1)
        self.assertEqual(qgc['mission']['items'][0]['command'],16)
        self.assertEqual(qgc['mission']['items'][-1]['command'],19)
        self.assertEqual(qgc['mission']['items'][0]['params'][4:6],
                         [plan['items'][1]['lat'],plan['items'][1]['lon']])

    def test_radius_scales_circle_and_square(self):
        for radius in [1,3,5,12]:
            p=make_plan(radius,(-3.12315,-41.764495))
            self.assertEqual(p['square_side_m'],2*radius)
            for item in p['items']:
                if item['phase']=='circle':
                    self.assertAlmostEqual(math.hypot(item['north'],item['east']),radius)
                if item['phase']=='square':
                    self.assertEqual(abs(item['north']),radius)
                    self.assertEqual(abs(item['east']),radius)
            self.assertEqual(p['items'][1]['north'],p['items'][37]['north'])
            self.assertAlmostEqual(p['items'][1]['east'],p['items'][37]['east'])

    def test_gps_roundtrip_and_offset(self):
        p=make_plan(3,(-3.12315,-41.764495),center=(12,-4))
        for item in p['items']:
            n,e=local_position(item['lat'],item['lon'],(-3.12315,-41.764495))
            self.assertAlmostEqual(n,item['north'],places=6)
            self.assertAlmostEqual(e,item['east'],places=6)

    def test_invalid_geometry(self):
        for radius in [0,-1,math.inf,math.nan]:
            with self.assertRaises(ValueError):make_plan(radius,(-3,-41))

    def test_route_length_scales_with_radius(self):
        a,b=[make_plan(r,(-3,-41)) for r in (3,5)]
        self.assertAlmostEqual(b['nominal_length_m']/a['nominal_length_m'],5/3)


class TuningTests(unittest.TestCase):
    def params(self):
        return dict(zip(TUNED,[.2,.2,.2,.1,.2,75]))

    def records(self,axis,achieved,output):
        return [dict(mavpackettype='PID_TUNING',axis=axis,phase='circle',
                     achieved=achieved,desired=achieved,FF=output,P=0,I=0,D=0) for _ in range(30)]

    def test_units_and_bounded_change(self):
        # 0.1 normalized steering for 1 rad/s means FF = 0.1, not 0.00175.
        rows=self.records(5,math.degrees(1),.1)+self.records(4,1,.15)
        candidate,evidence=propose(rows,self.params(),1,max_step=.5)
        self.assertAlmostEqual(evidence['steering']['output_per_response'],.1)
        self.assertAlmostEqual(candidate['ATC_STR_RAT_FF'],.1)
        self.assertAlmostEqual(candidate['CRUISE_THROTTLE'],37.5)
        for k,v in candidate.items():
            self.assertGreaterEqual(v,self.params()[k]*.5)
            self.assertLessEqual(v,self.params()[k]*1.5)

    def test_no_data_does_not_invent_tune(self):
        candidate,evidence=propose([],self.params(),1)
        self.assertIsNone(candidate)

    def test_saturation_excluded(self):
        candidate,evidence=propose(self.records(5,20,1),self.params(),1)
        self.assertIsNone(candidate)
        self.assertEqual(evidence['steering']['samples'],0)

    def test_nonfinite_pid_samples_are_rejected(self):
        candidate,evidence=propose(self.records(5,20,float('nan')),self.params(),1)
        self.assertIsNone(candidate)
        self.assertEqual(evidence['steering']['samples'],0)

    def test_speed_stage_preserves_steering(self):
        rows=self.records(5,20,.2)+self.records(4,1,.15)
        candidate,_=propose(rows,self.params(),1,stage='speed')
        for k in ['ATC_STR_RAT_FF','ATC_STR_RAT_P','ATC_STR_RAT_I']:
            self.assertEqual(candidate[k],self.params()[k])

    def test_completion_precedes_partial_score(self):
        incomplete=dict(score=100.1,quality_score=.1,completed=False)
        complete=dict(score=2,quality_score=2,completed=True)
        self.assertTrue(improves(complete,incomplete))
        self.assertFalse(improves(incomplete,complete))

    def test_intermediate_speed_progress_is_not_global_acceptance(self):
        baseline=dict(score=1,quality_score=1,completed=True,speed_rmse=.4)
        speed=dict(score=1.4,quality_score=1.4,completed=True,speed_rmse=.35,tracking_p95_m=2)
        self.assertTrue(stage_improves(speed,baseline,'speed',3))
        self.assertFalse(improves(speed,baseline))
        speed['completed']=False
        self.assertFalse(stage_improves(speed,baseline,'speed',3))

    def test_failed_or_missing_data_not_success(self):
        s=summarize([],make_plan(3,(-3,-41)),False)
        self.assertFalse(s['completed'])
        self.assertIsNone(s['score'])

    def test_nan_is_valid_json_null(self):
        value=json_safe({'a':float('nan'),'b':[float('inf'),1]})
        self.assertEqual(json.loads(json.dumps(value,allow_nan=False)),{'a':None,'b':[None,1]})


class ProtocolTests(unittest.TestCase):
    def test_upload_uses_integer_coordinates_even_for_legacy_request(self):
        link=Link.__new__(Link)
        link.sysid,link.compid=1,1
        sent=[]
        link.conn=SimpleNamespace(mav=SimpleNamespace(mission_item_int_send=lambda *a:sent.append(a)))
        item=dict(seq=1,frame=3,command=16,lat=-3.1231501,lon=-41.7644951,altitude=0)
        link.send_item(item,False)
        self.assertEqual(sent[0][3],6)
        self.assertEqual(sent[0][11],-31231501)
        self.assertEqual(sent[0][12],-417644951)

    def test_old_reached_event_cannot_end_new_trial(self):
        msg=SimpleNamespace(seq=43,get_type=lambda:'MISSION_ITEM_REACHED')
        self.assertFalse(mission_finished(msg,1,43))
        self.assertTrue(mission_finished(msg,43,43))

    def test_integer_parameter_is_quantized_before_confirmation(self):
        link=Link.__new__(Link)
        link.sysid,link.compid=1,1
        link.param_types={'CRUISE_THROTTLE':2}
        sent=[]
        link.conn=SimpleNamespace(mav=SimpleNamespace(param_set_send=lambda *a:sent.append(a)))
        link.wait=lambda *a,**kw:SimpleNamespace(param_id='CRUISE_THROTTLE',param_type=2,param_value=38.)
        self.assertEqual(link.param('CRUISE_THROTTLE',37.5),38)
        self.assertEqual(sent[0][3],38)

    def test_upload_retries_count_and_requires_ack(self):
        from pymavlink import mavutil
        link=Link.__new__(Link)
        link.sysid,link.compid=1,1
        link.source_system,link.source_component=246,191
        link.drain=lambda *a:None
        counts,items=[],[]
        link.conn=SimpleNamespace(mav=SimpleNamespace(mission_count_send=lambda *a:counts.append(a)))
        link.send_item=lambda item,integer:items.append(item['seq'])
        events=[TimeoutError(),
                SimpleNamespace(seq=0,get_type=lambda:'MISSION_REQUEST_INT'),
                SimpleNamespace(seq=1,get_type=lambda:'MISSION_REQUEST_INT'),
                SimpleNamespace(type=mavutil.mavlink.MAV_MISSION_ACCEPTED,get_type=lambda:'MISSION_ACK')]
        def wait(*a,**kw):
            event=events.pop(0)
            if isinstance(event,Exception):raise event
            return event
        link.wait=wait
        link.upload_mission([dict(seq=0),dict(seq=1)])
        self.assertEqual(len(counts),2)
        self.assertEqual(items,[0,1])


if __name__=='__main__':unittest.main()
