"""Deterministic policy, perception and orchestration regressions (no vehicle)."""
import copy
import math
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch, Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.docking_stations import load_stations
from mission_manager.policy import Battery, Machine, State
from mission_manager.geometry import Envelope, fuse, obstacle_in_sweep, reposition_route, acquisition_heading
from mission_manager.mission import bind, validate, requested, fingerprint

STATION=load_stations()[0]
def pose(f=0.,r=0.,h=0.,stamp=10.):
    return dict(forward=f,right=r,normal_yaw=h,stamp=stamp,rms=.005,n=200)
def mission():
    p=STATION['ponto_2m']
    return [dict(seq=i,command=16,frame=3,x=p['latitude'],y=p['longitude'],z=0,autocontinue=1) for i in range(2)]

class FusionTests(unittest.TestCase):
    def test_lidar_only(self):self.assertEqual(fuse(pose(),None,10)['fusion'],'lidar_only')
    def test_qr_valid_capped(self):
        q=dict(pose(.10),identity_ok=True,reprojection_px=.1,stationary=True)
        p=fuse(pose(),q,10)
        self.assertEqual(p['fusion'],'lidar_qr');self.assertGreater(p['forward'],0);self.assertLessEqual(p['forward'],.015)
    def test_loss_recovery(self):
        q=dict(pose(.02),identity_ok=True,reprojection_px=.1,stationary=True)
        self.assertEqual([fuse(pose(),x,10)['fusion'] for x in (q,None,q)],['lidar_qr','lidar_only','lidar_qr'])
    def test_divergence(self):
        q=dict(pose(.4),identity_ok=True,reprojection_px=.1,stationary=True)
        self.assertEqual(fuse(pose(),q,10)['forward'],0)
        self.assertEqual(fuse(pose(),q,10)['fusion'],'qr_divergent')
    def test_wrong_identity_motion_epoch(self):
        for change in ({'identity_ok':False},{'stationary':False},{'stamp':9.5}):
            q=dict(pose(.02),identity_ok=True,reprojection_px=.1,stationary=True);q.update(change)
            self.assertEqual(fuse(pose(),q,10)['fusion'],'qr_not_trusted')
    def test_stale_lidar_never_replaced_by_qr(self):
        with self.assertRaises(ValueError):fuse(pose(),dict(pose(stamp=12)),12)
    def test_invalid_lidar(self):
        for change in ({'rms':.04},{'n':20},{'right':float('nan')}):
            p=pose();p.update(change)
            with self.assertRaises(ValueError):fuse(p,None,10)

class GeometryTests(unittest.TestCase):
    def setUp(self):self.e=Envelope()
    def test_envelope_not_just_center(self):
        self.assertTrue(self.e.final(pose(),STATION))
        self.assertFalse(self.e.fits(pose(r=.25),STATION))
        self.assertFalse(self.e.outside(pose(f=.6),STATION))
        self.assertTrue(self.e.outside(pose(f=2),STATION,True))
    def test_different_vehicle(self):
        with self.assertRaises(ValueError):Envelope(width=1.).validate(STATION)
        e=Envelope(base_to_center_x=.05)
        self.assertAlmostEqual(e.target(pose(.05))['forward'],0)
    def test_corridor_heading_envelope(self):
        self.assertTrue(self.e.corridor(pose(.2),STATION))
        self.assertFalse(self.e.corridor(pose(.2,.15,.6),STATION))
    def test_obstacle_and_ground(self):
        self.assertTrue(obstacle_in_sweep([[.4,0,.2]]*5,.15,0,self.e))
        self.assertFalse(obstacle_in_sweep([[.4,0,0]]*5,.15,0,self.e))
        self.assertFalse(obstacle_in_sweep([[.4,.5,.2]]*5,.15,0,self.e))
        self.assertTrue(obstacle_in_sweep([[-.4,0,.2]]*5,-.15,0,self.e))
    def test_route_around_station_no_crossing(self):
        import numpy as np
        for a,l in ((-2.,0.),(0.,2.),(-1.5,-2.),(2.,1.)):
            route=[(-a,-l)]+reposition_route(pose(a,l),STATION,self.e)
            for start,end in zip(route,route[1:]):
                points=np.array(start)+(np.array(end)-start)*np.linspace(0,1,100)[:,None]
                self.assertFalse(np.any((abs(points[:,0])<1.)&(abs(points[:,1])<1.)))
            self.assertLess(route[-1][0],-1.2);self.assertEqual(route[-1][1],0.)
    def test_no_reposition_inside(self):
        with self.assertRaises(ValueError):reposition_route(pose(.2,.1),STATION,self.e)
    def test_acquisition_only_outside_and_without_translation(self):
        self.assertAlmostEqual(abs(acquisition_heading([-4.,0.,math.pi],STATION,self.e)),math.pi)
        self.assertAlmostEqual(acquisition_heading([4.,0.,0.],STATION,self.e),0.)
        for prior in ([.4,0.,0.],[1.,0.,0.],[0.,4.,0.],[9.,0.,0.],[float('nan'),0.,0.]):
            with self.assertRaises(ValueError):acquisition_heading(prior,STATION,self.e)
    def test_lidar_station_behind_heading_pivots_before_driving(self):
        from docking.visual_alignment import VisualAlignment
        c=VisualAlignment(allow_rear_target=True)
        phase,v,w,done=c.step(pose(-4,0,math.pi),now=10)
        self.assertEqual(v,0);self.assertNotEqual(w,0);self.assertFalse(done)
        with self.assertRaises(ValueError):VisualAlignment().step(pose(-4,0,math.pi))

class PerceptionSourceTests(unittest.TestCase):
    def perception(self, raw_ok):
        from mission_manager.perception import Perception
        o=Perception.__new__(Perception)
        msg=NS(header=NS(stamp=NS(sec=10,nanosec=0)))
        o.station=STATION;o.lock=threading.Lock();o.done=Mock();o.done.wait.side_effect=[False,True]
        o.link=NS();o.lidar=o.qr=o.points=None;o.error='';o.wall=0.;o.last_dump=time.monotonic()
        o.raw_sensor=NS(topic='raw',fresh_cloud=Mock(return_value=msg),pose=Mock(return_value=pose(2)))
        o.sensor=NS(topic='registered',fresh_cloud=Mock(return_value=msg),pose=Mock(return_value=pose(2.01)))
        if not raw_ok:o.raw_sensor.pose.side_effect=ValueError('not enough surfaces')
        return o,msg
    def test_current_scan_has_priority_and_same_sample_is_fitted(self):
        import numpy as np
        o,msg=self.perception(True)
        with patch('mission_manager.perception.cloud_xyz',return_value=np.array([[1.,0.,.2]])):
            o._lidar()
        self.assertEqual(o.lidar['cloud_source'],'raw');o.sensor.pose.assert_not_called()
        o.raw_sensor.pose.assert_called_once_with(o.link,STATION,msg,max_points=1200)
        self.assertEqual(o.error,'')
    def test_registered_fallback_requires_its_own_fit(self):
        import numpy as np
        o,msg=self.perception(False)
        with patch('mission_manager.perception.cloud_xyz',return_value=np.array([[1.,0.,.2]])):
            o._lidar()
        self.assertEqual(o.lidar['cloud_source'],'registered')
        o.sensor.pose.assert_called_once_with(o.link,STATION,msg,max_points=1200)
    def test_fallback_cannot_publish_an_older_epoch(self):
        import numpy as np
        o,msg=self.perception(False);o.lidar=pose(2.,stamp=10.1)
        with patch('mission_manager.perception.cloud_xyz',return_value=np.array([[1.,0.,.2]])):
            o._lidar()
        self.assertEqual(o.lidar['stamp'],10.1)
        self.assertIn('fora de ordem',o.error)
    def test_empty_cloud_cannot_authorize_exit_sweep(self):
        import numpy as np
        o,_=self.perception(True)
        with patch('mission_manager.perception.ros.age',return_value=0.), patch('mission_manager.perception.cloud_xyz',return_value=np.empty((0,3))):
            with self.assertRaisesRegex(ValueError,'vazia'):o.sweep()
    def test_stale_cloud_cannot_authorize_exit_sweep(self):
        o,_=self.perception(True)
        with patch('mission_manager.perception.ros.age',return_value=.8):
            with self.assertRaisesRegex(ValueError,'desatualizada'):o.sweep()

class MissionTests(unittest.TestCase):
    def test_binding_real_reach_required(self):
        items=mission();b=bind(items,STATION)
        self.assertFalse(requested(b,items,None,1,10,2.5))
        self.assertFalse(requested(b,items,1,1,15,2.5))
        self.assertTrue(requested(b,items,1,1,10,2.5))
    def test_upload_change_invalidates_binding(self):
        items=mission();b=bind(items,STATION);items[1]['x']+=.00001
        self.assertFalse(requested(b,items,1,1,10,2.5))
    def test_home_does_not_invalidate(self):
        a=mission();b=copy.deepcopy(a);b[0]['x']+=.1
        self.assertEqual(fingerprint(a),fingerprint(b))
    def test_relay_parachute_and_jumps_rejected(self):
        for cmd in (181,208,177):
            m=mission();m.append(dict(m[-1],seq=2,command=cmd))
            with self.assertRaises(ValueError):validate(m)
    def test_empty_invalid(self):
        with self.assertRaises(ValueError):validate([])
    def test_dock_waypoint_wrong_side(self):
        m=mission();m[-1]['y']=STATION['longitude']+.00002
        with self.assertRaises(ValueError):bind(m,STATION)

class BatteryTests(unittest.TestCase):
    def test_15_seconds_no_real_changes(self):
        b=Battery(True,20);b.observe(47,100);b.begin_charge(100,True,True)
        self.assertFalse(b.tick(107.5));self.assertEqual(b.soc,60)
        self.assertTrue(b.tick(115));self.assertEqual(b.soc,100);self.assertEqual(b.real,47)
        self.assertFalse(b.tick(116))
    def test_charge_guards(self):
        for enabled,sitl,parked in ((False,True,True),(True,False,True),(True,True,False)):
            with self.assertRaises(ValueError):Battery(enabled).begin_charge(1,sitl,parked)
    def test_real_unknown_stale(self):
        b=Battery();self.assertFalse(b.sufficient(10));b.observe(90,10)
        self.assertTrue(b.sufficient(11));self.assertFalse(b.sufficient(16))
    def test_minimum(self):
        self.assertFalse(Battery(True,29).sufficient(10));self.assertTrue(Battery(True,30).sufficient(10))

class StateTests(unittest.TestCase):
    def test_repeat_cycles(self):
        m=Machine();m.inside=False
        for _ in range(3):
            ok,_=m.request_start(True,True);self.assertTrue(ok)
            m.move(State.PREPARE,'test');m.move(State.MISSION,'confirmed');m.pending=False
            m.move(State.REQUESTED,'reached');m.move(State.ALIGN,'geometry');m.move(State.APPROACH,'aligned')
            m.confirmed_parked();m.move(State.CHARGING,'sim');m.move(State.CHARGED,'15s')
            ok,_=m.request_start(True,True);self.assertTrue(ok)
            m.move(State.PREPARE,'valid');m.move(State.EXIT,'guided');m.confirmed_outside()
            m.move(State.PREPARE,'clear');m.move(State.MISSION,'auto');m.pending=False;m.move(State.WAIT,'operator')
    def test_manual_guided_not_exit(self):
        m=Machine();m.confirmed_parked()
        self.assertFalse(m.pending);self.assertEqual(m.state,State.PARKED)
    def test_duplicate_and_concurrent(self):
        m=Machine();m.inside=True;m.move(State.PARKED,'geom')
        self.assertTrue(m.request_start(True,True)[0]);self.assertFalse(m.request_start(True,True)[0])
    def test_charge_queues_no_exit(self):
        m=Machine();m.confirmed_parked();m.move(State.CHARGING,'sim')
        self.assertTrue(m.request_start(True,False)[0]);self.assertEqual(m.state,State.CHARGING)
        with self.assertRaises(RuntimeError):m.move(State.EXIT,'too early')
    def test_invalid_battery_or_mission_stays_parked(self):
        for valid,enough in ((False,True),(True,False)):
            m=Machine();m.confirmed_parked();self.assertFalse(m.request_start(valid,enough)[0]);self.assertEqual(m.state,State.PARKED)
    def test_failure_requires_reconcile(self):
        m=Machine();m.inside=True;m.move(State.FAULT,'lost lidar')
        self.assertFalse(m.request_start(True,True)[0]);m.move(State.PARKED,'fresh geometry')

# Runtime integration uses acknowledgements and reported state through a fake
# link; these are software tests, NOT proof of physical motion or SITL trials.
class FakeLink:
    def __init__(self):
        self.calls=[];self.items=mission();self.latest={'HEARTBEAT':NS(custom_mode=4,base_mode=0)}
        self.sysid=1;self.conn=NS(mav=NS(mission_set_current_send=lambda *a:self.calls.append(('set_current',a[-1]))))
    def download_mission(self):return copy.deepcopy(self.items)
    def mode(self,name):
        self.calls.append(('mode',name));self.latest['HEARTBEAT'].custom_mode={'HOLD':4,'GUIDED':15,'AUTO':10}[name]
    def param(self,k,v=None):self.calls.append(('param',k,v));return 0 if v is None else v
    def command(self,*args):self.calls.append(('command',*args));self.latest['HEARTBEAT'].base_mode=128
    def wait(self,kind,predicate=lambda x:True):
        msg=NS(seq=1,base_mode=128,get_srcComponent=lambda:1)
        if not predicate(msg):raise TimeoutError('unconfirmed')
        self.calls.append(('confirmed',kind));return msg

class RuntimeTests(unittest.TestCase):
    def test_foreign_component_cannot_override_autopilot_state(self):
        from mission_manager.manager import ManagedLink
        link=ManagedLink.__new__(ManagedLink);link.sysid=1;link.tick=lambda:None
        original=NS(custom_mode=4);link.latest={'HEARTBEAT':original};link.last_seen={'HEARTBEAT':123.}
        foreign=NS(get_type=lambda:'HEARTBEAT',get_srcSystem=lambda:1,get_srcComponent=lambda:240)
        link.conn=NS(recv_match=lambda **kw:foreign)
        self.assertIsNone(link.recv());self.assertIs(link.latest['HEARTBEAT'],original)
        self.assertEqual(link.last_seen['HEARTBEAT'],123.)
    def owner(self,inside=True):
        from mission_manager.manager import Manager
        o=Manager.__new__(Manager);o.link=FakeLink();o.machine=Machine();o.machine.inside=inside
        o.machine.state=State.CHARGED if inside else State.WAIT;o.machine.pending=True
        o.items=mission();o.pending_hash=fingerprint(o.items);o.battery=Battery(True,100)
        o.ignore_cancel=False;o.reached=None;o.reached_at=0;o.event=lambda **kw:None
        o.envelope=Envelope()
        o.args=NS(station=None)
        return o
    def fake_motion(self,o,fail=False):
        def leave():
            self.assertEqual(o.link.latest['HEARTBEAT'].custom_mode,15)
            if fail:raise RuntimeError('exit interrupted')
            o.machine.confirmed_outside();o.link.calls.append(('exit_confirmed',))
        return NS(exit=leave,stop=lambda:o.link.mode('HOLD'),restore=lambda:None)
    def test_inside_exit_before_auto(self):
        o=self.owner()
        with patch('mission_manager.manager.Motion',return_value=self.fake_motion(o)):o.start_pending()
        self.assertLess(o.link.calls.index(('exit_confirmed',)),o.link.calls.index(('mode','AUTO')))
        self.assertIn(('confirmed','MISSION_CURRENT'),o.link.calls)
        self.assertEqual(o.machine.state,State.MISSION)
    def test_outside_start_no_exit(self):
        o=self.owner(False)
        with patch('mission_manager.manager.Motion',return_value=self.fake_motion(o)):o.start_pending()
        self.assertNotIn(('exit_confirmed',),o.link.calls);self.assertIn(('mode','AUTO'),o.link.calls)
    def test_exit_duration_is_not_charged_as_auto_mission_time(self):
        o=self.owner();o.battery.last=100.
        with patch('mission_manager.manager.Motion',return_value=self.fake_motion(o)), patch('mission_manager.manager.time.monotonic',return_value=123.):
            o.start_pending()
        o.battery.tick(124.,running=True)
        self.assertAlmostEqual(o.battery.soc,99.98)
    def test_exit_failure_no_auto(self):
        o=self.owner()
        with patch('mission_manager.manager.Motion',return_value=self.fake_motion(o,True)):
            with self.assertRaises(RuntimeError):o.start_pending()
        self.assertNotIn(('mode','AUTO'),o.link.calls)
    def test_missing_guided_confirmation_no_exit(self):
        o=self.owner();old=o.link.mode
        def mode(n):
            if n=='GUIDED':raise TimeoutError('No heartbeat')
            old(n)
        o.link.mode=mode
        with patch('mission_manager.manager.Motion',return_value=self.fake_motion(o)):
            with self.assertRaises(TimeoutError):o.start_pending()
        self.assertNotIn(('exit_confirmed',),o.link.calls);self.assertNotIn(('mode','AUTO'),o.link.calls)
    def test_mission_changed_no_exit(self):
        o=self.owner();o.link.items[-1]['x']+=.0001
        with self.assertRaises(RuntimeError):o.start_pending()
        self.assertNotIn(('mode','GUIDED'),o.link.calls)
    def test_duplicated_service(self):
        o=self.owner(False);o.machine.pending=False;o.busy=False;o.guard=threading.Lock();o.jobs=queue.Queue(maxsize=1)
        self.assertTrue(o.enqueue('start')[0]);self.assertFalse(o.enqueue('start')[0])
    def test_fault_response_contains_actual_failure_reason(self):
        o=self.owner(False);o.machine.pending=False;o.busy=False;o.guard=threading.Lock();o.jobs=queue.Queue(maxsize=1)
        o.machine.state=State.FAULT;o.last_error='Velocidade/atitude desatualizada'
        ok,message=o.enqueue('start_file','/tmp/mission.plan')
        self.assertFalse(ok);self.assertIn(o.last_error,message)
        self.assertIn('reconcile',message);self.assertTrue(o.jobs.empty())
    def test_binding_destination_preserves_physical_exit_station(self):
        import tempfile
        o=self.owner();o.machine.pending=False
        elsewhere=dict(STATION,nome='outra',latitude=STATION['latitude']+.01)
        o.stations=[elsewhere,STATION];o.station=elsewhere
        with tempfile.TemporaryDirectory() as folder:
            o.root=Path(folder);o.bind_current()
        self.assertEqual(o.binding['station'],STATION['nome'])
        self.assertEqual(o.station['nome'],'outra')
    def test_binding_rejects_destination_outside_requested_scope(self):
        o=self.owner();o.machine.pending=False
        elsewhere=dict(STATION,nome='outra',latitude=STATION['latitude']+.01)
        o.stations=[elsewhere,STATION];o.station=elsewhere;o.args.station='outra'
        with self.assertRaisesRegex(RuntimeError,'--station'):o.bind_current()

if __name__=='__main__':unittest.main()
