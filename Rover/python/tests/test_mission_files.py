"""File-start regressions: protocol, automatic binding, charge and exit ordering."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_manager.files import load_mission, verify_readback
from mission_manager.manager import Manager, ManagedLink, RequestRejected
from mission_manager.policy import State
from mission_manager.motion import Motion
from mission_manager.mission import requested
import test_mission_manager as fixtures
STATION,mission=fixtures.STATION,fixtures.mission


def wpl(items):
    lines=['QGC WPL 110']
    for i in items:
        fields=[i['seq'],int(i['seq']==0),i['frame'],i['command']]
        fields += [i.get('param%d'%k,0) for k in range(1,5)]
        fields += [i['x'],i['y'],i['z'],i['autocontinue']]
        lines.append('\t'.join(map(str,fields)))
    return '\n'.join(lines)+'\n'


class FileTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'test.waypoints'
        self.path.write_text(wpl(mission()))
    def test_wpl_and_plan_equivalent(self):
        a=load_mission(self.path)
        item=dict(type='SimpleItem',command=16,frame=3,autoContinue=True,
                  params=[0,0,0,0,a[1]['x'],a[1]['y'],0])
        self.path.write_text(json.dumps(dict(fileType='Plan',mission=dict(
            plannedHomePosition=[a[0]['x'],a[0]['y'],0],items=[item]))))
        b=load_mission(self.path)
        self.assertEqual(a[1:],b[1:])
    def test_takeoff_rejected(self):
        m=mission();m[1]['command']=22;self.path.write_text(wpl(m))
        with self.assertRaisesRegex(ValueError,'waypoint'):load_mission(self.path)
    def test_bad_sequence_and_nonfinite_and_pause_rejected(self):
        for change in ({'seq':4},{'param2':float('nan')},{'autocontinue':0}):
            m=mission();m[1].update(change);self.path.write_text(wpl(m))
            with self.assertRaises(ValueError):load_mission(self.path)
    def test_complex_plan_rejected(self):
        self.path.write_text(json.dumps(dict(fileType='Plan',mission=dict(
            plannedHomePosition=[0,0,0],items=[dict(type='ComplexItem')]))))
        with self.assertRaisesRegex(ValueError,'complexos'):load_mission(self.path)
    def test_full_parameters_preserved_on_wire(self):
        m=mission();m[1].update(param1=1,param2=.25,param3=.3,param4=4)
        self.path.write_text(wpl(m));item=load_mission(self.path)[1]
        link=ManagedLink.__new__(ManagedLink);link.sysid=1;link.compid=1
        link.conn=NS(mav=Mock());link.send_item(item)
        args=link.conn.mav.mission_item_int_send.call_args.args
        self.assertEqual(args[7:11],(1,.25,.3,4))
        self.assertEqual(args[11],round(item['x']*1e7))
    def test_readback_float_precision_and_frame_aliases(self):
        a=load_mission(self.path);a[1]['param2']=.1
        b=copy.deepcopy(a);b[0]['x']=0;b[1]['frame']=6;b[1]['param2']=.10000000149011612
        b[1]['x']=round(b[1]['x']*1e7)/1e7
        verify_readback(a,b)
    def test_readback_checks_params_coords_commands_count(self):
        a=load_mission(self.path)
        for change in ({'param2':.2},{'param4':1},{'x':0},{'command':17}):
            b=copy.deepcopy(a);b[1].update(change)
            with self.assertRaises(ValueError):verify_readback(a,b)
        with self.assertRaises(ValueError):verify_readback(a,a[:-1])


class FileStartTests(unittest.TestCase):
    def setUp(self):
        self.helper=fixtures.RuntimeTests()
        self.o=self.helper.owner(False);o=self.o
        o.machine.pending=False;o.stations=[STATION];o.station=STATION
        o.args.simulate_charge=True;o.fresh=lambda:True;o.motion_state=lambda:(0.,0.)
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        o.root=Path(self.tmp.name);self.path=o.root/'mission.waypoints';self.path.write_text(wpl(mission()))
        def upload(items):
            o.link.calls.append(('upload',));o.link.items=copy.deepcopy(items)
        o.link.upload_mission=Mock(side_effect=upload)
    def run_start(self):
        with patch('mission_manager.manager.Motion',return_value=self.helper.fake_motion(self.o)):
            self.o.request_start(str(self.path))
    def test_file_upload_autobind_start_outside(self):
        self.run_start();o=self.o
        self.assertEqual(o.machine.state,State.MISSION)
        self.assertEqual(o.binding['station'],STATION['nome'])
        self.assertTrue(requested(o.binding,o.items,1,1,10,2.5))
        self.assertEqual(len(list(o.root.glob('mission_backup_*.json'))),1)
        self.assertNotIn(('exit_confirmed',),o.link.calls)
    def test_inside_exit_then_auto(self):
        o=self.o;o.machine.inside=True;o.machine.state=State.CHARGED
        self.run_start()
        self.assertLess(o.link.calls.index(('exit_confirmed',)),o.link.calls.index(('mode','AUTO')))
        self.assertIsNotNone(o.binding)
    def test_legacy_trigger_also_autobinds(self):
        with patch('mission_manager.manager.Motion',return_value=self.helper.fake_motion(self.o)):
            self.o.request_start()
        self.assertIsNotNone(self.o.binding);self.o.link.upload_mission.assert_not_called()
    def test_invalid_file_does_not_upload_or_arm(self):
        self.path.write_text('not a mission')
        with self.assertRaises(RequestRejected):self.run_start()
        self.o.link.upload_mission.assert_not_called();self.assertEqual(self.o.link.calls,[])
    def test_wrong_destination_does_not_upload(self):
        m=mission();m[1]['y']+=.001;self.path.write_text(wpl(m))
        with self.assertRaises(RequestRejected):self.run_start()
        self.o.link.upload_mission.assert_not_called()
    def test_low_battery_does_not_upload(self):
        self.o.battery.soc=5
        with self.assertRaisesRegex(RequestRejected,'Bateria'):self.run_start()
        self.o.link.upload_mission.assert_not_called()
    def test_changed_readback_does_not_move(self):
        def upload(items):
            self.o.link.items=copy.deepcopy(items);self.o.link.items[1]['param2']=5
        self.o.link.upload_mission.side_effect=upload
        with self.assertRaisesRegex(ValueError,'Readback'):self.run_start()
        self.assertNotIn(('mode','AUTO'),self.o.link.calls)
        self.assertNotIn(('mode','GUIDED'),self.o.link.calls)
    def test_queued_during_charge_only_exits_after_15_seconds(self):
        o=self.o;o.machine.inside=True;o.machine.state=State.PARKED;o.battery.soc=10
        with patch('mission_manager.manager.time.monotonic',return_value=100.):
            o.begin_charge_if_parked();self.run_start()
        self.assertTrue(o.machine.pending);self.assertEqual(o.machine.state,State.CHARGING)
        self.assertNotIn(('mode','GUIDED'),o.link.calls)
        self.assertFalse(o.battery.tick(114.9));self.assertTrue(o.battery.tick(115.))
        o.machine.move(State.CHARGED,'test')
        with patch('mission_manager.manager.Motion',return_value=self.helper.fake_motion(o)):
            o.start_pending()
        self.assertEqual(o.battery.soc,100.)
        self.assertLess(o.link.calls.index(('exit_confirmed',)),o.link.calls.index(('mode','AUTO')))
    def test_bad_file_preserves_charge(self):
        o=self.o;o.machine.inside=True;o.machine.state=State.PARKED;o.battery.soc=10
        o.begin_charge_if_parked();started=o.battery.started;self.path.write_text('invalid')
        with self.assertRaises(RequestRejected):self.run_start()
        self.assertEqual(o.battery.started,started);self.assertEqual(o.machine.state,State.CHARGING)
    def test_charge_requires_disarmed_and_hold(self):
        o=self.o;o.machine.inside=True;o.machine.state=State.PARKED
        o.link.latest['HEARTBEAT'].base_mode=128
        with self.assertRaisesRegex(RuntimeError,'desarmamento'):o.begin_charge_if_parked()
        self.assertIsNone(o.battery.started)

    def test_start_waits_for_fresh_motion_before_upload(self):
        self.o.motion_state=Mock(side_effect=[RuntimeError('Velocidade/atitude desatualizada'),(0.,0.)])
        self.o.link.recv=Mock()
        self.run_start()
        self.o.link.recv.assert_called_once_with(.05)
        self.assertEqual(self.o.machine.state,State.MISSION)
        self.o.link.upload_mission.assert_called_once()

    def test_stale_start_is_rejected_without_fault_or_upload(self):
        o=self.o;o.machine.inside=True;o.machine.state=State.CHARGING;o.battery.started=1.
        o.motion_state=Mock(side_effect=RuntimeError('Velocidade/atitude desatualizada'))
        o.link.recv=Mock();o.args.mode_timeout=0
        with self.assertRaisesRegex(RequestRejected,'antes do upload'):self.run_start()
        o.link.upload_mission.assert_not_called()
        self.assertEqual(o.machine.state,State.CHARGING)
        self.assertEqual(o.battery.started,1.)
        self.assertFalse(o.machine.pending)

    def test_motion_resuming_during_wait_rejects_request(self):
        self.o.motion_state=Mock(side_effect=[RuntimeError('Velocidade/atitude desatualizada'),(.2,0.)])
        self.o.link.recv=Mock()
        with self.assertRaisesRegex(RequestRejected,'parada completa'):self.run_start()
        self.o.link.upload_mission.assert_not_called()

    def test_nonfinite_motion_rejects_request(self):
        self.o.motion_state=lambda:(float('nan'),0.)
        with self.assertRaisesRegex(RequestRejected,'nao finitas'):self.run_start()
        self.o.link.upload_mission.assert_not_called()


class ExitFreshnessTests(unittest.TestCase):
    def test_reverse_holds_command_between_fresh_repeated_observations(self):
        import numpy as np
        from mission_manager.geometry import Envelope
        def observation(back,stamp):return dict(forward=back,right=0.,normal_yaw=0.,stamp=stamp)
        samples=[observation(i*.1,1+i*.2) for i in range(1,31) for _ in range(2 if i<30 else 1)]
        current=[0.]
        def snapshot():
            p=samples.pop(0);current[0]=p['forward'];return p,np.empty((0,3))
        owner=NS(link=Mock(),envelope=Envelope(),station=STATION,machine=Mock(pending=True),
                 args=NS(exit_timeout=180,sensor_recovery=10),event=Mock(),
                 perception=NS(snapshot=snapshot))
        m=Motion(owner);m.command=Mock();m.check=Mock();m.drain=Mock()
        m.tune=Mock();m.allow_inside=Mock();m.stop=Mock()
        m.pose=Mock(side_effect=[observation(0.,1.),observation(3.,7.)])
        m.exit_observation=lambda **kw:(np.array([-current[0],0.,0.]),0.,np.empty((0,3)))
        m.exit()
        owner.machine.confirmed_outside.assert_called_once()
        m.stop.assert_called_once()
        self.assertTrue(m.command.call_args_list)
        self.assertTrue(all(c.args[0]<0 and c.args[1]==0 for c in m.command.call_args_list),
                        'A repeated but still-fresh pose must not insert stop pulses')

    def motion(self, errors):
        o=NS(args=NS(sensor_recovery=1.),event=Mock(),
             perception=NS(sensor=NS(lio=Mock(side_effect=errors)),sweep=Mock(return_value=['fresh scan'])))
        m=Motion.__new__(Motion);m.o=o;m.command=Mock();m.drain=Mock();m.check=Mock()
        return m
    def test_transient_lag_zeroes_command_until_fresh(self):
        m=self.motion([ValueError('Timestamp FAST-LIO invalido'),([1,2,3],.1)])
        self.assertEqual(m.exit_observation(),([1,2,3],.1,['fresh scan']))
        m.command.assert_called_once_with(0,0)
        m.o.perception.sweep.assert_called_once()
    def test_persistent_lag_times_out_with_zero_command(self):
        m=self.motion([ValueError('Timestamp FAST-LIO invalido')])
        with patch('mission_manager.motion.time.monotonic',side_effect=[0.,2.]):
            with self.assertRaisesRegex(RuntimeError,'nao recuperaram'):m.exit_observation()
        m.command.assert_called_once_with(0,0)
    def test_invalid_covariance_not_treated_as_latency(self):
        m=self.motion([ValueError('FAST-LIO inconsistente: covariancia de posicao excessiva')])
        with self.assertRaisesRegex(ValueError,'covariancia'):m.exit_observation()
        m.command.assert_called_once_with(0,0)

    def test_stop_keeps_hold_and_waits_for_fresh_stationary_samples(self):
        m=Motion.__new__(Motion);m.command=Mock();m.link=Mock();m.drain=Mock();m.check=Mock()
        m.o=NS(motion_state=Mock(side_effect=[RuntimeError('Velocidade/atitude desatualizada'),(0.,0.),(0.,0.)]))
        with patch('mission_manager.motion.time.monotonic',side_effect=[0.,.1,.3,.4,.5,.8,1.2]):m.stop()
        m.link.mode.assert_called_once_with('HOLD')
        self.assertEqual(m.command.call_count,2)

    def test_stop_does_not_confirm_persistent_stale_motion(self):
        m=Motion.__new__(Motion);m.command=Mock();m.link=Mock();m.drain=Mock();m.check=Mock()
        m.o=NS(motion_state=Mock(side_effect=RuntimeError('Velocidade/atitude desatualizada')))
        with patch('mission_manager.motion.time.monotonic',side_effect=[0.,.1,16.]):
            with self.assertRaisesRegex(RuntimeError,'Parada nao confirmada'):m.stop(disarm=True)
        m.link.disarm.assert_not_called()


if __name__=='__main__':unittest.main()
