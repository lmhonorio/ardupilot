import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_manager.charging import SITLCharger
from mission_manager.policy import Battery
from pymavlink import mavutil


class NativeChargeTests(unittest.TestCase):
    def owner(self):
        params={'SIM_BATT_VOLTAGE':12.6,'SIM_BATT_CAP_AH':0,'BATT_MONITOR':4,'BATT_CAPACITY':3300}
        link=NS(latest={'HEARTBEAT':NS(custom_mode=4,base_mode=0)},param=Mock(side_effect=params.__getitem__),command=Mock())
        def wait(kind,predicate,timeout):
            msg=NS(id=0,battery_remaining=100,current_consumed=0)
            if not predicate(msg):raise TimeoutError('missing confirmation')
            return msg
        link.wait=Mock(side_effect=wait)
        return NS(link=link,args=NS(simulate_charge=True),machine=NS(inside=True),fresh=lambda:True,
                  motion_state=lambda:(0.,0.),event=Mock())
    def test_native_soc_comes_from_telemetry_not_initial_value(self):
        b=Battery(True,100,native=True)
        self.assertIsNone(b.operational(10))
        b.observe(39,10);b.begin_charge(10,True,True)
        b.tick(17.5);self.assertEqual(b.soc,69.5)
        b.observe(69,17.5);self.assertEqual(b.operational(17.5),69)
        self.assertTrue(b.tick(25));self.assertEqual(b.soc,100)
        self.assertIsNone(b.operational(25))  # no confirmation, no invented 100%
        b.observe(100,25);self.assertEqual(b.operational(25),100)
    def test_native_discharge_uses_autopilot(self):
        b=Battery(True,native=True);b.observe(81,10);b.tick(10,True);b.tick(11,True)
        self.assertEqual(b.operational(11),81)
        b.observe(79,12);self.assertEqual(b.operational(12),79)
    def test_missing_battery_refuses_begin(self):
        with self.assertRaisesRegex(ValueError,'BATTERY_STATUS'):Battery(True,native=True).begin_charge(10,True,True)
    def test_command_updates_estimator_and_confirms_both_messages(self):
        o=self.owner();c=SITLCharger(o);c.prepare();c.sync(100,10,final=True)
        self.assertEqual(o.link.command.call_args_list[0].args,(mavutil.mavlink.MAV_CMD_BATTERY_RESET,1,100.))
        self.assertEqual([a.args[0] for a in o.link.wait.call_args_list],['BATTERY_STATUS','SYS_STATUS'])
    def test_guards_block_reset(self):
        for condition in ('armed','outside','mode','stale','moving','disabled'):
            o=self.owner();c=SITLCharger(o);c.prepare()
            if condition=='armed':o.link.latest['HEARTBEAT'].base_mode=128
            elif condition=='outside':o.machine.inside=False
            elif condition=='mode':o.link.latest['HEARTBEAT'].custom_mode=15
            elif condition=='stale':o.fresh=lambda:False
            elif condition=='moving':o.motion_state=lambda:(.2,0.)
            else:o.args.simulate_charge=False
            with self.assertRaises(RuntimeError):c.sync(100,10,True)
            o.link.command.assert_not_called()
    def test_unsupported_monitor_refused(self):
        o=self.owner();o.link.param.side_effect=lambda k:10 if k=='BATT_MONITOR' else 12
        with self.assertRaisesRegex(RuntimeError,'BATT_MONITOR'):SITLCharger(o).prepare()
        o.link.command.assert_not_called()
    def test_missing_telemetry_confirmation_is_failure(self):
        o=self.owner();o.link.wait.side_effect=TimeoutError('no telemetry');c=SITLCharger(o);c.prepare()
        with self.assertRaises(TimeoutError):c.sync(100,10,True)
        o.event.assert_not_called()
    def test_sys_status_must_confirm_full(self):
        o=self.owner();c=SITLCharger(o);c.prepare()
        def wait(kind,predicate,timeout):
            m=NS(id=0,battery_remaining=100 if kind=='BATTERY_STATUS' else 39,current_consumed=0)
            if not predicate(m):raise TimeoutError('SYS_STATUS not full')
            return m
        o.link.wait.side_effect=wait
        with self.assertRaises(TimeoutError):c.sync(100,10,True)
        o.event.assert_not_called()
    def test_native_reset_refusal_is_failure(self):
        o=self.owner();c=SITLCharger(o);c.prepare();o.link.command.side_effect=RuntimeError('denied')
        with self.assertRaises(RuntimeError):c.sync(100,10,True)
        o.link.wait.assert_not_called()


if __name__=='__main__':unittest.main()
