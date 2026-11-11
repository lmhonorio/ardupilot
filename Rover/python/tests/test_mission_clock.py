import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_manager.clock_sync import BootClock
from mission_manager.manager import Manager


class ClockTests(unittest.TestCase):
    def sample(self,c,t,offset=2.847,rtt=.02):
        send=Mock();c.request(100+t+offset,t,send)
        token=send.call_args.args[1]
        return c.response(int((100+t+rtt/2)*1e9),token,100+t+offset+rtt,t+rtt)
    def synced(self):
        c=BootClock()
        for t in (0.,.3,.6):self.sample(c,t)
        return c
    def test_measures_constant_offset_without_increasing_age_limit(self):
        c=self.synced();self.assertAlmostEqual(c.offset,2.847)
        self.assertAlmostEqual(c.age(101.,103.947,1.1),.1)
        self.assertAlmostEqual(c.age(100.,103.947,1.1),1.1)
    def test_requires_three_matched_samples(self):
        c=BootClock();self.sample(c,0.);self.sample(c,.3)
        with self.assertRaises(ValueError):c.age(100,102.847,.4)
    def test_foreign_response_cannot_set_clock(self):
        c=BootClock();self.assertFalse(c.response(100000000000,123,102.847,1.))
        self.assertIsNone(c.offset)
    def test_slow_roundtrip_is_not_used(self):
        c=BootClock();self.assertFalse(self.sample(c,0.,rtt=.4));self.assertIsNone(c.offset)
    def test_expired_sync_is_rejected(self):
        c=self.synced()
        with self.assertRaisesRegex(ValueError,'desatualizada'):c.age(100,102.847,40.)
    def test_clock_reset_requires_restart(self):
        c=self.synced();c.request(1.,1.,Mock())
        with self.assertRaisesRegex(ValueError,'reiniciado'):c.age(100,102.847,1.)
    def test_offset_jump_is_not_absorbed_as_delay(self):
        c=self.synced();self.assertFalse(self.sample(c,6.,offset=4.))
        with self.assertRaisesRegex(ValueError,'Salto'):c.age(100,104.,6.1)
    def test_manager_accepts_fresh_mapped_timestamp_but_rejects_old_one(self):
        o=Manager.__new__(Manager);o.clock_sync=self.synced()
        o.link=NS(last_seen={'ATTITUDE':1.,'VFR_HUD':1.},latest={
            'ATTITUDE':NS(time_boot_ms=101000,yawspeed=0.),'VFR_HUD':NS(groundspeed=0.)})
        with patch('mission_manager.manager.time.monotonic',return_value=1.1),patch('mission_manager.manager.ros.now',return_value=NS(sec=103,nanosec=947000000)):
            self.assertEqual(o.motion_state(),(0.,0.))
            o.link.latest['ATTITUDE'].time_boot_ms=100000
            with self.assertRaisesRegex(RuntimeError,'idade corrigida'):o.motion_state()


if __name__=='__main__':unittest.main()
