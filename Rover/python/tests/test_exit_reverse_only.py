"""Exercise Motion.exit with synthetic measurements, without moving a vehicle."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_manager.motion import Motion
from mission_manager.geometry import Envelope
from mission_manager.policy import Machine
import test_mission_manager as fixtures


class ReverseExitTests(unittest.TestCase):
    def motion(self, geometric_back=3., stopped_back=3., stopped_yaw=0., lio_scale=1.):
        machine=Machine();machine.inside=True;machine.pending=True
        owner=NS(link=fixtures.FakeLink(),envelope=Envelope(),station=fixtures.STATION,
                 machine=machine,args=NS(exit_timeout=60,sensor_recovery=10),event=Mock())
        m=Motion(owner);m.drain=Mock();m.check=Mock();m.tune=Mock();m.allow_inside=Mock()
        m.command=Mock();m.stop=Mock(side_effect=lambda:owner.link.mode('HOLD'))
        initial=fixtures.pose();initial['stamp']=0.
        m.pose=Mock(side_effect=[initial,fixtures.pose(geometric_back)])
        def obs(back,yaw=0.):return np.array([-back,0.,0.]),yaw,np.empty((0,3))
        samples=[obs(0),obs(0)]+[obs(i/10*lio_scale) for i in range(1,31)]+[obs(stopped_back,stopped_yaw)]
        m.exit_observation=Mock(side_effect=samples)
        walls=[]
        for i in range(1,31):
            p=fixtures.pose(i/10);p['stamp']=i*.5
            walls.append((p,np.empty((0,3))))
        owner.perception=NS(snapshot=Mock(side_effect=walls))
        return m

    def test_three_metres_then_hold_without_pivot(self):
        m=self.motion();m.exit()
        self.assertIs(m.o.machine.inside,False)
        calls=[c for c in m.link.calls if c[0]=='mode']
        self.assertEqual(calls,[('mode','GUIDED'),('mode','HOLD')])
        self.assertTrue(m.command.call_args_list)
        self.assertTrue(all(c.args[0]<0 and c.args[1]==0 for c in m.command.call_args_list))
        checked=next(c.kwargs for c in m.o.event.call_args_list if c.kwargs['kind']=='exit_check')
        self.assertEqual(checked['back'],3.)
        self.assertEqual(checked['strategy'],'reverse_only')
        # No AUTO command here: that remains after confirmation in the manager.
        self.assertNotIn(('mode','AUTO'),m.link.calls)

    def test_brakes_near_three_not_two_metres(self):
        m=self.motion();m.exit()
        speeds=[c.args[0] for c in m.command.call_args_list]
        self.assertEqual(speeds[19],-.12)  # at 2 m, one metre remains
        self.assertLess(abs(speeds[-1]),abs(speeds[19]))

    def test_accepts_stopping_tolerance_near_three(self):
        m=self.motion(geometric_back=2.97,stopped_back=3.04);m.exit()
        self.assertIs(m.o.machine.inside,False)

    def test_insufficient_geometric_distance_blocks_completion(self):
        m=self.motion(geometric_back=2.)
        with self.assertRaisesRegex(RuntimeError,'3 m'):m.exit()
        self.assertIs(m.o.machine.inside,True)

    def test_overshoot_blocks_completion(self):
        m=self.motion(geometric_back=3.3)
        with self.assertRaisesRegex(RuntimeError,'3 m'):m.exit()
        self.assertIs(m.o.machine.inside,True)

    def test_lio_translation_drift_does_not_extend_reverse(self):
        m=self.motion(lio_scale=.8,stopped_back=2.4);m.exit()
        self.assertIs(m.o.machine.inside,False)
        checked=next(c.kwargs for c in m.o.event.call_args_list if c.kwargs['kind']=='exit_check')
        self.assertAlmostEqual(checked['back'],2.4)
        self.assertAlmostEqual(checked['geometric_back'],3.)
        self.assertEqual(len(m.command.call_args_list),29)

    def test_no_lio_displacement_blocks_completion(self):
        m=self.motion(lio_scale=0.,stopped_back=0.)
        with self.assertRaisesRegex(RuntimeError,'3 m'):m.exit()
        self.assertIs(m.o.machine.inside,True)

    def test_wall_alignment_loss_blocks_motion(self):
        m=self.motion();p=fixtures.pose(.1);p.update(right=.2,stamp=1.)
        m.o.perception.snapshot.side_effect=[(p,np.empty((0,3)))]
        with self.assertRaisesRegex(RuntimeError,'paredes'):m.exit()
        m.command.assert_not_called()

    def test_wall_pose_jump_blocks_motion(self):
        m=self.motion();p=fixtures.pose(2.);p['stamp']=.1
        m.o.perception.snapshot.side_effect=[(p,np.empty((0,3)))]
        with self.assertRaisesRegex(RuntimeError,'Salto'):m.exit()
        m.command.assert_not_called()

    def test_final_heading_error_blocks_completion(self):
        m=self.motion(stopped_yaw=.3)
        with self.assertRaisesRegex(RuntimeError,'alinhamento'):m.exit()
        self.assertIs(m.o.machine.inside,True)

    def test_obstacle_blocks_reverse_and_auto(self):
        m=self.motion()
        with patch('mission_manager.motion.obstacle_in_sweep',return_value=True):
            with self.assertRaisesRegex(RuntimeError,'Obstaculo'):m.exit()
        m.command.assert_not_called();self.assertIs(m.o.machine.inside,True)

    def test_no_request_means_no_motion(self):
        m=self.motion();m.o.machine.pending=False
        with self.assertRaisesRegex(RuntimeError,'solicitacao'):m.exit()
        self.assertEqual(m.link.calls,[])


if __name__=='__main__':unittest.main()
