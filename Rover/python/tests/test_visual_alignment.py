"""Ensaio geometrico simplificado, sem substituir teste no Gazebo."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking'))
from visual_alignment import VisualAlignment, deve_frear, distancia_frenagem

class VisualAlignmentTest(unittest.TestCase):
    def test_user_pose_reaches_target_in_ideal_kinematic_model(self):
        controller=VisualAlignment(.12)
        pose={'forward':3.8538,'right':1.4685,'normal_yaw':-.4017}
        phases=set(); done=False
        for step in range(1800):
            phase,speed,rate,done=controller.step(pose); phases.add(phase)
            if done: break
            dt=.1; a=rate*dt; f=pose['forward']-speed*dt; r=pose['right']
            pose.update(forward=math.cos(a)*f+math.sin(a)*r,
                        right=-math.sin(a)*f+math.cos(a)*r,
                        normal_yaw=pose['normal_yaw']+a)
        self.assertTrue(done,(phase,pose))
        self.assertIn('aproximacao',phases)
        self.assertIn('entrada',phases)
        self.assertLessEqual(abs(pose['right']),.025)
        self.assertLessEqual(abs(pose['normal_yaw']),math.radians(2))

    def test_reject_overrun_and_nan(self):
        for pose in ({'forward':-.1,'right':0,'normal_yaw':0},
                     {'forward':float('nan'),'right':0,'normal_yaw':0}):
            with self.assertRaises(ValueError): VisualAlignment().step(pose)

    def test_close_aligned_rover_does_not_return_to_staging(self):
        phase,_,_,_=VisualAlignment().step({'forward':1.5,'right':0,'normal_yaw':0})
        self.assertEqual(phase,'entrada')

    def test_approach_between_six_and_eight_metres(self):
        controller=VisualAlignment(.08)
        pose={'forward':7.2,'right':0,'normal_yaw':0}
        for _ in range(3000):
            phase,speed,rate,done=controller.step(pose)
            if done: break
            self.assertEqual(rate,0)
            pose['forward']-=speed*.1
        self.assertTrue(done,pose)

    def test_freia_antes_do_alvo_e_sem_novas_manobras(self):
        self.assertGreater(distancia_frenagem(.2),distancia_frenagem(.05))
        self.assertTrue(deve_frear({'forward':.1},.2))
        self.assertTrue(deve_frear({'forward':-.05},0))
        self.assertFalse(deve_frear({'forward':.5},.2))
        self.assertTrue(deve_frear({'forward':.2},None))
        with self.assertRaises(ValueError):
            deve_frear({'forward':float('nan')},.1)

if __name__=='__main__':unittest.main()
