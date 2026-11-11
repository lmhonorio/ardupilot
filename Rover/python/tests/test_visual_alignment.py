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
        controller=VisualAlignment()
        pose={'forward':.9,'right':0,'normal_yaw':0}
        phase,speed,_,_=controller.step(pose,now=0)
        self.assertEqual(phase,'alinhar')
        self.assertEqual(speed,0)
        controller.step(pose,now=.4)
        phase,_,_,_=controller.step(pose,now=1.2)
        self.assertEqual(phase,'entrada')

    def test_entry_violation_stops_then_rechecks_without_loosening_limit(self):
        c=VisualAlignment();c.phase='entrada'
        bad={'forward':1.76,'right':-.25,'normal_yaw':0}
        phase,v,w,_=c.step(bad,now=0)
        self.assertEqual((v,w),(0,0))
        good={'forward':1.76,'right':-.01,'normal_yaw':0}
        self.assertGreater(c.step(good,now=.2)[1],0)
        self.assertEqual(c.step(bad,now=1)[1],0)
        with self.assertRaisesRegex(ValueError,'lateral=-0.250'):
            c.step(bad,now=2.1)

    def test_stationary_alignment_pivot_has_minimum_rate(self):
        c=VisualAlignment();c.phase='alinhar'
        h=math.radians(-6)
        p={'forward':.9*math.cos(h),'right':-.9*math.sin(h),'normal_yaw':h}
        _,v,w,_=c.step(p,now=0)
        self.assertEqual(v,0);self.assertGreaterEqual(w,.12)
        p['normal_yaw']=math.radians(-4)
        p['right']=-.9*math.sin(p['normal_yaw'])
        self.assertEqual(c.step(p,now=.2)[1],0)
        p['normal_yaw']=math.radians(-1)
        p['right']=-.9*math.sin(p['normal_yaw'])
        self.assertEqual(c.step(p,now=.4)[1],0)
        c.step(p,now=.8)
        self.assertGreater(c.step(p,now=1.5)[1],0)

    def test_translation_stops_before_pivot_and_waits_for_rotation_to_settle(self):
        c=VisualAlignment()
        p=dict(forward=3.,right=.5,normal_yaw=0.,groundspeed=.2,yawrate=0.)
        self.assertEqual(c.step(p,now=1)[1:3],(0.,0.))
        p['groundspeed']=0.
        self.assertEqual(c.step(p,now=1.2)[1],0.)
        self.assertGreater(c.step(p,now=1.3)[2],0.)
        # Near the requested course, an ongoing yaw rate must be braked.
        p.update(normal_yaw=.20,right=-.15,yawrate=.2)
        self.assertEqual(c.step(p,now=1.4)[1:3],(0.,0.))

    def test_offset_approaches_converge_with_separate_turn_and_translation(self):
        for lateral in (-.6,-.3,.3,.6):
            for heading in (-.35,.0,.35):
                c=VisualAlignment(.25)
                along=3.;h=heading;l=lateral
                for _ in range(2500):
                    pose=dict(forward=along*math.cos(h)+l*math.sin(h),
                              right=-along*math.sin(h)+l*math.cos(h),normal_yaw=h)
                    _,v,w,done=c.step(pose)
                    if along<.06:break
                    along-=v*math.cos(h)*.1;l-=v*math.sin(h)*.1;h+=w*.1
                self.assertLess(along,.06,(along,l,h))
                self.assertLess(abs(l),.04,(along,l,h))

    def test_heading_converges_when_skid_friction_blocks_turns_while_moving(self):
        # A plant model, not an ideal unicycle: turning is ineffective while
        # rolling, and speed/yaw respond with a 0.35 s lag as observed in SITL.
        for lateral,heading in ((.5,-.35),(-.5,.35),(.1,-.44),(-.1,.44)):
            c=VisualAlignment(.25)
            along=3.;h=heading;l=lateral;v=0.;w=0.
            for step in range(6000):
                pose=dict(forward=along*math.cos(h)+l*math.sin(h),
                          right=-along*math.sin(h)+l*math.cos(h),normal_yaw=h,
                          groundspeed=abs(v),yawrate=w)
                if c.phase=='entrada' and deve_frear(pose,abs(v)):break
                _,target_v,target_w,_=c.step(pose,now=step*.05)
                v+=(target_v-v)*.05/.35
                effective_w=target_w if abs(v)<.05 else 0.
                w+=(effective_w-w)*.05/.35
                along-=v*math.cos(h)*.05;l-=v*math.sin(h)*.05;h+=w*.05
            self.assertLess(along,.06,(along,l,h,c.phase))
            self.assertLess(abs(l),.04,(along,l,h))
            self.assertLess(abs(h),math.radians(3),(along,l,h))

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
        self.assertFalse(deve_frear({'forward':.066},.04))
        self.assertTrue(deve_frear({'forward':.03},.04))
        with self.assertRaises(ValueError):
            deve_frear({'forward':float('nan')},.1)

if __name__=='__main__':unittest.main()
