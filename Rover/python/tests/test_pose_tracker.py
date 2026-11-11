import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking'))
from pose_tracker import PoseTracker, PosePending
from visual_alignment import VisualAlignment


def raw(h=0,f=2.,r=0.):
    return dict(normal_yaw=h,marker_forward=f,marker_right=r,forward=f-.5335,
                right=r,down=0.,reprojection_px=.2,camera_range=f)


class TrackingTests(unittest.TestCase):
    def warm(self):
        t=PoseTracker()
        for i in range(8):
            try:t.update(raw(),i*.2,0.)
            except PosePending:pass
        return t

    def test_bootstrap_and_repeated_image_cannot_authorize_motion(self):
        t=PoseTracker()
        with self.assertRaises(PosePending):t.update(raw(),1,0)
        with self.assertRaisesRegex(PosePending,'repetido'):t.update(raw(),1,0)

    def test_wall_is_filtered_in_fixed_frame_while_rover_turns(self):
        t=self.warm();yaw=.08
        p=raw(h=yaw,f=2*math.cos(yaw),r=-2*math.sin(yaw))
        result=t.update(p,1.6,yaw)
        self.assertAlmostEqual(result['normal_yaw'],yaw)

    def test_outlier_stops_without_poisoning_wall_orientation(self):
        t=self.warm()
        with self.assertRaises(PosePending):t.update(raw(h=.3),1.6,0)
        p=t.update(raw(),1.8,0)
        self.assertAlmostEqual(p['normal_yaw'],0)
        self.assertAlmostEqual(p['right'],0)

    def test_turning_does_not_rotate_wall_from_biased_image_normals(self):
        t=self.warm()
        for i in range(12):
            yaw=.02*(i+1)
            p=raw(h=yaw+.08,f=2*math.cos(yaw),r=-2*math.sin(yaw))
            result=t.update(p,1.6+i*.2,yaw,stationary=False)
            self.assertAlmostEqual(result['normal_yaw'],yaw)
        for i in range(20):
            p=raw(h=yaw+.04,f=2*math.cos(yaw),r=-2*math.sin(yaw))
            result=t.update(p,4.+i*.2,yaw,stationary=True)
        self.assertAlmostEqual(t.axis,.04,delta=.003)

    def test_consistent_real_displacement_is_not_hidden(self):
        t=self.warm()
        p=t.update(raw(r=.09),1.6,0)
        self.assertAlmostEqual(p['right'],.09)
        with self.assertRaises(PosePending):t.update(raw(r=.6),1.8,0)

    def test_replay_captured_entry_failures(self):
        runs=json.loads((Path(__file__).with_name('fixtures')/'pose_entry_failures.json').read_text())
        accepted=0
        # Each short recorded retry contains the PnP jump that ended its run.
        for run in runs:
            if len(run['samples'])>30:continue
            tracker=PoseTracker(.5335);control=VisualAlignment();last=None
            for sample in run['samples']:
                try:last=tracker.update(sample['raw'],sample['t'],sample['yaw'])
                except PosePending:continue
                accepted+=1
                control.step(last,now=sample['t'])
                lateral=last['forward']*math.sin(last['normal_yaw'])+last['right']*math.cos(last['normal_yaw'])
                self.assertLess(abs(lateral),.20,run['run'])
        self.assertGreater(accepted,0)

if __name__=='__main__':unittest.main()
