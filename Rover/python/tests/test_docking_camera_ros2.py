"""Do not compute QR pose with old calibration or stale ROS 2 sensor data."""
import copy
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.docking_camera import DockingCamera, validate_pair
from common import ros2_runtime as ros


class CameraTests(unittest.TestCase):
    def setUp(self):
        header=types.SimpleNamespace(frame_id='camera_optical',
                                     stamp=types.SimpleNamespace(sec=10,nanosec=0))
        self.image=types.SimpleNamespace(width=640,height=360,header=header)
        self.info=types.SimpleNamespace(width=640,height=360,header=copy.deepcopy(header),
                                       k=[300.,0,320.,0,300.,180.,0,0,1.],d=[0.]*5,
                                       distortion_model='plumb_bob')

    def test_matching_calibration(self):
        validate_pair(self.image,self.info)

    def test_resolution_change_rejects_old_calibration(self):
        self.info.width=1280
        with self.assertRaisesRegex(RuntimeError,'Resolucao'):validate_pair(self.image,self.info)

    def test_wrong_frame_or_old_camera_info(self):
        self.info.header.frame_id='insta360'
        with self.assertRaisesRegex(RuntimeError,'frames'):validate_pair(self.image,self.info)
        self.info.header.frame_id='camera_optical'
        self.info.header.stamp.sec=8
        with self.assertRaisesRegex(RuntimeError,'instante'):validate_pair(self.image,self.info)

    def test_invalid_calibration(self):
        self.info.k[0]=float('nan')
        with self.assertRaisesRegex(RuntimeError,'calibracao'):validate_pair(self.image,self.info)
        self.info.k[0]=300.
        self.info.distortion_model='equidistant'
        with self.assertRaisesRegex(RuntimeError,'distorcao'):validate_pair(self.image,self.info)

    def test_snapshot_rejects_stale_or_pre_pan_image_before_conversion(self):
        camera=DockingCamera.__new__(DockingCamera)
        camera.lock=threading.Lock();camera.image=self.image;camera.info=self.info
        camera.wall=time.monotonic();camera.bridge=Mock()
        with patch.object(ros,'age',return_value=2):
            with self.assertRaisesRegex(RuntimeError,'Timestamp'):camera.snapshot()
        with patch.object(ros,'age',return_value=.1):
            with self.assertRaisesRegex(RuntimeError,'apos mover'):camera.snapshot(camera.wall+1)
            camera.wall-=2
            with self.assertRaisesRegex(RuntimeError,'desatualizada'):camera.snapshot()
        camera.bridge.imgmsg_to_cv2.assert_not_called()

    def test_nonfinite_joint_feedback_is_not_a_valid_camera_angle(self):
        joints=ros.JointFeedback.__new__(ros.JointFeedback)
        joints._wall=time.monotonic()
        joints._data=types.SimpleNamespace(header=self.image.header,name=['pan'],position=[float('nan')])
        with patch.object(ros,'age',return_value=0):
            with self.assertRaisesRegex(RuntimeError,'invalida'):joints.position('pan')

    def test_camera_pose_uses_joint_angle_at_exposure_not_latest_angle(self):
        joints=ros.JointFeedback.__new__(ros.JointFeedback)
        def sample(t,angle):
            return types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10,nanosec=int(t*1e9))),
                name=['pan'],position=[angle])
        joints._history=[sample(0.,.1),sample(.1,.3),sample(.2,.5)]
        joints._data=joints._history[-1];joints._wall=time.monotonic()
        with patch.object(ros,'age',return_value=0):
            stamp=types.SimpleNamespace(sec=10,nanosec=50000000)
            self.assertAlmostEqual(joints.position('pan',stamp),.2)
            self.assertAlmostEqual(joints.position('pan'),.5)
            stamp.sec=8
            with self.assertRaisesRegex(RuntimeError,'sincronizada'):joints.position('pan',stamp)

if __name__=='__main__':unittest.main()
