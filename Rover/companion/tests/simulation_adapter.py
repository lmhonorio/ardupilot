"""Native adapter contract: XYZ/order, range filtering, padding and exact time."""
import signal
import os
import subprocess
import time
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from livox_ros_driver2.msg import CustomMsg

rclpy.init();node=rclpy.create_node('adapter_contract')
source='/companion_test/sim_input';target='/companion_test/livox';visual='/companion_test/xyz'
pub=node.create_publisher(PointCloud2,source,1)
received={}
expected_sec=0
def receive(key,msg):
    if msg.header.stamp.sec==expected_sec:received[key]=msg
node.create_subscription(CustomMsg,target,lambda m:receive('livox',m),1)
node.create_subscription(PointCloud2,visual,lambda m:receive('xyz',m),1)
# Organized FLOAT32 scan with shuffled fields, extra bytes and row padding.
rng=np.random.default_rng(174)
xyz=rng.uniform(-12,12,(1024,3)).astype(np.float32)
xyz[:5]=[[0,0,0],[np.nan,1,1],[np.inf,1,1],[51,0,0],[.2,0,0]]
xyz[5:8]=[[1,2,3],[.35,0,0],[50,0,0]]
packed=np.zeros((32,32,4),np.float32);packed[:,:,0]=xyz[:,2].reshape(32,32)
packed[:,:,2]=xyz[:,0].reshape(32,32);packed[:,:,3]=xyz[:,1].reshape(32,32)
data=b''.join(row.tobytes()+bytes(16) for row in packed)
h=Header(frame_id='mid360_sensor');h.stamp.sec=12345;h.stamp.nanosec=678900123
msg=PointCloud2(header=h,height=32,width=32,point_step=16,row_step=32*16+16,data=data,
               fields=[PointField(name=k,offset=o,datatype=7,count=1) for k,o in [('z',0),('x',8),('y',12)]])
dist=np.linalg.norm(xyz,axis=1);expected=xyz[np.isfinite(xyz).all(axis=1)&(dist>.35)&(dist<50)]
try:
    for stride in (1,8,16):
        received.clear()
        expected_sec=12345+stride
        msg.header.stamp.sec=expected_sec
        proc=subprocess.Popen(['ros2','run','rover_companion_bridge','simulation_adapter','--ros-args',
            '-p','source:='+source,'-p','target:='+target,'-p','visual:='+visual,'-p','stride:='+str(stride)],start_new_session=True)
        try:
            end=time.monotonic()+20;last=0
            while time.monotonic()<end and len(received)<2:
                if time.monotonic()-last>.2:pub.publish(msg);last=time.monotonic()
                rclpy.spin_once(node,timeout_sec=.05)
            assert len(received)==2,'Native adapter did not publish both outputs'
            out=received['livox'];view=received['xyz'];want=expected[::stride]
            assert out.point_num==len(want)==len(out.points),(stride,out.point_num,len(want))
            np.testing.assert_array_equal([(p.x,p.y,p.z) for p in out.points],want)
            np.testing.assert_array_equal(point_cloud2.read_points_numpy(view),want)
            assert out.header.frame_id==view.header.frame_id==h.frame_id
            original_ns=h.stamp.sec*10**9+h.stamp.nanosec
            assert out.timebase==original_ns-100000000
            assert out.header.stamp.sec*10**9+out.header.stamp.nanosec==out.timebase
            assert view.header.stamp==h.stamp
            assert all((p.offset_time,p.reflectivity,p.tag,p.line)==(100000000,1,0,0) for p in out.points)
            print('PASS stride=%d: %d points; padding, fields, invalid ranges, XYZ and exact timestamps'%(stride,len(want)),flush=True)
        finally:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
finally:
    node.destroy_node();rclpy.shutdown()
