#!/usr/bin/python3.8
"""FAST-LIO 3D voxel map and recent, motion-compensated cloud for docking.

This consumes LiDAR/IMU odometry; no Gazebo truth, GPS or 2D LaserScan enters
mapping. Local geometry expires independently of the persistent display map.
"""
import argparse
from collections import OrderedDict
from datetime import datetime
import json
from pathlib import Path
import sys
import threading
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.ros_cloud import cloud_xyz
from common.cloud3d import checked_pose,LocalCloud,voxel_points


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--window',type=float,default=1.5,help='Janela local recente, segundos ROS')
    p.add_argument('--voxel',type=float,default=.04)
    p.add_argument('--max-points',type=int,default=250000)
    p.add_argument('--duration',type=float,default=0)
    p.add_argument('--output-dir',type=Path)
    a=p.parse_args()
    if not .2<=a.window<=3 or not .01<=a.voxel<=.2 or not 1000<=a.max_points<=1000000 or a.duration<0:p.error('Limites de mapa invalidos')
    import rospy,message_filters
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header,String
    rospy.init_node('mid360_slam_3d')
    output=a.output_dir or Path(__file__).resolve().parents[1]/'run/slam_3d'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=True)
    local_pub=rospy.Publisher('/rover_slam_3d/local_cloud',PointCloud2,queue_size=1)
    map_pub=rospy.Publisher('/rover_slam_3d/map',PointCloud2,queue_size=1,latch=True)
    status_pub=rospy.Publisher('/rover_slam_3d/status',String,queue_size=1,latch=True)
    lock=threading.RLock();recent=LocalCloud(a.window);voxels=OrderedDict()
    state={'frames':0,'rejected':0,'last_wall':0.,'last_stamp':0.,'last_map':0.,'last_seq':-1,'error':'aguardando FAST-LIO'}
    def reset(reason):
        recent.clear();voxels.clear();state['error']=reason;state['rejected']+=1
        state['last_map']=0.;state['last_stamp']=0.
        state['last_seq']=-1
        h=Header(stamp=rospy.Time.now(),frame_id='body')
        local_pub.publish(point_cloud2.create_cloud_xyz32(h,[]))
        h.frame_id='camera_init';map_pub.publish(point_cloud2.create_cloud_xyz32(h,[]))
    def receive(cloud,odom):
        with lock:
            try:
                stamp=cloud.header.stamp.to_sec();lag=(rospy.Time.now()-cloud.header.stamp).to_sec()
                if cloud.header.frame_id!='camera_init' or odom.header.frame_id!='camera_init' or odom.child_frame_id!='body':
                    raise ValueError('Frames FAST-LIO inesperados')
                if lag<-.05 or lag>.5:raise ValueError('Nuvem registrada desatualizada')
                position,rotation=checked_pose(odom)
                if state['last_seq']>=0 and odom.header.seq<=state['last_seq']:
                    raise ValueError('FAST-LIO reiniciado: origem do mapa invalidada')
                xyz=cloud_xyz(cloud)
                if len(xyz)<10:raise ValueError('Nuvem registrada vazia')
                local=recent.add(stamp,xyz,position,rotation)
                h=Header(stamp=cloud.header.stamp,frame_id='body')
                local_pub.publish(point_cloud2.create_cloud_xyz32(h,local.tolist()))
                # Display/export map: bounded XYZ voxels, never supplied to docking.
                for point in voxel_points(xyz,a.voxel):
                    key=tuple(np.floor(point/a.voxel).astype(int));voxels[key]=point;voxels.move_to_end(key)
                while len(voxels)>a.max_points:voxels.popitem(last=False)
                if stamp-state['last_map']>=1.:
                    h.frame_id='camera_init'
                    map_pub.publish(point_cloud2.create_cloud_xyz32(h,np.array(list(voxels.values())).tolist()))
                    state['last_map']=stamp
                state.update(frames=state['frames']+1,last_wall=time.monotonic(),last_stamp=stamp,last_seq=odom.header.seq,error='')
            except (ValueError,TypeError,np.linalg.LinAlgError) as e:
                reset(str(e));rospy.logwarn_throttle(3,'Mapa 3D rejeitou dados: %s',e)
    cloud_sub=message_filters.Subscriber('/fast_lio/cloud_registered',PointCloud2,queue_size=4,buff_size=32*1024*1024)
    odom_sub=message_filters.Subscriber('/fast_lio/odometry',Odometry,queue_size=30)
    # FAST-LIO stamps both at lidar_end_time. Exact matching prevents frame skew.
    sync=message_filters.TimeSynchronizer([cloud_sub,odom_sub],30);sync.registerCallback(receive)
    print('Mapa XYZ: /rover_slam_3d/map | docking: /rover_slam_3d/local_cloud | frame camera_init',flush=True)
    print('Exportacao ao encerrar:',output/'mapa.pcd',flush=True)
    start=time.monotonic();report=0;result=0
    try:
        while not rospy.is_shutdown():
            now=time.monotonic()
            with lock:
                if state['last_wall'] and now-state['last_wall']>1.:
                    reset('FAST-LIO sem atualizacao; nuvem local invalidada');state['last_wall']=0.
                if now>=report:
                    info=dict(state,voxels=len(voxels));status_pub.publish(json.dumps(info));print(info,flush=True);report=now+5
            if a.duration and now-start>=a.duration:break
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        cloud_sub.unregister();odom_sub.unregister()
        with lock:
            xyz=np.asarray(list(voxels.values()),dtype=float).reshape(-1,3)
            with (output/'mapa.pcd').open('w') as f:
                f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'%(len(xyz),len(xyz)))
                np.savetxt(f,xyz,fmt='%.6f')
            (output/'status.json').write_text(json.dumps(state,indent=2)+'\n')
            if not state['frames'] or state['error']:result=1
    return result


if __name__=='__main__':sys.exit(main())
