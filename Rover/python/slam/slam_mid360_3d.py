#!/usr/bin/python3
"""FAST-LIO 3D voxel map and recent, motion-compensated cloud for docking.

This consumes LiDAR/IMU odometry; no Gazebo truth, GPS or 2D LaserScan enters
mapping. Local geometry expires independently of the persistent display map.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
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
    p.add_argument('--wall-timeout',type=float,default=1.,help='Watchdog real de entrega; ate 3 s para SITL lento, sem relaxar timestamps ROS')
    p.add_argument('--output-dir',type=Path)
    a=p.parse_args()
    if not .2<=a.window<=3 or not .01<=a.voxel<=.2 or not 1000<=a.max_points<=1000000 or a.duration<0 or not .5<=a.wall_timeout<=3:p.error('Limites de mapa invalidos')
    from common import ros2_runtime as ros
    import message_filters
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header,String
    ros.init_node('mid360_slam_3d')
    output=a.output_dir or Path(__file__).resolve().parents[1]/'run/slam_3d'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=True)
    local_pub=ros.Publisher('/rover_slam_3d/local_cloud',PointCloud2,queue_size=1)
    map_pub=ros.Publisher('/rover_slam_3d/map',PointCloud2,queue_size=1,latch=True)
    status_pub=ros.Publisher('/rover_slam_3d/status',String,queue_size=1,latch=True)
    lock=threading.RLock();recent=LocalCloud(a.window);voxels=OrderedDict()
    state={'frames':0,'rejected':0,'last_wall':0.,'last_stamp':0.,'last_map':0.,'last_seq':-1,'error':'aguardando FAST-LIO'}
    def reset(reason,clear_map=True):
        if clear_map:recent.clear();voxels.clear()
        else:recent.invalidate_window()
        state['error']=reason;state['rejected']+=1
        state['last_map']=0.;state['last_stamp']=0.
        state['last_seq']=-1
        h=Header(stamp=ros.now(),frame_id='body')
        local_pub.publish(point_cloud2.create_cloud_xyz32(h,[]))
        if clear_map:
            h.frame_id='camera_init';map_pub.publish(point_cloud2.create_cloud_xyz32(h,[]))
    def receive(cloud,odom):
        with lock:
            try:
                stamp=ros.seconds(cloud.header.stamp);lag=ros.age(cloud.header.stamp)
                if cloud.header.frame_id!='camera_init' or odom.header.frame_id!='camera_init' or odom.child_frame_id!='body':
                    raise ValueError('Frames FAST-LIO inesperados')
                if lag<-.05:raise ValueError('Nuvem registrada no futuro/relogio reiniciado')
                if lag>.5:
                    # Late data cannot authorize docking, but does not invalidate
                    # the already measured display map in the same reference.
                    reset('Nuvem registrada desatualizada',clear_map=False)
                    ros.logwarn_throttle(3,'Nuvem local invalidada por atraso; mapa visual preservado')
                    return
                position,rotation=checked_pose(odom)
                xyz=cloud_xyz(cloud)
                if len(xyz)<10:raise ValueError('Nuvem registrada vazia')
                local=recent.add(stamp,xyz,position,rotation)
                h=Header(stamp=cloud.header.stamp,frame_id='body')
                local_pub.publish(point_cloud2.create_cloud_xyz32(h,local))
                # Display/export map: bounded XYZ voxels, never supplied to docking.
                for point in voxel_points(xyz,a.voxel):
                    key=tuple(np.floor(point/a.voxel).astype(int));voxels[key]=point;voxels.move_to_end(key)
                while len(voxels)>a.max_points:voxels.popitem(last=False)
                if stamp-state['last_map']>=1.:
                    h.frame_id='camera_init'
                    map_pub.publish(point_cloud2.create_cloud_xyz32(h,np.array(list(voxels.values()))))
                    state['last_map']=stamp
                state.update(frames=state['frames']+1,last_wall=time.monotonic(),last_stamp=stamp,last_seq=state['frames'],error='')
            except (ValueError,TypeError,np.linalg.LinAlgError) as e:
                reset(str(e));ros.logwarn_throttle(3,'Mapa 3D rejeitou dados: %s',e)
    cloud_sub=message_filters.Subscriber(ros.node(),PointCloud2,'/fast_lio/cloud_registered',qos_profile=ros.sensor_qos(4))
    odom_sub=message_filters.Subscriber(ros.node(),Odometry,'/fast_lio/odometry',qos_profile=ros.sensor_qos(30))
    # FAST-LIO stamps both at lidar_end_time. Exact matching prevents frame skew.
    sync=message_filters.TimeSynchronizer([cloud_sub,odom_sub],30);sync.registerCallback(receive)
    print('Mapa XYZ: /rover_slam_3d/map | docking: /rover_slam_3d/local_cloud | frame camera_init',flush=True)
    print('Exportacao ao encerrar:',output/'mapa.pcd',flush=True)
    start=time.monotonic();report=0;result=0
    try:
        while not ros.is_shutdown():
            now=time.monotonic()
            with lock:
                if state['last_wall'] and now-state['last_wall']>a.wall_timeout:
                    reset('FAST-LIO sem atualizacao; nuvem local invalidada',clear_map=False);state['last_wall']=0.
                if now>=report:
                    info=dict(state,voxels=len(voxels));status_pub.publish(json.dumps(info));print(info,flush=True);report=now+5
            if a.duration and now-start>=a.duration:break
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        ros.node().destroy_subscription(cloud_sub.sub);ros.node().destroy_subscription(odom_sub.sub)
        with lock:
            xyz=np.asarray(list(voxels.values()),dtype=float).reshape(-1,3)
            with (output/'mapa.pcd').open('w') as f:
                f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'%(len(xyz),len(xyz)))
                np.savetxt(f,xyz,fmt='%.6f')
            (output/'status.json').write_text(json.dumps(state,indent=2)+'\n')
            if not state['frames'] or state['error']:result=1
    return result


if __name__=='__main__':sys.exit(main())
