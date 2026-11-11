#!/usr/bin/env python3
"""Start an isolated stationary stack and require real sensor/SLAM outputs.

Run only in the isolated test-container network, never against a live rover.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROVER=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROVER/'python'))
from common import ros2_runtime as ros


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=90)
    p.add_argument('--glim',action='store_true')
    p.add_argument('--drive',action='store_true',help='Teste curto de atuacao, somente no SITL isolado')
    p.add_argument('--map2d',action='store_true')
    p.add_argument('--mavros',action='store_true')
    a=p.parse_args()
    if not Path('/.dockerenv').exists():p.error('Este teste exige o container isolado')
    output=ROVER/'ros2/run'/('validation_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True)
    print('Logs:',output,flush=True)
    children=[];logs=[]
    def start(name,cmd,cwd=ROVER):
        f=(output/(name+'.log')).open('w');logs.append(f)
        proc=subprocess.Popen(cmd,cwd=str(cwd),stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,proc));return proc
    counts=Counter();frames={};subscribers=[];report_at=0
    try:
        start('gazebo',['ros2','launch',str(ROVER/'ros2/launch/simulation.launch.py'),'gui:=false','insta360_enabled:=true'])
        sitl=output/'sitl'
        subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_sitl.py')),
                        '--run-dir',str(sitl),'--add-param-file',str(ROVER/'nrover_sem_desvio.param')],check=True)
        # A separate directory owns this test's EEPROM; motion requires --drive.
        start('sitl',[str(ROVER.parent/'build/sitl/bin/ardurover'),'-S','--model','JSON','--home','-3.123150,-41.764495,77,90',
                      '--defaults',str(sitl/'defaults.parm')],sitl)
        start('mavproxy',[str(ROVER/'ros2/venv/bin/mavproxy.py'),'--daemon','--master=tcp:127.0.0.1:5760',
                         '--out=udp:127.0.0.1:14555',
                         '--out=udp:127.0.0.1:14556','--out=udp:127.0.0.1:14557','--out=udp:127.0.0.1:14551',
                         '--cmd=set mavfwd true; rc 1 1500; rc 3 1500','--logfile='+str(output/'mav.tlog')])
        start('fastlio',[sys.executable,str(ROVER/'python/slam/fast_lio/run_fast_lio.py'),'--output-dir',str(output/'fast_lio')])
        start('map',[sys.executable,str(ROVER/'python/slam/slam_mid360_3d.py'),'--output-dir',str(output/'map')])
        start('obstacles',[sys.executable,str(ROVER/'python/obstacle_avoidance/manager.py')])
        if a.glim:start('glim',[sys.executable,str(ROVER/'python/slam/glim/run_glim.py'),'--output-dir',str(output/'glim')])
        if a.map2d:start('map2d',[sys.executable,str(ROVER/'python/slam/slam_mid360_2d.py')])
        if a.mavros:start('mavros',['ros2','launch',str(ROVER/'ros2/launch/mavros.launch.py'),
                                   'fcu_url:=udp://:14555@','namespace:=/rover_argo_N1/Instance1'])
        ros.init_node('rover_validation')
        from sensor_msgs.msg import PointCloud2,Imu,Image,LaserScan,JointState
        from nav_msgs.msg import Odometry,OccupancyGrid
        from common.ros_cloud import cloud_xyz
        def receive(key,msg):
            if key=='mavros' and not msg.connected:return
            counts[key]+=1
            frames[key]=getattr(msg,'header',None).frame_id
            if key=='cloud':frames['cloud_points']=len(cloud_xyz(msg))
            if key=='camera' and counts[key]==1:
                from cv_bridge import CvBridge
                import cv2
                cv2.imwrite(str(output/'camera.png'),CvBridge().imgmsg_to_cv2(msg,'bgr8'))
        ns='/rover_argo_N1/Instance1'
        topics=[('cloud',ns+'/livox/lidar',PointCloud2),('imu',ns+'/livox/imu',Imu),
                ('camera',ns+'/camera/image_raw',Image),('camera_info_test',ns+'/camera/insta360/image_raw',Image),
                ('hokuyo',ns+'/lidar/hokuyo/scan',LaserScan),('joints',ns+'/joint_states',JointState),
                ('odom','/fast_lio/odometry',Odometry),('map','/rover_slam_3d/map',PointCloud2),
                ('obstacles','/olhe/obstacles/scan',LaserScan)]
        if a.glim:topics += [('glim','/glim_ros/odom',Odometry)]
        if a.map2d:topics += [('map2d','/map',OccupancyGrid)]
        if a.mavros:
            from mavros_msgs.msg import State
            topics += [('mavros','/rover_argo_N1/Instance1/mavros/state',State)]
        for key,topic,kind in topics:
            subscribers.append(ros.Subscriber(topic,kind,lambda m,k=key:receive(k,m),queue_size=2))
        end=time.monotonic()+a.seconds
        while time.monotonic()<end and not ros.is_shutdown():
            dead=[(name,proc.returncode) for name,proc in children if proc.poll() is not None]
            if dead:raise RuntimeError('Processos encerrados: '+str(dead))
            if time.monotonic()>=report_at:
                print(dict(counts),flush=True);report_at=time.monotonic()+10
            time.sleep(.1)
        missing=[key for key,_,_ in topics if counts[key]<3]
        (output/'result.json').write_text(json.dumps({'counts':dict(counts),'frames':frames,'missing':missing},indent=2))
        if missing:raise RuntimeError('Sem dados suficientes: '+', '.join(missing))
        if a.drive:
            sys.path.insert(0,str(ROVER/'python/missions'))
            from mavproxy_link import Link
            from pymavlink import mavutil
            link=Link('udpin:127.0.0.1:14551',1,output,timeout=5,source_system=244)
            try:
                link.wait('HEARTBEAT',timeout=20)
                link.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                             mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE,500000)
                link.wait(['SIMSTATE','SIM_STATE'],timeout=15)
                link.ready()
                link.mode('HOLD');link.wait_stationary(timeout=60)
                origin=link.wait('GLOBAL_POSITION_INT',timeout=10)
                link.mode('GUIDED')
                link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
                link.wait('HEARTBEAT',lambda m:bool(m.base_mode&128),timeout=10)
                print('SITL isolado: teste de velocidade 0.2 m/s, sem missao.',flush=True)
                begin=ros.seconds(ros.now());wall=time.monotonic()
                while ros.seconds(ros.now())-begin<8 and time.monotonic()-wall<60:
                    link.conn.mav.set_position_target_local_ned_send(0,1,1,mavutil.mavlink.MAV_FRAME_BODY_NED,
                        1|2|4|64|128|256|1024,0,0,0,.2,0,0,0,0,0,0,0)
                    link.recv(.05)
                link.mode('HOLD');link.wait_stationary(timeout=30);link.disarm()
                final=link.wait('GLOBAL_POSITION_INT',timeout=10)
                distance=((final.lat-origin.lat)**2+(final.lon-origin.lon)**2)**.5*.01113
                (output/'motion.json').write_text(json.dumps({'distance_gps_m':distance,'disarmed':True},indent=2))
                if not .3<distance<3:raise RuntimeError('Deslocamento fora do esperado: %.3f m'%distance)
                print('PASS: movimento %.3f m, HOLD e desarmamento confirmados.'%distance,flush=True)
            finally:
                try:link.stop()
                finally:link.conn.close();link.log.close()
        else:
            print('PASS: sensores, proximidade e odometria/mapa publicaram dados. Rover nao foi armado.',flush=True)
    finally:
        for sub in subscribers:sub.unregister()
        for name,proc in reversed(children):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for name,proc in reversed(children):
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for f in logs:f.close()
        ros.signal_shutdown()

if __name__=='__main__':main()
