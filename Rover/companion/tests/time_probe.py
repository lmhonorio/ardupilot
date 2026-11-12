"""Read-only TIMESYNC/ROS clock comparison over an unused MAVProxy output."""
import argparse
import json
import os
import time
os.environ['MAVLINK20']='1'
from common import ros2_runtime as ros
from pymavlink import mavutil

p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=14557);p.add_argument('--duration',type=float,default=20);a=p.parse_args()
ros.init_node('companion_time_probe',anonymous=True)
link=mavutil.mavlink_connection('udpin:127.0.0.1:'+str(a.port),source_system=242,source_component=198,dialect='ardupilotmega')
pending={};rows=[];last=0.;end=time.monotonic()+a.duration;attitude=None
try:
    while time.monotonic()<end:
        now=time.monotonic()
        if link.clients and now-last>=.5 and ros.seconds(ros.now())>0:
            key=time.monotonic_ns();pending[key]=(ros.seconds(ros.now()),now)
            link.mav.timesync_send(0,key);last=now
        msg=link.recv_match(blocking=True,timeout=.01)
        if not msg or msg.get_srcSystem()!=1 or msg.get_srcComponent()!=1:continue
        if msg.get_type()=='ATTITUDE':attitude=msg.time_boot_ms*.001
        if msg.get_type()!='TIMESYNC' or msg.tc1<=0:continue
        start=pending.pop(msg.ts1,None)
        if start is None:continue
        clock=ros.seconds(ros.now());wall=time.monotonic();offset=(clock+start[0])/2-msg.tc1/1e9
        rows.append(dict(rtt=wall-start[1],ros_elapsed=clock-start[0],offset=offset,ros=clock,boot=msg.tc1/1e9,
                         attitude_boot=attitude))
finally:
    print(json.dumps(rows,indent=2));link.close();ros.signal_shutdown('probe complete')
