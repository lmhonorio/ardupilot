#!/usr/bin/env python3
"""Observa o ensaio isolado; nao comanda o sensor nem o rover."""
import argparse,json,math,time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu


def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=20);p.add_argument('--inputs-only',action='store_true');p.add_argument('--stationary',action='store_true');a=p.parse_args()
    rclpy.init();n=rclpy.create_node('lio_test_validation');q=QoSProfile(depth=2048,reliability=ReliabilityPolicy.BEST_EFFORT)
    counts={'cloud':0,'imu':0,'odom':0,'registered':0};times={k:[] for k in counts};stamps={k:[] for k in counts};positions=[];offsets=[];bad=[];sizes=[]
    def record(k,m):
        counts[k]+=1;times[k].append(time.monotonic());stamps[k].append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
    def cloud(m):
        record('cloud',m);f={f.name:f for f in m.fields}
        if 'timestamp' not in f or m.is_bigendian:bad.append('missing native timestamp');return
        dtype=np.dtype({'names':['t'],'formats':['<f8'],'offsets':[f['timestamp'].offset],'itemsize':m.point_step})
        x=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))['t']
        if x.size:
            base=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
            offsets.append((max(0.,float(x.min())-base),float(x.max())-base))
    def odom(m):
        record('odom',m);p=m.pose.pose.position;r=m.pose.pose.orientation
        v=[p.x,p.y,p.z,r.x,r.y,r.z,r.w]
        if not all(math.isfinite(x) for x in v):bad.append('nonfinite odometry')
        positions.append(v[:3])
    def registered(m):record('registered',m);sizes.append(m.width*m.height)
    n.create_subscription(PointCloud2,'/lio_test/points',cloud,q)
    n.create_subscription(Imu,'/lio_test/imu',lambda m:record('imu',m),q)
    n.create_subscription(Odometry,'/lio_test/odometry',odom,q)
    n.create_subscription(PointCloud2,'/lio_test/cloud_registered',registered,q)
    start=time.monotonic()
    try:
        while time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.1)
        gaps={k:[b-c for c,b in zip(v,v[1:])] for k,v in stamps.items()}
        required=['cloud','imu'] if a.inputs_only else list(counts)
        ok=all(counts[k]>10 and time.monotonic()-times[k][-1]<2 for k in required) and not bad
        ok=ok and all(min(gaps[k],default=0)>0 for k in required)
        ok=ok and bool(offsets) and min(x[0] for x in offsets)>=0 and max(x[1] for x in offsets)<500000000
        out={'passed':ok,'duration_s':a.seconds,'counts':counts,'hz':{k:(len(v)-1)/(v[-1]-v[0]) if len(v)>1 else 0 for k,v in times.items()},
             'max_stamp_gap_s':{k:max(v,default=0) for k,v in gaps.items()},'point_offset_ns_min':min((x[0] for x in offsets),default=None),
             'point_offset_ns_max':max((x[1] for x in offsets),default=None),'registered_points_min':min(sizes,default=0),
             'last_position':positions[-1] if positions else None,'position_span_m':np.ptp(np.array(positions),axis=0).tolist() if positions else None,'errors':bad}
        if a.stationary:
            out['stationary_passed']=bool(positions) and max(out['position_span_m'])<.1
            out['passed']=out['passed'] and out['stationary_passed']
        print(json.dumps(out,indent=2));return 0 if out['passed'] else 1
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
