#!/usr/bin/python3
"""Forward fresh sector distances to ArduPilot through a dedicated MAVProxy UDP output."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import argparse, os, threading, time
from pathlib import Path
import yaml
from projection import mav_distances


def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',default=str(Path(__file__).with_name('config.yaml')))
    args=p.parse_args(); config=yaml.safe_load(Path(args.config).read_text()); c=config['processing']; mc=config['mavlink']
    if not mc['endpoint'].startswith('udpin:127.0.0.1:'):
        raise ValueError('Somente UDP local permitido para esta integracao de simulacao')
    os.environ['MAVLINK20']='1'
    from pymavlink import mavutil
    from common import ros2_runtime as ros
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    ros.init_node('olhe_obstacle_mavlink_bridge')
    mav=mavutil.mavlink_connection(mc['endpoint'],source_system=mc['source_system'],source_component=mc['source_component'],dialect='ardupilotmega')
    lock=threading.RLock(); state={'scan':None,'wall':0.,'peer_wall':0.,'sent':0}
    def callback(msg):
        with lock: state.update(scan=msg,wall=time.monotonic())
    sub=ros.Subscriber(c['scan_topic'],LaserScan,callback,queue_size=1)
    status=ros.Publisher('/olhe/obstacles/bridge_status',String,queue_size=1)
    period=1.0/c['rate_hz']
    try:
        while not ros.is_shutdown():
            with lock:
                while mav.recv_match(blocking=False) is not None: state['peer_wall']=time.monotonic()
                scan,wall,peer=state['scan'],state['wall'],state['peer_wall']
                now=time.monotonic()
                if not peer or now-peer > 3:
                    result='SEM_MAVPROXY'
                elif scan is None or ros.seconds(scan.header.stamp) <= 0 or now-wall > c.get('wall_timeout_s', .5) or not -.05 <= ros.age(scan.header.stamp) <= c['temporal_window_s']:
                    result='SCAN_DESATUALIZADO'
                elif len(scan.ranges)!=72 or abs(scan.angle_increment - 2*3.141592653589793/72)>1e-5 or abs(scan.angle_min)>1e-5:
                    result='SCAN_INVALIDO'
                else:
                    distances=mav_distances(scan.ranges,c['min_distance_m'],c['max_distance_m'])
                    observed=sum(d!=65535 for d in distances)
                    if observed < mc['min_observed_sectors']:
                        result='COBERTURA_INSUFICIENTE=%d/72; NAO_ENVIADO'%observed
                    else:
                        mav.mav.obstacle_distance_send(ros.nanoseconds(scan.header.stamp)//1000,
                            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,distances,5,
                            round(c['min_distance_m']*100),round(c['max_distance_m']*100),
                            5.,0.,mavutil.mavlink.MAV_FRAME_BODY_FRD)
                        state['sent']+=1; result='ENVIADO=%d SETORES=%d/72%s'%(state['sent'],observed,' COBERTURA_PARCIAL' if observed<72 else '')
            status.publish(result); ros.loginfo_throttle(5,'ponte obstaculos: %s',result)
            time.sleep(period)
    finally:
        sub.unregister(); mav.close()
if __name__=='__main__': main()
