#!/usr/bin/env python3
"""One owner for MAVProxy, LIO, mapping and mission management in the companion."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]

def wait_sensors(children, simulation, cancelled, timeout=120):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    rclpy.init();node=rclpy.create_node('companion_startup',parameter_overrides=[Parameter('use_sim_time',value=simulation)])
    received={}
    def receive(name,msg):
        if name=='cloud' and msg.width*msg.height<10:return
        received[name]=(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,time.monotonic())
    node.create_subscription(Odometry,'/fast_lio/odometry',lambda m:receive('odom',m),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/rover_slam_3d/local_cloud',lambda m:receive('cloud',m),qos_profile_sensor_data)
    end=time.monotonic()+timeout
    print('Aguardando odometria e nuvem local recentes antes do gerenciador.',flush=True)
    try:
        while time.monotonic()<end and not cancelled():
            if any(p.poll() is not None for _,p,_ in children):raise RuntimeError('Servico encerrou durante inicializacao dos sensores')
            rclpy.spin_once(node,timeout_sec=.1)
            now=node.get_clock().now().nanoseconds/1e9
            if len(received)==2 and all(-.05<=now-stamp<=.5 and time.monotonic()-wall<2 for stamp,wall in received.values()):return
        raise RuntimeError('Nao chegaram nuvem local e odometria validas; gerenciador nao foi iniciado')
    finally:node.destroy_node();rclpy.shutdown()

def commands(config, run):
    profile=config['profile']
    if profile not in ('simulation','real'):raise ValueError('profile deve ser simulation ou real')
    real=profile=='real'
    if real and not config.get('hardware_configured',False):
        raise ValueError('Perfil real ainda nao calibrado: configure hardware.yaml e valide os dispositivos primeiro')
    py=sys.executable;ns=config['namespace'];cmd=[]
    master=config['mavproxy_master']
    if not real and not master.startswith('udpin:'):
        raise ValueError('Simulacao exige MAVProxy UDP, nunca serial')
    cmd.append(('mavproxy',['mavproxy.py','--master',master,'--baudrate',str(config.get('mavproxy_baud',115200)),
        '--source-system','243','--streamrate',str(config.get('mavproxy_streamrate',-1 if not real else 4)),
        '--default-modules','param,mode,output,cmdlong','--non-interactive','--cmd','set mavfwd true',
        '--out','udp:127.0.0.1:14551','--out','udp:127.0.0.1:14556','--out','udp:127.0.0.1:14557',
        '--logfile',str(run/'mavproxy.tlog')]))
    local_cloud=ns+'/livox/lidar_onboard'
    if not real:
        cmd.append(('sensor_ingress',['ros2','run','rover_companion_bridge','sensor_ingress','--ros-args',
                    '-p','source:='+ns+'/livox/lidar','-p','target:='+local_cloud]))
        cmd.append(('simulation_adapter',['ros2','run','rover_companion_bridge','simulation_adapter','--ros-args',
                    '-p','source:='+local_cloud,'-p','stride:='+str(config.get('fastlio_adapter_stride',4))]))
    lio=[py,'python/slam/fast_lio/run_fast_lio.py','--namespace',ns,
         '--point-filter',str(config.get('fastlio_point_filter',2)),
         '--adapter-stride',str(config.get('fastlio_adapter_stride',4)),
         '--map-voxel',str(config.get('fastlio_map_voxel',.10)),
         '--surface-voxel',str(config.get('fastlio_surface_voxel',.10)),
         '--output-dir',str(run/'fastlio')]
    if real:
        lio+=['--sensor-profile','real','--cloud-topic',config['livox_topic'],
              '--imu-topic',config['imu_topic'],'--calibration',config['calibration_file']]
        # Driver parameters are calibrated independently from simulator defaults.
        cmd.append(('livox',['ros2','run','livox_ros_driver2','livox_ros_driver2_node',
            '--ros-args','--params-file',config['livox_params']]))
    else:lio+=['--cloud-topic',local_cloud,'--external-simulation-adapter']
    cmd.append(('fastlio',lio))
    cmd.append(('map',[py,'python/slam/slam_mid360_3d.py','--wall-timeout','3',
        '--voxel',str(config.get('map_voxel',.06)),'--max-points',str(config.get('map_max_points',100000)),
        '--output-dir',str(run/'map')]))
    if config.get('obstacles',False):
        cmd.append(('obstacles',[py,'python/obstacle_avoidance/manager.py']))
    manager=[py,'python/mission_manager/manager.py','--namespace',ns,'--estacoes',config['stations'],
        '--vehicle','real' if real else 'sitl','--no-simulate-charge' if real else '--simulate-charge',
        '--output-dir',str(run/'manager'),'--recovery','/data/'+profile+'/recovery.json','--geometry-process']
    if config.get('no_qr',True):manager+=['--no-qr']
    if not real:manager+=['--raw-cloud',local_cloud]
    if real:manager+=['--envelope',config['envelope_file']]
    cmd.append(('manager',manager))
    return cmd

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--check',action='store_true')
    args=p.parse_args();config=yaml.safe_load(Path(args.config).read_text())
    run=Path('/data/runs')/datetime.now().strftime('%Y%m%d_%H%M%S');plan=commands(config,run)
    if args.check:print(json.dumps(plan,indent=2));return 0
    run.mkdir(parents=True);(Path('/data')/'latest.txt').write_text(str(run)+'\n')
    (Path('/data')/config['profile']).mkdir(parents=True,exist_ok=True)
    (run/'config.json').write_text(json.dumps(config,indent=2))
    env=os.environ.copy();env['ROVER_USE_SIM_TIME']='true' if config['profile']=='simulation' else 'false'
    if config['profile']=='real':env['ROVER_HARDWARE_CALIBRATION']=config['calibration_file']
    children=[];stopping=False
    def stop(sig,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        for name,argv in plan:
            if stopping:break
            if name=='manager':wait_sensors(children,config['profile']=='simulation',lambda:stopping)
            log=(run/(name+'.log')).open('w')
            proc=subprocess.Popen(argv,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,proc,log));print('Iniciado:',name,'PID',proc.pid,flush=True)
            time.sleep(1)
        while not stopping:
            for name,proc,_ in children:
                if proc.poll() is not None:raise RuntimeError('%s encerrou: %s; confira %s'%(name,proc.returncode,run))
            time.sleep(.5)
        return 0
    except Exception as exc:
        print('Stack interrompida:',exc,flush=True);return 1
    finally:
        # Stop the controller while telemetry is still present, before its sensors.
        for name,proc,log in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=18 if name=='manager' else 5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            log.close()

if __name__=='__main__':sys.exit(main())
