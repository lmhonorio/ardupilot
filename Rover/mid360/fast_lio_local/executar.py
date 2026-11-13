#!/usr/bin/env python3
import argparse,fcntl,json,os,signal,subprocess,sys,time
from datetime import datetime
from pathlib import Path
import yaml


def main():
    p=argparse.ArgumentParser();p.add_argument('--no-rviz',action='store_true');a=p.parse_args()
    root=Path(__file__).resolve().parent;(root/'run').mkdir(exist_ok=True)
    params=yaml.safe_load((root/'config.yaml').read_text())['/**']['ros__parameters']
    if params['cube_side_length']<=3*params['mapping']['det_range']:
        raise SystemExit('cube_side_length deve superar 3 x mapping.det_range nesta versao FAST-LIO')
    lock=(root/'run/lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('O teste local FAST-LIO ja esta rodando.')
    run=root/'run'/datetime.now().strftime('%Y%m%d_%H%M%S');run.mkdir()
    (root/'run/latest.txt').write_text(str(run)+'\n');(root/'run/pid').write_text(str(os.getpid())+'\n')
    stopping=False;children=[]
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    commands=[('adapter',[str(root/'ws/install/lib/mid360_lio_adapter/timing_adapter')]),
      ('fastlio',[str(root/'ws/install/lib/fast_lio/fastlio_mapping'),'--ros-args','--params-file',str(root/'config.yaml'),
                 '-r','__node:=fastlio_local_test','-r','/Odometry:=/lio_test/odometry','-r','/path:=/lio_test/path',
                 '-r','/cloud_registered:=/lio_test/cloud_registered','-r','/cloud_registered_body:=/lio_test/cloud_body',
                 '-r','/Laser_map:=/lio_test/map','-r','/cloud_effected:=/lio_test/effected']),
      ('transport',[sys.executable,str(root/'receber_teste.py'),'--stats',str(run/'transport.json')])]
    if not a.no_rviz:commands.append(('rviz',['rviz2','-d',str(root/'fastlio.rviz'),'--ros-args','-p','use_sim_time:=false']))
    result=0
    try:
        for name,argv in commands:
            log=(run/(name+'.log')).open('w')
            child=subprocess.Popen(argv,cwd=run,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,child,log));print(name,'PID',child.pid,flush=True)
        (run/'processes.json').write_text(json.dumps({name:child.pid for name,child,_ in children},indent=2))
        print('FAST-LIO exclusivo no computador, dominio 22. Logs:',run,flush=True)
        while not stopping:
            for name,child,_ in children:
                if child.poll() is not None:
                    print(name,'encerrou:',child.returncode,'; encerrando somente este teste.',flush=True)
                    result=child.returncode or 0;stopping=True;break
            time.sleep(.2)
    finally:
        for name,child,_ in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        end=time.monotonic()+8
        while time.monotonic()<end and any(c.poll() is None for _,c,_ in children):time.sleep(.1)
        for _,child,log in children:
            if child.poll() is None:os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            log.close()
        (root/'run/pid').unlink(missing_ok=True)
    return result


if __name__=='__main__':raise SystemExit(main())
