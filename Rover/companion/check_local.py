#!/usr/bin/env python3
"""Refuse a second controller/SLAM instance before starting the companion."""
from pathlib import Path
import sys
names=('mission_manager/manager.py','slam/fast_lio/run_fast_lio.py','slam/slam_mid360_3d.py',
       'docking/entrar_container.py','docking/aguardar_docking.py','docking/sair_container.py',
       'docking_lidar/entrar_lidar.py','docking_lidar/sair_lidar.py','obstacle_avoidance/manager.py')
found=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=proc.joinpath('cmdline').read_bytes().decode().split('\0')
    except (OSError,UnicodeDecodeError):continue
    if not args or 'python' not in Path(args[0]).name:continue
    if any(any(arg.endswith(name) for name in names) for arg in args[1:]):found.append((proc.name,args))
if found:
    print('Encerre os servicos locais antes de iniciar suas copias na Raspberry:')
    for pid,args in found:print('PID',pid,' '.join(args))
    sys.exit(1)
