#!/usr/bin/env python3
"""Copy a local mission to the companion, then call its ROS service from this PC."""
import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mission_file');p.add_argument('--remote',default=os.environ.get('ROVER_REMOTE','rover@192.168.0.167'))
    a=p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+',a.remote):p.error('Destino SSH invalido')
    path=Path(a.mission_file).expanduser().resolve()
    if not path.is_file() or path.suffix.lower() not in ('.plan','.waypoints'):p.error('Use arquivo .plan ou .waypoints existente')
    # Content-derived ASCII name avoids shell injection and concurrent overwrites.
    name='mission_'+hashlib.sha256(path.read_bytes()).hexdigest()[:16]+path.suffix.lower()
    subprocess.run(['ssh',a.remote,'mkdir -p ~/rover_companion/missions'],check=True)
    subprocess.run(['scp',str(path),a.remote+':rover_companion/missions/'+name],check=True)
    remote_path='/missions/'+name
    print('Arquivo no container:',remote_path,flush=True)
    import rclpy
    from rover_mission_interfaces.srv import StartMission
    rclpy.init();node=rclpy.create_node('companion_mission_client')
    try:
        client=node.create_client(StartMission,'/mission_manager/start_file')
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('Servico remoto indisponivel; confira DDS, container e ROS_DOMAIN_ID')
        future=client.call_async(StartMission.Request(mission_file=remote_path))
        rclpy.spin_until_future_complete(node,future,timeout_sec=20)
        if not future.done():raise RuntimeError('Sem resposta; confira status antes de repetir')
        result=future.result();print(result.message)
        if result.success:print('Pedido aceito; acompanhe /mission_manager/status. Nao significa missao concluida.')
        return 0 if result.success else 1
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':sys.exit(main())
