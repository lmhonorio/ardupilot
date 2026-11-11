#!/usr/bin/env python3
"""Submit a mission file to the running manager; the manager owns MAVLink."""
import argparse
from pathlib import Path
import sys
import rclpy
from rover_mission_interfaces.srv import StartMission


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission_file',help='Arquivo .waypoints ou .plan (mesmo computador do manager)')
    args=parser.parse_args()
    path=Path(args.mission_file).expanduser().resolve()
    if not path.is_file():parser.error('Arquivo nao encontrado: '+str(path))
    rclpy.init()
    node=rclpy.create_node('mission_file_client')
    try:
        client=node.create_client(StartMission,'/mission_manager/start_file')
        if not client.wait_for_service(timeout_sec=10.):
            print('Servico start_file indisponivel. Inicie/reinicie ./ros2/service.sh mission-manager.',file=sys.stderr)
            return 1
        future=client.call_async(StartMission.Request(mission_file=str(path)))
        rclpy.spin_until_future_complete(node,future,timeout_sec=15.)
        if not future.done():
            print('Sem resposta: confira status antes de repetir o pedido.',file=sys.stderr)
            return 1
        result=future.result()
        print(result.message)
        if result.success:
            print('Pedido recebido. Acompanhe last_request, state e battery em /mission_manager/status.')
        return 0 if result.success else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':sys.exit(main())
