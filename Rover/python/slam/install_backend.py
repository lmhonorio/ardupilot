#!/usr/bin/env python3
"""ROS 2 installer entrypoint. System packages require the reviewed sudo installer."""
from pathlib import Path
import subprocess
import sys
ROVER=Path(__file__).resolve().parents[2]
if __name__=='__main__':
    if not Path('/opt/ros/jazzy/setup.bash').exists():
        raise SystemExit('Execute primeiro: sudo bash '+str(ROVER/'ros2/install_ros2.sh'))
    sys.exit(subprocess.call(['bash',str(ROVER/'ros2/build.sh')]))
