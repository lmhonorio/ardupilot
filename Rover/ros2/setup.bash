# source ~/ardupilot/Rover/ros2/setup.bash
# Native GUI binaries cannot load the GTK/GIO modules bundled with a Snap IDE.
for rover_gui_var in GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH; do
  case "${!rover_gui_var}" in */snap/*) unset "$rover_gui_var";; esac
done
unset rover_gui_var
if [[ ${QT_XCB_GL_INTEGRATION:-} == none ]]; then unset QT_XCB_GL_INTEGRATION; fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy ainda nao instalado. Execute sudo bash ~/ardupilot/Rover/ros2/install_ros2.sh' >&2
  return 1
fi
if [[ ${ROS_VERSION:-2} != 2 ]]; then
  echo 'Removendo o ambiente ROS 1 herdado deste terminal antes de carregar Jazzy.' >&2
  unset ROS_VERSION ROS_DISTRO ROS_PACKAGE_PATH ROS_ETC_DIR ROS_ROOT ROS_MASTER_URI
  unset ROS_IP ROS_HOSTNAME ROSLISP_PACKAGE_DIRECTORIES ROS_WS
  for rover_var in PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH PATH; do
    rover_clean=''
    IFS=: read -ra rover_parts <<< "${!rover_var}"
    for rover_part in "${rover_parts[@]}"; do
      case "$rover_part" in *noetic*|*catkin_ws*|*scanner_ws*) continue;; esac
      [[ -z "$rover_part" ]] || rover_clean="${rover_clean:+$rover_clean:}$rover_part"
    done
    export "$rover_var=$rover_clean"
  done
  unset rover_var rover_clean rover_parts rover_part
fi
export ROVER_ROS2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
# Jazzy was sourced explicitly above. Do not replay colcon's captured parent
# workspaces: an older build may have recorded Noetic/catkin in setup.bash.
if [[ -f "$ROVER_ROS2_ROOT/install/local_setup.bash" ]]; then source "$ROVER_ROS2_ROOT/install/local_setup.bash"; fi
export PATH="$ROVER_ROS2_ROOT/venv/bin:/usr/bin:$PATH"
export PYTHONPATH="$ROVER_ROS2_ROOT/../python${PYTHONPATH:+:$PYTHONPATH}"
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROVER_ROS2_ROOT/install/lib/ardupilot_gazebo${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$ROVER_ROS2_ROOT/generated:$ROVER_ROS2_ROOT/assets/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export ROVER_USE_SIM_TIME=true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
