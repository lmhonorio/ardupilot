#!/usr/bin/env bash
# Instalacao nativa: Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / GLIM CPU.
set -euo pipefail
[[ $EUID == 0 ]] || { echo "Execute: sudo bash $0" >&2; exit 1; }
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_CODENAME:-} == noble ]] || {
  echo 'Este instalador exige Ubuntu 24.04 (noble).' >&2; exit 1;
}
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl ca-certificates gnupg software-properties-common
add-apt-repository -y universe
tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT
curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$tmpdir/release.json"
version=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$tmpdir/release.json")
curl -fSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.noble_all.deb" -o "$tmpdir/ros2-apt-source.deb"
dpkg -i "$tmpdir/ros2-apt-source.deb"
# Repositorio do autor do GLIM, com chave restrita a esta fonte.
curl -fsSL https://koide3.github.io/ppa/ubuntu2404/KEY.gpg -o "$tmpdir/glim.key"
gpg --batch --yes --dearmor -o /usr/share/keyrings/rover-glim.gpg "$tmpdir/glim.key"
echo 'deb [signed-by=/usr/share/keyrings/rover-glim.gpg] https://koide3.github.io/ppa/ubuntu2404 ./' > /etc/apt/sources.list.d/rover-glim.list
apt-get update
apt-get install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge ros-jazzy-xacro ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs-py ros-jazzy-message-filters ros-jazzy-tf-transformations \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions python3-dev \
  ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan ros-jazzy-mavros \
  ros-jazzy-glim-ros libgtsam-points-dev libiridescence-dev \
  ros-dev-tools python3-colcon-common-extensions python3-venv python3-pip \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-matplotlib \
  python3-serial python3-wxgtk4.0 python3-lxml python3-empy \
  build-essential cmake ninja-build git pkg-config libeigen3-dev libpcl-dev \
  libopencv-dev rapidjson-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libboost-all-dev libmetis-dev libglfw3-dev gnome-terminal geographiclib-tools
if [[ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  geographiclib-get-geoids egm96-5
fi
ldconfig
echo 'ROS 2 Jazzy, Gazebo Harmonic e GLIM instalados. O ambiente Noetic nao foi removido.'
echo 'Abra um terminal novo e use o setup ROS 2 do projeto; nao carregue o catkin_ws.'
