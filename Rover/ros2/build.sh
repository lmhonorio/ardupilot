#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Instale primeiro: sudo bash $root/install_ros2.sh" >&2; exit 1
fi
export PATH="/usr/bin:/bin:$PATH"
source "$root/setup.bash"
# Colcon/ROSIDL use the distro interpreter, not a pip-installed build tool.
export PATH="/usr/bin:/bin:$PATH"
export GZ_VERSION=harmonic
/usr/bin/python3 -m venv --system-site-packages "$root/venv"
"$root/venv/bin/python" -m pip install -r "$root/requirements.txt"
mkdir -p "$root/deps"
if [[ ! -d "$root/deps/ardupilot_gazebo/.git" ]]; then
  git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$root/deps/ardupilot_gazebo"
fi
if [[ ! -d "$root/deps/FAST_LIO/.git" ]]; then
  git clone --branch ros2 --depth 1 --filter=blob:none --sparse https://github.com/Ericsii/FAST_LIO.git "$root/deps/FAST_LIO"
  git -C "$root/deps/FAST_LIO" sparse-checkout set src include msg config launch rviz
fi
git -C "$root/deps/FAST_LIO" submodule update --init --recursive
# Keep builds reproducible; refuse to reset a dependency containing user edits.
while read -r name revision; do
  [[ "$name" != ikd-Tree ]] || continue
  actual=$(git -C "$root/deps/$name" rev-parse HEAD)
  if [[ "$actual" != "$revision" ]]; then
    [[ -z "$(git -C "$root/deps/$name" status --porcelain)" ]] || { echo "Dependencia modificada: $name" >&2; exit 1; }
    git -C "$root/deps/$name" fetch --depth 1 origin "$revision"
    git -C "$root/deps/$name" checkout --detach "$revision"
  fi
done < "$root/dependencies.lock"
/usr/bin/python3 "$root/../python/simulation/ros2/patch_fast_lio.py" "$root/deps/FAST_LIO"
cd "$root"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install \
  --base-paths src deps/ardupilot_gazebo deps/FAST_LIO \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$root/setup.bash"
"$root/venv/bin/python" "$root/../python/simulation/ros2/prepare_assets.py"
echo 'Compilacao e conversao concluidas. Execute ./ros2/service.sh gazebo.'
