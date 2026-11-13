#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
source "$here/../../ros2/setup.bash"
export LOCAL_LIO_ROOT="$here"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-1}
python - <<'PY'
import os,shutil
from pathlib import Path
root=Path(os.environ['LOCAL_LIO_ROOT']);src=root/'ws/src/fast_lio'
if not src.exists():
 shutil.copytree(root.parents[1]/'ros2/deps/FAST_LIO',src,ignore=shutil.ignore_patterns('.git','Log','PCD','build','install','log','__pycache__'))
for folder in ('Log','PCD'):(src/folder).mkdir(exist_ok=True)
for name in ('mat_pre.txt','mat_out.txt','dbg.txt','pos_log.txt'):
 p=src/'Log'/name
 if not p.exists() and not p.is_symlink():p.symlink_to('/dev/null')
PY
cd "$here/ws"
colcon build --merge-install --executor sequential --allow-overriding fast_lio \
  --base-paths src/fast_lio "$here/adapter" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
