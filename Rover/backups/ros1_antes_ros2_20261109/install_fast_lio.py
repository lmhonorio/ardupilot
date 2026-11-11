#!/usr/bin/python3
"""Fetch pinned official FAST-LIO2 source and build it in an isolated ROS workspace."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / '.deps/fast_lio_ws'
SOURCE = WORKSPACE / 'src/FAST_LIO'
COMMIT = '7cc4175de6f8ba2edf34bab02a42195b141027e9'
IKD_COMMIT = 'e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4'
BINARY = WORKSPACE / 'devel/lib/fast_lio/fastlio_mapping'


def fetch(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'rover-fast-lio-setup'})
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read()


def tree(repo, revision):
    return json.loads(fetch('https://api.github.com/repos/%s/git/trees/%s?recursive=1' % (repo, revision)))['tree']


def install_source():
    entries = []
    for repo, revision, prefix in [('hku-mars/FAST_LIO', COMMIT, ''),
                                   ('hku-mars/ikd-Tree', IKD_COMMIT, 'include/ikd-Tree/')]:
        for item in tree(repo, revision):
            name = item['path']
            if item['type'] != 'blob':
                continue
            if not prefix and not (name.startswith(('src/', 'include/', 'msg/', 'config/'))
                                   or name in ('CMakeLists.txt', 'package.xml', 'LICENSE', '.gitmodules')):
                continue
            if prefix and name not in ('ikd_Tree.cpp', 'ikd_Tree.h', 'README.md'):
                continue
            entries.append((repo, revision, prefix + name, name, item['sha']))

    def write_entry(entry):
        repo, revision, name, source_name, expected = entry
        path = SOURCE / name
        if path.exists():
            data = path.read_bytes()
        else:
            data = fetch('https://raw.githubusercontent.com/%s/%s/%s' % (repo, revision, source_name))
        digest = hashlib.sha1(('blob %d\0' % len(data)).encode() + data).hexdigest()
        if digest != expected:
            raise RuntimeError('Arquivo existente ou baixado difere da revisao oficial: %s' % path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists(): path.write_bytes(data)

    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(write_entry, entries))
    for name in ('Log', 'PCD'):
        (SOURCE / name).mkdir(parents=True, exist_ok=True)
    print('FAST-LIO2 oficial preparado:', COMMIT, flush=True)


def build():
    env = os.environ.copy()
    env.pop('CONDA_PREFIX', None)
    env.pop('CONDA_DEFAULT_ENV', None)
    env['PATH'] = '/usr/bin:/bin:/opt/ros/noetic/bin'
    env['CMAKE_PREFIX_PATH'] = '/home/grin/scanner_ws/devel:/opt/ros/noetic'
    env['PYTHONPATH'] = '/home/grin/scanner_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages'
    env['LD_LIBRARY_PATH'] = '/home/grin/scanner_ws/devel/lib:/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu'
    command = ['/opt/ros/noetic/bin/catkin_make', '-C', str(WORKSPACE), '-j1',
               '-DPYTHON_EXECUTABLE=/usr/bin/python3.8', '-DPYTHON_INCLUDE_DIR=/usr/include/python3.8',
               '-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so',
               '-DCMAKE_CXX_FLAGS_DEBUG=-O2 -g0']
    # Upstream does not declare an explicit CMake dependency from the
    # executable to Pose6D.h. Generate it first on a clean workspace.
    subprocess.run(command + ['fast_lio_generate_messages_cpp'], env=env, check=True)
    subprocess.run(command, env=env, check=True)
    if not BINARY.is_file(): raise RuntimeError('Compilacao nao produziu ' + str(BINARY))
    print('FAST-LIO2 compilado:', BINARY, flush=True)


def main():
    try:
        install_source()
        build()
        return 0
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print('Instalacao FAST-LIO falhou:', exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
