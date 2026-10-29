#!/usr/bin/python3
"""Install a user-local copy of ROS Noetic Hector Mapping (no sudo)."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

PYTHON_DIR = Path(__file__).resolve().parents[1]
LOCAL_ROOT = PYTHON_DIR / '.deps/hector'
LOCAL_BINARY = LOCAL_ROOT / 'opt/ros/noetic/lib/hector_mapping/hector_mapping'


def find_backend():
    for candidate in (Path('/opt/ros/noetic/lib/hector_mapping/hector_mapping'), LOCAL_BINARY):
        if candidate.is_file():
            return candidate
    return None


def check_backend(binary):
    result = subprocess.run(['ldd', str(binary)], capture_output=True, text=True)
    missing = [line.strip() for line in result.stdout.splitlines() if 'not found' in line]
    if result.returncode or missing:
        raise RuntimeError('Dependencias do Hector indisponiveis: ' + '; '.join(missing))


def install():
    cache = PYTHON_DIR / '.deps/cache'
    cache.mkdir(parents=True, exist_ok=True)
    if not shutil.which('apt-get') or not shutil.which('dpkg-deb'):
        raise RuntimeError('Este instalador requer apt-get e dpkg-deb (Ubuntu/ROS Noetic)')
    subprocess.run(['apt-get', 'download', 'ros-noetic-hector-mapping'], cwd=str(cache), check=True)
    archives = sorted(cache.glob('ros-noetic-hector-mapping_*.deb'), key=lambda p: p.stat().st_mtime)
    if not archives:
        raise RuntimeError('Download nao produziu o pacote Hector')
    subprocess.run(['dpkg-deb', '-x', str(archives[-1]), str(LOCAL_ROOT)], check=True)
    check_backend(LOCAL_BINARY)
    return LOCAL_BINARY


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Verificar sem baixar arquivos')
    args = parser.parse_args()
    try:
        binary = find_backend()
        if binary is None:
            if args.check:
                raise RuntimeError('Hector ausente; execute este instalador sem --check')
            binary = install()
        check_backend(binary)
        print('Hector Mapping pronto:', binary)
        return 0
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
