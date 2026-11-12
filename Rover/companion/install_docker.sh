#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Execute com sudo.' >&2; exit 1; }
classic=false
case "${1:-}" in
  --classic-store) classic=true; shift;;
  '') ;;
  *) echo 'Uso: sudo bash install_docker.sh [--classic-store]' >&2; exit 2;;
esac
[[ $# == 0 ]] || exit 2
docker_user=${SUDO_USER:-${ROVER_DOCKER_USER:-}}
[[ -n "$docker_user" && "$docker_user" != root ]] || { echo 'Use sudo a partir do usuario de operacao, ou defina ROVER_DOCKER_USER.' >&2; exit 2; }
id "$docker_user" >/dev/null
# Optional only for a new machine with small storage. Never hide existing images.
if $classic; then
  if command -v docker >/dev/null || [[ -e /etc/docker/daemon.json ]]; then
    echo '--classic-store exige instalacao nova, sem Docker/configuracao existente.' >&2; exit 2
  fi
fi
source /etc/os-release
[[ $ID == ubuntu && $VERSION_CODENAME == noble ]] || { echo 'Exige Ubuntu 24.04.' >&2; exit 1; }
if $classic; then
  install -d -m 0755 /etc/docker
  printf '%s\n' '{"features":{"containerd-snapshotter":false}}' > /etc/docker/daemon.json
fi
export DEBIAN_FRONTEND=noninteractive
apt-get -o Acquire::Retries=3 update
apt-get -o Acquire::Retries=3 install -y ca-certificates curl
install -m 0755 -d /etc/apt/keyrings
curl --retry 5 -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
cat > /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
apt-get -o Acquire::Retries=3 update
apt-get -o Acquire::Retries=3 install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
systemctl enable --now docker
usermod -aG docker "$docker_user"
docker version
