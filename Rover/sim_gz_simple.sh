#!/usr/bin/env bash
# Entrada atualizada para ROS 2 Jazzy e Gazebo Harmonic.
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/iniciar_simulacao_completa.sh" "$@"
