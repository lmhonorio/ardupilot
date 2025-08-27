#!/usr/bin/env bash
set -euo pipefail

# ===================== ENTRADAS =====================
DEFAULT_LOCATION="ARGO"
read -p "Digite a localização (padrão: $DEFAULT_LOCATION): " LOCATION
LOCATION=${LOCATION:-$DEFAULT_LOCATION}

read -p "Quantos robôs (N)? [3]: " N
N=${N:-3}

# ===================== PARAMS =====================
QGC_PORT=14550          # QGC recebe de todas as instâncias
MAV_BASE=14555          # porta base p/ MAVROS (instância 0)
STEP=10                 # incremento por instância: 14555, 14565, 14575...
SYSID0=1                # SYSID da instância 0
I0=0                    # índice base do --instance (-I)
FRAME="rover-skid"
MODEL="gazebo-rover"
PARAM_FILE="gzrover.param"
PAUSE=0.3

# Caminho do ArduPilot (ajuste se diferente)
ARDUPILOT_DIR="${HOME}/ardupilot"

# ===================== RESOLVE sim_vehicle.py =====================
SIMV_BIN="$(command -v sim_vehicle.py || true)"
if [[ -z "$SIMV_BIN" ]]; then
  CAND="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
  if [[ -x "$CAND" ]]; then
    SIMV_BIN="$CAND"
  else
    echo "[ERRO] sim_vehicle.py não encontrado no PATH nem em $CAND"
    echo "       Ajuste ARDUPILOT_DIR ou instale corretamente o ArduPilot."
    exit 1
  fi
fi

# ===================== TERMINAL HELPER =====================
pick_term() {
  if   command -v gnome-terminal >/dev/null 2>&1; then echo "gnome-terminal"
  elif command -v konsole         >/dev/null 2>&1; then echo "konsole"
  elif command -v mate-terminal   >/dev/null 2>&1; then echo "mate-terminal"
  elif command -v kitty           >/dev/null 2>&1; then echo "kitty"
  elif command -v alacritty       >/dev/null 2>&1; then echo "alacritty"
  elif command -v xterm           >/dev/null 2>&1; then echo "xterm"
  else echo ""; fi
}

open_term() {
  local title="$1"; shift
  local remote_cmd="$*"
  local term="$(pick_term)"

  # Sem GUI (DISPLAY vazio) ou sem terminal gráfico → roda no terminal atual
  if [[ -z "${DISPLAY:-}" || -z "$term" ]]; then
    echo "[WARN] Sem terminal gráfico disponível; rodando aqui mesmo."
    bash -lc "$remote_cmd"
    return
  fi

  case "$term" in
    gnome-terminal)
      gnome-terminal --title "$title" -- bash -lc "$remote_cmd" || {
        echo "[WARN] gnome-terminal falhou; rodando aqui."
        bash -lc "$remote_cmd"
      }
      ;;
    konsole)
      konsole --new-tab -p tabtitle="$title" -e bash -lc "$remote_cmd" || bash -lc "$remote_cmd" &
      ;;
    mate-terminal)
      mate-terminal --title "$title" -- bash -lc "$remote_cmd" || bash -lc "$remote_cmd"
      ;;
    kitty)
      kitty --title "$title" bash -lc "$remote_cmd" || bash -lc "$remote_cmd" &
      ;;
    alacritty)
      alacritty -t "$title" -e bash -lc "$remote_cmd" || bash -lc "$remote_cmd" &
      ;;
    xterm)
      xterm -T "$title" -hold -e bash -lc "$remote_cmd" || bash -lc "$remote_cmd" &
      ;;
  esac
}

# ===================== LOOP DE LANÇAMENTO =====================
for ((i=0; i<N; i++)); do
  inst=$((I0 + i))
  sysid=$((SYSID0 + i))
  mav_port=$((MAV_BASE + STEP*i))
  log="/tmp/sitl_${inst}.log"

  # Comando que será executado DENTRO de cada terminal:
  # 1) PATH=0 → PATH padrão → source ~/.bashrc
  # 2) (opcional) acrescenta ~/.local/bin
  # 3) executa sim_vehicle.py já resolvido (SIMV_BIN)
  # 4) log em /tmp/sitl_<I>.log e segura janela aberta no final
  remote_cmd=$(cat <<EOF
export PATH=0;
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games";
source ~/.bashrc 2>/dev/null || true;
[ -d "\$HOME/.local/bin" ] && export PATH="\$PATH:\$HOME/.local/bin";
echo "[ENV] PATH=\$PATH";
echo "[ENV] using: $SIMV_BIN";
"$SIMV_BIN" -D \
  --out=udp:127.0.0.1:${QGC_PORT} \
  --out=udp:127.0.0.1:${mav_port} \
  -L "${LOCATION}" -f ${FRAME} --model ${MODEL} \
  -I ${inst} --sysid ${sysid} \
  --console --add-param-file ${PARAM_FILE} \
  2>&1 | tee -a "$log" ;
echo; echo "Log: $log";
echo "Finalizado. Pressione ENTER para fechar.";
read
EOF
)

  echo "[SITL] Instância ${i}  (I=${inst}, SYSID=${sysid})  QGC:${QGC_PORT}  MAVROS:${mav_port}"
  open_term "SITL I=${inst} SYSID=${sysid} MAV=${mav_port}" "$remote_cmd"
  sleep "$PAUSE"
done

echo
echo "Lembrete: cada MAVROS deve escutar nessas portas (udp://:<porta>@):"
echo "  Instância 0 → ${MAV_BASE}, 1 → $((MAV_BASE+STEP)), 2 → $((MAV_BASE+STEP*2)) ..."
echo "Se 'não acontecer nada', veja os logs em /tmp/sitl_<I>.log."

