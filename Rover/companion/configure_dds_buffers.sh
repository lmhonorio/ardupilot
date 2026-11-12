#!/usr/bin/env bash
# Larger UDP socket buffers for fragmented PointCloud2 traffic. No IP/route changes.
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Execute com sudo: os limites dos sockets pertencem ao kernel.' >&2; exit 1; }
config=/etc/sysctl.d/70-rover-dds.conf
backup=/var/lib/rover-companion/dds-buffers-before.txt
install -d -m 0755 /var/lib/rover-companion
keys=(net.core.rmem_max net.core.rmem_default net.core.wmem_max net.core.wmem_default)
if [[ ! -f "$backup" ]]; then sysctl "${keys[@]}" > "$backup"; fi
temp=$(mktemp);trap 'rm -f "$temp"' EXIT
printf '# Rover ROS 2: UDP buffers; no addressing changes.\n' > "$temp"
for key in "${keys[@]}"; do
  value=$(sysctl -n "$key")
  (( value >= 8388608 )) || value=8388608
  printf '%s = %s\n' "$key" "$value" >> "$temp"
done
# Full PointCloud2 packets can fill Linux's separate IP reassembly budget even
# when UDP socket buffers are large enough. Record and tune this budget too.
fragment_backup=/var/lib/rover-companion/dds-fragments-before.txt
if [[ ! -f "$fragment_backup" ]]; then
  sysctl net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_time > "$fragment_backup"
fi
fragment_bytes=$(sysctl -n net.ipv4.ipfrag_high_thresh)
(( fragment_bytes >= 67108864 )) || fragment_bytes=67108864
fragment_seconds=$(sysctl -n net.ipv4.ipfrag_time)
(( fragment_seconds <= 3 )) || fragment_seconds=3
printf 'net.ipv4.ipfrag_high_thresh = %s\nnet.ipv4.ipfrag_time = %s\n' "$fragment_bytes" "$fragment_seconds" >> "$temp"
install -m 0644 "$temp" "$config"
sysctl --load "$config"
echo "Valores anteriores: $backup. Reinicie os processos ROS para recriar os sockets."
echo "Remontagem IP: valores anteriores em $fragment_backup."
