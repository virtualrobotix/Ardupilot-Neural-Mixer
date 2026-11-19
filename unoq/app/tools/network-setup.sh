#!/bin/bash
# Installs the ArduDuck network helper (tools/arduduck_network.py) as a systemd user service on the
# Uno Q. Run on the board as the arduino user, without sudo:
#
#   bash ~/ArduinoApps/arduhumanoid/tools/network-setup.sh            install or update, and start
#   bash ~/ArduinoApps/arduhumanoid/tools/network-setup.sh --remove   stop and remove
#
# The service starts at boot (the arduino user has lingering on). If the Wi-Fi does not connect in
# 45 s it turns the board into the access point "ArduDuck", initial Wi-Fi password "runrunrun".
set -euo pipefail
app="$(cd "$(dirname "$0")/.." && pwd)"
unit_dir="$HOME/.config/systemd/user"
unit="$unit_dir/arduduck-network.service"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

if [[ "${1:-}" == "--remove" ]]; then
  systemctl --user disable --now arduduck-network.service 2>/dev/null || true
  rm -f "$unit"
  systemctl --user daemon-reload
  echo "arduduck-network removed (the access point profile ArduDuck-AP is kept in NetworkManager)"
  exit 0
fi

mkdir -p "$unit_dir" "$app/data/network"
cat > "$unit" <<EOF
[Unit]
Description=ArduDuck network helper (Wi-Fi setup and access point fallback)

[Service]
ExecStart=/usr/bin/python3 -u $app/tools/arduduck_network.py $app/data/network
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
EOF
loginctl enable-linger "$(id -un)" 2>/dev/null || true
systemctl --user daemon-reload
systemctl --user enable arduduck-network.service
systemctl --user restart arduduck-network.service
sleep 1
systemctl --user --no-pager status arduduck-network.service | head -6
