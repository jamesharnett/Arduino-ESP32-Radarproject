#!/usr/bin/env bash
# RadarLink uConsole installer: packages, the Wi-Fi profile for the sensor node's
# network, the viewer autostart entry and/or the relay service, the firewall rule.
#
#   ./install.sh --ask-password                       # viewer only, join "RadarSystem"
#   ./install.sh --ask-password --viewer-autostart    # ... and start the viewer at login
#   ./install.sh --password 'secret' --relay --web-token s3cret   # headless relay + web view
#   ./install.sh --dry-run ...                        # print what would be done
#   ./install.sh --uninstall                          # remove what this script installed
#
# Safe to run again: every step is idempotent. Tested on Raspberry Pi OS (Bookworm)
# with NetworkManager; on other distributions the apt and nmcli steps can be skipped
# with --no-apt / --no-wifi and done by hand (see README, "Set up the uConsole").
set -euo pipefail

SSID="RadarSystem"
PASSWORD=""
ASK_PASSWORD=0
NODE_IP="192.168.4.1"
IFACE="wlan0"
VIEWER_AUTOSTART=0
RELAY=0
WEB_PORT=8080
WEB_TOKEN=""
DO_APT=1
DO_WIFI=1
CONNECT_NOW=0
DRY_RUN=0
UNINSTALL=0
PREFIX=""                      # root for SYSTEM files such as /etc/systemd (tests); user files follow $HOME

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --ssid) SSID="$2"; shift 2 ;;
    --password) PASSWORD="$2"; shift 2 ;;
    --ask-password) ASK_PASSWORD=1; shift ;;
    --node) NODE_IP="$2"; shift 2 ;;
    --iface) IFACE="$2"; shift 2 ;;
    --viewer-autostart) VIEWER_AUTOSTART=1; shift ;;
    --relay) RELAY=1; shift ;;
    --web-port) WEB_PORT="$2"; shift 2 ;;
    --web-token) WEB_TOKEN="$2"; shift 2 ;;
    --no-apt) DO_APT=0; shift ;;
    --no-wifi) DO_WIFI=0; shift ;;
    --connect) CONNECT_NOW=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --uninstall) UNINSTALL=1; shift ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
RUN_USER="${SUDO_USER:-$(id -un)}"
USER_HOME="${HOME}"
DATA_DIR="$USER_HOME/radarlink"
PYTHON="$(command -v python3 || echo /usr/bin/python3)"
AUTOSTART_DIR="$USER_HOME/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/radar-viewer.desktop"
RELAY_UNIT="$PREFIX/etc/systemd/system/radar-relay.service"
STEPS_DONE=()

say()  { printf '\n== %s\n' "$*"; }
note() { printf '   %s\n' "$*"; }
run()  { printf '   + %s\n' "$*"; [ "$DRY_RUN" = 1 ] || "$@"; }
as_root() {
  if [ "$(id -u)" = 0 ]; then run "$@"
  elif command -v sudo >/dev/null 2>&1; then run sudo "$@"
  else echo "need root for: $*" >&2; return 1; fi
}
have() { command -v "$1" >/dev/null 2>&1; }
write_file() {                 # write_file PATH <<EOF ... (content on stdin)
  local path="$1" dir
  dir="$(dirname "$path")"
  printf '   + write %s\n' "$path"
  if [ "$DRY_RUN" = 1 ]; then cat >/dev/null; return 0; fi
  mkdir -p "$dir"
  cat >"$path"
}

# ------------------------------------------------------------------ uninstall
if [ "$UNINSTALL" = 1 ]; then
  say "removing the viewer autostart entry"
  [ -e "$DESKTOP_FILE" ] && run rm -f "$DESKTOP_FILE" || note "not installed"
  say "removing the relay service"
  if [ -e "$RELAY_UNIT" ]; then
    have systemctl && as_root systemctl disable --now radar-relay || true
    as_root rm -f "$RELAY_UNIT"
    have systemctl && as_root systemctl daemon-reload || true
  else note "not installed"; fi
  if [ "$DO_WIFI" = 1 ] && have nmcli; then
    say "removing the Wi-Fi profile '$SSID'"
    if nmcli -t -f NAME con show 2>/dev/null | grep -qx -- "$SSID"; then run nmcli con delete "$SSID"; else note "no such profile"; fi
  fi
  note "kept: $DATA_DIR (baseline, alert log, snapshots), apt packages, firewall rules"
  exit 0
fi

# ---------------------------------------------------------------- preflight
say "RadarLink uConsole installer"
note "repository: $REPO"
note "user: $RUN_USER, home: $USER_HOME, python: $PYTHON"
[ "$DRY_RUN" = 1 ] && note "DRY RUN: nothing is changed"
if [ -r /proc/device-tree/model ] && grep -qi "raspberry\|clockwork\|uconsole" /proc/device-tree/model 2>/dev/null; then
  note "board: $(tr -d '\0' </proc/device-tree/model)"
fi
if [ ! -f "$HERE/radar_viewer.py" ]; then echo "run this from the repository's uconsole/ directory" >&2; exit 2; fi

# --------------------------------------------------------------------- apt
if [ "$DO_APT" = 1 ]; then
  say "packages"
  if have apt-get; then
    pkgs=(python3 python3-pygame python3-numpy iw)
    if [ "$DO_WIFI" = 1 ]; then pkgs+=(network-manager); fi
    as_root apt-get install -y --no-install-recommends "${pkgs[@]}"
    STEPS_DONE+=("packages: ${pkgs[*]}")
  else
    note "no apt-get here: install pygame and numpy yourself (pip install -r $HERE/requirements.txt)"
  fi
fi

# -------------------------------------------------------------------- wifi
if [ "$DO_WIFI" = 1 ]; then
  say "Wi-Fi profile '$SSID' (the sensor node's access point)"
  if ! have nmcli; then
    note "nmcli not found: this image does not use NetworkManager. Join '$SSID' with your network tool and"
    note "disable Wi-Fi power save (iw dev $IFACE set power_save off); keep your other uplink as the default route."
  else
    if [ -z "$PASSWORD" ] && [ "$ASK_PASSWORD" = 1 ]; then
      read -rs -p "   Wi-Fi password for '$SSID' (WIFI_PASS in the node's config.h): " PASSWORD; echo
    fi
    if nmcli -t -f NAME con show 2>/dev/null | grep -qx -- "$SSID"; then
      note "profile exists, updating it"
    else
      run nmcli con add type wifi con-name "$SSID" ifname "$IFACE" ssid "$SSID"
    fi
    # never-default: the node has no internet, keep the 4G/other uplink as the route out.
    # powersave 2 = off: a dozing Wi-Fi client delays and drops the node's small UDP datagrams.
    # autoconnect-retries 0 = keep trying forever (the node may boot after the uConsole).
    run nmcli con modify "$SSID" \
      ipv4.method auto ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled \
      802-11-wireless.powersave 2 connection.autoconnect yes connection.autoconnect-retries 0 \
      connection.autoconnect-priority 10
    if [ -n "$PASSWORD" ]; then
      if [ "$DRY_RUN" = 1 ]; then note "+ nmcli con modify $SSID wifi-sec.key-mgmt wpa-psk wifi-sec.psk ********"
      else nmcli con modify "$SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD"; note "+ nmcli con modify $SSID wifi-sec.key-mgmt wpa-psk wifi-sec.psk ********"; fi
    else
      note "no password given (--password / --ask-password): set it later with"
      note "  nmcli con modify '$SSID' wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<WIFI_PASS>'"
    fi
    if [ "$CONNECT_NOW" = 1 ]; then run nmcli con up "$SSID"; else note "connect with: nmcli con up '$SSID'   (or let autoconnect do it)"; fi
    STEPS_DONE+=("Wi-Fi profile $SSID (never-default, power save off, autoconnect)")
  fi
fi

# ------------------------------------------------------------------ viewer
if [ "$VIEWER_AUTOSTART" = 1 ]; then
  say "viewer autostart at login"
  write_file "$DESKTOP_FILE" <<DESKTOP
[Desktop Entry]
Type=Application
Name=RadarLink viewer
Comment=Radar and LIDAR view of the sensor node $NODE_IP
Exec=$PYTHON $HERE/radar_viewer.py --node $NODE_IP
Path=$HERE
Terminal=false
X-GNOME-Autostart-enabled=true
DESKTOP
  note "starts fullscreen with the desktop; ESC quits. Remove with --uninstall."
  STEPS_DONE+=("viewer autostart entry $DESKTOP_FILE")
fi

# ------------------------------------------------------------------- relay
if [ "$RELAY" = 1 ]; then
  say "relay service (headless: alarm daemon, web view on port $WEB_PORT, re-serves remote viewers)"
  run mkdir -p "$DATA_DIR"
  token_opt=""
  [ -n "$WEB_TOKEN" ] && token_opt=" --web-token $WEB_TOKEN"
  write_file "$RELAY_UNIT" <<UNIT
# generated by uconsole/install.sh; edit and run: sudo systemctl daemon-reload && sudo systemctl restart radar-relay
[Unit]
Description=RadarLink relay, alarm daemon and web view
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$HERE
ExecStart=$PYTHON $HERE/radar_relay.py --node $NODE_IP --web $WEB_PORT$token_opt --armed --baseline $DATA_DIR/baseline.json --alert-log $DATA_DIR/alerts.log --snapshot-dir $DATA_DIR/snapshots
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
UNIT
  if have systemctl && [ -z "$PREFIX" ]; then
    as_root systemctl daemon-reload
    as_root systemctl enable --now radar-relay
    note "follow it with: journalctl -u radar-relay -f"
  elif [ -n "$PREFIX" ]; then
    note "prefix set: service file written, systemd not touched"
  else
    note "no systemd: start by hand: $PYTHON $HERE/radar_relay.py --web $WEB_PORT"
  fi
  [ -z "$WEB_TOKEN" ] && note "no --web-token: anyone on the network can arm/disarm from the web page"
  STEPS_DONE+=("relay service $RELAY_UNIT, data in $DATA_DIR")
fi

# ---------------------------------------------------------------- firewall
if have ufw; then
  say "firewall"
  if ufw status 2>/dev/null | grep -q "Status: active"; then
    as_root ufw allow 4210/udp comment "RadarLink sensor node"
    [ "$RELAY" = 1 ] && as_root ufw allow "$WEB_PORT/tcp" comment "RadarLink web view"
    STEPS_DONE+=("ufw rules")
  else
    note "ufw is inactive, nothing to do"
  fi
fi

# ----------------------------------------------------------------- summary
say "done"
for s in "${STEPS_DONE[@]:-nothing installed (see --help)}"; do note "- $s"; done
note "check the setup with:   $PYTHON $HERE/doctor.py --ssid '$SSID' --node $NODE_IP --iface $IFACE"
note "run the viewer with:    $PYTHON $HERE/radar_viewer.py --node $NODE_IP"
[ "$RELAY" = 1 ] && note "web view:               http://$(hostname -I 2>/dev/null | awk '{print $1}' || echo '<this host>'):$WEB_PORT/${WEB_TOKEN:+?token=$WEB_TOKEN}"
exit 0
