#!/usr/bin/env bash
# RadarLink uConsole installer: packages, the Wi-Fi profile for the sensor node's
# network, the viewer autostart entry and/or the relay service, the firewall rule.
#
#   ./install.sh --ask-password                       # viewer only, join "RadarSystem"
#   ./install.sh --ask-password --viewer-autostart    # ... and start the viewer at login
#   sudo ./install.sh --password 'secret' --relay --web-token s3cret   # headless relay + web view
#   ./install.sh --dry-run ...                        # print what would be done
#   ./install.sh --uninstall                          # remove what this script installed
#
# Safe to run again: every step is idempotent. Tested on Raspberry Pi OS (Bookworm
# and Trixie) with NetworkManager; on other distributions the apt and nmcli steps can
# be skipped with --no-apt / --no-wifi and done by hand (see README, "Set up the
# uConsole"). System files (the relay unit, its token file) are written through
# sudo; user files (the autostart entry, the data directory) belong to the login
# user even when the script itself runs under sudo.
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
PREFIX=""                      # root for SYSTEM files such as /etc/systemd (tests); user files follow the user's home

usage() { sed -n '2,/^[^#]/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }

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
USER_HOME=""
if [ -n "${SUDO_USER:-}" ]; then USER_HOME="$(getent passwd "$SUDO_USER" 2>/dev/null | cut -d: -f6 || true)"; fi   # sudo resets HOME to /root
USER_HOME="${USER_HOME:-$HOME}"
RUN_GROUP="$(id -gn "$RUN_USER" 2>/dev/null || id -gn)"
DATA_DIR="$USER_HOME/radarlink"
PYTHON="$(command -v python3 || echo /usr/bin/python3)"
AUTOSTART_DIR="$USER_HOME/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/radar-viewer.desktop"
RELAY_UNIT="$PREFIX/etc/systemd/system/radar-relay.service"
RELAY_ENV="$PREFIX/etc/radarlink/relay.env"
STEPS_DONE=()

say()  { printf '\n== %s\n' "$*"; }
note() { printf '   %s\n' "$*"; }
run()  { printf '   + %s\n' "$*"; [ "$DRY_RUN" = 1 ] || "$@"; }
as_root() {
  if [ "$(id -u)" = 0 ]; then run "$@"
  elif command -v sudo >/dev/null 2>&1; then run sudo "$@"
  else echo "need root for: $*" >&2; return 1; fi
}
# system files: through root unless a --prefix redirects them (tests)
sys_install() { if [ -n "$PREFIX" ]; then run install "$@"; else as_root install "$@"; fi; }
sys_rm()      { if [ -n "$PREFIX" ]; then run rm -f "$@"; else as_root rm -f "$@"; fi; }
have() { command -v "$1" >/dev/null 2>&1; }
# user files: created for the login user, owned by them even under sudo
user_dir() {
  if [ "$(id -u)" = 0 ] && [ "$RUN_USER" != root ]; then run install -d -o "$RUN_USER" -g "$RUN_GROUP" "$1"
  else run mkdir -p "$1"; fi
}
own_user() { if [ "$(id -u)" = 0 ] && [ "$RUN_USER" != root ] && [ "$DRY_RUN" != 1 ]; then chown "$RUN_USER:$RUN_GROUP" "$1"; fi; }
write_file() {                 # write_file PATH <<EOF ... (user file, content on stdin)
  local path="$1"
  printf '   + write %s\n' "$path"
  if [ "$DRY_RUN" = 1 ]; then cat >/dev/null; return 0; fi
  user_dir "$(dirname "$path")" >/dev/null
  cat >"$path"
  own_user "$path"
}
# quoting for a Desktop Entry Exec= line: double quotes, backslash-escape \ " $ `, % doubled
dq() { local s=${1//\\/\\\\}; s=${s//\"/\\\"}; s=${s//\$/\\\$}; s=${s//\`/\\\`}; s=${s//%/%%}; printf '"%s"' "$s"; }
# quoting for a systemd ExecStart= line: double quotes, \ and " escaped, $ and % doubled
sq() { local s=${1//\\/\\\\}; s=${s//\"/\\\"}; s=${s//\$/\$\$}; s=${s//%/%%}; printf '"%s"' "$s"; }
# nmcli -t escapes ':' and '\' in names; compare literally after unescaping
profile_exists() { nmcli -t -f NAME con show 2>/dev/null | sed 's/\\\(.\)/\1/g' | grep -qxF -- "$1"; }

# ------------------------------------------------------------------ uninstall
if [ "$UNINSTALL" = 1 ]; then
  say "removing the viewer autostart entry"
  if [ -e "$DESKTOP_FILE" ]; then run rm -f "$DESKTOP_FILE"; else note "not installed"; fi
  say "removing the relay service"
  if [ -e "$RELAY_UNIT" ]; then
    if have systemctl && [ -z "$PREFIX" ]; then as_root systemctl disable --now radar-relay || true; fi
    sys_rm "$RELAY_UNIT"
    [ -e "$RELAY_ENV" ] && sys_rm "$RELAY_ENV"
    if have systemctl && [ -z "$PREFIX" ]; then as_root systemctl daemon-reload || true; fi
  else note "not installed"; fi
  if [ "$DO_WIFI" = 1 ] && have nmcli; then
    say "removing the Wi-Fi profile '$SSID'"
    if profile_exists "$SSID"; then run nmcli con delete "$SSID"; else note "no such profile"; fi
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
    pkgs=(python3 python3-pygame python3-numpy python3-serial iw)   # all preinstalled on the official uConsole image
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
      IFS= read -rs -p "   Wi-Fi password for '$SSID' (WIFI_PASS in the node's config.h): " PASSWORD; echo
    fi
    if profile_exists "$SSID"; then
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
    if [ "$CONNECT_NOW" = 1 ]; then
      if ! run nmcli con up "$SSID"; then note "could not connect now (node not up yet?); autoconnect keeps retrying"; fi
    else
      note "connect with: nmcli con up '$SSID'   (or let autoconnect do it)"
    fi
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
Exec=$(dq "$PYTHON") $(dq "$HERE/radar_viewer.py") --node $NODE_IP
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
  user_dir "$DATA_DIR"
  env_line=""
  if [ -n "$WEB_TOKEN" ]; then
    # the token stays out of the unit file and the process list: root-only env file
    tmp_env="$(mktemp)"; printf 'RADAR_WEB_TOKEN=%s\n' "$WEB_TOKEN" >"$tmp_env"
    printf '   + write %s (mode 0600, holds the web token)\n' "$RELAY_ENV"
    sys_install -D -m 0600 "$tmp_env" "$RELAY_ENV"; rm -f "$tmp_env"
    env_line="EnvironmentFile=$RELAY_ENV"
  fi
  tmp_unit="$(mktemp)"
  cat >"$tmp_unit" <<UNIT
# generated by uconsole/install.sh; edit and run: sudo systemctl daemon-reload && sudo systemctl restart radar-relay
[Unit]
Description=RadarLink relay, alarm daemon and web view
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$(sq "$HERE")
${env_line}
ExecStart=$(sq "$PYTHON") $(sq "$HERE/radar_relay.py") --node $NODE_IP --web $WEB_PORT --armed --baseline $(sq "$DATA_DIR/baseline.json") --alert-log $(sq "$DATA_DIR/alerts.log") --snapshot-dir $(sq "$DATA_DIR/snapshots")
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
UNIT
  printf '   + write %s\n' "$RELAY_UNIT"
  sys_install -D -m 0644 "$tmp_unit" "$RELAY_UNIT"; rm -f "$tmp_unit"
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
    note "ufw is inactive (or not queryable without root), nothing to do"
  fi
fi

# ----------------------------------------------------------------- summary
say "done"
for s in "${STEPS_DONE[@]:-nothing installed (see --help)}"; do note "- $s"; done
note "check the setup with:   $PYTHON $HERE/doctor.py --ssid '$SSID' --node $NODE_IP --iface $IFACE"
note "run the viewer with:    $PYTHON $HERE/radar_viewer.py --node $NODE_IP"
[ "$RELAY" = 1 ] && note "web view:               http://$(hostname -I 2>/dev/null | awk '{print $1}' || echo '<this host>'):$WEB_PORT/${WEB_TOKEN:+?token=$WEB_TOKEN}"
exit 0
