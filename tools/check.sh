#!/usr/bin/env bash
# One-command check: protocol copies in sync, host parser tests, Python tests and,
# when arduino-cli is installed, both firmware builds via their sketch.yaml profiles.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== protocol header copies"
python3 tools/check_protocol_sync.py

echo "== host parser tests (g++)"
g++ -std=c++11 -Wall -Wextra -Werror -I Firmware/sensor_node_esp32s3 Firmware/tests/host/test_parsers.cpp -o /tmp/radarlink_test_parsers
/tmp/radarlink_test_parsers

echo "== Python tests"
python3 -m unittest discover -s uconsole/tests

if command -v arduino-cli >/dev/null 2>&1; then
  echo "== firmware: sensor node (LIDAR on)"
  arduino-cli compile --profile xiao_esp32s3 --warnings all Firmware/sensor_node_esp32s3
  echo "== firmware: sensor node (LIDAR off)"
  tmp=$(mktemp -d); cp -r Firmware/sensor_node_esp32s3 "$tmp/sensor_node_esp32s3"
  sed -i 's/#define ENABLE_LIDAR       1/#define ENABLE_LIDAR       0/' "$tmp/sensor_node_esp32s3/config.h"
  arduino-cli compile --profile xiao_esp32s3 --warnings all "$tmp/sensor_node_esp32s3"; rm -rf "$tmp"
  echo "== firmware: GIGA display"
  arduino-cli compile --profile giga --warnings all Firmware/giga_display
else
  echo "== arduino-cli not found: firmware builds skipped (see README, Toolchain versions)"
fi
echo "all checks passed"
