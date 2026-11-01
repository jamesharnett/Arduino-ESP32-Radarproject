# Fixes prepared for the upstream repositories

The audit behind the RadarLink rewrite found defects in the original v1 code
and README. They are prepared as small, independent branches in this fork so
each can be opened as its own pull request against the author's repositories.
The branches are based on `main`, which is identical to the upstream `main`
(commit `c41c875`).

| Branch in this fork | Target repository | Files | Summary | Status |
|---|---|---|---|---|
| `fix/readme-wiring-and-facts` | `Stevee87/Arduino-ESP32-Radarproject` | `README.md` | 5 V boost into XIAO `BAT+` corrected (cell on BAT pads, boost feeds only the RD-03D); GIGA VIN 7–7.5 V or USB-C instead of 6 V; antennas; buzzer on 3V3; AP address 192.168.3.1; firmware paths; real enclosure files and sizes; BW4056 naming; board packages and Adafruit GFX dependency; duplicate caution; typo | **merged** upstream as #3 (2026-10-04) |
| `fix/wifiudp-include-case` | `Stevee87/Arduino-ESP32-Radarproject` | receiver `.ino` | `#include <WiFiUDP.h>` → `<WiFiUdp.h>`; the sketch did not compile on Linux | **merged** upstream as #5 (2026-10-04) |
| `fix/giga-receiver-robustness` | `Stevee87/Arduino-ESP32-Radarproject` | receiver `.ino` | check `WiFi.beginAP()` and show the failure on screen; drain all queued UDP datagrams per loop instead of one; `fb[12]` overflow → `fb[16]`; "R4" labels renamed | **merged** upstream as #2 (2026-10-04) |
| `fix/xiao-transmitter-nonblocking` | `Stevee87/Arduino-ESP32-Radarproject` | transmitter `.ino` | non-blocking Wi-Fi connect with retry instead of a 20 s block and `ESP.restart()`; the 650 ms radar re-configuration every 60 s turned into a timed step sequence; clustering keeps the speed with the larger magnitude | **merged** upstream as #4 (2026-10-04) |
| `fix/receiver-packet-format` in `jamesharnett/Radar-project-Uconsole` | `Stevee87/Radar-project-Uconsole` | `Firmware/uconsole_radar_receiver.py` | the Python receiver unpacked 29 of the 32 packet bytes and corrupted targets 2 and 3 (`"hhhB"` → `"hhhBx"` per target). The same change also exists as a `git am` patch on this fork's `fix/radar-project-uconsole-struct-format` branch, which is now redundant | **accepted**: upstream `main` carries the fix (comment reworded by the author) |
| `fix/firmware-1.1-packaging-and-readme` | `Stevee87/Arduino-ESP32-Radarproject` | `Firmware 1.1/…`, `README.md` | follow-up to the author's "Firmware 1.1" release: sketch folders renamed to match the sketch names (`rd03d_giga_receiver v1.1` → `rd03d_giga_receiver`, same for the sender) so the Arduino IDE opens them in place; README firmware paths updated; the RD-03D pin-table row (`BAT+ 3,7 V`) made consistent with the power paragraph it contradicts (5 V from the boost; the module's manual specifies 4.5–5.5 V). The PR text asks the author to confirm which supply he actually uses | open for you to submit |
| `fix/sketch-folder` in `jamesharnett/Radar-project-Uconsole` | `Stevee87/Radar-project-Uconsole` | `Firmware/rd03d_xiao_s3_transmitter/` | the transmitter `.ino` was flattened into `Firmware/` without its sketch folder; moved back so the Arduino IDE opens it | open for you to submit |

Each code branch was compiled for its board (esp32 core 3.3.12 for the XIAO,
Arduino Mbed OS GIGA Boards 4.6.0 for the GIGA) before it was pushed. After the
first four merged, the author re-published the sketches as "Firmware 1.1"; those
files keep every fix and are mirrored in `Firmware/legacy/` here. The
receiver robustness branch only builds on Linux once the include-name fix is
also merged; the two are kept separate so the one-line fix can land on its own.

## Opening the pull requests

From the fork on GitHub: *Pull requests → New → compare across forks*, base
`Stevee87/Arduino-ESP32-Radarproject:main`, head `<fork>:<branch>`. The commit
message of each branch is written to serve as the PR description.

For the `Radar-project-Uconsole` fix, open the pull request from
`jamesharnett/Radar-project-Uconsole:fix/receiver-packet-format` against
`Stevee87/Radar-project-Uconsole:main`.
