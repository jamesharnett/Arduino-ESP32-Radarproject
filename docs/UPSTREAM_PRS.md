# Fixes prepared for the upstream repositories

The audit behind the RadarLink rewrite found defects in the original v1 code
and README. They are prepared as small, independent branches in this fork so
each can be opened as its own pull request against the author's repositories.
The branches are based on `main`, which is identical to the upstream `main`
(commit `c41c875`).

| Branch in this fork | Target repository | Files | Summary |
|---|---|---|---|
| `fix/readme-wiring-and-facts` | `Stevee87/Arduino-ESP32-Radarproject` | `README.md` | 5 V boost into XIAO `BAT+` corrected (cell on BAT pads, boost feeds only the RD-03D); GIGA VIN 7–7.5 V or USB-C instead of 6 V; antennas; buzzer on 3V3; AP address 192.168.3.1; firmware paths; real enclosure files and sizes; BW4056 naming; board packages and Adafruit GFX dependency; duplicate caution; typo |
| `fix/wifiudp-include-case` | `Stevee87/Arduino-ESP32-Radarproject` | receiver `.ino` | `#include <WiFiUDP.h>` → `<WiFiUdp.h>`; the sketch did not compile on Linux |
| `fix/giga-receiver-robustness` | `Stevee87/Arduino-ESP32-Radarproject` | receiver `.ino` | check `WiFi.beginAP()` and show the failure on screen; drain all queued UDP datagrams per loop instead of one; `fb[12]` overflow → `fb[16]`; "R4" labels renamed |
| `fix/xiao-transmitter-nonblocking` | `Stevee87/Arduino-ESP32-Radarproject` | transmitter `.ino` | non-blocking Wi-Fi connect with retry instead of a 20 s block and `ESP.restart()`; the 650 ms radar re-configuration every 60 s turned into a timed step sequence; clustering keeps the speed with the larger magnitude |
| `fix/radar-project-uconsole-struct-format` | `Stevee87/Radar-project-Uconsole` | `upstream-patches/…/0001-*.patch` | the Python receiver unpacked 29 of the 32 packet bytes and corrupted targets 2 and 3; shipped as a `git am` patch with instructions because this fork has no fork of that repository |

Each code branch was compiled for its board (esp32 core 2.0.9 for the XIAO, whose
APIs used here are unchanged in 3.x; Arduino Mbed OS GIGA Boards 4.6.0 for the GIGA)
before it was pushed. The
receiver robustness branch only builds on Linux once the include-name fix is
also merged; the two are kept separate so the one-line fix can land on its own.

## Opening the pull requests

From the fork on GitHub: *Pull requests → New → compare across forks*, base
`Stevee87/Arduino-ESP32-Radarproject:main`, head `<fork>:<branch>`. The commit
message of each branch is written to serve as the PR description.

For the `Radar-project-Uconsole` fix, fork that repository first and follow
`upstream-patches/Radar-project-Uconsole/README.md` on the
`fix/radar-project-uconsole-struct-format` branch.
