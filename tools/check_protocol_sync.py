#!/usr/bin/env python3
"""Fails when the two copies of radar_protocol.h differ. ``--fix`` copies the canonical one over."""
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
CANONICAL = ROOT / "Firmware" / "sensor_node_esp32s3" / "radar_protocol.h"
COPIES = [ROOT / "Firmware" / "giga_display" / "radar_protocol.h"]


def main() -> int:
    fix = "--fix" in sys.argv
    status = 0
    for copy in COPIES:
        if copy.exists() and copy.read_bytes() == CANONICAL.read_bytes():
            print(f"ok      {copy.relative_to(ROOT)}")
            continue
        if fix:
            shutil.copyfile(CANONICAL, copy)
            print(f"updated {copy.relative_to(ROOT)}")
        else:
            print(f"DIFFERS {copy.relative_to(ROOT)} (run with --fix)")
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
