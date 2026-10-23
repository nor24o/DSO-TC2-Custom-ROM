#!/usr/bin/env python3
"""
FNIRSI DSO-TC2 Custom ROM - Live RTT Log Reader
Polls the SEGGER RTT circular buffer in target SRAM via SWD.
"""

import os
import sys
import subprocess
import time

def main():
    proj_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    script_path = os.path.join(proj_dir, "tools", "temp_rtt.jlink")

    script_content = """mem 0x2000039C, 0x400
Exit
"""
    with open(script_path, "w") as f:
        f.write(script_content)

    jlink_cmd = [
        r"C:\Users\horva\.platformio\packages\tool-jlink\JLink.exe",
        "-device", "STM32F103C6",
        "-if", "SWD",
        "-speed", "4000",
        "-autoconnect", "1",
        "-NoGui", "1",
        "-CommanderScript", script_path
    ]

    try:
        res = subprocess.run(jlink_cmd, capture_output=True, text=True, timeout=5)
        for line in res.stdout.splitlines():
            if "20000" in line:
                print(line)
    finally:
        if os.path.exists(script_path):
            os.remove(script_path)

if __name__ == "__main__":
    main()
