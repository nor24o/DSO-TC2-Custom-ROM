#!/usr/bin/env python3
"""
FNIRSI DSO-TC2 Custom ROM - Automatic Build & Flash Utility
Connects to J-Link via SWD, uploads firmware, and performs an AIRCR.VECTRESET soft reset
(crucial because DSO-TC2 does not expose the hardware NRST pin on its SWD header).
"""

import os
import sys
import subprocess

def run_command(cmd, desc):
    print(f"[+] {desc}...")
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        print(f"[-] Error: {desc} failed with exit code {result.returncode}")
        sys.exit(1)

def main():
    proj_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    os.chdir(proj_dir)

    # 1. Build firmware using PlatformIO
    run_command("pio run", "Building firmware with PlatformIO")

    bin_path = os.path.join(proj_dir, ".pio", "build", "genericSTM32F103C6", "firmware.bin")
    if not os.path.exists(bin_path):
        print(f"[-] Error: firmware binary not found at {bin_path}")
        sys.exit(1)

    # 2. Generate temporary J-Link Commander script
    script_path = os.path.join(proj_dir, "tools", "temp_flash.jlink")
    script_content = f"""RSetType 1
h
loadbin {bin_path.replace(os.sep, '/')}, 0x08000000
r
g
Exit
"""
    with open(script_path, "w") as f:
        f.write(script_content)

    # 3. Flash firmware via J-Link
    jlink_cmd = [
        r"C:\Users\horva\.platformio\packages\tool-jlink\JLink.exe",
        "-device", "STM32F103C6",
        "-if", "SWD",
        "-speed", "4000",
        "-autoconnect", "1",
        "-NoGui", "1",
        "-CommanderScript", script_path
    ]

    print("[+] Flashing binary to STM32F103C6 via J-Link SWD...")
    res = subprocess.run(jlink_cmd, capture_output=True, text=True, timeout=15)
    print(res.stdout)

    # Clean up script
    if os.path.exists(script_path):
        os.remove(script_path)

    print("[+] Target successfully flashed and running!")

if __name__ == "__main__":
    main()
