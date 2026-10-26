#!/usr/bin/env python3
"""
Auto-Flasher: Waits for user to hold down Power / OK button,
catches the SWD target instantly, and flashes the safe firmware.
"""

import os
import sys
import time
import subprocess

PROJ_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
JLINK_EXE = r"C:\Users\horva\.platformio\packages\tool-jlink\JLink.exe"
SCRIPT_PATH = os.path.join(PROJ_DIR, "tools", "flash_tester.jlink")

def try_flash():
    cmd = [
        JLINK_EXE,
        "-device", "STM32L083RZ",
        "-if", "SWD",
        "-speed", "1000",
        "-autoconnect", "1",
        "-NoGui", "1",
        "-CommanderScript", SCRIPT_PATH
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = res.stdout.decode(errors="ignore")
    if ("Programming flash [100%]" in out or 
        ("O.K." in out and "Flash download:" in out) or
        ("O.K." in out and "Contents already match" in out)):
        print("\n[+] SUCCESS! STM32L083RZ successfully programmed!")
        return True
    return False

def main():
    print("=" * 60)
    print("   HOLD DOWN THE POWER / OK BUTTON NOW...")
    print("   Waiting for continuous power to flash STM32L083...")
    print("=" * 60)
    
    start_time = time.time()
    attempts = 0
    while time.time() - start_time < 60:
        attempts += 1
        sys.stdout.write(f"\r[*] Attempt {attempts}... ")
        sys.stdout.flush()
        if try_flash():
            print("[+] Done! You can release the button now.")
            sys.exit(0)
        time.sleep(0.3)
        
    print("\n[-] Timed out after 30 seconds.")
    sys.exit(1)

if __name__ == "__main__":
    main()
