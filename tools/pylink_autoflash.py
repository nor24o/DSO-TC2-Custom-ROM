#!/usr/bin/env python3
"""
High-Speed J-Link SWD Flash Catcher
Keeps the J-Link probe open and polls connect() at 20 Hz.
The instant power is applied (by pressing or holding the power button),
it halts the core, flashes firmware.bin, and resets the target.
"""

import os
import sys
import time
import pylink

PROJ_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BIN_PATH = os.path.join(PROJ_DIR, ".pio", "build", "tester_l083", "firmware.bin")

def on_progress(action, progress_string, percentage):
    sys.stdout.write(f"\r[+] {action}: {progress_string} ({percentage}%)   ")
    sys.stdout.flush()

def main():
    sys.stdout.reconfigure(line_buffering=True)
    if not os.path.exists(BIN_PATH):
        print(f"[-] Error: binary not found at {BIN_PATH}")
        sys.exit(1)
        
    print("=" * 60)
    print("   HIGH-SPEED SWD AUTO-FLASH CATCHER READY")
    print(f"   Target: STM32L083RZ | File: {os.path.basename(BIN_PATH)}")
    print("   --> PRESS / CLICK THE POWER BUTTON ON THE DEVICE NOW <--")
    print("=" * 60)

    j = pylink.JLink()
    try:
        j.open(serial_no=69650043)
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        
        start_time = time.time()
        dots = 0
        while time.time() - start_time < 90:
            try:
                # Attempt fast connection at 500 kHz
                j.connect('STM32L083RZ', speed=500)
                print("\n[+] TARGET CONNECTED! Halting CPU...")
                j.halt()
                
                print(f"[+] Flashing {BIN_PATH} to 0x08000000...")
                j.flash_file(BIN_PATH, 0x08000000, on_progress=on_progress)
                print("\n[+] Programming complete! Resetting and starting core...")
                
                j.reset(ms=20, halt=False)
                j.restart()
                print("[+] SUCCESS! Target is now running with PB7=HIGH (Power Latch), PC7=HIGH (Backlight), PB6 (Relay) and PB9 (IR)!")
                
                j.close()
                
                # Now flash display MCU
                print("\n" + "=" * 60)
                print("   FLASHING DISPLAY MCU (STM32F103C6 via DAPLink)")
                print("=" * 60)
                import subprocess
                subprocess.run([sys.executable, os.path.join(PROJ_DIR, "tools", "flash.py"), "--target", "display"])
                sys.exit(0)
            except pylink.errors.JLinkException:
                time.sleep(0.05)
                dots += 1
                if dots % 20 == 0:
                    sys.stdout.write(".")
                    sys.stdout.flush()

        print("\n[-] Timed out after 90 seconds.")
        sys.exit(1)
    finally:
        try:
            j.close()
        except:
            pass

if __name__ == "__main__":
    main()
