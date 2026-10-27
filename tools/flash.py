#!/usr/bin/env python3
"""
FNIRSI DSO-TC2 Dual-MCU Flashing Utility
- Target 'display': STM32F103C6 via CMSIS-DAP / DAPLink (LU_2022_8888) using OpenOCD
- Target 'tester':  STM32L083RZ via Segger J-Link (69650043) using JLink.exe
- Target 'all':     Flashes both targets sequentially
"""

import os
import sys
import argparse
import subprocess

PROJ_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OPENOCD_EXE = r"C:\Users\horva\.platformio\packages\tool-openocd\bin\openocd.exe"
OPENOCD_SCRIPTS = r"C:\Users\horva\.platformio\packages\tool-openocd\scripts"
JLINK_EXE = r"C:\Users\horva\.platformio\packages\tool-jlink\JLink.exe"

def run_cmd(cmd, desc):
    print(f"\n[+] {desc}...")
    res = subprocess.run(cmd, shell=True)
    if res.returncode != 0:
        print(f"[-] Error: {desc} failed with exit code {res.returncode}")
        sys.exit(1)

def flash_display():
    print("\n" + "=" * 60)
    print("   FLASHING DISPLAY MCU (STM32F103C6 via DAPLink)")
    print("=" * 60)
    
    run_cmd("pio run -e display_f103", "Compiling display_f103 firmware")
    
    bin_path = os.path.join(PROJ_DIR, ".pio", "build", "display_f103", "firmware.bin")
    if not os.path.exists(bin_path):
        print(f"[-] Error: binary not found at {bin_path}")
        sys.exit(1)
        
    bin_posix = bin_path.replace(os.sep, "/")
    openocd_cmd = [
        OPENOCD_EXE,
        "-s", OPENOCD_SCRIPTS,
        "-f", "interface/cmsis-dap.cfg",
        "-c", "adapter serial LU_2022_8888; transport select swd",
        "-f", "target/stm32f1x.cfg",
        "-c", f"reset_config none; program {bin_posix} 0x08000000 reset exit"
    ]
    print(f"[+] Executing OpenOCD flash to STM32F103C6...")
    res = subprocess.run(openocd_cmd)
    if res.returncode != 0:
        print(f"[-] Flash failed with code {res.returncode}")
        sys.exit(1)
    print("[+] STM32F103C6 flashed and restarted successfully!")

def flash_tester():
    print("\n" + "=" * 60)
    print("   FLASHING TESTER MCU (STM32L083RZ via Segger J-Link)")
    print("=" * 60)
    
    run_cmd("pio run -e tester_l083", "Compiling tester_l083 firmware")
    
    bin_path = os.path.join(PROJ_DIR, ".pio", "build", "tester_l083", "firmware.bin")
    if not os.path.exists(bin_path):
        print(f"[-] Error: binary not found at {bin_path}")
        sys.exit(1)
        
    script_path = os.path.join(PROJ_DIR, "tools", "temp_flash_l083.jlink")
    script_content = f"""h
loadfile {bin_path.replace(os.sep, '/')}, 0x08000000
r
g
q
"""
    with open(script_path, "w") as f:
        f.write(script_content)
        
    jlink_cmd = [
        JLINK_EXE,
        "-SelectEmuBySN", "69650043",
        "-device", "STM32L083RZ",
        "-if", "SWD",
        "-speed", "1000",
        "-autoconnect", "1",
        "-NoGui", "1",
        "-CommanderScript", script_path
    ]
    print(f"[+] Executing J-Link flash to STM32L083RZ...")
    res = subprocess.run(jlink_cmd, timeout=20)
    
    try:
        os.remove(script_path)
    except OSError:
        pass
        
    if res.returncode != 0:
        print(f"[-] Flash failed with code {res.returncode}")
        sys.exit(1)
    print("[+] STM32L083RZ flashed and restarted successfully!")

def main():
    parser = argparse.ArgumentParser(description="FNIRSI DSO-TC2 Dual-MCU Flashing Tool")
    parser.add_argument("pos_target", nargs="?", choices=["display", "tester", "all"], default=None,
                        help="Optional positional target: display, tester, or all")
    parser.add_argument("--target", choices=["display", "tester", "all"], default="all",
                        help="Select target MCU to flash (display, tester, or all)")
    args = parser.parse_args()

    target = args.pos_target if args.pos_target else args.target
    os.chdir(PROJ_DIR)
    
    if target == "display":
        flash_display()
    elif target == "tester":
        flash_tester()
    elif target == "all":
        flash_tester()
        flash_display()

if __name__ == "__main__":
    main()
