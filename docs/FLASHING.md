# FNIRSI DSO-TC2 Complete Flashing & Programming Guide

This guide provides exhaustive, step-by-step instructions on **what to flash**, **how to flash**, **which programmers to use**, and **how to handle the unique hardware reset and power-latch quirks** of the FNIRSI DSO-TC2 dual-microcontroller architecture.

---

## 1. Dual-MCU Target Architecture

The FNIRSI DSO-TC2 runs on **two separate ARM microcontrollers** that must each be programmed with their respective firmware binary:

```
                           +----------------------------------------+
                           |           FNIRSI DSO-TC2 PCB           |
                           +----------------------------------------+
                                     |                     |
                  SWD Header 1       |                     |  SWD Header 2
                 (DAPLink / OpenOCD) |                     | (Segger J-Link)
                                     v                     v
                         +-----------------------+     +-----------------------+
                         |   DISPLAY / HOST MCU  |     |   TESTER / SYSTEM MCU |
                         |      STM32F103C6      |     |     STM32L083RZT6     |
                         |   (ARM Cortex-M3)     |     |   (ARM Cortex-M0+)    |
                         |  72 MHz, 32 KB Flash  |     |  32 MHz, 192 KB Flash |
                         +-----------------------+     +-----------------------+
                                     |                             |
                                     +======== UART Link ==========+
                                           PA9 (TX) <-> PA10 (RX)
                                               @ 38,400 baud
```

### Firmware Target Reference Table

| Parameter | Display / Host MCU | Tester / System MCU |
| :--- | :--- | :--- |
| **Microcontroller** | `STM32F103C6` (LQFP-48) | `STM32L083RZT6` (LQFP-64) |
| **CPU Core** | ARM Cortex-M3 @ 72 MHz | ARM Cortex-M0+ @ 32 MHz |
| **Flash Memory Size** | 32 KB (32,768 bytes) | 192 KB (196,608 bytes) |
| **SRAM Size** | 10 KB (10,240 bytes) | 20 KB (20,480 bytes) |
| **PlatformIO Environment**| `display_f103` | `tester_l083` |
| **Source Directory** | `src/display/` | `src/tester/` |
| **Output ELF Binary** | `.pio/build/display_f103/firmware.elf` | `.pio/build/tester_l083/firmware.elf` |
| **Output Raw Binary** | `.pio/build/display_f103/firmware.bin` | `.pio/build/tester_l083/firmware.bin` |
| **Target Flash Address** | `0x08000000` | `0x08000000` |
| **Recommended Programmer**| CMSIS-DAP / DAPLink (or ST-Link v2) | Segger J-Link (V9 / EDU / Mini / Clone) |
| **Programming Software** | OpenOCD (`openocd.exe`) | J-Link Commander (`JLink.exe`) |

---

## 2. Hardware SWD Programming Headers & Pinout

Each microcontroller has a dedicated 4-pin SWD programming header on the PCB.

```
       SWD Header Pinout (Both MCUs)
     +-----+-------------------------+
     | Pin | Signal Name             |
     +-----+-------------------------+
     |  1  | VCC (3.3V Power / VTref)|
     |  2  | SWDIO (Data)            |
     |  3  | SWCLK (Clock)           |
     |  4  | GND (System Ground)     |
     +-----+-------------------------+
```

### Microcontroller SWD Pin Mapping

| Signal | Display MCU (`STM32F103C6`) | Tester MCU (`STM32L083RZ`) | Notes |
| :--- | :---: | :---: | :--- |
| **`VCC`** | 3.3V rail | 3.3V rail | Connect to programmer VDD / VTref sense pin |
| **`SWDIO`** | **`PA13`** (Pin 34) | **`PA13`** (Pin 46) | Bi-directional SWD debug data |
| **`SWCLK`** | **`PA14`** (Pin 37) | **`PA14`** (Pin 49) | SWD debug clock line |
| **`GND`** | Ground | Ground | Common system reference ground |

---

## 3. Critical Hardware Quirks & Flashing Rules

Before attempting to flash, review these three critical hardware constraints:

### ⚠️ Quirk 1: Missing Hardware Reset Pin (`nRESET` Not Connected)
* Neither SWD header exposes the hardware reset (`NRST`) pin of the MCU.
* **Impact**: Hardware pin resets (`nRESET` pulses) will fail or time out.
* **Requirement**: Flashing tools **must** use software reset commands:
  - OpenOCD: `reset_config none;` with soft reset (`reset exit`).
  - Segger J-Link: Software reset via `r` (`AIRCR.SYSRESETREQ`).

### ⚠️ Quirk 2: Cortex-M0+ Reset Incompatibility (`VECTRESET` Will Lock Up)
* On ARM Cortex-M3/M4 (like `STM32F103`), `AIRCR.VECTRESET` (bit 0) resets only the CPU core.
* **On ARM Cortex-M0+ (`STM32L083`), `VECTRESET` is not implemented in silicon**.
* **Impact**: If you execute `RSetType 1` (VECTRESET) in J-Link on the STM32L083, the J-Link DAP reports `CPU could not be halted`, `Failed to power up DAP`, or `SYSRESETREQ has confused core`.
* **Requirement**: Always use standard software reset (`Reset type: NORMAL` / `AIRCR.SYSRESETREQ`) at SWD speed 1000–4000 kHz.

### ⚠️ Quirk 3: Active Power-Hold Soft-Latch (`PB7`)
* Power to both microcontrollers is maintained by a 3.3V buck regulator controlled by a soft latch on **`PB7`** of the Tester MCU (`STM32L083`).
* When the MCU boots, firmware immediately asserts `digitalWrite(PB7, HIGH)` to keep power alive.
* **Impact**: If the MCU is erased, halted, or reset during flashing without USB-C power connected, the power regulator turns off, causing the debugger connection to drop mid-flash.
* **Requirement**: **Keep a USB-C cable connected** to the DSO-TC2 during flashing, OR hold down the red **POWER** button until programming finishes.

---

## 4. Quick-Start Automated Flashing

The repository includes a Python utility [`tools/flash.py`](../tools/flash.py) that handles compilation and flashing automatically.

### Flash Both Microcontrollers (Recommended)
```bash
python tools/flash.py --target all
```
*Compiles both targets, flashes the STM32F103C6 via OpenOCD DAPLink, then flashes the STM32L083RZ via Segger J-Link.*

### Flash Only the Display MCU
```bash
python tools/flash.py --target display
```

### Flash Only the Tester MCU
```bash
python tools/flash.py --target tester
```

---

## 5. Manual Step-by-Step Flashing Procedures

### 5.1 Flashing the Display MCU (`STM32F103C6`) via OpenOCD & DAPLink

#### Step 1: Compile Firmware
```bash
pio run -e display_f103
```
*Output binary:* `.pio/build/display_f103/firmware.bin`

#### Step 2: Connect Programmer
Wire your CMSIS-DAP / DAPLink programmer to the Display MCU SWD header:
* `DAPLink 3V3` $\to$ `Display SWD Pin 1 (VCC)`
* `DAPLink SWDIO` $\to$ `Display SWD Pin 2 (SWDIO / PA13)`
* `DAPLink SWCLK` $\to$ `Display SWD Pin 3 (SWCLK / PA14)`
* `DAPLink GND` $\to$ `Display SWD Pin 4 (GND)`

#### Step 3: Flash with OpenOCD
Execute OpenOCD from your terminal (adjust path to `openocd.exe` as needed):
```bash
openocd -s "C:/Users/<USER>/.platformio/packages/tool-openocd/scripts" \
  -f interface/cmsis-dap.cfg \
  -c "adapter serial LU_2022_8888; transport select swd; adapter speed 1000" \
  -f target/stm32f1x.cfg \
  -c "reset_config none; program .pio/build/display_f103/firmware.bin 0x08000000 reset exit"
```

*Expected output:*
```text
Info : [stm32f1x.cpu] Cortex-M3 r1p1 processor detected
Info : device id = 0x10006412
Info : flash size = 32 KiB
** Programming Started **
** Programming Finished **
** Resetting Target **
```

---

### 5.2 Flashing the Tester MCU (`STM32L083RZT6`) via Segger J-Link

#### Step 1: Compile Firmware
```bash
pio run -e tester_l083
```
*Output binary:* `.pio/build/tester_l083/firmware.bin`

#### Step 2: Connect Programmer
Wire your Segger J-Link to the Tester MCU SWD header:
* `J-Link Pin 1 (VTref)` $\to$ `Tester SWD Pin 1 (VCC)`
* `J-Link Pin 7 (SWDIO)` $\to$ `Tester SWD Pin 2 (SWDIO / PA13)`
* `J-Link Pin 9 (SWCLK)` $\to$ `Tester SWD Pin 3 (SWCLK / PA14)`
* `J-Link Pin 4 (GND)`   $\to$ `Tester SWD Pin 4 (GND)`
* *Plug in USB-C to the DSO-TC2 to ensure uninterrupted 3.3V power.*

#### Step 3: Flash with JLink.exe
Create a temporary command script `flash_tester.jlink`:
```text
device STM32L083RZ
si SWD
speed 1000
h
loadfile .pio/build/tester_l083/firmware.bin, 0x08000000
r
g
q
```

Execute J-Link Commander:
```bash
JLink.exe -NoGui 1 -ExitOnError 1 -CommanderScript flash_tester.jlink
```

*Expected output:*
```text
Device "STM32L083RZ" selected.
Found Cortex-M0 r0p1, Little endian.
Downloading file [.pio/build/tester_l083/firmware.bin]...
Comparing flash   [100%] Done.
Erasing flash     [100%] Done.
Programming flash [100%] Done.
Reset: Halt core after reset via DEMCR.VC_CORERESET.
Reset: Reset device via AIRCR.SYSRESETREQ.
```

---

## 6. Real-Time Telemetry & Diagnostics (J-Link RTT)

The Tester MCU firmware integrates **Segger Real-Time Terminal (RTT)**, streaming live telemetry and diagnostic traces over SWD with zero UART latency and zero CPU overhead.

### Stream Live Logs
With the J-Link connected to the Tester MCU:
```bash
python tools/stream_rtt.py
```

### What You Will See on RTT:
* **Boot Self-Test**: ADC calibration, supply voltage ($V_{CC}$), battery voltage on `PA2`, and SMA voltage on `PA6`.
* **Zener Boost Test**: Aerosemi MT3608 28V step-up rail verification.
* **Component Testing**: Step-by-step Karl-Heinz matrix evaluation, diode forward voltages ($V_f$), BJT beta ($h_{FE}$), MOSFET gate thresholds, and capacitor ESR.
* **1-Wire Discovery**: Live 64-bit laser ROM serial codes and digital temperatures (°C / °F) from **DS18B20 / DS18S20 / DS1822**.
* **Keypad Events**: Debounced key press/release notifications with exact millisecond timestamps and bitmasks.
* **Signal Generator Status**: Frequency presets, active waveforms (Square, Pulse, Triangle, Sine, Ramp), and duty cycle updates on `PB8`.

---

## 7. Troubleshooting & Recovery

| Issue | Root Cause | Solution |
| :--- | :--- | :--- |
| **`Cannot connect to target`** | Device powered off or floating `PB7` | Plug in USB-C cable or hold down the red `POWER` button while initiating the flash. |
| **`Target not halted` / `VECTRESET error`** | J-Link script used `RSetType 1` on Cortex-M0+ | Use `Reset type: NORMAL` / `AIRCR.SYSRESETREQ` in J-Link configuration. |
| **`OpenOCD timed out waiting for reset`** | `NRST` pin is not connected to SWD header | Add `reset_config none;` to your OpenOCD command line. |
| **White screen after flashing Display MCU** | ST7789 inversion bit misconfigured | The LCD is a **Normally White (TN)** panel; ensure `0x20` (`INVOFF`) is sent, not `0x21` (`INVON`). |
| **Buttons unresponsive on boot** | USART1 baud rate mismatch | Inter-MCU communication is locked at **38,400 baud** due to the `RS0102YH8` level-shifter bandwidth. Verify both firmwares use 38,400 baud. |
| **Flash size overflow on STM32F103** | Firmware binary exceeds 32 KB | The STM32F103C6 has strictly 32 KB flash. Ensure compiler optimization is set to `-Os` in `platformio.ini`. |
