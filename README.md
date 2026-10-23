# FNIRSI DSO-TC2 High-Performance Custom ROM

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange?logo=platformio)](https://platformio.org/)
[![Target MCU](https://img.shields.io/badge/MCU-STM32F103C6%20%2F%20CH32F103-blue?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c6.html)
[![Display Refresh](https://img.shields.io/badge/Display%20Engine-186%20FPS-brightgreen)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An open-source, ultra-high-performance Custom ROM and reverse-engineering framework for the **FNIRSI DSO-TC2** handheld multi-function instrument (Oscilloscope, Transistor / Component Tester, and Signal Generator).

---

## 📷 Photographic Verification

![FNIRSI DSO-TC2 Custom ROM Running Live](docs/images/dso_tc2_screen.jpg)
*Figure 1: FNIRSI DSO-TC2 running the custom graphics engine with crisp text, sub-pixel grid, and live ADC telemetry.*

---

## 🚀 Key Features

- **Blazing Fast Display Engine (186 FPS)**:
  - Direct 16-bit 8080-II parallel bus driver using direct register manipulation (`GPIOB->ODR`, `GPIOC->BSRR/BRR`).
  - Cycle-exact inline assembly write strobing (`PC14`) respecting the STM32 RTC power domain pin slew rates.
  - Differential bounding-span column rendering—updates moving waveforms and telemetry with zero ghosting and zero leftover pixels.
- **Hardware Architecture Reverse-Engineered**:
  - Full pinout mapping for all 48 pins of the main display/oscilloscope MCU.
  - Discovery of the dual-MCU architecture (MindMotion MM32F103 + WCH CH32F103 / STM32F103).
  - Inter-MCU serial communication link identified on `PA9` (TX) and `PA10` (RX).
- **Component Tester Diagnostic & Pin Explorer**:
  - Live 20 Hz telemetry scanning of all 7 ADC channels (`PA0` through `PA6`).
  - Automatic change detection on the 1-2-3 test socket taps (`PA4`, `PA5`, `PA6`).
  - Front-panel keypad monitor.
- **Support for Aftermarket Replacement Chips**:
  - Optimized for soldered STM32F103C6 / CH32F103 chips with **10 KB SRAM** and **32 KB Flash**.
  - Correct stack pointer placement preventing immediate HardFaults observed with standard C8 configurations.
- **One-Click Flash & Run Tooling**:
  - Python flashing utility using `AIRCR.VECTRESET` soft resets, bypassing the board's missing hardware `NRST` pin.

---

## 🔬 Reverse-Engineering Discoveries

### 1. Normally-White (TN) Panel Inversion Fix
The 2.4-inch 320×240 TFT LCD on the DSO-TC2 is a **Normally White (TN)** panel:
- Standard ST7789 libraries issue command `0x21` (`INVON`), which inverts every pixel, turning black backgrounds bright white and yellow traces dark blue.
- **The Fix**: The initialization sequence explicitly issues `0x20` (`INVOFF`). Black renders at `0x0000`, white at `0xFFFF`, and colors match true RGB565 values.

### 2. Dual-Wired Bit 0 (`D0` on `PA7` & `PB0`)
The PCB routes data bit 0 to both `PB0` and `PA7`. The firmware drives both pins synchronously during parallel bus transfers to ensure signal integrity across revisions.

### 3. Missing NRST Pin on SWD Header
The SWD header on the PCB exposes only `3.3V`, `GND`, `SWDIO` (`PA13`), and `SWCLK` (`PA14`). Because hardware reset is not connected:
- Standard J-Link / ST-Link uploads time out trying to pull `NRST`.
- **The Fix**: All scripts use `AIRCR.VECTRESET` (`RSetType 1` in SEGGER J-Link), executing instantaneous software core resets.

---

## 📌 Complete Pin Assignment (LQFP48)

| Signal / Role | MCU Pins | Description |
| :--- | :--- | :--- |
| **TFT Data Bus (16-bit)** | `PB0` – `PB15`, `PA7` | 16-bit parallel 8080 data bus (`D0` to `D15`) |
| **TFT Control Lines** | `PC13` (`DCX`), `PC14` (`WRX`), `PC15` (`RDX`), `PA8` (`RESX`), `PA15` (`CSX`) | Hardware control strobes |
| **Clock & Debug** | `PD0`/`PD1` (8 MHz Crystal), `PA13` (`SWDIO`), `PA14` (`SWCLK`) | System clock and SWD programming |
| **Oscilloscope Input** | `PA0` (ADC Channel 0) | MCX probe input via analog buffer |
| **Signal Generator** | `PA1` (ADC1 / Timer 2 CH2) | PWM / waveform output |
| **Battery Monitor** | `PA2` (ADC Channel 2) | Battery voltage divider |
| **Front Keypad** | `PA3` (ADC Channel 3) | Analog resistor ladder for buttons |
| **Test Socket 1-2-3** | `PA4`, `PA5`, `PA6` | ZIF socket analog sense lines |
| **Inter-MCU Link** | `PA9` (TX), `PA10` (RX) | USART1 serial link to MM32 tester MCU |
| **USB-C Interface** | `PA11` (D-), `PA12` (D+) | Virtual disk bootloader / USB communication |

*For complete details, see [docs/PINOUT.md](docs/PINOUT.md).*

---

## 🛠️ Building & Flashing

### Prerequisites
1. [PlatformIO CLI](https://platformio.org/) or VSCode with the PlatformIO extension.
2. [SEGGER J-Link Software](https://www.segger.com/downloads/jlink/) (connected to the DSO-TC2 SWD header).
3. Python 3.8+.

### 1. Build the Firmware
```bash
pio run
```

### 2. Flash and Run in One Step
```bash
python tools/flash.py
```
*This compiles the binary, uploads it to address `0x08000000`, triggers an `AIRCR.VECTRESET`, and resumes execution automatically.*

### 3. View Live Diagnostics (RTT)
```bash
python tools/read_rtt.py
```

---

## 📂 Repository Structure

```
├── docs/
│   ├── images/
│   │   └── dso_tc2_screen.jpg      # Photographic verification
│   ├── HARDWARE.md                 # Complete hardware architecture & dual-MCU details
│   └── PINOUT.md                   # Full 48-pin microcontroller map
├── include/
│   ├── font8x8.h                   # ASCII 8x8 font definition
│   └── tft.h                       # High-speed TFT driver interface
├── src/
│   ├── font8x8.cpp                 # Font rendering routines
│   ├── main.cpp                    # Active Component Tester Hardware Mapper
│   └── tft.cpp                     # 186 FPS ST7789 16-bit parallel driver
├── tools/
│   ├── flash.py                    # Automated J-Link upload & soft-reset tool
│   └── read_rtt.py                 # Live SEGGER RTT reader
├── platformio.ini                  # PlatformIO project configuration
├── LICENSE                         # MIT License
└── README.md                       # Main documentation
```

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
