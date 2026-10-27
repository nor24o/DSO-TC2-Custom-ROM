# FNIRSI DSO-TC2 High-Performance Custom ROM

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange?logo=platformio)](https://platformio.org/)
[![Display MCU](https://img.shields.io/badge/Display%20MCU-STM32F103C6-blue?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c6.html)
[![Tester MCU](https://img.shields.io/badge/Tester%20MCU-STM32L083RZ-green?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32l083rz.html)
[![Display Refresh](https://img.shields.io/badge/Display%20Engine-186%20FPS-brightgreen)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An open-source, ultra-high-performance Custom ROM and reverse-engineering framework for the **FNIRSI DSO-TC2** handheld multi-function test instrument (Oscilloscope, Graphical Transistor / Component Tester, and 38 kHz Infrared Remote Protocol Decoder).

---

## 📷 Photographic Verification

![FNIRSI DSO-TC2 Custom ROM Running Live](docs/images/dso_tc2_screen.jpg)
*Figure 1: FNIRSI DSO-TC2 running the custom graphics engine with crisp text, sub-pixel grid, and live telemetry.*

---

## 🚀 Key Features

- **Dual-MCU Distributed Architecture**:
  - **Tester MCU (`STM32L083RZT6`, LQFP-64)**: Manages the 6-button physical panel, soft power latch (`PB7`), LCD backlight PWM (`PC7`), audio piezo engine (`PC6`), oscilloscope input relay (`PB6`), 38 kHz IR demodulator (`PB9`), and Karl-Heinz style 1-2-3 component tester matrix.
  - **Display MCU (`STM32F103C6`, LQFP-48)**: Drives the 2.4" 320×240 TFT LCD via direct 16-bit 8080 parallel bus at **186 Frames Per Second (FPS)**.
  - **Synchronized High-Speed Telemetry Link**: Blazing-fast 115,200-baud UART protocol (`PA9`/`PA10` + redundant `PA2`) with **instant 0 ms event-driven key transmission** streaming real-time component measurements, remote codes, and Zener voltages with zero latency.
- **Graphical Component Tester (Screen 1)**:
  - **Full Multi-Junction Analysis**: Automatic detection and pinout identification for BJTs (**NPN** / **PNP**), MOSFETs (**N-Channel** / **P-Channel** with body diode verification), and diodes with exact forward voltage $V_f$.
  - **True Capacitor vs. Resistor Discrimination**: Multi-sample decay filtering prevents capacitive charge transients from being falsely identified as 1.6 M$\Omega$ resistors. Accurately measures from pF up to thousands of µF.
  - **Real-Time Schematic Symbols**: Dynamic electronic schematic glyphs (BJT, MOSFET, Diode, Resistor, Capacitor) with active pin badges ($B, C, E, G, D, S, A, K$) and visual 3-pin ZIF socket highlights.
- **Dedicated 38 kHz IR Remote Protocol Decoder (Screen 2)**:
  - Hardware EXTI timer engine decoding NEC, Samsung, and Sony pulse-distance encoded protocols.
  - Clean non-overlapping display dashboard: Size-2 Command Hex/Dec, 16-bit Address, 32-bit Raw code, dynamic 32-bit bitstream visualizer, and live 3-key history log.
- **Dedicated High-Voltage Zener Diode Tester (Screen 3)**:
  - Measures reverse breakdown voltage ($V_z$) up to **30V** on the front `K` (Cathode) and `A` (Anode) terminals.
  - Utilizes the onboard **Aerosemi MT3608 2A Step-Up Boost Converter** (~28V rail) with automatic current-limiting protection.
  - Dedicated electronic Zener schematic graphic with iconic bent cathode wings and dynamic voltage classification (`Vz Breakdown`, `Open Rail >28V`, `Forward Diode Vf`, `Short Circuit`).
- **Physical Keypad & Power Management**:
  - 6 physical buttons mapped: `POWER` (`PB13`), `OK / <` (`PB14`), `MENU / AUTO` (`PB15`), `UP / ^` (`PC8`), `DOWN / v` (`PC9`), and `TEST / >` (`PA8`).
  - Active power-hold soft latch (`PB7 = HIGH` on boot; hold POWER > 1.5s for graceful shutdown).
  - Backlight dimming (`MENU + UP/DOWN`) and relay toggle (`MENU + TEST`).
  - Screen cycle order via MENU: `Component Tester` $\rightarrow$ `IR Decoder` $\rightarrow$ `Zener Tester` $\rightarrow$ `Console`.

---

## 📌 Complete Pinout & Hardware Architecture

Complete hardware schematics, ASCII wiring diagrams, and pin assignments are documented in:
👉 **[docs/PINOUT.md](docs/PINOUT.md)** (Full pin tables and graphical schemes for both MCUs)  
👉 **[docs/HARDWARE.md](docs/HARDWARE.md)** (Reverse engineering findings, circuit theory, and power design)

### Quick Pin Summary

| Subsystem | MCU | Active Pins | Function |
| :--- | :---: | :---: | :--- |
| **TFT 16-Bit Parallel Bus** | STM32F103C6 | `PB0..PB15`, `PA7` | 16-bit data bus (`D0`..`D15`) |
| **TFT Control Strobes** | STM32F103C6 | `PC13` (`DCX`), `PC14` (`WRX`), `PC15` (`RDX`), `PA15` (`CSX`), `PA8` (`RESX`) | Bus timing & command select |
| **Inter-MCU UART** | Both | `PA9` (TX) $\leftrightarrow$ `PA10` (RX), `PA2` (redundant) | 38,400 baud packet protocol |
| **1-2-3 Socket ADC Sense**| STM32L083RZ | `PA3` (Pin 1), `PA4` (Pin 2), `PA5` (Pin 3) | 12-bit analog voltage taps |
| **680 $\Omega$ Low Probes** | STM32L083RZ | `PA7` ($R_{L1}$), `PB1` ($R_{L2}$), `PB10` ($R_{L3}$) | Low-range resistance/capacitance probes |
| **470 k$\Omega$ High Probes**| STM32L083RZ| `PB0` ($R_{H1}$), `PB2` ($R_{H2}$), `PB11` ($R_{H3}$) | High-range resistance/capacitance probes |
| **6-Button Front Panel** | STM32L083RZ | `PB13`, `PB14`, `PB15`, `PC8`, `PC9`, `PA8` | Active LOW navigation keys |
| **System Peripherals** | STM32L083RZ | `PB7` (Power Latch), `PC7` (Backlight), `PC6` (Buzzer), `PB6` (Relay), `PB9` (IR Rx) | Board control & audio |

---

## 🛠️ Building & Dual-MCU Flashing

### Prerequisites
1. [PlatformIO CLI](https://platformio.org/) or VSCode with PlatformIO.
2. [SEGGER J-Link](https://www.segger.com/downloads/jlink/) (connected to the STM32L083 SWD header).
3. CMSIS-DAP / DAPLink (connected to the STM32F103 SWD header).
4. Python 3.8+.

### Compile Both Targets
```bash
pio run
```

### Flash Both MCUs in One Automated Step
```bash
python tools/pylink_autoflash.py
```
*This tool automatically compiles both targets, catches the STM32L083 via Segger J-Link (press the front POWER button to power up), flashes the binary, and immediately proceeds to program the STM32F103 display controller via OpenOCD DAPLink.*

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
All contributions and custom firmware code copyright (c) 2026 nor24o.
