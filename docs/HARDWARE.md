# FNIRSI DSO-TC2 Hardware Architecture & Reverse Engineering Reference

## 1. System Overview

The **FNIRSI DSO-TC2** is a handheld 3-in-1 test instrument combining:
- A digital storage oscilloscope (2.5 MS/s, 200 kHz bandwidth)
- A full-featured component & transistor tester (LCR, ESR, BJT, MOSFET, Diodes)
- A PWM signal / function generator
- A high-voltage Zener diode breakdown tester (up to ~24–30V)
- An IR remote protocol decoder

The device is housed in a compact handheld casing with a 1500 mAh rechargeable lithium-ion battery, a USB-C interface for charging and firmware updates, and a 2.4-inch 320×240 color TFT display.

```mermaid
graph TD
    subgraph DSO_TC2 ["FNIRSI DSO-TC2 System Architecture"]
        Batt["1500mAh LiPo Battery"] --> PowerIC["Charge & Power Management"]
        PowerIC --> Boost["30V DC-DC Boost Converter"]
        Boost --> ZenerSocket["Zener Socket (K-A-A)"]
        
        subgraph MCU2 ["MCU 2: WCH CH32F103 / STM32F103 (LQFP48) - This Custom ROM"]
            DisplayCtrl["16-bit Parallel LCD Engine"]
            ScopeADC["Fast ADC (PA0)"]
            USBC["USB-C Interface (PA11/PA12)"]
            Keypad["Button Sense"]
        end
        
        subgraph MCU1 ["MCU 1: MindMotion MM32F103RET6 (LQFP64)"]
            TesterCode["Karl-Heinz Component Tester"]
            ResistorNet["680Ω / 470kΩ Test Networks"]
            IRSensor["38kHz IR Demodulator"]
        end

        DisplayCtrl --> LCD["2.4' 320x240 Color TFT (ST7789)"]
        ScopeADC --> MCX["MCX Probe Input"]
        ResistorNet --> ZIF["1-2-3 Test Socket & SMD Pads"]
        
        MCU1 <== "UART Link (PA9 / PA10)" ==> MCU2
    end
```

---

## 2. Dual-Microcontroller Architecture

Teardowns and reverse-engineering reveal that FNIRSI splits the workload between two distinct ARM Cortex-M3 microcontrollers:

### Microcontroller 1: MindMotion MM32F103RET6 (LQFP64)
- **Role**: Component & Transistor Tester Subsystem
- **Core**: ARM Cortex-M3 @ 72–96 MHz, 512 KB Flash, 64 KB SRAM
- **Functions**:
  - Ports of Karl-Heinz Kübbeler & Markus Frejek's open-source *TransistorTester* firmware.
  - Direct connection to the 1-2-3 ZIF socket via precision 680 $\Omega$ and 470 $k\Omega$ resistor networks and Runic RS2227 analog switches.
  - Controls the 30V step-up boost converter for Zener diode breakdown testing.
  - Decodes 38 kHz NEC infrared remote signals.

### Microcontroller 2: WCH CH32F103 / STM32F103C6 (LQFP48) — **Active Target of this Custom ROM**
- **Role**: Display Controller, Oscilloscope Front-End & Host Interface
- **Core**: ARM Cortex-M3 @ 72 MHz, 32 KB Flash, 10 KB SRAM
- **Functions**:
  - Directly drives the 2.4-inch 320×240 TFT LCD via an ultra-fast **16-bit 8080 parallel bus**.
  - Acquires analog oscilloscope samples via ADC channel 0 (`PA0`).
  - Manages the USB-C interface (`PA11` / `PA12`).
  - Communicates with MCU 1 over USART1 (`PA9` TX, `PA10` RX).

---

## 3. Display Subsystem (ST7789 2.4" 320×240 TFT)

### 3.1 Bus Interface: 16-bit 8080-II Parallel
The display uses a 16-bit parallel bus for maximum frame rates. Data lines `D0` to `D15` are mapped directly to `PB0` through `PB15` of the MCU. Because `D0` is wired in parallel to both `PA7` and `PB0`, firmware drives both pins synchronously:

| TFT Signal | MCU Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **`D0`** | **`PB0` & `PA7`** | Data Bit 0 | Driven synchronously on both ports |
| **`D1`** | **`PB1`** | Data Bit 1 | Direct `GPIOB->ODR` register write |
| **`D2`** | **`PB2`** | Data Bit 2 | Direct `GPIOB->ODR` register write |
| **`D3`** | **`PB3`** | Data Bit 3 | Requires JTAG release (`AFIO_MAPR_SWJ_CFG_JTAGDISABLE`) |
| **`D4`** | **`PB4`** | Data Bit 4 | Requires JTAG release (`AFIO_MAPR_SWJ_CFG_JTAGDISABLE`) |
| **`D5` – `D15`** | **`PB5` – `PB15`** | Data Bits 5–15 | Direct `GPIOB->ODR` register write |
| **`DCX` (RS)** | **`PC13`** | Data / Command Select | LOW = Command, HIGH = Parameter/Pixel Data |
| **`WRX`** | **`PC14`** | Write Strobe Clock | Active LOW; data latched on rising edge |
| **`RDX`** | **`PC15`** | Read Strobe Clock | Held permanently HIGH in output mode |
| **`CSX`** | **`PA15`** | Chip Select | Active LOW |
| **`RESX`** | **`PA8`** | Hardware Reset | Active LOW pulse on initialization |

### 3.2 Critical Display Finding: Normally-White (TN) Panel
The LCD panel on the DSO-TC2 is a **Normally White (TN)** panel, not a Normally Black (IPS) panel:
- **`0x20` (INVOFF)**: Must be used. `0x0000` renders deep pitch black, and `0xFFFF` renders bright white.
- If `0x21` (INVON) is sent, the entire display inverts (black becomes bright white, and text/waveform colors turn inverted blue).

### 3.3 High-Speed Performance Optimization
By utilizing cycle-exact inline assembly write strobes on `PC14` and differential bounding-span column rendering, the custom ROM achieves **186 Frames Per Second (FPS)**.

---

## 4. Power & Programming Notes

### 4.1 Missing SWD Hardware Reset Pin
The SWD header on the DSO-TC2 PCB exposes only 4 pins:
- **`VCC`** (3.3V)
- **`SWDIO`** (`PA13`)
- **`SWCLK`** (`PA14`)
- **`GND`**

Because the hardware **`NRST` pin is NOT connected to the programming header**, standard J-Link or ST-Link hardware resets will fail or timeout.
The firmware and flashing tools must use **`AIRCR.VECTRESET`** (`RSetType 1` in SEGGER J-Link) to perform clean software resets after uploading binaries.
