# FNIRSI DSO-TC2 Complete Dual-MCU Pinout & Hardware Schematic Reference

This document provides the definitive, hardware-verified pinout and schematic architecture for both microcontrollers on the FNIRSI DSO-TC2 Custom ROM:
1. **Tester / System MCU**: `STM32L083RZT6` (ARM Cortex-M0+ @ 32 MHz, LQFP-64) — replaces damaged MindMotion MM32.
2. **Display / Host MCU**: `STM32F103C6` (ARM Cortex-M3 @ 72 MHz, LQFP-48) — drives the 16-bit parallel TFT.

---

## 1. System Interconnection Block Diagram

```mermaid
graph TD
    subgraph PowerSystem ["Power & Peripheral Subsystem"]
        Batt["3.7V 1500mAh LiPo"] --> PwrReg["3.3V Step-Down DC-DC"]
        PwrReg --> MCU_L083
        PwrReg --> MCU_F103
        PwrReg --> Backlight["2.4' LCD Backlight LED"]
        PwrReg --> Buzzer["Piezo Sound Generator"]
        PwrReg --> Relay["Attenuator Switch Relay"]
    end

    subgraph TesterMCU ["MCU 1: STM32L083RZT6 (LQFP-64) - System & Measurement Engine"]
        L083_PwrLatch["PB7: Power Latch (Active HIGH)"]
        L083_BL["PC7: Backlight PWM"]
        L083_Buzz["PC6: Audio Piezo"]
        L083_Relay["PB6: Oscilloscope Relay"]
        L083_IR["PB9: 38kHz IR Demodulator EXTI"]
        
        L083_Keys["Buttons (Active LOW):
        PB13: POWER | PB14: OK (<)
        PB15: MENU  | PC8:  UP (^)
        PC9:  DOWN  | PA8:  TEST (>)"]
        
        L083_Tester["1-2-3 Component Tester Matrix:
        Sense: PA3 (S1), PA4 (S2), PA5 (S3)
        680Ω RL: PA7 (RL1), PB1 (RL2), PB10 (RL3)
        470kΩ RH: PB0 (RH1), PB2 (RH2), PB11 (RH3)"]
        
        L083_TX["PA9: USART1_TX (115200)
        PA2: USART2_TX (115200)"]
    end

    subgraph DisplayMCU ["MCU 2: STM32F103C6 (LQFP-48) - 186 FPS Graphics Host"]
        F103_RX["PA10: USART1_RX (115200)"]
        F103_ADC["PA0: Scope ADC Input"]
        
        F103_LCD_Ctrl["LCD Control:
        PC13: DCX (RS)
        PC14: WRX (Write Strobe)
        PC15: RDX (Tied HIGH)
        PA15: CSX (Chip Select)
        PA8:  RESX (Reset)"]
        
        F103_LCD_Data["16-Bit Parallel Data Bus:
        PB0..PB15 (D0..D15) + PA7 (D0 sync)"]
    end

    subgraph Peripherals ["External Hardware & Sockets"]
        ZIF["1-2-3 ZIF Socket & SMD Pads"]
        LCD["2.4' 320x240 ST7789 Color TFT"]
        IR_Sensor["38kHz Infrared Receiver Diode"]
        BNC["MCX Oscilloscope Probe Jack"]
    end

    L083_PwrLatch -.->|Keep Alive| PwrReg
    L083_BL --> Backlight
    L083_Buzz --> Buzzer
    L083_Relay --> Relay
    IR_Sensor --> L083_IR
    
    L083_Tester <==>|RL / RH / Sense| ZIF
    L083_TX ==>|"Telemetry Link (38,400 baud)"| F103_RX
    
    F103_LCD_Ctrl --> LCD
    F103_LCD_Data ==> LCD
    BNC --> F103_ADC
```

---

## 2. Tester MCU: STM32L083RZT6 (LQFP-64) Pinout

### 2.1 Complete Hardware Pinout Table

| LQFP-64 Pin # | Pin Name | Peripheral | Hardware Net / Target | Active Status | Description |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **14** | `PA0` | `ADC_IN0` | MT3608 Zener Divider Tap| **ACTIVE** | High-voltage breakdown voltage sense (0-30V DC via resistor divider) |
| **16** | `PA2` | `ADC_IN2` | Li-Po Battery 2:1 Divider | **ACTIVE** | Li-Po battery voltage monitor (samples ~2.15V at 4.3V battery rail) |
| **17** | `PA3` | `ADC_IN3` | ZIF Socket Pin 1 Sense | **ACTIVE** | Direct 12-bit analog voltage measurement tap for Socket 1 |
| **20** | `PA4` | `ADC_IN4` | ZIF Socket Pin 2 Sense | **ACTIVE** | Direct 12-bit analog voltage measurement tap for Socket 2 |
| **21** | `PA5` | `ADC_IN5` | ZIF Socket Pin 3 Sense | **ACTIVE** | Direct 12-bit analog voltage measurement tap for Socket 3 |
| **22** | `PA6` | `ADC_IN6` | External SMA Connector | **ACTIVE** | External DC Voltmeter tap (0 - 16.0V DC via 127k / 27k divider) |
| **23** | `PA7` | `GPIO / RL1`| Socket Pin 1 $680\,\Omega$ | **ACTIVE** | Low-range probe resistor ($R_{L1}$) for Karl-Heinz tester matrix |
| **24** | `PC4` | `GPIO` | MT3608 Boost Enable | **ACTIVE** | Powers on the Aerosemi MT3608 28V step-up boost converter for Zener tests |
| **26** | `PB0` | `GPIO / RH1`| Socket Pin 1 $470\,\text{k}\Omega$ | **ACTIVE** | High-range probe resistor ($R_{H1}$) for Karl-Heinz tester matrix |
| **27** | `PB1` | `GPIO / RL2`| Socket Pin 2 $680\,\Omega$ | **ACTIVE** | Low-range probe resistor ($R_{L2}$) for Karl-Heinz tester matrix |
| **28** | `PB2` | `GPIO / RH2`| Socket Pin 2 $470\,\text{k}\Omega$ | **ACTIVE** | High-range probe resistor ($R_{H2}$) for Karl-Heinz tester matrix |
| **29** | `PB10`| `GPIO / RL3`| Socket Pin 3 $680\,\Omega$ | **ACTIVE** | Low-range probe resistor ($R_{L3}$) for Karl-Heinz tester matrix |
| **30** | `PB11`| `GPIO / RH3`| Socket Pin 3 $470\,\text{k}\Omega$ | **ACTIVE** | High-range probe resistor ($R_{H3}$) for Karl-Heinz tester matrix |
| **34** | `PB13`| `GPIO / EXTI`| Button: **POWER** | **ACTIVE** | Active LOW, internal pull-up. Hold >1.5s for soft power shutdown |
| **35** | `PB14`| `GPIO / EXTI`| Button: **OK** / `<` | **ACTIVE** | Active LOW. Left navigation & SigGen waveform mode / duty cycle selector |
| **36** | `PB15`| `GPIO / EXTI`| Button: **MENU** / `AUTO` | **ACTIVE** | Active LOW. Instrument screen cycling (Tester -> IR -> Zener -> SigGen) |
| **37** | `PC6` | `GPIO / PWM`| Buzzer / Piezo Audio | **ACTIVE** | Acoustic continuity beeper, melodic test chimes, and tactile key clicks |
| **38** | `PC7` | `GPIO / PWM`| LCD Backlight LED | **ACTIVE** | Backlight brightness control (Active HIGH / PWM dimming) |
| **39** | `PC8` | `GPIO / EXTI`| Button: **UP** / `^` | **ACTIVE** | Active LOW. Value increment / SigGen frequency step up |
| **40** | `PC9` | `GPIO / EXTI`| Button: **DOWN** / `v` | **ACTIVE** | Active LOW. Value decrement / SigGen frequency step down |
| **41** | `PA8` | `GPIO / EXTI`| Button: **TEST** / `HOLD` / `>`| **ACTIVE** | Active LOW. Triggers component test / Zener sample / SigGen output toggle |
| **42** | `PA9` | `USART1_TX`| Inter-MCU Primary TX | **ACTIVE** | Transmits telemetry packets (`#TC2:`) to Display MCU @ 38,400 baud |
| **43** | `PA10`| `USART1_RX`| Inter-MCU Primary RX | **ACTIVE** | Receives commands from Display MCU @ 38,400 baud |
| **46** | `PA13`| `SWDIO` | SWD Debug Data | **ACTIVE** | Segger J-Link programming line |
| **49** | `PA14`| `SWCLK` | SWD Debug Clock | **ACTIVE** | Segger J-Link programming clock |
| **58** | `PB6` | `GPIO` | Oscilloscope Range Relay| **ACTIVE** | Solid-state relay for 1X / 10X input attenuation |
| **59** | `PB7` | `GPIO` | Power-Hold Soft Latch | **ACTIVE** | Driven HIGH on boot to maintain 3.3V DC-DC power regulator |
| **61** | `PB8` | `GPIO / TIM6`| Signal Generator Output | **ACTIVE** | Programmable PWM, Square, Pulse, and Ramp generator (10 Hz – 100 kHz) |
| **62** | `PB9` | `GPIO / EXTI`| 38 kHz IR Demodulator | **ACTIVE** | Captures NEC / Samsung / Sony infrared remote pulse-distance codes |

---

### 2.2 Component Tester Resistor Network Architecture

```
                  STM32L083RZ Pins
               ----------------------
               [ PA7 ] ----/\/\/\----+  (680 Ω RL1)
                            680 Ω    |
               [ PB0 ] ----/\/\/\----+  (470 kΩ RH1)
                            470 kΩ   |
                                     +-----> ZIF Pin 1 (Socket 1)
               [ PA3 ] --------------+  (Direct ADC Channel 3)
                                     
               ----------------------
               [ PB1 ] ----/\/\/\----+  (680 Ω RL2)
                            680 Ω    |
               [ PB2 ] ----/\/\/\----+  (470 kΩ RH2)
                            470 kΩ   |
                                     +-----> ZIF Pin 2 (Socket 2)
               [ PA4 ] --------------+  (Direct ADC Channel 4)
                                     
               ----------------------
               [ PB10] ----/\/\/\----+  (680 Ω RL3)
                            680 Ω    |
               [ PB11] ----/\/\/\----+  (470 kΩ RH3)
                            470 kΩ   |
                                     +-----> ZIF Pin 3 (Socket 3)
               [ PA5 ] --------------+  (Direct ADC Channel 5)
```

---

## 3. Display MCU: STM32F103C6 (LQFP-48) Pinout

### 3.1 Complete Pinout Table

| LQFP-48 Pin # | Pin Name | Default Function | Hardware Net / Connection | Mode / State | Description |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | `VBAT` | Backup Power | Tied to 3.3V digital rail | Power | System VDD |
| **2** | `PC13` | Tamper / RTC | **LCD `DCX` (RS)** | Output | Data / Command Select (LOW=Cmd, HIGH=Data) |
| **3** | `PC14` | OSC32_IN | **LCD `WRX`** | Output | Write Strobe Clock (Inline assembly cycle) |
| **4** | `PC15` | OSC32_OUT | **LCD `RDX`** | Output | Read Strobe (Tied permanently HIGH) |
| **5** | `PD0` | OSC_IN | 8 MHz Crystal Input | Clock | HSE Clock Generator |
| **6** | `PD1` | OSC_OUT | 8 MHz Crystal Output | Clock | HSE Clock Generator |
| **7** | `NRST` | Master Reset | Internal RC (unrouted to SWD) | System | Software reset via AIRCR.VECTRESET |
| **8** | `VSSA` | Analog Ground | System Ground | Power | Clean Analog Ground |
| **9** | `VDDA` | Analog Power | 3.3V Filtered Rail | Power | Clean Analog VDD |
| **10** | `PA0` | `ADC12_IN0` | Scope MCX Analog Front-End | Input | High-speed ADC sampling input |
| **11** | `PA1` | `ADC_IN1 / TIM2` | Signal Generator Output | Reserved | PWM / DAC Waveform Generator |
| **12** | `PA2` | `ADC12_IN2` | Unconnected / NC | NC | (Battery divider resides on Tester MCU PA2) |
| **17** | `PA7` | `GPIO` | **LCD `D0` Parallel Tap** | Output | Synchronously mirrored with `PB0` for bus speed |
| **18** | `PB0` | `GPIO` | **LCD `D0`** | Output | 16-Bit Parallel Data Bit 0 |
| **19** | `PB1` | `GPIO` | **LCD `D1`** | Output | 16-Bit Parallel Data Bit 1 |
| **20** | `PB2` | `BOOT1 / GPIO` | **LCD `D2`** | Output | 16-Bit Parallel Data Bit 2 |
| **21** | `PB10`| `GPIO` | **LCD `D10`** | Output | 16-Bit Parallel Data Bit 10 |
| **22** | `PB11`| `GPIO` | **LCD `D11`** | Output | 16-Bit Parallel Data Bit 11 |
| **25** | `PB12`| `GPIO` | **LCD `D12`** | Output | 16-Bit Parallel Data Bit 12 |
| **26** | `PB13`| `GPIO` | **LCD `D13`** | Output | 16-Bit Parallel Data Bit 13 |
| **27** | `PB14`| `GPIO` | **LCD `D14`** | Output | 16-Bit Parallel Data Bit 14 |
| **28** | `PB15`| `GPIO` | **LCD `D15`** | Output | 16-Bit Parallel Data Bit 15 |
| **29** | `PA8` | `GPIO` | **LCD `RESX`** | Output | Display Hardware Reset (Active LOW pulse) |
| **30** | `PA9` | `USART1_TX` | Inter-MCU Link TX | Output | Telemetry / Command TX @ 38,400 baud |
| **31** | `PA10`| `USART1_RX` | Inter-MCU Link RX | Input | Telemetry RX from L083 Tester @ 38,400 baud |
| **32** | `PA11`| `USB_DM` | USB-C D- Line | USB | Factory Bootloader / VCP |
| **33** | `PA12`| `USB_DP` | USB-C D+ Line | USB | Factory Bootloader / VCP |
| **34** | `PA13`| `JTMS / SWDIO` | DAPLink SWD Data | Debug | OpenOCD programming line |
| **37** | `PA14`| `JTCK / SWCLK` | DAPLink SWD Clock | Debug | OpenOCD programming clock |
| **38** | `PA15`| `GPIO` | **LCD `CSX`** | Output | Display Chip Select (Active LOW) |
| **39** | `PB3` | `GPIO` | **LCD `D3`** | Output | Data Bit 3 (JTAG released via AFIO) |
| **40** | `PB4` | `GPIO` | **LCD `D4`** | Output | Data Bit 4 (JTAG released via AFIO) |
| **41** | `PB5` | `GPIO` | **LCD `D5`** | Output | 16-Bit Parallel Data Bit 5 |
| **42** | `PB6` | `GPIO` | **LCD `D6`** | Output | 16-Bit Parallel Data Bit 6 |
| **43** | `PB7` | `GPIO` | **LCD `D7`** | Output | 16-Bit Parallel Data Bit 7 |
| **44** | `PB8` | `GPIO` | **LCD `D8`** | Output | 16-Bit Parallel Data Bit 8 |
| **45** | `PB9` | `GPIO` | **LCD `D9`** | Output | 16-Bit Parallel Data Bit 9 |
| **46** | `BOOT0`| `Boot Select` | Pulled to GND | Boot | Default flash boot |

---

### 3.2 Graphical 16-Bit Parallel Display Bus Architecture

```
                 STM32F103C6                          ST7789 2.4" TFT (320x240)
              +-----------------+                    +-------------------------+
              |            PA15 |------------------->| CSX   (Chip Select, Act-L)
              |            PC13 |------------------->| DCX   (Cmd: 0 / Data: 1)
              |            PC14 |------------------->| WRX   (Write Clock Strobe)
              |            PC15 |------------------->| RDX   (Read Tied High 3.3V)
              |             PA8 |------------------->| RESX  (Reset Pulse, Act-L)
              |                 |                    |                         |
              |        PB0, PA7 |===================>| D0    (Data Bit 0)      |
              |             PB1 |===================>| D1    (Data Bit 1)      |
              |             PB2 |===================>| D2    (Data Bit 2)      |
              |             PB3 |===================>| D3    (Data Bit 3)      |
              |             PB4 |===================>| D4    (Data Bit 4)      |
              |             PB5 |===================>| D5    (Data Bit 5)      |
              |             PB6 |===================>| D6    (Data Bit 6)      |
              |             PB7 |===================>| D7    (Data Bit 7)      |
              |             PB8 |===================>| D8    (Data Bit 8)      |
              |             PB9 |===================>| D9    (Data Bit 9)      |
              |            PB10 |===================>| D10   (Data Bit 10)     |
              |            PB11 |===================>| D11   (Data Bit 11)     |
              |            PB12 |===================>| D12   (Data Bit 12)     |
              |            PB13 |===================>| D13   (Data Bit 13)     |
              |            PB14 |===================>| D14   (Data Bit 14)     |
              |            PB15 |===================>| D15   (Data Bit 15)     |
              +-----------------+                    +-------------------------+
```

---

## 4. Keypad Pin Assignment & Multi-Function Navigation

The 6 front panel buttons connect directly to the **Tester MCU (`STM32L083RZ`)** with internal pull-ups enabled:

```
                  +-----------------------------------+
                  |        FNIRSI DSO-TC2 PANEL       |
                  |                                   |
                  |          [ UP: PC8 ]              |
                  |               ^                   |
                  |   [ < ]               [ > ]       |
                  | [OK: PB14]          [TEST: PA8]   |
                  |               v                   |
                  |         [ DOWN: PC9 ]             |
                  |                                   |
                  |    [POWER: PB13]  [MENU: PB15]    |
                  +-----------------------------------+
```

### Button Functions per Instrument Screen

| Button | Pin | Screen 0: Component Tester | Screen 1: IR Decoder | Screen 2: Zener (30V) | Screen 3: SigGen & Voltmeter |
| :--- | :---: | :--- | :--- | :--- | :--- |
| **`POWER`** | `PB13` | Click: Audio click<br>Hold >1.5s: Power Off | Same | Same | Same |
| **`MENU`**  | `PB15` | Cycle to Screen 1 | Cycle to Screen 2 | Cycle to Screen 3 | Cycle to Screen 0 |
| **`TEST`**  | `PA8`  | Run Component / 1-Wire Test | Clear Decoded History | Trigger Zener Sample | Toggle Output ON / OFF |
| **`OK`**    | `PB14` | Navigation confirm | Clear buffer | Standby | Cycle Waveform Mode (Square/Pulse/Ramp) |
| **`UP`**    | `PC8`  | Value increment | Scroll history | Value increment | Frequency step up (10Hz - 100kHz) |
| **`DOWN`**  | `PC9`  | Value decrement | Scroll history | Value decrement | Frequency step down (10Hz - 100kHz)|

### Global Hardware Modifier Shortcuts
* **`MENU` + `UP` / `DOWN`**: Increments / decrements LCD backlight brightness (`PC7` PWM, range 30..255).
* **`MENU` + `TEST`**: Manually toggles the solid-state oscilloscope input attenuator relay (`PB6`).

---

## 5. Front-Panel 14-Pin ZIF Socket & Terminal Mapping

The front multi-connector ZIF socket is divided into the **Component Tester (1-2-3)** matrix and the **High-Voltage Zener Diode (K-A-A)** socket:

```
          +---------------------------------------------------+
          |         14-PIN FRONT ZIF SOCKET PINOUT            |
          +---------------------------------------------------+
          |  Top Row:    [ 1 ] [ 2 ] [ 3 ] [ 1 ] [ 2 ] [ 3 ] [ 2 ]  |
          |  Bottom Row: [ K ] [ A ] [ A ] [ 1 ] [ 2 ] [ 3 ] [ 3 ]  |
          +---------------------------------------------------+
```

### Terminal Descriptions

| Socket Terminal | Net Connection | MCU Pin | Connected Test Circuit | Target Components |
| :---: | :---: | :---: | :--- | :--- |
| **`1`** | Socket Pin 1 | `PA3` (Sense)<br>`PA7` ($R_{L1}$)<br>`PB0` ($R_{H1}$) | $680\,\Omega$ & $470\,\text{k}\Omega$ probes to $V_{CC}$/GND | Resistors, Capacitors, Inductors, BJTs, MOSFETs, Diodes, DS18B20 GND/VDD/DQ |
| **`2`** | Socket Pin 2 | `PA4` (Sense)<br>`PB1` ($R_{L2}$)<br>`PB2` ($R_{H2}$) | $680\,\Omega$ & $470\,\text{k}\Omega$ probes to $V_{CC}$/GND | Same as above |
| **`3`** | Socket Pin 3 | `PA5` (Sense)<br>`PB10` ($R_{L3}$)<br>`PB11` ($R_{H3}$) | $680\,\Omega$ & $470\,\text{k}\Omega$ probes to $V_{CC}$/GND | Same as above |
| **`K`** | Zener Cathode | `PA0` (Sense) | MT3608 Boost rail tap through current limiter | High-voltage Zener cathode (bar end) |
| **`A`** | Zener Anode | `GND` | Ground return rail | High-voltage Zener anode (triangle end) |

*Note: In the 1-2-3 socket, all pins labeled `1` are internally bridged on the PCB, all pins labeled `2` are bridged, and all pins labeled `3` are bridged. You can plug components into any matching 1, 2, or 3 holes.*

---

## 6. Top-Edge Connector Pinout

The top edge of the FNIRSI DSO-TC2 features three physical ports:

```
          +-------------------------------------------------------------+
          |                      TOP EDGE OF CASING                     |
          +-------------------------------------------------------------+
          |   [ IN (SMA / MCX) ]       [ SIGGEN / PWM ]       [ USB-C ] |
          +-------------------------------------------------------------+
```

1. **`IN` (SMA / MCX Connector)**:
   - **Internal Connection**: Wired to **`PA6`** (`ADC_IN6`) on the Tester MCU through a precision $100\,\text{k}\Omega$ series and $27\,\text{k}\Omega$ shunt resistive divider ($\text{Ratio} = \frac{127}{27} \approx 4.704$).
   - **Voltage Measurement Range**: **$0.0\text{V}$ to $16.0\text{V}$ DC** unipolar.
   - **Scope Mode**: Routed through the solid-state input relay (`PB6`) to **`PA0`** on the Display MCU for high-speed oscilloscope sampling (up to 40V max with 10X attenuation).

2. **`SIGGEN / PWM` Output**:
   - **Internal Connection**: Driven directly by **`PB8`** on the Tester MCU (`STM32L083RZ`) via high-speed bare-metal `TIM6` interrupt.
   - **Frequency Range**: **$10\text{ Hz}$ to $100\text{ kHz}$**.
   - **Waveforms**: 50% Square Wave, 10%/25%/75%/90% Pulse, Triangle, Sine, and 80% Sawtooth / Ramp.
   - **Output Level**: Fixed $3.3\text{V}$ CMOS logic level ($0.0\text{V}$ to $3.3\text{V}_{pp}$).

3. **`USB-C` Port**:
   - **Pin 1 / 12**: `GND`
   - **Pin 4 / 9**: `VBUS` (5V charging input for onboard 1500 mAh Li-Po battery)
   - **Pin A6 / B6**: `USB_DP` (wired to Display MCU `PA12`)
   - **Pin A7 / B7**: `USB_DM` (wired to Display MCU `PA11`)

