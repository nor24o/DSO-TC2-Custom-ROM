# STM32F103C6 / CH32F103 (LQFP48) Complete Pinout Reference

This document details the exact assignment of all 48 pins of the main microcontroller on the FNIRSI DSO-TC2.

## Pinout Map

| Pin # | Pin Name | Default Function | DSO-TC2 Hardware Connection | Status in Custom ROM |
| :--- | :--- | :--- | :--- | :--- |
| **1** | `VBAT` | Backup Power | Connected to `VDD` / 3.3V power rail | Power |
| **2** | `PC13` | Tamper / RTC / GPIO | LCD `DCX` (Data / Command Select) | **Active (TFT)** |
| **3** | `PC14` | OSC32_IN / GPIO | LCD `WRX` (Write Strobe Clock) | **Active (TFT)** |
| **4** | `PC15` | OSC32_OUT / GPIO | LCD `RDX` (Read Strobe Clock, tied High) | **Active (TFT)** |
| **5** | `PD0` | OSC_IN | 8 MHz Crystal Oscillator Input | **Active (Clock)** |
| **6** | `PD1` | OSC_OUT | 8 MHz Crystal Oscillator Output | **Active (Clock)** |
| **7** | `NRST` | Master Reset | Onboard RC filter (not on SWD header) | Reset |
| **8** | `VSSA` | Analog Ground | System Ground | Ground |
| **9** | `VDDA` | Analog Power | 3.3V Filtered Analog Rail | Power |
| **10** | `PA0` | ADC12_IN0 | Oscilloscope MCX Analog Input Buffer | Available / Monitored |
| **11** | `PA1` | ADC12_IN1 / TIM2_CH2 | Signal Generator / PWM Output | Available / Monitored |
| **12** | `PA2` | ADC12_IN2 / TIM2_CH3 | Battery Voltage Divider Monitor | Available / Monitored |
| **13** | `PA3` | ADC12_IN3 / TIM2_CH4 | Front Keypad Resistor Ladder Sense | Available / Monitored |
| **14** | `PA4` | ADC12_IN4 / SPI1_NSS | Test Socket Pin 1 / Analog Mux Tap | Available / Monitored |
| **15** | `PA5` | ADC12_IN5 / SPI1_SCK | Test Socket Pin 2 / Analog Mux Tap | Available / Monitored |
| **16** | `PA6` | ADC12_IN6 / SPI1_MISO | Test Socket Pin 3 / Analog Mux Tap | Available / Monitored |
| **17** | `PA7` | ADC12_IN7 / SPI1_MOSI | LCD Data Bit 0 (`D0` parallel line) | **Active (TFT)** |
| **18** | `PB0` | ADC12_IN8 / TIM3_CH3 | LCD Data Bit 0 (`D0`) | **Active (TFT)** |
| **19** | `PB1` | ADC12_IN9 / TIM3_CH4 | LCD Data Bit 1 (`D1`) | **Active (TFT)** |
| **20** | `PB2` | BOOT1 / GPIO | LCD Data Bit 2 (`D2`) | **Active (TFT)** |
| **21** | `PB10` | I2C2_SCL / USART3_TX | LCD Data Bit 10 (`D10`) | **Active (TFT)** |
| **22** | `PB11` | I2C2_SDA / USART3_RX | LCD Data Bit 11 (`D11`) | **Active (TFT)** |
| **23** | `VSS_1` | Digital Ground | System Ground | Ground |
| **24** | `VDD_1` | Digital Power | 3.3V Digital Rail | Power |
| **25** | `PB12` | SPI2_NSS / TIM1_BKIN | LCD Data Bit 12 (`D12`) | **Active (TFT)** |
| **26** | `PB13` | SPI2_SCK / TIM1_CH1N | LCD Data Bit 13 (`D13`) | **Active (TFT)** |
| **27** | `PB14` | SPI2_MISO / TIM1_CH2N | LCD Data Bit 14 (`D14`) | **Active (TFT)** |
| **28** | `PB15` | SPI2_MOSI / TIM1_CH3N | LCD Data Bit 15 (`D15`) | **Active (TFT)** |
| **29** | `PA8` | TIM1_CH1 / MCO | LCD `RESX` (Display Hardware Reset) | **Active (TFT)** |
| **30** | `PA9` | USART1_TX / TIM1_CH2 | Inter-MCU UART TX (to MM32 Tester MCU) | Available / Monitored |
| **31** | `PA10` | USART1_RX / TIM1_CH3 | Inter-MCU UART RX (from MM32 Tester MCU)| Available / Monitored |
| **32** | `PA11` | USB_DM / CAN_RX | USB-C D- line (Factory USB Bootloader) | Available / Monitored |
| **33** | `PA12` | USB_DP / CAN_TX | USB-C D+ line (Factory USB Bootloader) | Available / Monitored |
| **34** | `PA13` | JTMS / SWDIO | SWD Data (J-Link Debugger) | **Active (SWD)** |
| **35** | `VSS_2` | Digital Ground | System Ground | Ground |
| **36** | `VDD_2` | Digital Power | 3.3V Digital Rail | Power |
| **37** | `PA14` | JTCK / SWCLK | SWD Clock (J-Link Debugger) | **Active (SWD)** |
| **38** | `PA15` | JTDI / TIM2_CH1 | LCD `CSX` (Display Chip Select) | **Active (TFT)** |
| **39** | `PB3` | JTDO / TIM2_CH2 | LCD Data Bit 3 (`D3` - JTAG released) | **Active (TFT)** |
| **40** | `PB4` | NJTRST / TIM3_CH1 | LCD Data Bit 4 (`D4` - JTAG released) | **Active (TFT)** |
| **41** | `PB5` | I2C1_SMBA / TIM3_CH2 | LCD Data Bit 5 (`D5`) | **Active (TFT)** |
| **42** | `PB6` | I2C1_SCL / TIM4_CH1 | LCD Data Bit 6 (`D6`) | **Active (TFT)** |
| **43** | `PB7` | I2C1_SDA / TIM4_CH2 | LCD Data Bit 7 (`D7`) | **Active (TFT)** |
| **44** | `PB8` | TIM4_CH3 / CAN_RX | LCD Data Bit 8 (`D8`) | **Active (TFT)** |
| **45** | `PB9` | TIM4_CH4 / CAN_TX | LCD Data Bit 9 (`D9`) | **Active (TFT)** |
| **46** | `BOOT0` | Boot Config | Pulled to Ground via 10k resistor | Boot |
| **47** | `VSS_3` | Digital Ground | System Ground | Ground |
| **48** | `VDD_3` | Digital Power | 3.3V Digital Rail | Power |

---

## Summary Statistics

- **Total Pins**: 48
- **Power & System Pins**: 11 (`VDD`, `VSS`, `VBAT`, `NRST`, `BOOT0`, `PD0`, `PD1`)
- **Total GPIOs**: 37
- **GPIOs Used by Display**: 22 (`PB0..PB15`, `PA7`, `PA8`, `PA15`, `PC13`, `PC14`, `PC15`)
- **GPIOs Used by SWD Debug**: 2 (`PA13`, `PA14`)
- **Free / Unused GPIOs Available for Instruments**: **11** (`PA0`–`PA6`, `PA9`–`PA12`)
