/**
 * @file main_tester.cpp
 * @brief FNIRSI DSO-TC2 Secondary MCU Firmware (STM32L083RZT6)
 * Pin Discovery & Inter-MCU UART Transmitter at 38400 baud
 */

#include <Arduino.h>

#if defined(USART1)
HardwareSerial SerialUART1(USART1);
#endif

// Monitored button pins with internal pull-up
static const uint32_t MONITORED_PINS[] = {
    // Port A
    PA0, PA1, PA4, PA5, PA6, PA7, PA8, PA11, PA12, PA15,
    // Port B
    PB0, PB1, PB2, PB3, PB4, PB5, PB7, PB8, PB9, PB12, PB13, PB14, PB15,
    // Port C
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC11, PC12, PC13,
    // Port D
    PD2
};
static const size_t NUM_MONITORED = sizeof(MONITORED_PINS) / sizeof(MONITORED_PINS[0]);

// VREFINT calibration value
#define VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FF80078))
#define VREFINT_CAL_VREF 3000

static uint32_t packet_seq = 0;
static char last_active_pin[16] = "IDLE";

static uint32_t read_vcc_mv() {
    analogReadResolution(12);
    uint32_t raw_vref = analogRead(AVREF);
    if (raw_vref == 0) return 3300;
    uint32_t cal = *VREFINT_CAL_ADDR;
    if (cal == 0 || cal == 0xFFFF) cal = 1662;
    return (VREFINT_CAL_VREF * cal) / raw_vref;
}

void setup() {
    // Initialize candidate hardware UARTs at 38400 baud
    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(38400);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    // Candidate bitbang pins (PB10, PC10, PB6) set as inputs for now to avoid bus contention
    pinMode(PB6, INPUT_PULLUP);
    pinMode(PB10, INPUT_PULLUP);
    pinMode(PC10, INPUT_PULLUP);

    // Configure all monitored pins as INPUT_PULLUP
    for (size_t i = 0; i < NUM_MONITORED; i++) {
        pinMode(MONITORED_PINS[i], INPUT_PULLUP);
    }
}

void loop() {
    // Scan all monitored pins for button presses (pulled LOW)
    uint32_t active_count = 0;
    for (size_t i = 0; i < NUM_MONITORED; i++) {
        if (digitalRead(MONITORED_PINS[i]) == LOW) {
            uint32_t p = MONITORED_PINS[i];
            active_count++;
            int port = (p >> 4) & 0xF;
            int pin_num = p & 0xF;
            char port_char = (port == 0) ? 'A' : (port == 1) ? 'B' : (port == 2) ? 'C' : 'D';
            snprintf(last_active_pin, sizeof(last_active_pin), "P%c%d", port_char, pin_num);
        }
    }

    packet_seq++;
    uint16_t reg_a = (uint16_t)GPIOA->IDR;
    uint16_t reg_b = (uint16_t)GPIOB->IDR;
    uint16_t reg_c = (uint16_t)GPIOC->IDR;
    uint32_t vcc_mv = read_vcc_mv();

    char buf[128];
    // Format: #TC2:<seq>,<pin_name>,<reg_a>,<reg_b>,<reg_c>,<vcc_mv>,<uptime_ms>*\n
    snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%04X,%04X,%04X,%lu,%lu*\n",
             packet_seq,
             (active_count > 0) ? last_active_pin : "IDLE",
             reg_a, reg_b, reg_c,
             vcc_mv, millis());

    // Send on PA9 (USART1)
#if defined(USART1)
    SerialUART1.print(buf);
#endif

    // Send on PA2 (USART2)
    Serial.print(buf);

    delay(50); // 20 Hz
}
