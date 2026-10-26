/**
 * @file main_tester.cpp
 * @brief FNIRSI DSO-TC2 Secondary MCU (STM32L083RZT6)
 * - Differential Keypad Pin Detector with baseline auto-calibration
 * - Reads physical UP, DOWN, LEFT, RIGHT, OK buttons
 * - Measures exact VCC via calibrated VREFINT
 * - Inter-MCU Telemetry at 38400 baud over USART1 (PA9 TX) & USART2 (PA2 TX)
 */

#include <Arduino.h>

#if defined(USART1)
HardwareSerial SerialUART1(USART1);
#endif

// All candidate pins to monitor for front-panel buttons
static const uint32_t MONITORED_PINS[] = {
    // Port A
    PA0, PA1, PA4, PA5, PA6, PA7, PA8, PA11, PA12, PA15,
    // Port B
    PB0, PB1, PB2, PB3, PB4, PB5, PB7, PB8, PB9, PB12, PB13, PB14, PB15,
    // Port C
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13,
    // Port D
    PD2
};
static const size_t NUM_MONITORED = sizeof(MONITORED_PINS) / sizeof(MONITORED_PINS[0]);

// Baseline idle state (1 = idling HIGH with pull-up, 0 = tied to GND on PCB)
static uint8_t idle_state[NUM_MONITORED];

// VREFINT calibration value
#define VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FF80078))
#define VREFINT_CAL_VREF 3000

static uint32_t packet_seq = 0;
static char curr_pressed_pin[16] = "NONE";
static char last_pressed_pin[16] = "NONE";

static uint32_t read_vcc_mv() {
    analogReadResolution(12);
    uint32_t raw_vref = analogRead(AVREF);
    if (raw_vref == 0) return 3300;
    uint32_t cal = *VREFINT_CAL_ADDR;
    if (cal == 0 || cal == 0xFFFF) cal = 1662;
    return (VREFINT_CAL_VREF * cal) / raw_vref;
}

void setup() {
    // 1. Initialize candidate hardware UARTs at 38400 baud
    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(38400);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    // 2. Configure all monitored pins as INPUT_PULLUP
    for (size_t i = 0; i < NUM_MONITORED; i++) {
        pinMode(MONITORED_PINS[i], INPUT_PULLUP);
    }

    // 3. Auto-calibrate baseline idle state after pull-ups settle
    delay(50);
    for (size_t i = 0; i < NUM_MONITORED; i++) {
        idle_state[i] = digitalRead(MONITORED_PINS[i]);
    }
}

void loop() {
    // Differential Scan: Only detect pins that were IDLE HIGH and are now PULLED LOW
    uint32_t active_count = 0;
    curr_pressed_pin[0] = '\0';

    for (size_t i = 0; i < NUM_MONITORED; i++) {
        // If pin normally idles HIGH, and is currently LOW -> BUTTON PRESSED!
        if (idle_state[i] == HIGH && digitalRead(MONITORED_PINS[i]) == LOW) {
            uint32_t p = MONITORED_PINS[i];
            active_count++;
            int port = (p >> 4) & 0xF;
            int pin_num = p & 0xF;
            char port_char = (port == 0) ? 'A' : (port == 1) ? 'B' : (port == 2) ? 'C' : 'D';
            
            char pin_str[16];
            snprintf(pin_str, sizeof(pin_str), "P%c%d", port_char, pin_num);

            if (curr_pressed_pin[0] == '\0') {
                strncpy(curr_pressed_pin, pin_str, sizeof(curr_pressed_pin) - 1);
            }
            strncpy(last_pressed_pin, pin_str, sizeof(last_pressed_pin) - 1);
        }
    }

    if (active_count == 0) {
        strncpy(curr_pressed_pin, "NONE", sizeof(curr_pressed_pin) - 1);
    }

    packet_seq++;
    uint16_t reg_a = (uint16_t)GPIOA->IDR;
    uint16_t reg_b = (uint16_t)GPIOB->IDR;
    uint16_t reg_c = (uint16_t)GPIOC->IDR;
    uint32_t vcc_mv = read_vcc_mv();

    // Format: #TC2:<seq>,<curr_pressed>,<last_pressed>,<reg_a>,<reg_b>,<reg_c>,<vcc_mv>,<uptime_ms>*\n
    char buf[128];
    snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%s,%04X,%04X,%04X,%lu,%lu*\n",
             packet_seq,
             curr_pressed_pin,
             last_pressed_pin,
             reg_a, reg_b, reg_c,
             vcc_mv, millis());

#if defined(USART1)
    SerialUART1.print(buf);
#endif
    Serial.print(buf);

    delay(40); // 25 Hz telemetry rate
}
