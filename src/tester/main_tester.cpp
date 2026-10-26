/**
 * @file main_tester.cpp
 * @brief FNIRSI DSO-TC2 Secondary MCU (STM32L083RZT6)
 * - Front Keypad Precision Pin Decoder
 * - Confirmed buttons: RIGHT=PA8, LEFT=PB14, MENU=PB15
 * - Auto-detects UP, DOWN, OK across PB12, PB13, Port C, Port A
 * - Interactive Relay Seeker on command
 */

#include <Arduino.h>

#if defined(USART1)
HardwareSerial SerialUART1(USART1);
#endif

// All candidate pins to monitor
static const uint32_t ALL_PINS[] = {
    // Port A
    PA0, PA1, PA4, PA5, PA6, PA7, PA8, PA11, PA12, PA15,
    // Port B
    PB0, PB1, PB2, PB3, PB4, PB5, PB7, PB8, PB9, PB12, PB13, PB14, PB15,
    // Port C
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13,
    // Port D
    PD2
};
static const size_t NUM_PINS = sizeof(ALL_PINS) / sizeof(ALL_PINS[0]);

// Baseline state
static uint8_t base_state[NUM_PINS];

// Relay Seeker candidate pin index (-1 = idle)
static int relay_seek_pin = -1;
static uint32_t relay_pulse_end = 0;

#define VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FF80078))
#define VREFINT_CAL_VREF 3000

static uint32_t read_vcc_mv() {
    analogReadResolution(12);
    uint32_t raw_vref = analogRead(AVREF);
    if (raw_vref == 0) return 3300;
    uint32_t cal = *VREFINT_CAL_ADDR;
    if (cal == 0 || cal == 0xFFFF) cal = 1662;
    return (VREFINT_CAL_VREF * cal) / raw_vref;
}

void setup() {
    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(38400);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    // Set all monitored pins as INPUT_PULLUP
    for (size_t i = 0; i < NUM_PINS; i++) {
        pinMode(ALL_PINS[i], INPUT_PULLUP);
    }
    delay(50);

    for (size_t i = 0; i < NUM_PINS; i++) {
        base_state[i] = digitalRead(ALL_PINS[i]);
    }
}

void loop() {
    // 1. Process Relay Pulse if active
    if (relay_seek_pin >= 0 && millis() >= relay_pulse_end) {
        digitalWrite(relay_seek_pin, LOW);
        pinMode(relay_seek_pin, INPUT_PULLUP); // restore to input
        relay_seek_pin = -1;
    }

    // 2. Scan for active button
    char active_btn[24] = "NONE";
    char active_pin_name[16] = "NONE";

    // Read registers directly for fastest detection
    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    // Check confirmed buttons:
    // RIGHT: PA8 (bit 8 of GPIOA)
    if (!(ra & (1U << 8))) {
        strcpy(active_btn, "RIGHT");
        strcpy(active_pin_name, "PA8");
    }
    // LEFT: PB14 (bit 14 of GPIOB)
    else if (!(rb & (1U << 14))) {
        strcpy(active_btn, "LEFT");
        strcpy(active_pin_name, "PB14");
    }
    // MENU: PB15 (bit 15 of GPIOB)
    else if (!(rb & (1U << 15))) {
        strcpy(active_btn, "MENU");
        strcpy(active_pin_name, "PB15");
    }
    // Check PB13
    else if (!(rb & (1U << 13))) {
        strcpy(active_btn, "PB13");
        strcpy(active_pin_name, "PB13");
    }
    // Check PB12
    else if (!(rb & (1U << 12))) {
        strcpy(active_btn, "PB12");
        strcpy(active_pin_name, "PB12");
    }
    else {
        // Scan other pins for any transition from base_state
        for (size_t i = 0; i < NUM_PINS; i++) {
            if (base_state[i] == HIGH && digitalRead(ALL_PINS[i]) == LOW) {
                uint32_t p = ALL_PINS[i];
                int port = (p >> 4) & 0xF;
                int pin_num = p & 0xF;
                char port_char = (port == 0) ? 'A' : (port == 1) ? 'B' : (port == 2) ? 'C' : 'D';
                snprintf(active_pin_name, sizeof(active_pin_name), "P%c%d", port_char, pin_num);
                strncpy(active_btn, active_pin_name, sizeof(active_btn) - 1);
                break;
            }
        }
    }

    // 3. Telemetry broadcast at 25 Hz
    static uint32_t last_tx = 0;
    static uint32_t seq = 0;

    if (millis() - last_tx >= 40) {
        last_tx = millis();
        seq++;
        uint32_t vcc = read_vcc_mv();

        // Format: #TC2:<seq>,<btn_name>,<pin_name>,<ra>,<rb>,<rc>,<vcc>,<uptime>*\n
        char buf[128];
        snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%s,%04X,%04X,%04X,%lu,%lu*\n",
                 seq, active_btn, active_pin_name, ra, rb, rc, vcc, millis());

#if defined(USART1)
        SerialUART1.print(buf);
#endif
        Serial.print(buf);
    }
}
