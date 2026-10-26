/**
 * @file main_tester.cpp
 * @brief FNIRSI DSO-TC2 Secondary MCU (STM32L083RZT6)
 * - Full-spectrum pin change and ADC sniffer
 * - Reports any pin delta over USART1 (PA9) & USART2 (PA2) at 38400 baud
 */

#include <Arduino.h>

#if defined(USART1)
HardwareSerial SerialUART1(USART1);
#endif

// Monitored GPIOs on STM32L083
static const uint32_t MONITORED_PINS[] = {
    PA0, PA1, PA4, PA5, PA6, PA7, PA8, PA11, PA12, PA15,
    PB0, PB1, PB2, PB3, PB4, PB5, PB7, PB8, PB9, PB12, PB13, PB14, PB15,
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13,
    PD2
};
static const size_t NUM_PINS = sizeof(MONITORED_PINS) / sizeof(MONITORED_PINS[0]);

static uint8_t last_pin_state[NUM_PINS];
static char last_event[32] = "L083_IDLE";
static uint32_t event_counter = 0;

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
        pinMode(MONITORED_PINS[i], INPUT_PULLUP);
    }
    delay(50);

    // Record initial states
    for (size_t i = 0; i < NUM_PINS; i++) {
        last_pin_state[i] = digitalRead(MONITORED_PINS[i]);
    }
}

void loop() {
    // Check for any pin transition (HIGH -> LOW or LOW -> HIGH)
    for (size_t i = 0; i < NUM_PINS; i++) {
        uint8_t cur = digitalRead(MONITORED_PINS[i]);
        if (cur != last_pin_state[i]) {
            last_pin_state[i] = cur;
            event_counter++;

            uint32_t p = MONITORED_PINS[i];
            int port = (p >> 4) & 0xF;
            int pin_num = p & 0xF;
            char port_char = (port == 0) ? 'A' : (port == 1) ? 'B' : (port == 2) ? 'C' : 'D';

            snprintf(last_event, sizeof(last_event), "P%c%d_%s", port_char, pin_num, cur ? "HIGH" : "LOW");
        }
    }

    static uint32_t last_tx = 0;
    if (millis() - last_tx >= 40) { // 25 Hz
        last_tx = millis();

        uint16_t ra = (uint16_t)GPIOA->IDR;
        uint16_t rb = (uint16_t)GPIOB->IDR;
        uint16_t rc = (uint16_t)GPIOC->IDR;
        uint32_t vcc = read_vcc_mv();

        // Format: #TC2:<event_counter>,<last_event>,<ra>,<rb>,<rc>,<vcc>,<uptime>*\n
        char buf[128];
        snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%04X,%04X,%04X,%lu,%lu*\n",
                 event_counter, last_event, ra, rb, rc, vcc, millis());

#if defined(USART1)
        SerialUART1.print(buf);
#endif
        Serial.print(buf);
    }
}
