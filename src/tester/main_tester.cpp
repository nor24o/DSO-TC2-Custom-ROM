/**
 * @file main_tester.cpp
 * @brief FNIRSI DSO-TC2 Secondary MCU (STM32L083RZT6)
 * - Rock-solid Keypad & Telemetry Engine (Brownout-Safe)
 * - Confirmed Buttons:
 *   * UP    = PC8  (Pin 39)
 *   * DOWN  = PC9  (Pin 40)
 *   * LEFT  = PB14 (Pin 35)
 *   * RIGHT = PA8  (Pin 41)
 *   * OK    = PB9  (Pin 62)
 *   * MENU  = PB15 (Pin 36)
 * - Safe GPIO Initialization (leaves PC6 as input to prevent power cutoff)
 * - Soft Backlight control on PC7
 */

#include <Arduino.h>
#include <RTTStream.h>

RTTStream rtt;

#if defined(USART1)
HardwareSerial SerialUART1(USART1);
#endif

// Pin Definitions
#define PIN_BTN_UP     PC8
#define PIN_BTN_DOWN   PC9
#define PIN_BTN_LEFT   PB14
#define PIN_BTN_RIGHT  PA8
#define PIN_BTN_MENU   PB15

// Hardware Peripherals
#define PIN_PWR_HOLD   PB7
#define PIN_BACKLIGHT  PC7
#define PIN_BUZZER     PC6
#define PIN_RELAY      PB6
#define PIN_IR_RX      PB9

// Candidate pins to hunt for OK button
static const uint32_t CANDIDATE_PINS[] = {
    PB8, PB10, PB11, PB12, PB13,
    PA0, PA1, PA4, PA5, PA6, PA7, PA11, PA12, PA15,
    PC0, PC1, PC2, PC3, PC4, PC5, PC10, PC11, PC13,
    PD2
};
static const size_t NUM_CANDIDATES = sizeof(CANDIDATE_PINS) / sizeof(CANDIDATE_PINS[0]);

static uint8_t backlight_level = 255;
static bool relay_state = false;

// Direct Hardware Audio Generator for PC6 (Works for both piezo & active buzzers)
static void beep(uint16_t freq_hz = 2700, uint16_t dur_ms = 40) {
    if (freq_hz == 0 || dur_ms == 0) return;
    uint32_t half_period_us = 1000000UL / (2 * freq_hz);
    if (half_period_us == 0) half_period_us = 1;
    uint32_t cycles = ((uint32_t)dur_ms * 1000UL) / (2 * half_period_us);
    
    for (uint32_t i = 0; i < cycles; i++) {
        GPIOC->BSRR = (1U << 6); // PC6 HIGH
        delayMicroseconds(half_period_us);
        GPIOC->BRR  = (1U << 6); // PC6 LOW
        delayMicroseconds(half_period_us);
    }
}

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
    // 1. Immediately latch battery power ON via PB7
    pinMode(PIN_PWR_HOLD, OUTPUT);
    digitalWrite(PIN_PWR_HOLD, HIGH);

    // 2. Turn on LCD Backlight via PC7
    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);

    // 3. Initialize Buzzer on PC6 (idle LOW)
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    // 4. Oscilloscope Relay / Switch (PB6, default LOW)
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);

    // 5. Infrared Receiver (PB9, idle HIGH)
    pinMode(PIN_IR_RX, INPUT_PULLUP);

    // Startup Chime (loud, clear 3-tone chirp)
    beep(2000, 60);
    delay(40);
    beep(2800, 80);
    delay(40);
    beep(3600, 100);

    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(38400);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    // 6. Configure 5 confirmed front buttons with internal pullups
    pinMode(PIN_BTN_UP,    INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,  INPUT_PULLUP);
    pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
    pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU,  INPUT_PULLUP);

    // 7. Configure candidate pins to hunt for OK button
    for (size_t i = 0; i < NUM_CANDIDATES; i++) {
        pinMode(CANDIDATE_PINS[i], INPUT_PULLUP);
    }

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 COMPLETE HARDWARE ENGINE ONLINE       ");
    rtt.println("   POWER LATCH: PB7=HIGH | BACKLIGHT: PC7=HIGH          ");
    rtt.println("   RELAY: PB6            | IR RECEIVER: PB9             ");
    rtt.println("   UP=PC8, DOWN=PC9, LEFT=PB14, RIGHT=PA8, MENU=PB15    ");
    rtt.println("========================================================\n");
}

void loop() {
    // Read hardware registers directly for zero latency
    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    // 1. Decode confirmed 5 buttons (active LOW)
    bool b_up    = !(rc & (1U << 8));   // PC8
    bool b_down  = !(rc & (1U << 9));   // PC9
    bool b_left  = !(rb & (1U << 14));  // PB14
    bool b_right = !(ra & (1U << 8));   // PA8
    bool b_menu  = !(rb & (1U << 15));  // PB15

    // 2. Infrared Receiver on PB9 (pulled LOW by demodulated 38 kHz carrier)
    bool ir_pulse = !(rb & (1U << 9));  // PB9
    static uint32_t last_ir_time = 0;
    static uint32_t ir_pulse_count = 0;
    static bool prev_ir_pulse = false;
    if (ir_pulse && !prev_ir_pulse) {
        ir_pulse_count++;
        last_ir_time = millis();
        beep(3200, 10);
        rtt.printf(">> [IR EVENT] Pulse detected on PB9! Total pulses: %lu\n", ir_pulse_count);
    }
    prev_ir_pulse = ir_pulse;
    bool ir_active = (millis() - last_ir_time < 300);

    // 3. Scan candidate pins to hunt for OK button
    bool b_ok = false;
    static char ok_pin_name[16] = "???";
    for (size_t i = 0; i < NUM_CANDIDATES; i++) {
        if (digitalRead(CANDIDATE_PINS[i]) == LOW) {
            b_ok = true;
            uint32_t p = CANDIDATE_PINS[i];
            int port = (p >> 4) & 0xF;
            int pin_num = p & 0xF;
            char port_char = (port == 0) ? 'A' : (port == 1) ? 'B' : (port == 2) ? 'C' : 'D';
            snprintf(ok_pin_name, sizeof(ok_pin_name), "P%c%d", port_char, pin_num);
            break;
        }
    }

    // 4. Oscilloscope Relay Control (PB6): Hold MENU and press RIGHT to toggle!
    static uint32_t last_relay_toggle = 0;
    if (b_menu && b_right && (millis() - last_relay_toggle > 400)) {
        last_relay_toggle = millis();
        relay_state = !relay_state;
        digitalWrite(PIN_RELAY, relay_state ? HIGH : LOW);
        beep(relay_state ? 4000 : 1600, 30);
        rtt.printf(">> [RELAY PB6] Toggled to: %s\n", relay_state ? "ON (HIGH)" : "OFF (LOW)");
    }

    // 5. Pack into 8-bit status mask
    // bit 0: UP, bit 1: DOWN, bit 2: LEFT, bit 3: RIGHT, bit 4: OK, bit 5: MENU, bit 6: RELAY, bit 7: IR_ACTIVE
    uint8_t btn_mask = (b_up        ? 0x01 : 0) |
                       (b_down      ? 0x02 : 0) |
                       (b_left      ? 0x04 : 0) |
                       (b_right     ? 0x08 : 0) |
                       (b_ok        ? 0x10 : 0) |
                       (b_menu      ? 0x20 : 0) |
                       (relay_state ? 0x40 : 0) |
                       (ir_active   ? 0x80 : 0);

    const char* active_btn = "NONE";
    const char* active_pin = "NONE";

    if (b_ok)             { active_btn = "OK";    active_pin = ok_pin_name; }
    else if (ir_active)   { active_btn = "IR_RX"; active_pin = "PB9";       }
    else if (b_up)        { active_btn = "UP";    active_pin = "PC8";       }
    else if (b_down)      { active_btn = "DOWN";  active_pin = "PC9";       }
    else if (b_left)      { active_btn = "LEFT";  active_pin = "PB14";      }
    else if (b_right)     { active_btn = "RIGHT"; active_pin = "PA8";       }
    else if (b_menu)      { active_btn = "MENU";  active_pin = "PB15";      }

    // Print to RTT on button transitions & play key click sound
    static uint8_t prev_mask = 0;
    if ((btn_mask & 0x3F) != (prev_mask & 0x3F)) {
        if ((btn_mask & 0x3F) > (prev_mask & 0x3F)) {
            beep(2800, 18); // Crisp key click sound
        }
        prev_mask = btn_mask;
        if ((btn_mask & 0x3F) != 0) {
            rtt.printf(">> [KEY EVENT] ACTIVE: %s (PIN: %s, MASK: 0x%02X)\n",
                       active_btn, active_pin, btn_mask);
        } else {
            rtt.println(">> [KEY EVENT] ALL RELEASED (IDLE)");
        }
    }

    // Backlight adjustment: Hold MENU and press UP or DOWN
    static uint32_t last_bl_change = 0;
    if (b_menu && (millis() - last_bl_change > 150)) {
        if (b_up && backlight_level < 255) {
            backlight_level = (backlight_level > 230) ? 255 : (backlight_level + 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = millis();
            beep(3600, 15);
            rtt.printf(">> Backlight UP: %u/255\n", backlight_level);
        } else if (b_down && backlight_level > 30) {
            backlight_level = (backlight_level < 55) ? 30 : (backlight_level - 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = millis();
            beep(1800, 15);
            rtt.printf(">> Backlight DOWN: %u/255\n", backlight_level);
        }
    }

    // 25 Hz Telemetry Packet to Display MCU
    static uint32_t last_tx = 0;
    static uint32_t seq = 0;

    if (millis() - last_tx >= 40) {
        last_tx = millis();
        seq++;
        uint32_t vcc = read_vcc_mv();

        // Packet format: #TC2:<seq>,<btn_str>,<pin_str>,<mask_hex>,<bl_val>,<vcc>,<uptime>*\n
        char buf[128];
        snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%s,%02X,%u,%lu,%lu*\n",
                 seq, active_btn, active_pin, btn_mask, backlight_level, vcc, millis());

#if defined(USART1)
        SerialUART1.print(buf);
#endif
        Serial.print(buf);
    }
}
