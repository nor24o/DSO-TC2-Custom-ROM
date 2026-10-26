#include <Arduino.h>
#include <HardwareSerial.h>
#include <Stream.h>
#include <SEGGER_RTT.h>

class RTTStream : public Stream {
public:
    RTTStream() {}
    int available() override { return SEGGER_RTT_HasKey() ? 1 : 0; }
    int read() override { return SEGGER_RTT_GetKey(); }
    int peek() override { return -1; }
    void flush() override {}
    size_t write(uint8_t c) override {
        SEGGER_RTT_Write(0, &c, 1);
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        SEGGER_RTT_Write(0, buffer, size);
        return size;
    }
};

static RTTStream rtt;

#if defined(USART1)
HardwareSerial SerialUART1(USART1);
#endif

// ============================================================================
// FNIRSI DSO-TC2 HARDWARE PIN DEFINITIONS (STM32L083RZ LQFP-64)
// ============================================================================
// Front Panel Buttons (Active LOW, all contiguous on Pins 34..41!)
#define PIN_BTN_PWR    PB13   // Pin 34: POWER Button
#define PIN_BTN_OK     PB14   // Pin 35: OK / ENTER Button (Left Navigation <)
#define PIN_BTN_MENU   PB15   // Pin 36: MENU / AUTO Button
#define PIN_BTN_UP     PC8    // Pin 39: UP Button (^)
#define PIN_BTN_DOWN   PC9    // Pin 40: DOWN Button (v)
#define PIN_BTN_TEST   PA8    // Pin 41: TEST / HOLD Button (Right Navigation >)

// Hardware Peripherals
#define PIN_BUZZER     PC6    // Pin 37: Audio Generator (Active / Passive buzzer)
#define PIN_BACKLIGHT  PC7    // Pin 38: LCD Backlight (TIM22_CH2 PWM / GPIO)
#define PIN_RELAY      PB6    // Pin 58: Oscilloscope Solid-State Relay / Range Switch
#define PIN_PWR_HOLD   PB7    // Pin 59: Power-Hold Soft-Latch (Active HIGH)
#define PIN_IR_RX      PB9    // Pin 62: 38 kHz Demodulated Infrared Receiver

static uint8_t backlight_level = 255;
static bool relay_state = false;

// ============================================================================
// BUZZER & AUDIO GENERATOR ENGINE
// ============================================================================
// Square-wave audio generation for passive piezo transducers
static void beep_square(uint16_t freq_hz = 2700, uint16_t dur_ms = 40) {
    if (freq_hz == 0 || dur_ms == 0) return;
    uint32_t half_period_us = 1000000UL / (2 * freq_hz);
    if (half_period_us == 0) half_period_us = 1;
    uint32_t cycles = ((uint32_t)dur_ms * 1000UL) / (2 * half_period_us);
    
    for (uint32_t i = 0; i < cycles; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delayMicroseconds(half_period_us);
        digitalWrite(PIN_BUZZER, LOW);
        delayMicroseconds(half_period_us);
    }
}

// Steady DC pulse for active buzzers
static void beep_dc(uint16_t dur_ms = 30) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(dur_ms);
    digitalWrite(PIN_BUZZER, LOW);
}

// Universal key-click sound: generates both a DC pulse and an AC chirp
// Guarantees audible click regardless of active or passive buzzer type
static void beep_click() {
    beep_dc(15);
    beep_square(2800, 15);
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

    // 6. Configure all 6 physical buttons with internal pullups
    pinMode(PIN_BTN_PWR,  INPUT_PULLUP);
    pinMode(PIN_BTN_OK,   INPUT_PULLUP);
    pinMode(PIN_BTN_TEST, INPUT_PULLUP);
    pinMode(PIN_BTN_UP,   INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU, INPUT_PULLUP);

    // Startup Sound (Tests both DC active buzzer and AC frequency chirp)
    beep_dc(80);
    delay(40);
    beep_square(2000, 50);
    delay(30);
    beep_square(2800, 60);
    delay(30);
    beep_square(3600, 80);

    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(38400);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 COMPLETE HARDWARE ENGINE ONLINE       ");
    rtt.println("   POWER LATCH: PB7=HIGH | BACKLIGHT: PC7=HIGH          ");
    rtt.println("   RELAY: PB6            | IR RECEIVER: PB9             ");
    rtt.println("   BUZZER: PC6 (Active/Passive Dual-Driver)             ");
    rtt.println("   PWR=PB13, OK=PB14, TEST=PA8, UP=PC8, DN=PC9, MN=PB15 ");
    rtt.println("========================================================\n");
}

void loop() {
    // Read hardware registers directly for zero latency
    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    // 1. Decode all 6 physical front buttons (active LOW)
    bool b_up    = !(rc & (1U << 8));   // PC8
    bool b_down  = !(rc & (1U << 9));   // PC9
    bool b_ok    = !(rb & (1U << 14));  // PB14: OK / ENTER (Left button <)
    bool b_test  = !(ra & (1U << 8));   // PA8:  TEST / HOLD (Right button >)
    bool b_menu  = !(rb & (1U << 15));  // PB15: MENU / AUTO
    bool b_pwr   = !(rb & (1U << 13));  // PB13: POWER button

    // 2. Infrared Receiver on PB9 (pulled LOW by demodulated 38 kHz carrier)
    bool ir_pulse = !(rb & (1U << 9));  // PB9
    static uint32_t last_ir_time = 0;
    static uint32_t ir_pulse_count = 0;
    static bool prev_ir_pulse = false;
    if (ir_pulse && !prev_ir_pulse) {
        ir_pulse_count++;
        last_ir_time = millis();
        beep_square(3200, 10);
        rtt.printf(">> [IR EVENT] Pulse detected on PB9! Total pulses: %lu\n", ir_pulse_count);
    }
    prev_ir_pulse = ir_pulse;
    bool ir_active = (millis() - last_ir_time < 300);

    // 3. Oscilloscope Relay Control (PB6): Hold MENU and press TEST (PA8) to toggle
    static uint32_t last_relay_toggle = 0;
    if (b_menu && b_test && (millis() - last_relay_toggle > 400)) {
        last_relay_toggle = millis();
        relay_state = !relay_state;
        digitalWrite(PIN_RELAY, relay_state ? HIGH : LOW);
        beep_square(relay_state ? 4000 : 1600, 30);
        rtt.printf(">> [RELAY PB6] Toggled to: %s\n", relay_state ? "ON (HIGH)" : "OFF (LOW)");
    }

    // 4. Diagnostic Buzzer Test: Hold MENU and press OK (PB14) to trigger multi-mode test
    static uint32_t last_buzz_test = 0;
    if (b_menu && b_ok && (millis() - last_buzz_test > 1000)) {
        last_buzz_test = millis();
        rtt.println(">> [BUZZER TEST RUNNING]");
        rtt.println(">> Step 1: DC HIGH 150ms (active buzzer test)");
        beep_dc(150);
        delay(100);
        rtt.println(">> Step 2: 2.7kHz Square Wave 200ms (passive buzzer test)");
        beep_square(2700, 200);
        delay(100);
        rtt.println(">> Step 3: DC LOW 150ms (inverted test)");
        digitalWrite(PIN_BUZZER, LOW);
        delay(150);
        digitalWrite(PIN_BUZZER, HIGH);
        delay(50);
        digitalWrite(PIN_BUZZER, LOW);
        rtt.println(">> [BUZZER TEST COMPLETE]");
    }

    // 5. Soft Power-Off: Hold POWER button for > 1.5 seconds
    static uint32_t pwr_press_start = 0;
    if (b_pwr) {
        if (pwr_press_start == 0) pwr_press_start = millis();
        if (millis() - pwr_press_start > 1500) {
            rtt.println(">> [POWER] Long press detected! Shutting down system...");
            beep_square(3600, 80);
            delay(40);
            beep_square(2400, 80);
            delay(40);
            beep_square(1200, 120);
            // Release soft latch
            digitalWrite(PIN_PWR_HOLD, LOW);
            digitalWrite(PIN_BACKLIGHT, LOW);
            while (1) {
                digitalWrite(PIN_PWR_HOLD, LOW);
                delay(100);
            }
        }
    } else {
        pwr_press_start = 0;
    }

    // 6. Pack into 8-bit status mask
    // bit 0: UP, bit 1: DOWN, bit 2: OK, bit 3: TEST, bit 4: MENU, bit 5: POWER, bit 6: RELAY, bit 7: IR_ACTIVE
    uint8_t btn_mask = (b_up        ? 0x01 : 0) |
                       (b_down      ? 0x02 : 0) |
                       (b_ok        ? 0x04 : 0) |
                       (b_test      ? 0x08 : 0) |
                       (b_menu      ? 0x10 : 0) |
                       (b_pwr       ? 0x20 : 0) |
                       (relay_state ? 0x40 : 0) |
                       (ir_active   ? 0x80 : 0);

    const char* active_btn = "NONE";
    const char* active_pin = "NONE";

    if (b_pwr)            { active_btn = "POWER"; active_pin = "PB13"; }
    else if (b_ok)        { active_btn = "OK";    active_pin = "PB14"; }
    else if (b_test)      { active_btn = "TEST";  active_pin = "PA8";  }
    else if (b_up)        { active_btn = "UP";    active_pin = "PC8";  }
    else if (b_down)      { active_btn = "DOWN";  active_pin = "PC9";  }
    else if (b_menu)      { active_btn = "MENU";  active_pin = "PB15"; }
    else if (ir_active)   { active_btn = "IR_RX"; active_pin = "PB9";  }

    // Print to RTT on button transitions & play key click sound
    static uint8_t prev_mask = 0;
    uint8_t current_keys = btn_mask & 0x3F;
    uint8_t prev_keys    = prev_mask & 0x3F;

    if (current_keys != prev_keys) {
        if (current_keys > prev_keys) {
            beep_click(); // Play click sound
        }
        prev_mask = btn_mask;
        if (current_keys != 0) {
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
            beep_square(3600, 15);
            rtt.printf(">> Backlight UP: %u/255\n", backlight_level);
        } else if (b_down && backlight_level > 30) {
            backlight_level = (backlight_level < 55) ? 30 : (backlight_level - 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = millis();
            beep_square(1800, 15);
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
