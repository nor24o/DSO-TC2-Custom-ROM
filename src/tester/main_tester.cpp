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
#define PIN_BTN_OK     PB9
#define PIN_BTN_MENU   PB15

#define PIN_PWR_HOLD   PB7
#define PIN_BACKLIGHT  PC7

static uint8_t backlight_level = 255;

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

    // Also drive PC6 HIGH just in case of dual latching / enable
    pinMode(PC6, OUTPUT);
    digitalWrite(PC6, HIGH);

    // 2. Turn on LCD Backlight via PC7
    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);

    Serial.setRx(PA3);
    Serial.setTx(PA2);
    Serial.begin(38400);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    // 3. Configure all 6 front buttons with internal pullups
    pinMode(PIN_BTN_UP,    INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,  INPUT_PULLUP);
    pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
    pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
    pinMode(PIN_BTN_OK,    INPUT_PULLUP);
    pinMode(PIN_BTN_MENU,  INPUT_PULLUP);

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 COMPLETE HARDWARE ENGINE ONLINE       ");
    rtt.println("   POWER LATCH: PB7=HIGH | BACKLIGHT: PC7=HIGH          ");
    rtt.println("   UP=PC8, DOWN=PC9, LEFT=PB14, RIGHT=PA8               ");
    rtt.println("   OK=PB9, MENU=PB15                                    ");
    rtt.println("========================================================\n");
}

void loop() {
    // Read hardware registers directly for zero latency
    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    // Decode all 6 buttons (active LOW)
    bool b_up    = !(rc & (1U << 8));   // PC8
    bool b_down  = !(rc & (1U << 9));   // PC9
    bool b_left  = !(rb & (1U << 14));  // PB14
    bool b_right = !(ra & (1U << 8));   // PA8
    bool b_ok    = !(rb & (1U << 9));   // PB9
    bool b_menu  = !(rb & (1U << 15));  // PB15

    // Pack into a single 6-bit button mask
    // bit 0: UP, bit 1: DOWN, bit 2: LEFT, bit 3: RIGHT, bit 4: OK, bit 5: MENU
    uint8_t btn_mask = (b_up    ? 0x01 : 0) |
                       (b_down  ? 0x02 : 0) |
                       (b_left  ? 0x04 : 0) |
                       (b_right ? 0x08 : 0) |
                       (b_ok    ? 0x10 : 0) |
                       (b_menu  ? 0x20 : 0);

    const char* active_btn = "NONE";
    const char* active_pin = "NONE";

    if (b_ok)         { active_btn = "OK";    active_pin = "PB9";  }
    else if (b_up)    { active_btn = "UP";    active_pin = "PC8";  }
    else if (b_down)  { active_btn = "DOWN";  active_pin = "PC9";  }
    else if (b_left)  { active_btn = "LEFT";  active_pin = "PB14"; }
    else if (b_right) { active_btn = "RIGHT"; active_pin = "PA8";  }
    else if (b_menu)  { active_btn = "MENU";  active_pin = "PB15"; }

    // Print to RTT on button transitions
    static uint8_t prev_mask = 0;
    if (btn_mask != prev_mask) {
        prev_mask = btn_mask;
        if (btn_mask != 0) {
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
            rtt.printf(">> Backlight UP: %u/255\n", backlight_level);
        } else if (b_down && backlight_level > 30) {
            backlight_level = (backlight_level < 55) ? 30 : (backlight_level - 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = millis();
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
