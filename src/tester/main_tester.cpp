#include <Arduino.h>
#include <HardwareSerial.h>
#include <Stream.h>
#include <math.h>
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
// HARDWARE PIN DEFINITIONS (STM32L083RZ LQFP-64)
// ============================================================================
// 6 Physical Front Buttons (Active LOW, Pins 34..41)
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

// Component Tester 1-2-3 Socket Direct ADC Sense Channels
#define PIN_SOCK_1     PA3    // Pin 17: Socket 1 (ADC_IN3)
#define PIN_SOCK_2     PA4    // Pin 20: Socket 2 (ADC_IN4)
#define PIN_SOCK_3     PA5    // Pin 21: Socket 3 (ADC_IN5)

// Component Tester Precision Resistor Probe Lines (Hardware Verified)
#define PIN_RL1        PA7    // Pin 23: S1 680 ohm probe
#define PIN_RH1        PB0    // Pin 26: S1 470k ohm probe
#define PIN_RL2        PB1    // Pin 27: S2 680 ohm probe
#define PIN_RH2        PB2    // Pin 28: S2 470k ohm probe
#define PIN_RL3        PB10   // Pin 29: S3 680 ohm probe
#define PIN_RH3        PB11   // Pin 30: S3 470k ohm probe

extern "C" uint16_t adc_read_value(PinName pin, uint32_t resolution);

static uint8_t backlight_level = 255;
static bool relay_state = false;

// ============================================================================
// AUDIO GENERATOR ENGINE
// ============================================================================
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

static void beep_dc(uint16_t dur_ms = 25) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(dur_ms);
    digitalWrite(PIN_BUZZER, LOW);
}

static void beep_click() {
    beep_dc(12);
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

static void release_probes() {
    pinMode(PIN_SOCK_1, INPUT);
    pinMode(PIN_SOCK_2, INPUT);
    pinMode(PIN_SOCK_3, INPUT);

    pinMode(PIN_RL1, INPUT);
    pinMode(PIN_RH1, INPUT);
    pinMode(PIN_RL2, INPUT);
    pinMode(PIN_RH2, INPUT);
    pinMode(PIN_RL3, INPUT);
    pinMode(PIN_RH3, INPUT);
}

// Discharge all 3 socket terminals to GND
static void discharge_socket(uint16_t ms = 30) {
    pinMode(PIN_RL1, OUTPUT);
    digitalWrite(PIN_RL1, LOW);
    pinMode(PIN_RL2, OUTPUT);
    digitalWrite(PIN_RL2, LOW);
    pinMode(PIN_RL3, OUTPUT);
    digitalWrite(PIN_RL3, LOW);
    delay(ms);
    release_probes();
}

// ============================================================================
// COMPONENT TESTER MEASUREMENT ENGINE
// ============================================================================
struct ComponentResult {
    char type[16];     // "NONE", "RESISTOR", "DIODE", "CAPACITOR", "SHORT"
    char value[24];    // "1.00 kOhm", "100.0 nF", "Vf=0.68V"
    char pinout[20];   // "Pins 1 - 2", "1(A) -> 2(K)"
    uint16_t s1_mv;
    uint16_t s2_mv;
    uint16_t s3_mv;
};

static ComponentResult comp_res = {"NONE", "No component", "---", 0, 0, 0};

// Capacitance measurement between two terminals
// Returns capacitance in pF, or 0 if not a capacitor
static uint32_t measure_capacitance(uint32_t pin_rl_a, uint32_t pin_rh_a, PinName sock_a,
                                    uint32_t pin_rl_b, PinName sock_b,
                                    uint32_t vcc_mv) {
    // 1. Discharge socket thoroughly
    discharge_socket(35);

    // 2. Connect Terminal B to direct GND
    pinMode(pin_rl_b, OUTPUT);
    digitalWrite(pin_rl_b, LOW);

    // 3. Configure Terminal A as digital Input (floating, no pull)
    uint8_t p = (sock_a & 0x0F);
    GPIOA->MODER &= ~(3U << (p * 2));
    GPIOA->PUPDR &= ~(3U << (p * 2));

    // Verify baseline voltage is low
    if ((GPIOA->IDR >> p) & 1U) {
        discharge_socket(50);
        GPIOA->MODER &= ~(3U << (p * 2));
        GPIOA->PUPDR &= ~(3U << (p * 2));
        if ((GPIOA->IDR >> p) & 1U) {
            release_probes();
            return 0;
        }
    }

    // Step A: Charge through RH (470k ohm) for 20 pF .. 1 uF
    pinMode(pin_rh_a, OUTPUT);
    digitalWrite(pin_rh_a, HIGH);
    uint32_t t_start = micros();

    // Fast register polling (<150 ns per cycle)
    while (!((GPIOA->IDR >> p) & 1U)) {
        if ((micros() - t_start) > 200000UL) break; // 200 ms timeout (>0.5 uF)
    }
    uint32_t t_elapsed = micros() - t_start;
    pinMode(pin_rh_a, INPUT); // Float charge drive immediately!

    // If it flipped in under 12 microseconds, it is purely OPEN socket pin stray capacitance!
    if (t_elapsed < 12) {
        discharge_socket(15);
        return 0;
    }

    // If it tripped within 200 ms:
    if (t_elapsed < 200000UL) {
        // Verify charge retention: a capacitor holds its voltage, a resistor collapses to GND
        delayMicroseconds(600);
        uint32_t adc_hold = adc_read_value(sock_a, 12);
        if (adc_hold < 900) {
            // Dropped below ~0.7V -> resistive or leaky, not a capacitor
            discharge_socket(20);
            return 0;
        }

        // Calculate capacitance from time constant tau:
        // V_hold / VCC ratio gives exact k = -ln(1 - V/Vcc)
        float v_ratio = (float)adc_hold / 4095.0f;
        float k = (v_ratio > 0.15f && v_ratio < 0.98f) ? -logf(1.0f - v_ratio) : 0.8223f;

        // C (pF) = (t_us * 1000) / (k * 470)
        uint32_t cap_pf = (uint32_t)(((float)t_elapsed * 1000.0f) / (k * 470.0f));

        // Deduct baseline PCB stray (~22 pF)
        if (cap_pf > 22) cap_pf -= 22; else cap_pf = 0;
        discharge_socket(30);
        return cap_pf;
    }

    // Step B: Large Capacitor (> 0.5 uF up to 5000 uF) charged via RL (680 ohm)
    discharge_socket(80);
    pinMode(pin_rl_b, OUTPUT);
    digitalWrite(pin_rl_b, LOW);

    GPIOA->MODER &= ~(3U << (p * 2));
    GPIOA->PUPDR &= ~(3U << (p * 2));

    pinMode(pin_rl_a, OUTPUT);
    digitalWrite(pin_rl_a, HIGH);
    t_start = micros();

    while (!((GPIOA->IDR >> p) & 1U)) {
        if ((micros() - t_start) > 3500000UL) break; // 3.5s timeout (~7000 uF)
    }
    t_elapsed = micros() - t_start;
    pinMode(pin_rl_a, INPUT);

    if (t_elapsed > 100 && t_elapsed < 3500000UL) {
        delayMicroseconds(1000);
        uint32_t adc_hold = adc_read_value(sock_a, 12);
        if (adc_hold >= 900) {
            float v_ratio = (float)adc_hold / 4095.0f;
            float k = (v_ratio > 0.15f && v_ratio < 0.98f) ? -logf(1.0f - v_ratio) : 0.8223f;
            // C_nF = (t_us * 1000) / (k * 680)
            uint32_t cap_nf = (uint32_t)(((float)t_elapsed * 1000.0f) / (k * 680.0f));
            discharge_socket(60);
            return cap_nf * 1000UL; // in pF
        }
    }

    discharge_socket(30);
    return 0;
}

static void test_pair(uint32_t pin_rl_a, uint32_t pin_rh_a, PinName sock_a, uint8_t idx_a,
                      uint32_t pin_rl_b, uint32_t pin_rh_b, PinName sock_b, uint8_t idx_b,
                      uint32_t vcc_mv, bool &found) {
    if (found) return;

    // 1. Forward probe: drive A HIGH through 680 ohm, drive B LOW (direct GND)
    pinMode(pin_rl_a, OUTPUT);
    digitalWrite(pin_rl_a, HIGH);
    pinMode(pin_rl_b, OUTPUT);
    digitalWrite(pin_rl_b, LOW);
    delayMicroseconds(200);

    uint32_t adc_va = adc_read_value(sock_a, 12);
    uint32_t adc_vb = adc_read_value(sock_b, 12);
    uint32_t va_fwd = (adc_va * vcc_mv) / 4095;
    uint32_t vb_fwd = (adc_vb * vcc_mv) / 4095;

    release_probes();
    delayMicroseconds(100);

    // 2. Reverse probe: drive B HIGH through 680 ohm, drive A LOW (direct GND)
    pinMode(pin_rl_b, OUTPUT);
    digitalWrite(pin_rl_b, HIGH);
    pinMode(pin_rl_a, OUTPUT);
    digitalWrite(pin_rl_a, LOW);
    delayMicroseconds(200);

    uint32_t adc_va_rev = adc_read_value(sock_a, 12);
    uint32_t adc_vb_rev = adc_read_value(sock_b, 12);
    uint32_t va_rev = (adc_va_rev * vcc_mv) / 4095;
    uint32_t vb_rev = (adc_vb_rev * vcc_mv) / 4095;

    release_probes();

    bool fwd_conducts = (vb_fwd > 70) && (va_fwd > vb_fwd);
    bool rev_conducts = (va_rev > 70) && (vb_rev > va_rev);

    if (fwd_conducts && rev_conducts) {
        // Symmetric DC conduction -> RESISTOR or SHORT!
        found = true;
        uint32_t delta_v = va_fwd - vb_fwd;
        uint32_t r_ohm = (680UL * delta_v) / (vb_fwd > 0 ? vb_fwd : 1);

        if (r_ohm < 10) {
            strcpy(comp_res.type, "SHORT");
            snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%lu Ohm", r_ohm, (delta_v * 680UL / vb_fwd) % 10);
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", idx_a, idx_b);
        } else {
            strcpy(comp_res.type, "RESISTOR");
            if (r_ohm < 1000) {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu Ohm", r_ohm);
            } else if (r_ohm < 1000000) {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%02lu kOhm", r_ohm / 1000, (r_ohm % 1000) / 10);
            } else {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%02lu MOhm", r_ohm / 1000000, (r_ohm % 1000000) / 10000);
            }
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", idx_a, idx_b);
        }
        rtt.printf(">> [COMPONENT DETECTED] %s: %s (%s)\n", comp_res.type, comp_res.value, comp_res.pinout);
        return;
    } else if (fwd_conducts && !rev_conducts) {
        // DIODE: Anode = A, Cathode = B
        found = true;
        strcpy(comp_res.type, "DIODE");
        uint32_t vf_mv = va_fwd - vb_fwd;
        snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", vf_mv / 1000, (vf_mv % 1000) / 10);
        snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d(A) -> %d(K)", idx_a, idx_b);
        rtt.printf(">> [COMPONENT DETECTED] DIODE: %s (%s)\n", comp_res.value, comp_res.pinout);
        return;
    } else if (!fwd_conducts && rev_conducts) {
        // DIODE: Anode = B, Cathode = A
        found = true;
        strcpy(comp_res.type, "DIODE");
        uint32_t vf_mv = vb_rev - va_rev;
        snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", vf_mv / 1000, (vf_mv % 1000) / 10);
        snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d(A) -> %d(K)", idx_b, idx_a);
        rtt.printf(">> [COMPONENT DETECTED] DIODE: %s (%s)\n", comp_res.value, comp_res.pinout);
        return;
    }

    // If NO DC conduction was found, test for CAPACITANCE!
    uint32_t cap_pf = measure_capacitance(pin_rl_a, pin_rh_a, sock_a, pin_rl_b, sock_b, vcc_mv);
    if (cap_pf > 30) {
        found = true;
        strcpy(comp_res.type, "CAPACITOR");
        if (cap_pf < 1000) {
            snprintf(comp_res.value, sizeof(comp_res.value), "%lu pF", cap_pf);
        } else if (cap_pf < 1000000) {
            uint32_t nf_x10 = (cap_pf + 50) / 100;
            snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%lu nF", nf_x10 / 10, nf_x10 % 10);
        } else {
            uint32_t uf_x10 = (cap_pf + 50000) / 100000;
            snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%lu uF", uf_x10 / 10, uf_x10 % 10);
        }
        snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", idx_a, idx_b);
        rtt.printf(">> [COMPONENT DETECTED] CAPACITOR: %s (%s)\n", comp_res.value, comp_res.pinout);
        return;
    }
}

static void run_component_test() {
    rtt.println(">> [TESTER] Running full component measurement (Resistor, Diode, Capacitor)...");
    beep_square(3200, 30);

    uint32_t vcc = read_vcc_mv();
    bool found = false;

    // Read resting voltages
    comp_res.s1_mv = (adc_read_value(PA_3, 12) * vcc) / 4095;
    comp_res.s2_mv = (adc_read_value(PA_4, 12) * vcc) / 4095;
    comp_res.s3_mv = (adc_read_value(PA_5, 12) * vcc) / 4095;

    // Test Pair 1 - 2
    test_pair(PIN_RL1, PIN_RH1, PA_3, 1, PIN_RL2, PIN_RH2, PA_4, 2, vcc, found);

    // Test Pair 2 - 3
    test_pair(PIN_RL2, PIN_RH2, PA_4, 2, PIN_RL3, PIN_RH3, PA_5, 3, vcc, found);

    // Test Pair 1 - 3
    test_pair(PIN_RL1, PIN_RH1, PA_3, 1, PIN_RL3, PIN_RH3, PA_5, 3, vcc, found);

    if (!found) {
        strcpy(comp_res.type, "NONE");
        strcpy(comp_res.value, "No component");
        strcpy(comp_res.pinout, "Pins 1-2-3 Open");
        rtt.println(">> [TESTER] Result: No component detected in 1-2-3 socket.");
    }

    release_probes();
}

void setup() {
    // 1. Latch battery power ON
    pinMode(PIN_PWR_HOLD, OUTPUT);
    digitalWrite(PIN_PWR_HOLD, HIGH);

    // 2. LCD Backlight
    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);

    // 3. Buzzer
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    // 4. Oscilloscope Relay
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);

    // 5. Infrared Receiver
    pinMode(PIN_IR_RX, INPUT_PULLUP);

    // 6. Buttons
    pinMode(PIN_BTN_PWR,  INPUT_PULLUP);
    pinMode(PIN_BTN_OK,   INPUT_PULLUP);
    pinMode(PIN_BTN_TEST, INPUT_PULLUP);
    pinMode(PIN_BTN_UP,   INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU, INPUT_PULLUP);

    // 7. Component Tester Probes high-Z
    release_probes();

    // Startup Chime
    beep_dc(60);
    delay(30);
    beep_square(2000, 40);
    delay(20);
    beep_square(2800, 50);
    delay(20);
    beep_square(3600, 70);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 COMPLETE HARDWARE ENGINE ONLINE       ");
    rtt.println("   COMPONENT TESTER MEASUREMENT SYSTEM READY            ");
    rtt.println("========================================================\n");
}

void loop() {
    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    bool b_up    = !(rc & (1U << 8));   // PC8
    bool b_down  = !(rc & (1U << 9));   // PC9
    bool b_ok    = !(rb & (1U << 14));  // PB14: OK / ENTER (<)
    bool b_test  = !(ra & (1U << 8));   // PA8:  TEST / HOLD (>)
    bool b_menu  = !(rb & (1U << 15));  // PB15: MENU / AUTO
    bool b_pwr   = !(rb & (1U << 13));  // PB13: POWER

    // IR Receiver on PB9
    bool ir_pulse = !(rb & (1U << 9));
    static uint32_t last_ir_time = 0;
    static uint32_t ir_pulse_count = 0;
    static bool prev_ir_pulse = false;
    if (ir_pulse && !prev_ir_pulse) {
        ir_pulse_count++;
        last_ir_time = millis();
        beep_square(3200, 10);
        rtt.printf(">> [IR EVENT] Pulse on PB9! Count: %lu\n", ir_pulse_count);
    }
    prev_ir_pulse = ir_pulse;
    bool ir_active = (millis() - last_ir_time < 300);

    // Relay Toggle: Hold MENU and press TEST (PA8)
    static uint32_t last_relay_toggle = 0;
    if (b_menu && b_test && (millis() - last_relay_toggle > 400)) {
        last_relay_toggle = millis();
        relay_state = !relay_state;
        digitalWrite(PIN_RELAY, relay_state ? HIGH : LOW);
        beep_square(relay_state ? 4000 : 1600, 30);
        rtt.printf(">> [RELAY PB6] Toggled to: %s\n", relay_state ? "ON (HIGH)" : "OFF (LOW)");
    }

    // Backlight Brightness: Hold MENU and press UP/DOWN
    static uint32_t last_bl_change = 0;
    if (b_menu && (millis() - last_bl_change > 150)) {
        if (b_up && backlight_level < 255) {
            backlight_level = (backlight_level > 230) ? 255 : (backlight_level + 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = millis();
            beep_square(3600, 15);
        } else if (b_down && backlight_level > 30) {
            backlight_level = (backlight_level < 55) ? 30 : (backlight_level - 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = millis();
            beep_square(1800, 15);
        }
    }

    // Run Component Test: Press TEST / HOLD (PA8) without MENU
    static uint32_t last_test_run = 0;
    if (b_test && !b_menu && (millis() - last_test_run > 600)) {
        last_test_run = millis();
        run_component_test();
    }

    // Soft Power-Off: Hold POWER for > 1.5 seconds
    static uint32_t pwr_press_start = 0;
    if (b_pwr) {
        if (pwr_press_start == 0) pwr_press_start = millis();
        if (millis() - pwr_press_start > 1500) {
            rtt.println(">> [POWER] Long press detected! Shutting down...");
            beep_square(3600, 80);
            delay(40);
            beep_square(2400, 80);
            delay(40);
            beep_square(1200, 120);
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

    // Status Mask & Active Key
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

    // Key click audio
    static uint8_t prev_mask = 0;
    uint8_t current_keys = btn_mask & 0x3F;
    uint8_t prev_keys    = prev_mask & 0x3F;

    if (current_keys != prev_keys) {
        if (current_keys > prev_keys) {
            beep_click();
        }
        prev_mask = btn_mask;
        if (current_keys != 0) {
            rtt.printf(">> [KEY EVENT] ACTIVE: %s (PIN: %s, MASK: 0x%02X)\n",
                       active_btn, active_pin, btn_mask);
        }
    }

    // Read live socket resting voltages periodically
    static uint32_t last_volt_read = 0;
    if (millis() - last_volt_read >= 250) {
        last_volt_read = millis();
        uint32_t vcc = read_vcc_mv();
        comp_res.s1_mv = (adc_read_value(PA_3, 12) * vcc) / 4095;
        comp_res.s2_mv = (adc_read_value(PA_4, 12) * vcc) / 4095;
        comp_res.s3_mv = (adc_read_value(PA_5, 12) * vcc) / 4095;
    }

    // 25 Hz Telemetry Packet to Display MCU
    static uint32_t last_tx = 0;
    static uint32_t seq = 0;

    if (millis() - last_tx >= 40) {
        last_tx = millis();
        seq++;
        uint32_t vcc = read_vcc_mv();

        char buf[160];
        snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%s,%02X,%u,%lu,%lu,%s,%s,%s,%u,%u,%u*\n",
                 seq, active_btn, active_pin, btn_mask, backlight_level, vcc, millis(),
                 comp_res.type, comp_res.value, comp_res.pinout,
                 comp_res.s1_mv, comp_res.s2_mv, comp_res.s3_mv);

#if defined(USART1)
        SerialUART1.print(buf);
#endif
    }
}
