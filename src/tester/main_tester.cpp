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
#define PIN_SIGGEN     PB8    // Pin 61: Programmable PWM / Square Wave Signal Generator
#define PIN_BATT_ADC   PA2    // Pin 16: Li-Po Battery 2:1 Voltage Divider (ADC_IN2)
#define PIN_SMA_ADC    PA6    // Pin 22: External SMA/MCX Connector Tap (ADC_IN6)

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
// ZENER DIODE HIGH-VOLTAGE BREAKDOWN MEASUREMENT (MT3608 BOOST CONVERTER)
// ============================================================================
struct ZenerResult {
    uint32_t zener_mv;
    char     status[24];
    bool     active;
    PinName  adc_pin;
    uint32_t en_pin;
};

static volatile ZenerResult zener_res = {0, "STANDBY", false, PA_0, 0xFF};

// Candidate ADC pins on STM32L083 (Excluded PA2 for Batt, PA6 for SMA)
static const PinName CANDIDATE_ZENER_ADC[] = {
    PA_0, PA_1, PC_0, PC_1, PC_2, PC_3, PC_4, PC_5
};
static const size_t NUM_ZENER_ADC = sizeof(CANDIDATE_ZENER_ADC) / sizeof(CANDIDATE_ZENER_ADC[0]);

// Candidate MT3608 Boost EN control pins (Excluded PB8 for SigGen)
static const uint32_t CANDIDATE_ZENER_EN[] = {
    PC4, PC5, PB5, PC0, PC1, PC2, PC3, PC10, PC11, PC12
};
static const size_t NUM_ZENER_EN = sizeof(CANDIDATE_ZENER_EN) / sizeof(CANDIDATE_ZENER_EN[0]);

// ============================================================================
// ============================================================================
// BARE-METAL HARDWARE TIMER (TIM6) SIGNAL GENERATOR (DIRECT IOPORT BIT-BANG)
// RAM VECTOR DISPATCH / ZERO HAL WRAPPER OVERHEAD / 100 kHz UNLOCKED
// ============================================================================
#define BSRR_PB8_HI  (1U << 8)
#define BSRR_PB8_LO  (1U << 24)

static const uint32_t SIGGEN_PRESETS[] = {
    10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000
};
static const uint8_t NUM_SIGGEN_PRESETS = sizeof(SIGGEN_PRESETS) / sizeof(SIGGEN_PRESETS[0]);
static uint8_t siggen_idx = 6; // Default: 1000 Hz (1.0 kHz)

enum SiggenWaveMode {
    WAVE_SQR  = 0, // Square (50% Duty)
    WAVE_TRI  = 1, // Triangle Wave
    WAVE_SINE = 2, // Sine Wave
    WAVE_SAW  = 3, // Sawtooth / Ramp Wave
    WAVE_P10  = 4, // Pulse (10% Duty)
    WAVE_P25  = 5, // Pulse (25% Duty)
    WAVE_P75  = 6, // Pulse (75% Duty)
    WAVE_P90  = 7  // Pulse (90% Duty)
};
static const uint8_t NUM_WAVE_MODES = 8;
static uint8_t siggen_wave_mode = 0; // Default: SQR (50%)
static bool siggen_on = false;

static volatile uint16_t siggen_arr_hi = 16000;
static volatile uint16_t siggen_arr_lo = 16000;

// High-speed bare-metal ISR executing directly from RAM vector table (no HAL, no std::function overhead)
static void siggen_isr_fast(void) {
    TIM6->SR = 0; // Clear update interrupt flag
    if (siggen_arr_hi == siggen_arr_lo) {
        GPIOB->ODR ^= (1U << 8); // Single-cycle toggle for symmetric square waves
    } else {
        if (GPIOB->ODR & (1U << 8)) {
            GPIOB->BSRR = BSRR_PB8_LO; // Set PB8 LOW (0.0V)
            TIM6->ARR = siggen_arr_lo;
        } else {
            GPIOB->BSRR = BSRR_PB8_HI; // Set PB8 HIGH (3.3V)
            TIM6->ARR = siggen_arr_hi;
        }
    }
}

// 256-byte aligned RAM vector table (48 entries * 4 bytes = 192 bytes)
__attribute__((aligned(256))) static uint32_t ram_vector_table[48];

static void init_ram_vectors() {
    uint32_t *orig_vectors = (uint32_t *)SCB->VTOR;
    for (int i = 0; i < 48; i++) {
        ram_vector_table[i] = orig_vectors[i];
    }
    // Route TIM6_DAC_IRQn (IRQ 17) directly to our ultra-lean siggen_isr_fast
    ram_vector_table[16 + TIM6_DAC_IRQn] = (uint32_t)siggen_isr_fast;
    __disable_irq();
    SCB->VTOR = (uint32_t)ram_vector_table;
    __DSB();
    __enable_irq();
}

static void update_siggen() {
    // 1. Disable TIM6 and NVIC IRQ
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    if (RCC->APB1ENR & RCC_APB1ENR_TIM6EN) {
        TIM6->CR1 = 0;
        TIM6->DIER = 0;
        TIM6->SR = 0;
    }

    if (!siggen_on) {
        pinMode(PIN_SIGGEN, OUTPUT);
        digitalWrite(PIN_SIGGEN, LOW);
        return;
    }

    pinMode(PIN_SIGGEN, OUTPUT);
    digitalWrite(PIN_SIGGEN, LOW);
    GPIOB->OSPEEDR |= (3U << 16); // High Speed slew rate on PB8

    // 2. Enable Clocks for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    __asm__ volatile("nop; nop");

    // 3. Determine Duty Percentage from Wave Mode
    uint8_t duty_pct = 50;
    switch (siggen_wave_mode) {
        case WAVE_SQR:  duty_pct = 50; break;
        case WAVE_TRI:  duty_pct = 50; break;
        case WAVE_SINE: duty_pct = 50; break;
        case WAVE_SAW:  duty_pct = 80; break;
        case WAVE_P10:  duty_pct = 10; break;
        case WAVE_P25:  duty_pct = 25; break;
        case WAVE_P75:  duty_pct = 75; break;
        case WAVE_P90:  duty_pct = 90; break;
        default:        duty_pct = 50; break;
    }

    // 4. Calculate Timer Prescaler & AutoReload for Target Frequency
    uint32_t target_freq = SIGGEN_PRESETS[siggen_idx];
    uint32_t psc = 0;
    uint32_t total_ticks = 0;

    if (target_freq < 100) {
        psc = 319; // 32 MHz / 320 = 100 kHz timer clock
        total_ticks = 100000UL / target_freq;
    } else if (target_freq < 10000) {
        psc = 31; // 32 MHz / 32 = 1 MHz timer clock
        total_ticks = 1000000UL / target_freq;
    } else {
        psc = 0; // 32 MHz timer clock
        total_ticks = 32000000UL / target_freq;
    }

    uint32_t hi_ticks = (total_ticks * (uint32_t)duty_pct) / 100UL;
    if (hi_ticks < 1) hi_ticks = 1;
    if (hi_ticks >= total_ticks) hi_ticks = total_ticks - 1;
    uint32_t lo_ticks = total_ticks - hi_ticks;
    if (lo_ticks < 1) lo_ticks = 1;

    siggen_arr_hi = (uint16_t)(hi_ticks - 1);
    siggen_arr_lo = (uint16_t)(lo_ticks - 1);

    // 5. Configure TIM6 registers
    TIM6->PSC = (uint16_t)psc;
    TIM6->ARR = siggen_arr_lo; // Start with low phase
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0;
    TIM6->DIER = TIM_DIER_UIE; // Enable update interrupt

    NVIC_SetPriority(TIM6_DAC_IRQn, 0); // Highest priority for jitter-free output
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM6->CR1 = TIM_CR1_CEN; // Counter enable
}

// ============================================================================
// BATTERY & SMA EXTERNAL VOLTAGE SAMPLER (PA2 & PA6)
// ============================================================================
static uint32_t read_vcc_mv();

static uint32_t sample_battery_mv() {
    uint32_t vcc = read_vcc_mv();
    uint32_t raw = adc_read_value(PA_2, 12);
    // 2:1 hardware divider: Vbatt = Vadc * 2
    uint32_t vadc = (raw * vcc) / 4095;
    return vadc * 2;
}

static uint32_t sample_sma_mv() {
    uint32_t vcc = read_vcc_mv();
    uint32_t raw = adc_read_value(PA_6, 12);
    uint32_t vadc_mv = (raw * vcc) / 4095;
    // Hardware input divider on SMA connector: 100k series + 27k to GND (Ratio: 127/27 = 4.704)
    uint32_t vin_mv = (vadc_mv * 127UL) / 27UL;
    return vin_mv;
}

// Instrument Modes: 0 = Tester, 1 = IR Decoder, 2 = Zener, 3 = SigGen & Voltmeter
static uint8_t instrument_mode = 0;

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

// ============================================================================
// INFRARED REMOTE DECODER ENGINE (Multi-Protocol: NEC / Samsung / Sony / RAW)
// ============================================================================
struct IrDecodedData {
    char protocol[12];
    uint16_t address;
    uint8_t command;
    uint32_t raw_code;
    uint32_t repeat_count;
    uint32_t last_rx_ms;
    bool has_new;
};

static volatile IrDecodedData ir_data = {"NONE", 0, 0, 0, 0, 0, false};

#define IR_RING_SIZE 128
static volatile uint16_t ir_edge_buf[IR_RING_SIZE];
static volatile uint8_t  ir_buf_head = 0;
static volatile uint8_t  ir_buf_tail = 0;
static volatile uint32_t ir_last_edge_us = 0;
static volatile uint32_t ir_edge_count = 0;

// High-speed ISR on PB9 (captures falling edges of 38 kHz demodulated bursts)
static void ir_exti_isr() {
    uint32_t now = micros();
    static uint32_t prev_edge = 0;
    uint32_t dt = now - prev_edge;

    // Reject ultra-short carrier spikes (< 320 us) WITHOUT updating prev_edge!
    if (dt < 320) return;

    prev_edge = now;
    ir_last_edge_us = now;
    ir_edge_count++;

    if (dt > 65000) dt = 65000;

    uint8_t next = (ir_buf_head + 1) % IR_RING_SIZE;
    if (next != ir_buf_tail) {
        ir_edge_buf[ir_buf_head] = (uint16_t)dt;
        ir_buf_head = next;
    }
}

// Background IR Protocol Parser (Runs in loop())
static void process_ir() {
    static enum { IR_IDLE, IR_IN_FRAME } state = IR_IDLE;
    static uint32_t raw_shift = 0;
    static uint8_t  bit_count = 0;
    static char current_proto[12] = "NEC";

    while (ir_buf_tail != ir_buf_head) {
        uint16_t dt = ir_edge_buf[ir_buf_tail];
        ir_buf_tail = (ir_buf_tail + 1) % IR_RING_SIZE;

        // 1. Leader Detection
        if (dt >= 11500 && dt <= 16500) {
            // NEC Leader Pulse (9ms LOW + 4.5ms HIGH = ~13.5ms)
            state = IR_IN_FRAME;
            bit_count = 0;
            raw_shift = 0;
            strcpy(current_proto, "NEC");
            continue;
        } else if (dt >= 9500 && dt < 11500) {
            // NEC Repeat Code (9ms LOW + 2.25ms HIGH = ~11.25ms)
            if (ir_data.raw_code != 0) {
                ir_data.repeat_count++;
                ir_data.last_rx_ms = millis();
                ir_data.has_new = true;
            }
            state = IR_IDLE;
            continue;
        } else if (dt >= 7500 && dt < 9500) {
            // Samsung Leader Pulse (4.5ms LOW + 4.5ms HIGH = ~9.0ms)
            state = IR_IN_FRAME;
            bit_count = 0;
            raw_shift = 0;
            strcpy(current_proto, "SAMS");
            continue;
        } else if (dt >= 2400 && dt <= 3600 && state == IR_IDLE) {
            // Sony SIRC Leader Pulse (2.4ms LOW + 0.6ms HIGH = ~3.0ms)
            state = IR_IN_FRAME;
            bit_count = 0;
            raw_shift = 0;
            strcpy(current_proto, "SONY");
            continue;
        }

        // 2. Data Bits (Pulse-Distance Decoded)
        if (state == IR_IN_FRAME) {
            if (dt >= 650 && dt < 1650) {
                // Bit 0 (~1.125 ms)
                raw_shift = (raw_shift >> 1); // LSB first
                bit_count++;
            } else if (dt >= 1650 && dt <= 3000) {
                // Bit 1 (~2.25 ms)
                raw_shift = (raw_shift >> 1) | 0x80000000UL; // LSB first
                bit_count++;
            } else if (dt < 650) {
                // Minor noise spike during frame — ignore without aborting
                continue;
            } else {
                // Inter-frame gap or stop bit (> 3000 us)
                if (bit_count >= 16) {
                    goto commit_ir_frame;
                } else {
                    state = IR_IDLE;
                }
            }

            if (bit_count == 32) {
                goto commit_ir_frame;
            }
        }
    }

    // 3. Frame Flush on Inactivity Timeout (>18ms silence after receiving bits)
    if (state == IR_IN_FRAME && (micros() - ir_last_edge_us > 18000UL)) {
        if (bit_count >= 12) {
            goto commit_ir_frame;
        } else {
            state = IR_IDLE;
        }
    }
    return;

commit_ir_frame:
    state = IR_IDLE;
    ir_data.raw_code = raw_shift;

    if (bit_count == 32) {
        uint8_t a0 = raw_shift & 0xFF;
        uint8_t a1 = (raw_shift >> 8) & 0xFF;
        uint8_t c0 = (raw_shift >> 16) & 0xFF;
        uint8_t c1 = (raw_shift >> 24) & 0xFF;

        if ((uint8_t)(c0 ^ c1) == 0xFF) {
            strcpy((char*)ir_data.protocol, current_proto);
            ir_data.address = (a0 == (uint8_t)~a1) ? a0 : (a0 | ((uint16_t)a1 << 8));
            ir_data.command = c0;
        } else {
            strcpy((char*)ir_data.protocol, "RAW32");
            ir_data.address = a0 | ((uint16_t)a1 << 8);
            ir_data.command = c0;
        }
    } else {
        strcpy((char*)ir_data.protocol, "RAW");
        ir_data.address = bit_count; // Store received bit count as address
        ir_data.command = raw_shift & 0xFF;
    }
    ir_data.repeat_count = 0;
    ir_data.last_rx_ms = millis();
    ir_data.has_new = true;
    bit_count = 0;
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
    char type[16];     // "NONE", "RESISTOR", "DIODE", "CAPACITOR", "NPN", "PNP", "N-MOS", "P-MOS", "SHORT"
    char value[24];    // "1.00 kOhm", "100.0 nF", "Vf=0.68V"
    char pinout[28];   // "Pins 1 - 2", "1(A) -> 2(K)", "B:1 C:2 E:3", "G:1 D:2 S:3"
    uint16_t s1_mv;
    uint16_t s2_mv;
    uint16_t s3_mv;
};

static ComponentResult comp_res = {"NONE", "No component", "Pins 1-2-3 Open", 0, 0, 0};

struct SocketDef {
    uint32_t pin_rl;
    uint32_t pin_rh;
    PinName adc_pin;
    uint32_t pin_direct;
    uint8_t idx;
};
static const SocketDef PROBES[3] = {
    {PIN_RL1, PIN_RH1, PA_3, PIN_SOCK_1, 1},
    {PIN_RL2, PIN_RH2, PA_4, PIN_SOCK_2, 2},
    {PIN_RL3, PIN_RH3, PA_5, PIN_SOCK_3, 3}
};

struct DiodeDetection {
    uint8_t a;     // Anode index (1..3)
    uint8_t k;     // Cathode index (1..3)
    uint16_t vf_mv;
};

static void test_success_chime() {
    beep_square(2400, 20);
    delay(15);
    beep_square(3400, 30);
}

// Returns diode forward voltage in mV if conducting from a_idx -> b_idx, or 0 if non-conducting
static uint16_t probe_diode_direction(uint8_t a_idx, uint8_t b_idx, uint32_t vcc_mv) {
    const SocketDef& probe_a = PROBES[a_idx];
    const SocketDef& probe_b = PROBES[b_idx];

    // Forward drive via RL (680R): A=HIGH, B=LOW
    pinMode(probe_a.pin_rl, OUTPUT);
    digitalWrite(probe_a.pin_rl, HIGH);
    pinMode(probe_b.pin_rl, OUTPUT);
    digitalWrite(probe_b.pin_rl, LOW);
    delayMicroseconds(400);

    uint32_t va = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    uint32_t vb = (adc_read_value(probe_b.adc_pin, 12) * vcc_mv) / 4095;
    release_probes();
    delayMicroseconds(100);

    // Reverse drive: B=HIGH, A=LOW
    pinMode(probe_b.pin_rl, OUTPUT);
    digitalWrite(probe_b.pin_rl, HIGH);
    pinMode(probe_a.pin_rl, OUTPUT);
    digitalWrite(probe_a.pin_rl, LOW);
    delayMicroseconds(400);

    uint32_t va_rev = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    release_probes();

    // Diode criteria:
    // 1. Current flows into cathode in forward direction (vb > 60 mV)
    // 2. Forward drop: 150 mV <= Vf <= 2800 mV
    // 3. Reverse direction is non-conducting (va_rev < 80 mV)
    if (vb > 60 && va > vb && (va - vb >= 150) && (va - vb <= 2800) && va_rev < 80) {
        return (uint16_t)(va - vb);
    }
    return 0;
}

// Returns true if resistor or short detected between a_idx and b_idx
static bool test_resistor(uint8_t a_idx, uint8_t b_idx, uint32_t vcc_mv) {
    const SocketDef& probe_a = PROBES[a_idx];
    const SocketDef& probe_b = PROBES[b_idx];

    // 1. RL Low-Range Test (15 Ohm .. 25 kOhm)
    pinMode(probe_a.pin_rl, OUTPUT);
    digitalWrite(probe_a.pin_rl, HIGH);
    pinMode(probe_b.pin_rl, OUTPUT);
    digitalWrite(probe_b.pin_rl, LOW);
    delayMicroseconds(400);

    uint32_t va_fwd = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    uint32_t vb_fwd = (adc_read_value(probe_b.adc_pin, 12) * vcc_mv) / 4095;
    release_probes();
    delayMicroseconds(100);

    pinMode(probe_b.pin_rl, OUTPUT);
    digitalWrite(probe_b.pin_rl, HIGH);
    pinMode(probe_a.pin_rl, OUTPUT);
    digitalWrite(probe_a.pin_rl, LOW);
    delayMicroseconds(400);

    uint32_t vb_rev = (adc_read_value(probe_b.adc_pin, 12) * vcc_mv) / 4095;
    uint32_t va_rev = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    release_probes();

    // Check Short / Continuity (< 15 Ohm)
    if (vb_fwd > 1200 && va_rev > 1200) {
        int32_t diff_fwd = abs((int32_t)va_fwd - (int32_t)vb_fwd);
        int32_t diff_rev = abs((int32_t)vb_rev - (int32_t)va_rev);
        if (diff_fwd < 140 && diff_rev < 140) {
            strcpy(comp_res.type, "CONTINUITY");
            uint32_t avg_diff = (diff_fwd + diff_rev) / 2;
            uint32_t r_ohm = (680UL * avg_diff) / (vb_fwd > 0 ? vb_fwd : 1);
            if (r_ohm < 2) {
                snprintf(comp_res.value, sizeof(comp_res.value), "< 1.0 Ohm BEEP");
            } else {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%lu Ohm BEEP", r_ohm, (avg_diff * 6800UL / vb_fwd) % 10);
            }
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", probe_a.idx, probe_b.idx);
            rtt.printf(">> [CONTINUITY BEEP] SHORT: %s (%s)\n", comp_res.value, comp_res.pinout);
            beep_square(3200, 70);
            return true;
        }
    }

    // Check RL Resistor
    bool fwd_conducts = (vb_fwd > 55) && (va_fwd > vb_fwd);
    bool rev_conducts = (va_rev > 55) && (vb_rev > va_rev);
    if (fwd_conducts && rev_conducts) {
        strcpy(comp_res.type, "RESISTOR");
        uint32_t delta_v = va_fwd - vb_fwd;
        uint32_t r_ohm = (680UL * delta_v) / (vb_fwd > 0 ? vb_fwd : 1);
        if (r_ohm < 1000) {
            snprintf(comp_res.value, sizeof(comp_res.value), "%lu Ohm", r_ohm);
        } else {
            snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%02lu kOhm", r_ohm / 1000, (r_ohm % 1000) / 10);
        }
        snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", probe_a.idx, probe_b.idx);
        rtt.printf(">> [COMPONENT DETECTED] RESISTOR (RL): %s (%s)\n", comp_res.value, comp_res.pinout);
        test_success_chime();
        return true;
    }

    // 2. RH High-Range Test (25 kOhm .. 2.5 MOhm)
    pinMode(probe_a.pin_rh, OUTPUT);
    digitalWrite(probe_a.pin_rh, HIGH);
    pinMode(probe_b.pin_rh, OUTPUT);
    digitalWrite(probe_b.pin_rh, LOW);
    delayMicroseconds(1000);

    uint32_t va_rh_1 = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    uint32_t vb_rh_1 = (adc_read_value(probe_b.adc_pin, 12) * vcc_mv) / 4095;

    // Transient check: sample again after 5 ms.
    // If vb_rh decays, it's a capacitor charging, NOT a resistor!
    delay(5);
    uint32_t vb_rh_2 = (adc_read_value(probe_b.adc_pin, 12) * vcc_mv) / 4095;
    release_probes();
    delayMicroseconds(200);

    pinMode(probe_b.pin_rh, OUTPUT);
    digitalWrite(probe_b.pin_rh, HIGH);
    pinMode(probe_a.pin_rh, OUTPUT);
    digitalWrite(probe_a.pin_rh, LOW);
    delayMicroseconds(1000);

    uint32_t va_rh_rev_1 = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    delay(5);
    uint32_t va_rh_rev_2 = (adc_read_value(probe_a.adc_pin, 12) * vcc_mv) / 4095;
    release_probes();

    // Symmetrical steady conduction:
    // Voltage must be above 380 mV AND must be STEADY (|sample1 - sample2| < 90 mV)
    bool fwd_steady = (abs((int32_t)vb_rh_1 - (int32_t)vb_rh_2) < 90);
    bool rev_steady = (abs((int32_t)va_rh_rev_1 - (int32_t)va_rh_rev_2) < 90);

    if (fwd_steady && rev_steady && vb_rh_2 > 380 && va_rh_rev_2 > 380) {
        uint32_t delta_v = va_rh_1 - vb_rh_1;
        uint32_t r_ohm = (470000UL * delta_v) / (vb_rh_2 > 0 ? vb_rh_2 : 1);
        if (r_ohm <= 2500000UL) {
            strcpy(comp_res.type, "RESISTOR");
            if (r_ohm < 1000000) {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%01lu kOhm", r_ohm / 1000, (r_ohm % 1000) / 100);
            } else {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%02lu MOhm", r_ohm / 1000000, (r_ohm % 1000000) / 10000);
            }
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", probe_a.idx, probe_b.idx);
            rtt.printf(">> [COMPONENT DETECTED] RESISTOR (RH): %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            return true;
        }
    }

    return false;
}

// Returns true if a capacitor was detected and measured, filling comp_res
static bool test_capacitance(uint8_t a_idx, uint8_t b_idx, uint32_t vcc_mv) {
    discharge_socket(40);

    const SocketDef& probe_a = PROBES[a_idx];
    const SocketDef& probe_b = PROBES[b_idx];

    // Connect Terminal B to direct GND
    pinMode(probe_b.pin_rl, OUTPUT);
    digitalWrite(probe_b.pin_rl, LOW);

    // Terminal A as floating digital input
    uint8_t p = (probe_a.adc_pin & 0x0F);
    GPIOA->MODER &= ~(3U << (p * 2));
    GPIOA->PUPDR &= ~(3U << (p * 2));

    if ((GPIOA->IDR >> p) & 1U) {
        discharge_socket(40);
        GPIOA->MODER &= ~(3U << (p * 2));
        if ((GPIOA->IDR >> p) & 1U) {
            release_probes();
            return false;
        }
    }

    // Phase 1: Small/Medium caps (25 pF .. 1 uF) charged via RH (470k)
    pinMode(probe_a.pin_rh, OUTPUT);
    digitalWrite(probe_a.pin_rh, HIGH);
    uint32_t t_start = micros();

    while (!((GPIOA->IDR >> p) & 1U)) {
        if ((micros() - t_start) > 220000UL) break; // 220 ms timeout
    }
    uint32_t t_elapsed = micros() - t_start;
    pinMode(probe_a.pin_rh, INPUT);

    if (t_elapsed < 220000UL) {
        // Open socket stray flips in ~10 us
        if (t_elapsed < 20) {
            discharge_socket(15);
            return false;
        }
        // Verify voltage hold after 600 us (real caps hold charge)
        delayMicroseconds(600);
        uint32_t adc_hold = adc_read_value(probe_a.adc_pin, 12);
        if (adc_hold < 800) {
            // Dropped significantly -> resistor/leaky
            discharge_socket(20);
            return false;
        }

        // C_pF = (t_us * 1000) / 470
        uint32_t cap_pf = (t_elapsed * 1000UL) / 470UL;
        if (cap_pf > 30) {
            uint32_t cap_eff = (cap_pf > 25) ? (cap_pf - 25) : cap_pf; // Subtract baseline stray
            strcpy(comp_res.type, "CAPACITOR");
            if (cap_eff < 1000) {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu pF", cap_eff);
            } else {
                uint32_t nf_x10 = (cap_eff + 50) / 100;
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%lu nF", nf_x10 / 10, nf_x10 % 10);
            }
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", probe_a.idx, probe_b.idx);
            rtt.printf(">> [COMPONENT DETECTED] CAPACITOR: %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            discharge_socket(30);
            return true;
        }
    }

    // Phase 2: Large caps (> 0.5 uF .. 5000 uF) charged via RL (680R)
    discharge_socket(80);
    pinMode(probe_b.pin_rl, OUTPUT);
    digitalWrite(probe_b.pin_rl, LOW);

    GPIOA->MODER &= ~(3U << (p * 2));
    GPIOA->PUPDR &= ~(3U << (p * 2));

    pinMode(probe_a.pin_rl, OUTPUT);
    digitalWrite(probe_a.pin_rl, HIGH);
    t_start = micros();

    while (!((GPIOA->IDR >> p) & 1U)) {
        if ((micros() - t_start) > 3500000UL) break; // 3.5s timeout (~5000 uF)
    }
    t_elapsed = micros() - t_start;
    pinMode(probe_a.pin_rl, INPUT);

    if (t_elapsed > 200 && t_elapsed < 3500000UL) {
        // C_nF = (t_us) / 680
        uint32_t cap_nf = t_elapsed / 680UL;
        if (cap_nf >= 100) {
            strcpy(comp_res.type, "CAPACITOR");
            if (cap_nf < 1000) {
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu nF", cap_nf);
            } else {
                uint32_t uf_x10 = (cap_nf + 50) / 100;
                snprintf(comp_res.value, sizeof(comp_res.value), "%lu.%lu uF", uf_x10 / 10, uf_x10 % 10);
            }
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "Pins %d - %d", probe_a.idx, probe_b.idx);
            rtt.printf(">> [COMPONENT DETECTED] CAPACITOR: %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            discharge_socket(60);
            return true;
        }
    }

    discharge_socket(30);
    return false;
}

// ============================================================================
// 1-WIRE DIGITAL TEMPERATURE SENSOR ENGINE (DS18B20 / DS18S20 / DS1822)
// ============================================================================
static void onewire_set_low(uint32_t dq_pin) {
    pinMode(dq_pin, OUTPUT);
    digitalWrite(dq_pin, LOW);
}

static void onewire_release(uint32_t dq_pin) {
    pinMode(dq_pin, INPUT_PULLUP);
}

static bool onewire_reset_pulse(uint32_t dq_pin) {
    onewire_release(dq_pin);
    delayMicroseconds(20);
    if (digitalRead(dq_pin) == LOW) return false;

    onewire_set_low(dq_pin);
    delayMicroseconds(480);
    onewire_release(dq_pin);
    delayMicroseconds(70);

    uint8_t presence = digitalRead(dq_pin);
    delayMicroseconds(410);

    if (digitalRead(dq_pin) == LOW) return false;
    return (presence == LOW);
}

static void onewire_write_bit(uint32_t dq_pin, uint8_t b) {
    if (b) {
        onewire_set_low(dq_pin);
        delayMicroseconds(6);
        onewire_release(dq_pin);
        delayMicroseconds(64);
    } else {
        onewire_set_low(dq_pin);
        delayMicroseconds(60);
        onewire_release(dq_pin);
        delayMicroseconds(10);
    }
}

static uint8_t onewire_read_bit(uint32_t dq_pin) {
    onewire_set_low(dq_pin);
    delayMicroseconds(5);
    onewire_release(dq_pin);
    delayMicroseconds(10);
    uint8_t b = digitalRead(dq_pin);
    delayMicroseconds(55);
    return b;
}

static void onewire_write_byte(uint32_t dq_pin, uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        onewire_write_bit(dq_pin, (byte >> i) & 1);
    }
}

static uint8_t onewire_read_byte(uint32_t dq_pin) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (onewire_read_bit(dq_pin)) {
            byte |= (1 << i);
        }
    }
    return byte;
}

static uint8_t onewire_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        uint8_t inbyte = *data++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static bool probe_1wire_device(uint8_t gnd_idx, uint8_t dq_idx, uint8_t vdd_idx) {
    // Power pins: GND = LOW, VDD = HIGH
    pinMode(PROBES[gnd_idx].pin_rl, OUTPUT);
    digitalWrite(PROBES[gnd_idx].pin_rl, LOW);
    pinMode(PROBES[gnd_idx].pin_direct, OUTPUT);
    digitalWrite(PROBES[gnd_idx].pin_direct, LOW);

    pinMode(PROBES[vdd_idx].pin_rl, OUTPUT);
    digitalWrite(PROBES[vdd_idx].pin_rl, HIGH);
    pinMode(PROBES[vdd_idx].pin_direct, OUTPUT);
    digitalWrite(PROBES[vdd_idx].pin_direct, HIGH);

    uint32_t dq_pin = PROBES[dq_idx].pin_direct;
    pinMode(PROBES[dq_idx].pin_rl, INPUT_PULLUP);
    pinMode(PROBES[dq_idx].pin_rh, INPUT_PULLUP);
    onewire_release(dq_pin);

    delay(30); // Power-on settle for DS18B20 internal logic (25-30ms required)

    if (!onewire_reset_pulse(dq_pin)) {
        release_probes();
        return false;
    }

    // Read ROM (0x33)
    onewire_write_byte(dq_pin, 0x33);
    uint8_t rom[8];
    for (int i = 0; i < 8; i++) {
        rom[i] = onewire_read_byte(dq_pin);
    }

    if (onewire_crc8(rom, 7) != rom[7] || rom[0] == 0x00 || rom[0] == 0xFF) {
        release_probes();
        return false;
    }

    if (rom[0] == 0x28 || rom[0] == 0x10 || rom[0] == 0x22) {
        if (!onewire_reset_pulse(dq_pin)) { release_probes(); return false; }
        onewire_write_byte(dq_pin, 0xCC);
        onewire_write_byte(dq_pin, 0x44);

        uint32_t t_start = millis();
        while (onewire_read_bit(dq_pin) == 0) {
            if (millis() - t_start > 150) break;
        }

        if (!onewire_reset_pulse(dq_pin)) { release_probes(); return false; }
        onewire_write_byte(dq_pin, 0xCC);
        onewire_write_byte(dq_pin, 0xBE);

        uint8_t sp[9];
        for (int i = 0; i < 9; i++) {
            sp[i] = onewire_read_byte(dq_pin);
        }

        if (onewire_crc8(sp, 8) == sp[8]) {
            int16_t raw_temp = (int16_t)((sp[1] << 8) | sp[0]);
            float tc = 0;
            if (rom[0] == 0x10) {
                tc = (float)(raw_temp >> 1) - 0.25f + ((float)(sp[7] - sp[6]) / (float)(sp[7] ? sp[7] : 1));
            } else {
                tc = (float)raw_temp * 0.0625f;
            }
            float tf = tc * 1.8f + 32.0f;

            strcpy(comp_res.type, (rom[0] == 0x28) ? "DS18B20" : (rom[0] == 0x10) ? "DS18S20" : "DS1822");
            snprintf(comp_res.value, sizeof(comp_res.value), "%d.%01dC (%dF)", 
                     (int)tc, abs((int)(tc * 10) % 10), (int)tf);
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d:GND %d:DQ %d:VDD",
                     PROBES[gnd_idx].idx, PROBES[dq_idx].idx, PROBES[vdd_idx].idx);

            rtt.printf(">> [1-WIRE DETECTED] %s: %s | %s | ROM: %02X-%02X%02X%02X%02X%02X%02X-%02X\n",
                       comp_res.type, comp_res.value, comp_res.pinout,
                       rom[0], rom[6], rom[5], rom[4], rom[3], rom[2], rom[1], rom[7]);

            test_success_chime();
            release_probes();
            return true;
        }
    }

    strcpy(comp_res.type, "1-WIRE");
    snprintf(comp_res.value, sizeof(comp_res.value), "ROM:%02X%02X%02X%02X", rom[0], rom[1], rom[2], rom[3]);
    snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d:GND %d:DQ %d:VDD",
             PROBES[gnd_idx].idx, PROBES[dq_idx].idx, PROBES[vdd_idx].idx);
    rtt.printf(">> [1-WIRE] Generic 1-Wire device found! %s\n", comp_res.value);
    test_success_chime();
    release_probes();
    return true;
}

// ============================================================================
// DHT11 / DHT22 SENSOR ENGINE
// ============================================================================
static bool probe_dht_sensor(uint8_t vdd_idx, uint8_t dat_idx, uint8_t gnd_idx) {
    pinMode(PROBES[vdd_idx].pin_rl, OUTPUT); digitalWrite(PROBES[vdd_idx].pin_rl, HIGH);
    pinMode(PROBES[vdd_idx].pin_direct, OUTPUT); digitalWrite(PROBES[vdd_idx].pin_direct, HIGH);

    pinMode(PROBES[gnd_idx].pin_rl, OUTPUT); digitalWrite(PROBES[gnd_idx].pin_rl, LOW);
    pinMode(PROBES[gnd_idx].pin_direct, OUTPUT); digitalWrite(PROBES[gnd_idx].pin_direct, LOW);

    uint32_t dat_pin = PROBES[dat_idx].pin_direct;
    pinMode(PROBES[dat_idx].pin_rl, INPUT);
    pinMode(PROBES[dat_idx].pin_rh, INPUT);
    pinMode(dat_pin, INPUT_PULLUP);
    delay(20);

    if (digitalRead(dat_pin) == LOW) {
        release_probes();
        return false;
    }

    pinMode(dat_pin, OUTPUT);
    digitalWrite(dat_pin, LOW);
    delay(20);
    pinMode(dat_pin, INPUT_PULLUP);
    delayMicroseconds(35);

    if (digitalRead(dat_pin) != LOW) {
        release_probes();
        return false;
    }

    uint32_t to = micros();
    while (digitalRead(dat_pin) == LOW) {
        if (micros() - to > 120) { release_probes(); return false; }
    }
    to = micros();
    while (digitalRead(dat_pin) == HIGH) {
        if (micros() - to > 120) { release_probes(); return false; }
    }

    uint8_t data[5] = {0};
    for (int i = 0; i < 40; i++) {
        to = micros();
        while (digitalRead(dat_pin) == LOW) {
            if (micros() - to > 80) { release_probes(); return false; }
        }
        uint32_t t_start = micros();
        to = micros();
        while (digitalRead(dat_pin) == HIGH) {
            if (micros() - to > 100) { release_probes(); return false; }
        }
        uint32_t pulse_len = micros() - t_start;
        if (pulse_len > 45) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4] || (data[0] == 0 && data[2] == 0)) {
        release_probes();
        return false;
    }

    strcpy(comp_res.type, "DHT11");
    snprintf(comp_res.value, sizeof(comp_res.value), "H:%u%% T:%u C", data[0], data[2]);
    snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d:VDD %d:DAT %d:GND",
             PROBES[vdd_idx].idx, PROBES[dat_idx].idx, PROBES[gnd_idx].idx);
    rtt.printf(">> [DHT DETECTED] DHT11: %s (%s)\n", comp_res.value, comp_res.pinout);

    test_success_chime();
    release_probes();
    return true;
}

static void run_component_test() {
    rtt.println(">> [TESTER] Running full component test (BJT, MOSFET, Diode, Resistor, Capacitor, 1-Wire)...");
    beep_square(3200, 30);

    uint32_t vcc = read_vcc_mv();

    // Read resting voltages
    comp_res.s1_mv = (adc_read_value(PA_3, 12) * vcc) / 4095;
    comp_res.s2_mv = (adc_read_value(PA_4, 12) * vcc) / 4095;
    comp_res.s3_mv = (adc_read_value(PA_5, 12) * vcc) / 4095;

    // ------------------------------------------------------------------------
    // STAGE 0: PROBE DIGITAL SENSORS (DS18B20 1-WIRE & DHT11 / DHT22)
    // Scan all 6 permutations across socket terminals 1-2-3
    // ------------------------------------------------------------------------
    static const uint8_t SENSOR_PERMS[6][3] = {
        {0, 1, 2}, // 1:GND 2:DQ  3:VDD
        {2, 1, 0}, // 3:GND 2:DQ  1:VDD
        {1, 0, 2}, // 2:GND 1:DQ  3:VDD
        {1, 2, 0}, // 2:GND 3:DQ  1:VDD
        {0, 2, 1}, // 1:GND 3:DQ  2:VDD
        {2, 0, 1}  // 3:GND 1:DQ  2:VDD
    };

    rtt.println(">> [TESTER] Probing 1-Wire DS18B20 across 6 socket permutations...");
    for (uint8_t p = 0; p < 6; p++) {
        if (probe_1wire_device(SENSOR_PERMS[p][0], SENSOR_PERMS[p][1], SENSOR_PERMS[p][2])) {
            release_probes();
            return;
        }
    }

    rtt.println(">> [TESTER] Probing DHT11 / DHT22 across 6 socket permutations...");
    for (uint8_t p = 0; p < 6; p++) {
        if (probe_dht_sensor(SENSOR_PERMS[p][0], SENSOR_PERMS[p][1], SENSOR_PERMS[p][2])) {
            release_probes();
            return;
        }
    }

    // ------------------------------------------------------------------------
    // STAGE 1: SCAN ALL 6 DIODE COMBINATIONS (Pairs 1-2, 2-3, 1-3 in both directions)
    // ------------------------------------------------------------------------
    DiodeDetection found_diodes[6];
    uint8_t num_diodes = 0;

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            if (i == j) continue;
            uint16_t vf = probe_diode_direction(i, j, vcc);
            if (vf > 0) {
                found_diodes[num_diodes++] = {PROBES[i].idx, PROBES[j].idx, vf};
            }
        }
    }

    // ------------------------------------------------------------------------
    // STAGE 2: ANALYZE DIODE JUNCTIONS (NPN, PNP, DUAL DIODE, MOSFET, SINGLE DIODE)
    // ------------------------------------------------------------------------
    if (num_diodes == 2) {
        // Case A: Common Anode -> NPN Transistor!
        if (found_diodes[0].a == found_diodes[1].a) {
            uint8_t base = found_diodes[0].a;
            uint8_t em = (found_diodes[0].vf_mv >= found_diodes[1].vf_mv) ? found_diodes[0].k : found_diodes[1].k;
            uint8_t col = (em == found_diodes[0].k) ? found_diodes[1].k : found_diodes[0].k;
            uint32_t avg_vf = (found_diodes[0].vf_mv + found_diodes[1].vf_mv) / 2;

            strcpy(comp_res.type, "NPN");
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "B:%d C:%d E:%d", base, col, em);
            snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", avg_vf / 1000, (avg_vf % 1000) / 10);
            rtt.printf(">> [COMPONENT DETECTED] NPN BJT: %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            release_probes();
            return;
        }

        // Case B: Common Cathode -> PNP Transistor!
        if (found_diodes[0].k == found_diodes[1].k) {
            uint8_t base = found_diodes[0].k;
            uint8_t em = (found_diodes[0].vf_mv >= found_diodes[1].vf_mv) ? found_diodes[0].a : found_diodes[1].a;
            uint8_t col = (em == found_diodes[0].a) ? found_diodes[1].a : found_diodes[0].a;
            uint32_t avg_vf = (found_diodes[0].vf_mv + found_diodes[1].vf_mv) / 2;

            strcpy(comp_res.type, "PNP");
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "B:%d C:%d E:%d", base, col, em);
            snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", avg_vf / 1000, (avg_vf % 1000) / 10);
            rtt.printf(">> [COMPONENT DETECTED] PNP BJT: %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            release_probes();
            return;
        }

        // Dual Diode (Common terminal or series)
        strcpy(comp_res.type, "DIODE");
        snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d->%d & %d->%d",
                 found_diodes[0].a, found_diodes[0].k, found_diodes[1].a, found_diodes[1].k);
        snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%u,%umV", found_diodes[0].vf_mv, found_diodes[1].vf_mv);
        rtt.printf(">> [COMPONENT DETECTED] DUAL DIODE: %s (%s)\n", comp_res.value, comp_res.pinout);
        test_success_chime();
        release_probes();
        return;

    } else if (num_diodes == 1) {
        // Exactly 1 Diode found. Check if third pin acts as a MOSFET Gate!
        uint8_t da = found_diodes[0].a;
        uint8_t dk = found_diodes[0].k;
        uint8_t gate = 6 - da - dk; // The remaining pin (1+2+3 = 6)
        uint8_t g_idx = gate - 1;
        uint8_t da_idx = da - 1;
        uint8_t dk_idx = dk - 1;

        // Test N-MOSFET (Body Diode: Source=da, Drain=dk, Gate=gate)
        // 1. Discharge Gate to 0V
        pinMode(PROBES[g_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[g_idx].pin_rl, LOW);
        delay(5);
        pinMode(PROBES[g_idx].pin_rl, INPUT); // High-Z gate

        // Measure baseline V_DS off
        pinMode(PROBES[dk_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[dk_idx].pin_rl, HIGH);
        pinMode(PROBES[da_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[da_idx].pin_rl, LOW);
        delayMicroseconds(300);
        uint32_t vs_off = (adc_read_value(PROBES[da_idx].adc_pin, 12) * vcc) / 4095;
        release_probes();

        // 2. Charge Gate to 3.3V
        pinMode(PROBES[g_idx].pin_rh, OUTPUT);
        digitalWrite(PROBES[g_idx].pin_rh, HIGH);
        delay(8);
        pinMode(PROBES[g_idx].pin_rh, INPUT); // Keep Gate charged

        // Measure V_DS on
        pinMode(PROBES[dk_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[dk_idx].pin_rl, HIGH);
        pinMode(PROBES[da_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[da_idx].pin_rl, LOW);
        delayMicroseconds(300);
        uint32_t vs_on = (adc_read_value(PROBES[da_idx].adc_pin, 12) * vcc) / 4095;
        release_probes();

        // If Gate charge increased Source conduction (vs_on > vs_off + 250 mV):
        if (vs_on > vs_off + 250) {
            // Discharge gate to confirm it turns off
            pinMode(PROBES[g_idx].pin_rl, OUTPUT);
            digitalWrite(PROBES[g_idx].pin_rl, LOW);
            delay(5);
            release_probes();

            strcpy(comp_res.type, "N-MOS");
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "G:%d D:%d S:%d", gate, dk, da);
            snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", 
                     (uint32_t)found_diodes[0].vf_mv / 1000, ((uint32_t)found_diodes[0].vf_mv % 1000) / 10);
            rtt.printf(">> [COMPONENT DETECTED] N-CH MOSFET: %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            return;
        }

        // Test P-MOSFET (Body Diode: Drain=da, Source=dk, Gate=gate)
        // Charge gate HIGH -> off, pull gate LOW -> on
        pinMode(PROBES[g_idx].pin_rh, OUTPUT);
        digitalWrite(PROBES[g_idx].pin_rh, HIGH);
        delay(8);
        pinMode(PROBES[g_idx].pin_rh, INPUT);

        pinMode(PROBES[da_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[da_idx].pin_rl, HIGH);
        pinMode(PROBES[dk_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[dk_idx].pin_rl, LOW);
        delayMicroseconds(300);
        uint32_t vk_off = (adc_read_value(PROBES[dk_idx].adc_pin, 12) * vcc) / 4095;
        release_probes();

        // Pull gate to 0V
        pinMode(PROBES[g_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[g_idx].pin_rl, LOW);
        delay(5);
        pinMode(PROBES[g_idx].pin_rl, INPUT);

        pinMode(PROBES[da_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[da_idx].pin_rl, HIGH);
        pinMode(PROBES[dk_idx].pin_rl, OUTPUT);
        digitalWrite(PROBES[dk_idx].pin_rl, LOW);
        delayMicroseconds(300);
        uint32_t vk_on = (adc_read_value(PROBES[dk_idx].adc_pin, 12) * vcc) / 4095;
        release_probes();

        if (vk_on > vk_off + 250) {
            strcpy(comp_res.type, "P-MOS");
            snprintf(comp_res.pinout, sizeof(comp_res.pinout), "G:%d D:%d S:%d", gate, da, dk);
            snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", 
                     (uint32_t)found_diodes[0].vf_mv / 1000, ((uint32_t)found_diodes[0].vf_mv % 1000) / 10);
            rtt.printf(">> [COMPONENT DETECTED] P-CH MOSFET: %s (%s)\n", comp_res.value, comp_res.pinout);
            test_success_chime();
            return;
        }

        // Pure Single Diode
        strcpy(comp_res.type, "DIODE");
        snprintf(comp_res.pinout, sizeof(comp_res.pinout), "%d(A) -> %d(K)", da, dk);
        snprintf(comp_res.value, sizeof(comp_res.value), "Vf=%lu.%02luV", 
                 (uint32_t)found_diodes[0].vf_mv / 1000, ((uint32_t)found_diodes[0].vf_mv % 1000) / 10);
        rtt.printf(">> [COMPONENT DETECTED] DIODE: %s (%s)\n", comp_res.value, comp_res.pinout);
        test_success_chime();
        release_probes();
        return;
    }

    // ------------------------------------------------------------------------
    // STAGE 3: TEST RESISTORS & CAPACITORS ACROSS ALL 3 PAIRS (1-2, 2-3, 1-3)
    // ------------------------------------------------------------------------
    static const uint8_t PAIRS[3][2] = {{0, 1}, {1, 2}, {0, 2}};

    // Test for Resistors first
    for (uint8_t p = 0; p < 3; p++) {
        if (test_resistor(PAIRS[p][0], PAIRS[p][1], vcc)) {
            release_probes();
            return;
        }
    }

    // Test for Capacitors
    for (uint8_t p = 0; p < 3; p++) {
        if (test_capacitance(PAIRS[p][0], PAIRS[p][1], vcc)) {
            release_probes();
            return;
        }
    }

    // No component found
    strcpy(comp_res.type, "NONE");
    strcpy(comp_res.value, "No component");
    strcpy(comp_res.pinout, "Pins 1-2-3 Open");
    rtt.println(">> [TESTER] Result: No component detected in 1-2-3 socket.");
    release_probes();
}

static void zener_init_probe() {
    uint32_t vcc = read_vcc_mv();
    rtt.println(">> [ZENER PROBE] Scanning candidate ADC & MT3608 Boost EN pins...");

    uint16_t baseline[NUM_ZENER_ADC];
    PinName best_adc = PA_0;
    uint16_t max_mv = 0;

    for (size_t i = 0; i < NUM_ZENER_ADC; i++) {
        uint32_t raw = adc_read_value(CANDIDATE_ZENER_ADC[i], 12);
        baseline[i] = (raw * vcc) / 4095;
        rtt.printf("   ADC Pin idx %u: %u mV\n", (unsigned int)i, baseline[i]);
        if (baseline[i] > max_mv) {
            max_mv = baseline[i];
            best_adc = CANDIDATE_ZENER_ADC[i];
        }
    }

    if (max_mv > 1000) {
        zener_res.adc_pin = best_adc;
        zener_res.en_pin = 0xFF;
        rtt.printf(">> [ZENER PROBE] Boost rail live! Detected Zener ADC pin with %u mV\n", max_mv);
        return;
    }

    for (size_t e = 0; e < NUM_ZENER_EN; e++) {
        uint32_t en_pin = CANDIDATE_ZENER_EN[e];
        pinMode(en_pin, OUTPUT);
        digitalWrite(en_pin, HIGH);
        delay(6);

        for (size_t i = 0; i < NUM_ZENER_ADC; i++) {
            uint32_t raw = adc_read_value(CANDIDATE_ZENER_ADC[i], 12);
            uint16_t mv = (raw * vcc) / 4095;
            if (mv > baseline[i] + 400 && mv > 1000) {
                zener_res.adc_pin = CANDIDATE_ZENER_ADC[i];
                zener_res.en_pin = en_pin;
                rtt.printf(">> [ZENER PROBE] FOUND! EN Pin: 0x%lX | ADC Pin idx: %u (%u mV)\n", en_pin, (unsigned int)i, mv);
                return;
            }
        }
        digitalWrite(en_pin, LOW);
        pinMode(en_pin, INPUT);
    }

    zener_res.adc_pin = PA_0;
    zener_res.en_pin = 0xFF;
    rtt.println(">> [ZENER PROBE] Defaulting to PA_0 (Pin 14) for Zener sense.");
}

static void sample_zener() {
    if (zener_res.en_pin != 0xFF) {
        pinMode(zener_res.en_pin, OUTPUT);
        digitalWrite(zener_res.en_pin, HIGH);
        delay(4);
    }

    uint32_t vcc = read_vcc_mv();
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_read_value(zener_res.adc_pin, 12);
        delayMicroseconds(60);
    }
    uint32_t raw = sum / 16;
    uint32_t adc_mv = (raw * vcc) / 4095;

    // Resistor divider scale (ratio ~10.9x: 100k+10k / 10k)
    uint32_t vz_mv = (adc_mv * 109) / 10;
    zener_res.zener_mv = vz_mv;

    if (vz_mv > 24000) {
        strncpy((char*)zener_res.status, "OPEN (>28V)", sizeof(zener_res.status) - 1);
    } else if (vz_mv >= 1000) {
        snprintf((char*)zener_res.status, sizeof(zener_res.status), "Vz = %.2f V", vz_mv / 1000.0f);
    } else if (vz_mv >= 350) {
        snprintf((char*)zener_res.status, sizeof(zener_res.status), "Vf = %.2f V (FWD)", vz_mv / 1000.0f);
    } else {
        strncpy((char*)zener_res.status, "SHORT CIRCUIT", sizeof(zener_res.status) - 1);
    }
    zener_res.active = true;
}

void setup() {
    // 1. Latch battery power ON
    pinMode(PIN_PWR_HOLD, OUTPUT);
    digitalWrite(PIN_PWR_HOLD, HIGH);

    // Initialize RAM vector table for ultra-fast bare-metal timer interrupt
    init_ram_vectors();

    // 2. LCD Backlight
    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);

    // 3. Buzzer
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    // 4. Oscilloscope Relay
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);

    // 5. Infrared Receiver (Demodulated active LOW on PB9)
    pinMode(PIN_IR_RX, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_IR_RX), ir_exti_isr, FALLING);

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

    // Ensure USART2 clock is disabled so PA2 and PA3 are purely ADC
    if (__HAL_RCC_USART2_IS_CLK_ENABLED()) {
        __HAL_RCC_USART2_CLK_DISABLE();
    }
    pinMode(PIN_BATT_ADC, INPUT_ANALOG);
    pinMode(PIN_SMA_ADC, INPUT_ANALOG);

    // Initialize Zener boost & ADC sensor
    zener_init_probe();
    sample_zener();

    // Probe ALL candidate analog pins on STM32L083 to identify the battery divider
    uint32_t vcc_boot = read_vcc_mv();
    static const PinName PROBE_PINS[] = {
        PA_0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7,
        PB_0, PB_1, PC_0, PC_1, PC_2, PC_3, PC_4, PC_5
    };
    static const char* const PROBE_NAMES[] = {
        "PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7",
        "PB0", "PB1", "PC0", "PC1", "PC2", "PC3", "PC4", "PC5"
    };
    rtt.println(">> [ADC PIN SCAN] Probing all analog pins for Battery Divider:");
    for (size_t i = 0; i < sizeof(PROBE_PINS)/sizeof(PROBE_PINS[0]); i++) {
        uint32_t r = adc_read_value(PROBE_PINS[i], 12);
        uint32_t mv = (r * vcc_boot) / 4095;
        rtt.printf("   %s: %lu mV (raw %lu)\n", PROBE_NAMES[i], mv, r);
    }

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 COMPLETE HARDWARE ENGINE ONLINE       ");
    rtt.println("   SOLID 38,400 BAUD TELEMETRY LINK ACTIVE (RS0102)     ");
    rtt.println("   ZENER HIGH-VOLTAGE & COMPONENT TESTER READY          ");
    rtt.println("========================================================\n");
}

// ============================================================================
// INDUSTRIAL-GRADE BUTTON DEBOUNCER (40 ms integrator / contact-chatter lockout)
// ============================================================================
struct ButtonDebouncer {
    bool raw_state;
    bool stable_state;
    bool pressed;   // One-shot edge on stable press
    bool released;  // One-shot edge on stable release
    uint32_t last_transition_ms;

    void update(bool raw, uint32_t now) {
        pressed = false;
        released = false;
        if (raw != raw_state) {
            raw_state = raw;
            last_transition_ms = now;
        }
        if ((now - last_transition_ms) >= 40) { // 40 ms debounce window
            if (raw_state != stable_state) {
                stable_state = raw_state;
                if (stable_state) {
                    pressed = true;
                } else {
                    released = true;
                }
            }
        }
    }
};

static ButtonDebouncer btn_up_db;
static ButtonDebouncer btn_down_db;
static ButtonDebouncer btn_ok_db;
static ButtonDebouncer btn_test_db;
static ButtonDebouncer btn_menu_db;
static ButtonDebouncer btn_pwr_db;

void loop() {
    uint32_t now_ms = millis();

    // Process background IR demodulator pulse train from PB9
    process_ir();

    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    bool raw_up    = !(rc & (1U << 8));   // PC8
    bool raw_down  = !(rc & (1U << 9));   // PC9
    bool raw_ok    = !(rb & (1U << 14));  // PB14: OK / ENTER (<)
    bool raw_test  = !(ra & (1U << 8));   // PA8:  TEST / HOLD (>)
    bool raw_menu  = !(rb & (1U << 15));  // PB15: MENU / AUTO
    bool raw_pwr   = !(rb & (1U << 13));  // PB13: POWER

    btn_up_db.update(raw_up, now_ms);
    btn_down_db.update(raw_down, now_ms);
    btn_ok_db.update(raw_ok, now_ms);
    btn_test_db.update(raw_test, now_ms);
    btn_menu_db.update(raw_menu, now_ms);
    btn_pwr_db.update(raw_pwr, now_ms);

    bool b_up    = btn_up_db.stable_state;
    bool b_down  = btn_down_db.stable_state;
    bool b_ok    = btn_ok_db.stable_state;
    bool b_test  = btn_test_db.stable_state;
    bool b_menu  = btn_menu_db.stable_state;
    bool b_pwr   = btn_pwr_db.stable_state;

    bool test_pressed = btn_test_db.pressed;
    bool up_pressed   = btn_up_db.pressed;
    bool down_pressed = btn_down_db.pressed;
    bool menu_pressed = btn_menu_db.pressed;

    // IR Receiver on PB9 & Decoded Command Handler
    if (ir_data.has_new) {
        ir_data.has_new = false;
        beep_square(4200, 25); // crisp high confirmation chime
        rtt.printf(">> [IR DECODE] Proto: %s | Addr: 0x%04X | Cmd: 0x%02X | Raw: 0x%08lX | Rpt: %lu\n",
                   ir_data.protocol, ir_data.address, ir_data.command, ir_data.raw_code, ir_data.repeat_count);

        char ir_pkt[96];
        snprintf(ir_pkt, sizeof(ir_pkt), "#IR:%s,%04X,%02X,%08lX,%lu\n",
                 ir_data.protocol, ir_data.address, ir_data.command, ir_data.raw_code, ir_data.repeat_count);
#if defined(USART1)
        SerialUART1.print(ir_pkt);
#endif
    }
    bool ir_active = (millis() - ir_data.last_rx_ms < 400) || (micros() - ir_last_edge_us < 350000UL);

    // Relay Toggle: Hold MENU and press TEST (PA8)
    static uint32_t last_relay_toggle = 0;
    if (b_menu && test_pressed && (now_ms - last_relay_toggle > 400)) {
        last_relay_toggle = now_ms;
        relay_state = !relay_state;
        digitalWrite(PIN_RELAY, relay_state ? HIGH : LOW);
        beep_square(relay_state ? 4000 : 1600, 30);
        rtt.printf(">> [RELAY PB6] Toggled to: %s\n", relay_state ? "ON (HIGH)" : "OFF (LOW)");
    }

    // Backlight Brightness: Hold MENU and press UP/DOWN
    static uint32_t last_bl_change = 0;
    if (b_menu && (now_ms - last_bl_change > 150)) {
        if (up_pressed && backlight_level < 255) {
            backlight_level = (backlight_level > 230) ? 255 : (backlight_level + 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = now_ms;
            beep_square(3600, 15);
        } else if (down_pressed && backlight_level > 30) {
            backlight_level = (backlight_level < 55) ? 30 : (backlight_level - 25);
            analogWrite(PIN_BACKLIGHT, backlight_level);
            last_bl_change = now_ms;
            beep_square(1800, 15);
        }
    }

    // Menu navigation: Switch instruments (0 = Tester, 1 = IR Decoder, 2 = Zener, 3 = SigGen & Voltmeter)
    // Only trigger if MENU alone was pressed (not held as a modifier for relay or backlight)
    if (menu_pressed && !b_test && !b_up && !b_down) {
        instrument_mode = (instrument_mode + 1) % 4;
        beep_square(3200, 20);
        rtt.printf(">> [INSTRUMENT SWITCH] Mode: %u\n", instrument_mode);
    }

    // Instrument-specific actions for TEST, UP, DOWN
    if (!b_menu) {
        if (instrument_mode == 0) { // TESTER
            if (test_pressed) {
                run_component_test();
                sample_zener();
            }
        } else if (instrument_mode == 1) { // IR DECODER
            if (test_pressed) {
                ir_data.has_new = false;
                ir_data.raw_code = 0;
                ir_data.repeat_count = 0;
                beep_square(2400, 20);
                rtt.println(">> [IR DECODER] Cleared buffer on TEST press.");
            }
        } else if (instrument_mode == 2) { // ZENER
            if (test_pressed) {
                sample_zener();
                beep_square(2800, 25);
                rtt.printf(">> [ZENER] Sample triggered: %lu mV (%s)\n", zener_res.zener_mv, zener_res.status);
            }
        } else if (instrument_mode == 3) { // SIGGEN & VOLTMETER
            if (test_pressed) {
                siggen_on = !siggen_on;
                update_siggen();
                beep_square(siggen_on ? 3600 : 1800, 30);
                rtt.printf(">> [SIGGEN PB8] Output: %s (%lu Hz, WaveMode: %u)\n",
                           siggen_on ? "ON" : "OFF", SIGGEN_PRESETS[siggen_idx], siggen_wave_mode);
            }
            if (btn_ok_db.pressed) {
                siggen_wave_mode = (siggen_wave_mode + 1) % NUM_WAVE_MODES;
                if (siggen_on) update_siggen();
                beep_square(2800, 20);
                rtt.printf(">> [SIGGEN PB8] Waveform changed to mode: %u\n", siggen_wave_mode);
            }
            if (up_pressed) {
                if (siggen_idx < NUM_SIGGEN_PRESETS - 1) {
                    siggen_idx++;
                    if (siggen_on) update_siggen();
                    beep_square(3200, 15);
                    rtt.printf(">> [SIGGEN PB8] Frequency UP -> %lu Hz\n", SIGGEN_PRESETS[siggen_idx]);
                }
            }
            if (down_pressed) {
                if (siggen_idx > 0) {
                    siggen_idx--;
                    if (siggen_on) update_siggen();
                    beep_square(2200, 15);
                    rtt.printf(">> [SIGGEN PB8] Frequency DOWN -> %lu Hz\n", SIGGEN_PRESETS[siggen_idx]);
                }
            }
        }
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

    // 5-minute Auto Power-Off (Battery Saver)
    static uint32_t last_activity_time = 0;
    if (last_activity_time == 0) last_activity_time = millis();
    if (b_up || b_down || b_ok || b_test || b_menu || b_pwr || ir_active) {
        last_activity_time = millis();
    }
    if (millis() - last_activity_time > 300000UL) {
        rtt.println(">> [AUTO POWER-OFF] Inactivity timeout (5 min). Conserving Li-Po battery...");
        beep_square(3000, 60);
        delay(30);
        beep_square(2000, 80);
        delay(30);
        beep_square(1000, 120);
        digitalWrite(PIN_PWR_HOLD, LOW);
        digitalWrite(PIN_BACKLIGHT, LOW);
        while (1) {
            digitalWrite(PIN_PWR_HOLD, LOW);
            delay(100);
        }
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

    // Debounced Key Click Audio & Event Flag
    bool any_pressed = btn_up_db.pressed || btn_down_db.pressed || btn_ok_db.pressed ||
                       btn_test_db.pressed || btn_menu_db.pressed || btn_pwr_db.pressed;
    bool any_released = btn_up_db.released || btn_down_db.released || btn_ok_db.released ||
                        btn_test_db.released || btn_menu_db.released || btn_pwr_db.released;
    bool key_changed = any_pressed || any_released;

    if (any_pressed) {
        beep_click();
        rtt.printf(">> [KEY EVENT] ACTIVE: %s (PIN: %s, MASK: 0x%02X)\n",
                   active_btn, active_pin, btn_mask);
    }

    // Read live socket resting voltages periodically
    static uint32_t last_volt_read = 0;
    if (millis() - last_volt_read >= 250) {
        last_volt_read = millis();
        uint32_t vcc = read_vcc_mv();
        comp_res.s1_mv = (adc_read_value(PA_3, 12) * vcc) / 4095;
        comp_res.s2_mv = (adc_read_value(PA_4, 12) * vcc) / 4095;
        comp_res.s3_mv = (adc_read_value(PA_5, 12) * vcc) / 4095;
        sample_zener();
    }

    // Periodic IR Activity Log to RTT (shows raw pulses detected on PB9)
    static uint32_t last_ir_rtt_report = 0;
    static uint32_t prev_ir_total = 0;
    if (ir_edge_count != prev_ir_total && (millis() - last_ir_rtt_report >= 500)) {
        last_ir_rtt_report = millis();
        rtt.printf(">> [IR PULSE ACTIVITY] PB9 saw %lu new edges (Total: %lu)\n",
                   ir_edge_count - prev_ir_total, ir_edge_count);
        prev_ir_total = ir_edge_count;
    }

    // High-Speed Telemetry: 12.5 Hz periodic (80ms) OR Instant on key change (38,400 baud)
    static uint32_t last_tx = 0;
    static uint32_t seq = 0;

    if (key_changed || (millis() - last_tx >= 80)) {
        last_tx = millis();
        seq++;
        uint32_t vcc = read_vcc_mv();
        uint16_t batt_mv = (uint16_t)sample_battery_mv();
        uint16_t sma_mv  = (uint16_t)sample_sma_mv();

        char buf[224];
        snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%s,%02X,%u,%lu,%lu,%s,%s,%s,%u,%u,%u,%lu,%s,%u,%u,%lu,%u,%u,%u\n",
                 seq, active_btn, active_pin, btn_mask, backlight_level, vcc, millis(),
                 comp_res.type, comp_res.value, comp_res.pinout,
                 comp_res.s1_mv, comp_res.s2_mv, comp_res.s3_mv,
                 zener_res.zener_mv, zener_res.status,
                 batt_mv, sma_mv,
                 SIGGEN_PRESETS[siggen_idx], (unsigned int)(siggen_on ? 1 : 0),
                 (unsigned int)siggen_wave_mode,
                 (unsigned int)instrument_mode);

#if defined(USART1)
        SerialUART1.print(buf);
#endif
    }
}
