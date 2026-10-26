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

// Power hold & peripherals
#define PIN_PWR_HOLD   PB7
#define PIN_BACKLIGHT  PC7
#define PIN_BUZZER     PC6
#define PIN_RELAY      PB6
#define PIN_IR_RX      PB9

// Buttons
#define PIN_BTN_PWR    PB13
#define PIN_BTN_OK     PB14
#define PIN_BTN_MENU   PB15
#define PIN_BTN_UP     PC8
#define PIN_BTN_DOWN   PC9
#define PIN_BTN_TEST   PA8

// Direct PinName definitions for ADC
extern "C" uint16_t adc_read_value(PinName pin, uint32_t resolution);

static uint32_t read_adc(PinName pin) {
    return adc_read_value(pin, 12);
}

// Resistor Mapping storage
struct ResistorMap {
    char rl_name[8];
    char rh_name[8];
};
static ResistorMap socket_map[3] = {
    {"???", "???"},
    {"???", "???"},
    {"???", "???"}
};

struct CandidatePin {
    const char* name;
    uint32_t pin;
};

static const CandidatePin CANDIDATES[] = {
    // Port C
    {"PC0", PC0}, {"PC1", PC1}, {"PC2", PC2}, {"PC3", PC3},
    {"PC4", PC4}, {"PC5", PC5}, {"PC10", PC10}, {"PC11", PC11},
    {"PC12", PC12}, {"PC13", PC13},
    // Port B
    {"PB0", PB0}, {"PB1", PB1}, {"PB2", PB2}, {"PB8", PB8},
    {"PB10", PB10}, {"PB11", PB11}, {"PB12", PB12},
    // Port A
    {"PA0", PA0}, {"PA1", PA1}, {"PA2", PA2}, {"PA6", PA6},
    {"PA7", PA7}, {"PA11", PA11}, {"PA12", PA12}, {"PA15", PA15},
    // Port D
    {"PD2", PD2}
};
static const size_t NUM_CANDIDATES = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

static void scan_resistors() {
    rtt.println("\n========================================================");
    rtt.println("   SCANNING FOR COMPONENT TESTER RESISTOR NETWORKS      ");
    rtt.println("   SOCKET 1: PA3 | SOCKET 2: PA4 | SOCKET 3: PA5       ");
    rtt.println("========================================================");

    // Set all candidates to floating inputs
    for (size_t i = 0; i < NUM_CANDIDATES; i++) {
        pinMode(CANDIDATES[i].pin, INPUT);
    }

    const PinName SOCKS[3] = {PA_3, PA_4, PA_5};

    for (size_t s = 0; s < 3; s++) {
        PinName sock_pin = SOCKS[s];
        const char* sock_str = (s == 0 ? "PA3" : s == 1 ? "PA4" : "PA5");
        rtt.printf("\n--- PROBING SOCKET %d (%s) ---\n", s + 1, sock_str);

        uint32_t best_rl_delta = 0;
        const char* best_rl_name = "???";
        uint32_t best_rh_delta = 0;
        const char* best_rh_name = "???";

        for (size_t i = 0; i < NUM_CANDIDATES; i++) {
            uint32_t p = CANDIDATES[i].pin;
            const char* name = CANDIDATES[i].name;

            // Drive candidate HIGH
            pinMode(p, OUTPUT);
            digitalWrite(p, HIGH);
            delay(3);
            uint32_t val_high = read_adc(sock_pin);

            // Drive candidate LOW
            digitalWrite(p, LOW);
            delay(3);
            uint32_t val_low = read_adc(sock_pin);

            // Restore candidate to floating INPUT
            pinMode(p, INPUT);

            int32_t delta = (int32_t)val_high - (int32_t)val_low;

            if (delta > 80) {
                rtt.printf("   Candidate %-5s -> HIGH=%4lu, LOW=%4lu, DELTA=+%4ld\n",
                           name, val_high, val_low, delta);

                if (delta > 2500) {
                    if ((uint32_t)delta > best_rl_delta) {
                        best_rl_delta = (uint32_t)delta;
                        best_rl_name = name;
                    }
                } else if (delta > 150 && delta < 2200) {
                    if ((uint32_t)delta > best_rh_delta) {
                        best_rh_delta = (uint32_t)delta;
                        best_rh_name = name;
                    }
                }
            }
        }

        strncpy(socket_map[s].rl_name, best_rl_name, sizeof(socket_map[s].rl_name) - 1);
        strncpy(socket_map[s].rh_name, best_rh_name, sizeof(socket_map[s].rh_name) - 1);
        rtt.printf(">> SOCKET %d (%s) RESULT: RL (~680 ohm) = %s, RH (~470k) = %s\n",
                   s + 1, sock_str, best_rl_name, best_rh_name);
    }

    rtt.println("\n--------------------------------------------------------");
    rtt.println("   COMPONENT TESTER RESISTOR MAPPING SUMMARY:           ");
    for (int s = 0; s < 3; s++) {
        rtt.printf("   Socket %d (PA%d): RL=%-5s  RH=%-5s\n",
                   s + 1, s + 3, socket_map[s].rl_name, socket_map[s].rh_name);
    }
    rtt.println("--------------------------------------------------------\n");
}

void setup() {
    pinMode(PIN_PWR_HOLD, OUTPUT);
    digitalWrite(PIN_PWR_HOLD, HIGH);

    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);

    pinMode(PIN_IR_RX, INPUT_PULLUP);

    pinMode(PIN_BTN_PWR,  INPUT_PULLUP);
    pinMode(PIN_BTN_OK,   INPUT_PULLUP);
    pinMode(PIN_BTN_TEST, INPUT_PULLUP);
    pinMode(PIN_BTN_UP,   INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU, INPUT_PULLUP);

#if defined(USART1)
    SerialUART1.setRx(PA10);
    SerialUART1.setTx(PA9);
    SerialUART1.begin(38400);
#endif

    rtt.println("\n\n========================================================");
    rtt.println("   DSO-TC2 COMPONENT TESTER RESISTOR SCANNER ONLINE     ");
    rtt.println("========================================================\n");

    scan_resistors();
}

void loop() {
    uint16_t ra = (uint16_t)GPIOA->IDR;
    uint16_t rb = (uint16_t)GPIOB->IDR;
    uint16_t rc = (uint16_t)GPIOC->IDR;

    bool b_up    = !(rc & (1U << 8));
    bool b_down  = !(rc & (1U << 9));
    bool b_ok    = !(rb & (1U << 14));
    bool b_test  = !(ra & (1U << 8));
    bool b_menu  = !(rb & (1U << 15));
    bool b_pwr   = !(rb & (1U << 13));

    // Press TEST to re-run scan
    static uint32_t last_scan = 0;
    if (b_test && (millis() - last_scan > 1000)) {
        last_scan = millis();
        scan_resistors();
    }

    // 25 Hz Telemetry to Display
    static uint32_t last_tx = 0;
    static uint32_t seq = 0;
    if (millis() - last_tx >= 40) {
        last_tx = millis();
        seq++;

        uint8_t btn_mask = (b_up ? 0x01 : 0) | (b_down ? 0x02 : 0) |
                           (b_ok ? 0x04 : 0) | (b_test ? 0x08 : 0) |
                           (b_menu ? 0x10 : 0) | (b_pwr ? 0x20 : 0);

        char buf[160];
        snprintf(buf, sizeof(buf), "#TC2:%lu,%s,%s,%02X,%u,%lu,%lu,%s,%s,%s,%s,%s,%s*\n",
                 seq, b_test ? "TEST" : b_ok ? "OK" : "NONE", "PA8", btn_mask, 255, 3300UL, (unsigned long)millis(),
                 socket_map[0].rl_name, socket_map[0].rh_name,
                 socket_map[1].rl_name, socket_map[1].rh_name,
                 socket_map[2].rl_name, socket_map[2].rh_name);

#if defined(USART1)
        SerialUART1.print(buf);
#endif
    }
}
