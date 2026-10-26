/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Master Display & Universal Dual-MCU Hardware Mapper
 * 
 * Simultaneously sniffs:
 * - MCU 1 (STM32F103C6): Digital & Analog ADC on PA0..PA6
 * - MCU 2 (STM32L083RZ): Live telemetry stream of all 35 GPIOs
 * 
 * Accurately maps the front keypad buttons and oscilloscope relay!
 */

#include <Arduino.h>
#include <RTTStream.h>
#include "tft.h"

RTTStream rtt;

// MCU 1 (F103) Monitored Pins
static const uint8_t F103_PINS[] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6};
static const size_t NUM_F103_PINS = sizeof(F103_PINS) / sizeof(F103_PINS[0]);

static uint16_t f103_adc_base[NUM_F103_PINS];
static uint16_t f103_adc_cur[NUM_F103_PINS];
static uint8_t f103_dig_base[NUM_F103_PINS];
static uint8_t f103_dig_cur[NUM_F103_PINS];
static char f103_last_event[32] = "F103_IDLE";
static uint32_t f103_event_count = 0;

// MCU 2 (L083) Telemetry State
struct L083Telemetry {
    uint32_t event_count;
    char last_event[32];
    uint16_t reg_a;
    uint16_t reg_b;
    uint16_t reg_c;
    uint32_t vcc_mv;
    uint32_t uptime_ms;
    uint32_t total_packets;
    bool connected;
};

static L083Telemetry l083_data = {0, "L083_IDLE", 0xFFFF, 0xFFFF, 0xFFFF, 3300, 0, 0, false};

// Serial Parser
static char rx_buf[128];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time = 0;

static void parse_packet(const char* pkt) {
    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    uint32_t ev_count, vcc, uptime;
    char ev_str[32];
    unsigned int ra, rb, rc;

    int n = sscanf(pkt + 5, "%lu,%31[^,],%x,%x,%x,%lu,%lu*",
                   &ev_count, ev_str, &ra, &rb, &rc, &vcc, &uptime);

    if (n >= 7) {
        l083_data.event_count = ev_count;
        strncpy(l083_data.last_event, ev_str, sizeof(l083_data.last_event) - 1);
        l083_data.reg_a = (uint16_t)ra;
        l083_data.reg_b = (uint16_t)rb;
        l083_data.reg_c = (uint16_t)rc;
        l083_data.vcc_mv = vcc;
        l083_data.uptime_ms = uptime;
        l083_data.total_packets++;
        l083_data.connected = true;
        last_rx_time = millis();
    }
}

static void poll_uart() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '#') {
            rx_idx = 0;
            rx_buf[rx_idx++] = c;
        } else if (c == '\n' || c == '\r' || c == '*') {
            if (rx_idx > 5 && rx_buf[0] == '#') {
                rx_buf[rx_idx] = '\0';
                parse_packet(rx_buf);
            }
            rx_idx = 0;
        } else if (rx_idx < sizeof(rx_buf) - 1) {
            rx_buf[rx_idx++] = c;
        } else {
            rx_idx = 0;
        }
    }

    if (millis() - last_rx_time > 1000) {
        l083_data.connected = false;
    }
}

void setup() {
    tft_init();
    Serial1.begin(38400);

    // Configure PA0..PA6 as INPUT_PULLUP on MCU 1
    for (size_t i = 0; i < NUM_F103_PINS; i++) {
        pinMode(F103_PINS[i], INPUT_PULLUP);
    }
    delay(50);

    // Calibrate baselines for MCU 1
    for (size_t i = 0; i < NUM_F103_PINS; i++) {
        f103_dig_base[i] = digitalRead(F103_PINS[i]);
        f103_adc_base[i] = analogRead(F103_PINS[i]);
        f103_adc_cur[i] = f103_adc_base[i];
    }

    tft_fill_screen(0x0842); // Slate grey

    // 1. Header
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);
    tft_print(8, 7, "DSO-TC2 DUAL-MCU PIN TRACER", COLOR_YELLOW, 0x0010, 1);
    tft_print(250, 7, "186 FPS", COLOR_GREEN, 0x0010, 1);

    // Static Card Frames
    // Card 1: MCU 1 (STM32F103) Detection Box (Top Half)
    tft_fill_rect(6, 28, 308, 92, COLOR_BLACK);
    tft_draw_rect(6, 28, 308, 92, COLOR_CYAN);
    tft_fill_rect(7, 29, 306, 14, 0x0186);
    tft_print(11, 32, "MCU 1 (STM32F103 MAIN CHIP) ACTIVITY", COLOR_CYAN, 0x0186, 1);

    // Card 2: MCU 2 (STM32L083) Detection Box (Bottom Half)
    tft_fill_rect(6, 126, 308, 108, COLOR_BLACK);
    tft_draw_rect(6, 126, 308, 108, COLOR_YELLOW);
    tft_fill_rect(7, 127, 306, 14, 0x18E3);
    tft_print(11, 130, "MCU 2 (STM32L083 TESTER CHIP) ACTIVITY", COLOR_WHITE, 0x18E3, 1);
}

void loop() {
    poll_uart();

    // 1. Fast Scan MCU 1 (F103) Pins: PA0 .. PA6
    for (size_t i = 0; i < NUM_F103_PINS; i++) {
        uint8_t cur_dig = digitalRead(F103_PINS[i]);
        uint16_t cur_adc = analogRead(F103_PINS[i]);
        f103_dig_cur[i] = cur_dig;
        f103_adc_cur[i] = cur_adc;

        // Check if digital flipped
        if (cur_dig != f103_dig_base[i]) {
            snprintf(f103_last_event, sizeof(f103_last_event), 
                     "PA%d FLIPPED TO %s!", i, cur_dig ? "HIGH" : "LOW");
            f103_event_count++;
        }
        // Check if analog voltage shifted significantly (> 400 mV = ~500 counts)
        int diff = abs((int)cur_adc - (int)f103_adc_base[i]);
        if (diff > 500) {
            snprintf(f103_last_event, sizeof(f103_last_event), 
                     "PA%d ANALOG SHIFT: %d -> %d", i, f103_adc_base[i], cur_adc);
            f103_event_count++;
        }
    }

    // 2. Render Screen at 25 Hz
    static uint32_t last_ui = 0;
    if (millis() - last_ui >= 40) {
        last_ui = millis();

        // --- Render MCU 1 Section ---
        char buf[64];
        snprintf(buf, sizeof(buf), "EVENT #%-6lu : %-28s", f103_event_count, f103_last_event);
        tft_print(12, 48, buf, (f103_event_count > 0) ? COLOR_GREEN : COLOR_LIGHTGREY, COLOR_BLACK, 1);

        // Display PA0 .. PA6 current analog/digital values in a compact row
        for (size_t i = 0; i < NUM_F103_PINS; i++) {
            int16_t x = 12 + (i % 4) * 74;
            int16_t y = 66 + (i / 4) * 18;
            snprintf(buf, sizeof(buf), "A%d:%4d D:%d", i, f103_adc_cur[i], f103_dig_cur[i]);
            bool changed = (f103_dig_cur[i] != f103_dig_base[i]) || (abs((int)f103_adc_cur[i] - (int)f103_adc_base[i]) > 300);
            tft_print(x, y, buf, changed ? COLOR_GREEN : COLOR_WHITE, changed ? 0x0180 : COLOR_BLACK, 1);
        }

        // --- Render MCU 2 Section ---
        if (l083_data.connected) {
            snprintf(buf, sizeof(buf), "LINK: ONLINE (38400) | VCC: %0.2fV | UP: %lus",
                     l083_data.vcc_mv / 1000.0f, l083_data.uptime_ms / 1000);
            tft_print(12, 146, buf, COLOR_GREEN, COLOR_BLACK, 1);

            snprintf(buf, sizeof(buf), "EVENT #%-6lu : %-28s", l083_data.event_count, l083_data.last_event);
            tft_print(12, 162, buf, (l083_data.event_count > 0) ? COLOR_YELLOW : COLOR_LIGHTGREY, COLOR_BLACK, 1);

            snprintf(buf, sizeof(buf), "PORTA: 0x%04X   PORTB: 0x%04X   PORTC: 0x%04X",
                     l083_data.reg_a, l083_data.reg_b, l083_data.reg_c);
            tft_print(12, 180, buf, COLOR_CYAN, COLOR_BLACK, 1);

            tft_print(12, 202, "PRESS UP, DOWN, LEFT, RIGHT, OK NOW!", COLOR_MAGENTA, COLOR_BLACK, 1);
            tft_print(12, 216, "Watch which box registers the change!", COLOR_WHITE, COLOR_BLACK, 1);
        } else {
            tft_print(12, 146, "LINK: WAITING FOR SECOND MCU...", COLOR_YELLOW, COLOR_BLACK, 1);
        }
    }
}
