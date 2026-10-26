/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Master Display Console & Relay Tracer (STM32F103C6)
 * - 186 FPS ST7789 TFT Graphics Engine
 * - Inter-MCU Telemetry from STM32L083RZ at 38400 baud
 * - Interactive Button Mapper & Relay Coil Exerciser
 */

#include <Arduino.h>
#include <RTTStream.h>
#include "tft.h"

RTTStream rtt;

// Candidate Relay Pins on the STM32F103C6
static const uint8_t RELAY_CANDIDATE_PINS[] = {PA1, PA2, PA3, PA4, PA5, PA6};
static const size_t NUM_RELAY_PINS = sizeof(RELAY_CANDIDATE_PINS) / sizeof(RELAY_CANDIDATE_PINS[0]);
static int current_relay_pin_idx = 0;
static bool relay_state = false;

// Telemetry State from Secondary MCU (STM32L083RZ)
struct L083Telemetry {
    uint32_t seq;
    char curr_pressed[16];
    char last_pressed[16];
    uint16_t reg_a;
    uint16_t reg_b;
    uint16_t reg_c;
    uint32_t vcc_mv;
    uint32_t uptime_ms;
    uint32_t last_rx_time;
    uint32_t total_packets;
    bool connected;
};

static L083Telemetry l083_data = {0, "NONE", "NONE", 0xFFFF, 0xFFFF, 0xFFFF, 3300, 0, 0, 0, false};

// Serial Parser Buffer & Diagnostic Counters
static char rx_buf[128];
static uint8_t rx_idx = 0;
static uint32_t raw_rx_byte_count = 0;
static uint8_t last_rx_byte = 0;

static void parse_packet(const char* pkt) {
    // Format: #TC2:<seq>,<curr_pressed>,<last_pressed>,<reg_a>,<reg_b>,<reg_c>,<vcc_mv>,<uptime_ms>*
    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    uint32_t seq, vcc, uptime;
    char curr_p[16], last_p[16];
    unsigned int ra, rb, rc;

    int n = sscanf(pkt + 5, "%lu,%15[^,],%15[^,],%x,%x,%x,%lu,%lu*",
                   &seq, curr_p, last_p, &ra, &rb, &rc, &vcc, &uptime);

    if (n >= 8) {
        l083_data.seq = seq;
        strncpy(l083_data.curr_pressed, curr_p, sizeof(l083_data.curr_pressed) - 1);
        strncpy(l083_data.last_pressed, last_p, sizeof(l083_data.last_pressed) - 1);
        l083_data.reg_a = (uint16_t)ra;
        l083_data.reg_b = (uint16_t)rb;
        l083_data.reg_c = (uint16_t)rc;
        l083_data.vcc_mv = vcc;
        l083_data.uptime_ms = uptime;
        l083_data.last_rx_time = millis();
        l083_data.total_packets++;
        l083_data.connected = true;
    }
}

static void poll_uart() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        raw_rx_byte_count++;
        last_rx_byte = (uint8_t)c;

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

    if (millis() - l083_data.last_rx_time > 1000) {
        l083_data.connected = false;
    }
}

void setup() {
    tft_init();

    // Initialize USART1 on PA9 (TX) and PA10 (RX) at 38400 baud
    Serial1.begin(38400);

    // Initialize candidate relay pins as outputs (default LOW)
    for (size_t i = 0; i < NUM_RELAY_PINS; i++) {
        pinMode(RELAY_CANDIDATE_PINS[i], OUTPUT);
        digitalWrite(RELAY_CANDIDATE_PINS[i], LOW);
    }

    // Initial background
    tft_fill_screen(0x0842); // Dark slate grey/navy

    // 1. Header Banner
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);
    tft_print(8, 7, "DSO-TC2 DUAL-MCU CONSOLE", COLOR_YELLOW, 0x0010, 1);
    tft_print(250, 7, "186 FPS", COLOR_GREEN, 0x0010, 1);

    // Static Card Frames
    // Card 1: Inter-MCU Link Status (Left Top)
    tft_fill_rect(6, 28, 150, 74, COLOR_BLACK);
    tft_draw_rect(6, 28, 150, 74, COLOR_DARKCYAN);
    tft_fill_rect(7, 29, 148, 14, 0x0186);
    tft_print(11, 32, "MCU LINK (USART1)", COLOR_CYAN, 0x0186, 1);

    // Card 2: Secondary MCU Hardware Stats (Right Top)
    tft_fill_rect(162, 28, 152, 74, COLOR_BLACK);
    tft_draw_rect(162, 28, 152, 74, COLOR_DARKCYAN);
    tft_fill_rect(163, 29, 150, 14, 0x0186);
    tft_print(168, 32, "STM32L083 TESTER", COLOR_CYAN, 0x0186, 1);

    // Card 3: Keypad Pin Detector Card (Middle)
    tft_fill_rect(6, 106, 308, 64, COLOR_BLACK);
    tft_draw_rect(6, 106, 308, 64, COLOR_DARKGREY);
    tft_fill_rect(7, 107, 306, 14, 0x18E3);
    tft_print(11, 110, "KEYPAD PIN DETECTOR (PRESS ANY BUTTON)", COLOR_WHITE, 0x18E3, 1);

    // Card 4: Oscilloscope Relay Exerciser (Bottom)
    tft_fill_rect(6, 174, 308, 60, COLOR_BLACK);
    tft_draw_rect(6, 174, 308, 60, 0x0186);
    tft_fill_rect(7, 175, 306, 14, 0x0186);
    tft_print(11, 178, "OSCILLOSCOPE RELAY EXERCISER", COLOR_YELLOW, 0x0186, 1);
}

void loop() {
    static uint32_t last_ui_time = 0;
    static uint32_t last_sec_time = 0;
    static uint32_t prev_packets = 0;
    static uint32_t pkt_rate = 0;
    static uint32_t last_relay_toggle = 0;

    poll_uart();

    // Cycle relay test every 1.5 seconds across PA1 .. PA6
    if (millis() - last_relay_toggle >= 1500) {
        last_relay_toggle = millis();
        
        // Turn previous pin OFF
        digitalWrite(RELAY_CANDIDATE_PINS[current_relay_pin_idx], LOW);

        // Advance to next pin
        current_relay_pin_idx = (current_relay_pin_idx + 1) % NUM_RELAY_PINS;
        
        // Pulse new pin HIGH
        digitalWrite(RELAY_CANDIDATE_PINS[current_relay_pin_idx], HIGH);
        relay_state = true;
    } else if (relay_state && (millis() - last_relay_toggle >= 150)) {
        // Return pulse to LOW after 150 ms
        digitalWrite(RELAY_CANDIDATE_PINS[current_relay_pin_idx], LOW);
        relay_state = false;
    }

    // Calculate packet rate per second
    if (millis() - last_sec_time >= 1000) {
        last_sec_time = millis();
        pkt_rate = l083_data.total_packets - prev_packets;
        prev_packets = l083_data.total_packets;
    }

    // Refresh UI at 25 Hz
    if (millis() - last_ui_time >= 40) {
        last_ui_time = millis();

        // 1. Render Link Status
        if (l083_data.connected) {
            tft_print(12, 46, "STATUS: ONLINE", COLOR_GREEN, COLOR_BLACK, 1);
            char pbuf[32];
            snprintf(pbuf, sizeof(pbuf), "PKTS  : %-7lu", l083_data.total_packets);
            tft_print(12, 59, pbuf, COLOR_WHITE, COLOR_BLACK, 1);
            snprintf(pbuf, sizeof(pbuf), "RATE  : %2lu pkt/s", pkt_rate);
            tft_print(12, 72, pbuf, COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(12, 85, "BAUD  : 38400 8N1  ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        } else {
            static uint8_t dot = 0;
            dot = (dot + 1) % 4;
            char sbuf[32] = "SEARCHING";
            for (int d = 0; d < dot; d++) strcat(sbuf, ".");
            for (int d = dot; d < 3; d++) strcat(sbuf, " ");
            tft_print(12, 46, sbuf, COLOR_YELLOW, COLOR_BLACK, 1);
            
            char bbuf[32];
            snprintf(bbuf, sizeof(bbuf), "BYTES : %-7lu", raw_rx_byte_count);
            tft_print(12, 59, bbuf, (raw_rx_byte_count > 0) ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK, 1);

            snprintf(bbuf, sizeof(bbuf), "LAST  : 0x%02X '%c' ", last_rx_byte, (last_rx_byte >= 32 && last_rx_byte <= 126) ? (char)last_rx_byte : '.');
            tft_print(12, 72, bbuf, (raw_rx_byte_count > 0) ? COLOR_YELLOW : COLOR_DARKGREY, COLOR_BLACK, 1);

            tft_print(12, 85, "BAUD  : 38400 8N1  ", COLOR_DARKGREY, COLOR_BLACK, 1);
        }

        // 2. Render Secondary MCU Telemetry
        if (l083_data.connected) {
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "VCC : %0.2f V", l083_data.vcc_mv / 1000.0f);
            tft_print(168, 46, tbuf, COLOR_GREEN, COLOR_BLACK, 1);

            uint32_t sec = l083_data.uptime_ms / 1000;
            snprintf(tbuf, sizeof(tbuf), "UP  : %02lu:%02lu:%02lu", sec / 3600, (sec % 3600) / 60, sec % 60);
            tft_print(168, 59, tbuf, COLOR_WHITE, COLOR_BLACK, 1);

            snprintf(tbuf, sizeof(tbuf), "PORTA: 0x%04X", l083_data.reg_a);
            tft_print(168, 72, tbuf, COLOR_CYAN, COLOR_BLACK, 1);

            snprintf(tbuf, sizeof(tbuf), "PORTB: 0x%04X", l083_data.reg_b);
            tft_print(168, 85, tbuf, COLOR_CYAN, COLOR_BLACK, 1);
        } else {
            tft_print(168, 46, "VCC : --.-- V ", COLOR_DARKGREY, COLOR_BLACK, 1);
            tft_print(168, 59, "UP  : --:--:--", COLOR_DARKGREY, COLOR_BLACK, 1);
            tft_print(168, 72, "PORTA: ------ ", COLOR_DARKGREY, COLOR_BLACK, 1);
            tft_print(168, 85, "PORTB: ------ ", COLOR_DARKGREY, COLOR_BLACK, 1);
        }

        // 3. Render Keypad Pin Detector Card
        bool is_pressed = (strcmp(l083_data.curr_pressed, "NONE") != 0 && l083_data.connected);
        char kbuf[64];

        if (is_pressed) {
            snprintf(kbuf, sizeof(kbuf), "NOW ACTIVE : >> PIN %s IS PRESSED! <<    ", l083_data.curr_pressed);
            tft_print(12, 126, kbuf, COLOR_GREEN, 0x0180, 1);
        } else {
            tft_print(12, 126, "NOW ACTIVE : [ NO BUTTON PRESSED (IDLE) ]", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        }

        snprintf(kbuf, sizeof(kbuf), "LAST LATCHED: PIN %-6s (L083 Pin)        ", l083_data.last_pressed);
        tft_print(12, 142, kbuf, (strcmp(l083_data.last_pressed, "NONE") != 0) ? COLOR_YELLOW : COLOR_DARKGREY, COLOR_BLACK, 1);

        snprintf(kbuf, sizeof(kbuf), "PORTC RAW   : 0x%04X                        ", l083_data.reg_c);
        tft_print(12, 155, kbuf, COLOR_CYAN, COLOR_BLACK, 1);

        // 4. Render Oscilloscope Relay Exerciser Card
        char rbuf[64];
        uint8_t current_pin = RELAY_CANDIDATE_PINS[current_relay_pin_idx];
        int pin_num = current_pin & 0xF;
        
        snprintf(rbuf, sizeof(rbuf), "TESTING PIN: PA%d   [%s] ", 
                 pin_num, relay_state ? "PULSE HIGH (CLICK?)" : "IDLE LOW          ");
        tft_print(12, 194, rbuf, relay_state ? COLOR_GREEN : COLOR_WHITE, COLOR_BLACK, 1);

        tft_print(12, 210, "Listen for mechanical click from the casing!", COLOR_CYAN, COLOR_BLACK, 1);
    }
}
