/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Dual-MCU Master Display & Telemetry Console (STM32F103C6)
 * 
 * Drives the 2.4" 320x240 ST7789 TFT at 186 FPS.
 * Receives live inter-MCU telemetry from the STM32L083RZ over USART1 (PA10 RX @ 115200 baud).
 * Renders real-time connection health, L083 hardware metrics, and live interactive keypad state.
 */

#include <Arduino.h>
#include <RTTStream.h>
#include "tft.h"

RTTStream rtt;

// Telemetry State from Secondary MCU (STM32L083RZ)
struct L083Telemetry {
    uint32_t seq;
    char active_pin[16];
    uint16_t reg_a;
    uint16_t reg_b;
    uint16_t reg_c;
    uint32_t vcc_mv;
    uint32_t uptime_ms;
    uint32_t last_rx_time;
    uint32_t total_packets;
    bool connected;
};

static L083Telemetry l083_data = {0, "IDLE", 0xFFFF, 0xFFFF, 0xFFFF, 3300, 0, 0, 0, false};

// Serial Parser Buffer & Diagnostic Counters
static char rx_buf[128];
static uint8_t rx_idx = 0;
static uint32_t raw_rx_byte_count = 0;
static uint8_t last_rx_byte = 0;

static void parse_packet(const char* pkt) {
    // Format: #TC2:<seq>,<pin_name>,<reg_a>,<reg_b>,<reg_c>,<vcc_mv>,<uptime_ms>*
    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    uint32_t seq, vcc, uptime;
    char pin_str[16];
    unsigned int ra, rb, rc;

    int n = sscanf(pkt + 5, "%lu,%15[^,],%x,%x,%x,%lu,%lu*",
                   &seq, pin_str, &ra, &rb, &rc, &vcc, &uptime);

    if (n >= 7) {
        l083_data.seq = seq;
        strncpy(l083_data.active_pin, pin_str, sizeof(l083_data.active_pin) - 1);
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

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 DUAL-MCU TELEMETRY CONSOLE            ");
    rtt.println("   MCU 1: STM32F103C6 | MCU 2: STM32L083RZT6            ");
    rtt.println("========================================================");

    // Initial background
    tft_fill_screen(0x0842); // Dark slate grey/navy

    // 1. Header Banner
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);
    tft_print(8, 7, "DSO-TC2 DUAL-MCU SYSTEM", COLOR_YELLOW, 0x0010, 1);
    tft_print(250, 7, "186 FPS", COLOR_GREEN, 0x0010, 1);

    // Static Card Frames
    // Card 1: Inter-MCU Link Status (Left Top)
    tft_fill_rect(6, 30, 150, 76, COLOR_BLACK);
    tft_draw_rect(6, 30, 150, 76, COLOR_DARKCYAN);
    tft_fill_rect(7, 31, 148, 14, 0x0186);
    tft_print(11, 34, "MCU LINK (USART1)", COLOR_CYAN, 0x0186, 1);

    // Card 2: Secondary MCU Hardware Stats (Right Top)
    tft_fill_rect(162, 30, 152, 76, COLOR_BLACK);
    tft_draw_rect(162, 30, 152, 76, COLOR_DARKCYAN);
    tft_fill_rect(163, 31, 150, 14, 0x0186);
    tft_print(168, 34, "STM32L083 TESTER", COLOR_CYAN, 0x0186, 1);

    // Card 3: Interactive Front Keypad Visualizer (Middle)
    tft_fill_rect(6, 112, 308, 74, COLOR_BLACK);
    tft_draw_rect(6, 112, 308, 74, COLOR_DARKGREY);
    tft_fill_rect(7, 113, 306, 14, 0x18E3);
    tft_print(11, 116, "FRONT KEYPAD & PIN DETECTOR", COLOR_WHITE, 0x18E3, 1);

    // Card 4: Bottom Status Bar
    tft_fill_rect(6, 192, 308, 42, COLOR_BLACK);
    tft_draw_rect(6, 192, 308, 42, 0x0186);
}

void loop() {
    static uint32_t last_ui_time = 0;
    static uint32_t last_sec_time = 0;
    static uint32_t prev_packets = 0;
    static uint32_t pkt_rate = 0;

    poll_uart();

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
            tft_print(12, 50, "STATUS: ONLINE", COLOR_GREEN, COLOR_BLACK, 1);
            char pbuf[32];
            snprintf(pbuf, sizeof(pbuf), "PKTS  : %-7lu", l083_data.total_packets);
            tft_print(12, 64, pbuf, COLOR_WHITE, COLOR_BLACK, 1);
            snprintf(pbuf, sizeof(pbuf), "RATE  : %2lu pkt/s", pkt_rate);
            tft_print(12, 78, pbuf, COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(12, 92, "BAUD  : 38400 8N1  ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        } else {
            static uint8_t dot = 0;
            dot = (dot + 1) % 4;
            char sbuf[32] = "SEARCHING";
            for (int d = 0; d < dot; d++) strcat(sbuf, ".");
            for (int d = dot; d < 3; d++) strcat(sbuf, " ");
            tft_print(12, 50, sbuf, COLOR_YELLOW, COLOR_BLACK, 1);
            
            char bbuf[32];
            snprintf(bbuf, sizeof(bbuf), "BYTES : %-7lu", raw_rx_byte_count);
            tft_print(12, 64, bbuf, (raw_rx_byte_count > 0) ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK, 1);

            snprintf(bbuf, sizeof(bbuf), "LAST  : 0x%02X '%c' ", last_rx_byte, (last_rx_byte >= 32 && last_rx_byte <= 126) ? (char)last_rx_byte : '.');
            tft_print(12, 78, bbuf, (raw_rx_byte_count > 0) ? COLOR_YELLOW : COLOR_DARKGREY, COLOR_BLACK, 1);

            int pa10_lvl = digitalRead(PA10);
            snprintf(bbuf, sizeof(bbuf), "PA10 LEVEL: %d   ", pa10_lvl);
            tft_print(12, 92, bbuf, pa10_lvl ? COLOR_GREEN : COLOR_RED, COLOR_BLACK, 1);
        }

        // 2. Render Secondary MCU Telemetry
        if (l083_data.connected) {
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "VCC : %0.2f V", l083_data.vcc_mv / 1000.0f);
            tft_print(168, 50, tbuf, COLOR_GREEN, COLOR_BLACK, 1);

            uint32_t sec = l083_data.uptime_ms / 1000;
            snprintf(tbuf, sizeof(tbuf), "UP  : %02lu:%02lu:%02lu", sec / 3600, (sec % 3600) / 60, sec % 60);
            tft_print(168, 64, tbuf, COLOR_WHITE, COLOR_BLACK, 1);

            snprintf(tbuf, sizeof(tbuf), "PORTA: 0x%04X", l083_data.reg_a);
            tft_print(168, 78, tbuf, COLOR_CYAN, COLOR_BLACK, 1);

            snprintf(tbuf, sizeof(tbuf), "PORTB: 0x%04X", l083_data.reg_b);
            tft_print(168, 92, tbuf, COLOR_CYAN, COLOR_BLACK, 1);
        } else {
            tft_print(168, 50, "VCC : --.-- V ", COLOR_DARKGREY, COLOR_BLACK, 1);
            tft_print(168, 64, "UP  : --:--:--", COLOR_DARKGREY, COLOR_BLACK, 1);
            tft_print(168, 78, "PORTA: ------ ", COLOR_DARKGREY, COLOR_BLACK, 1);
            tft_print(168, 92, "PORTB: ------ ", COLOR_DARKGREY, COLOR_BLACK, 1);
        }

        // 3. Render Interactive Keypad Visualizer
        // Layout:
        //          [ UP ]
        //   [ LEFT ] [ OK ] [ RIGHT ]
        //          [ DOWN ]
        bool is_pressed = (strcmp(l083_data.active_pin, "IDLE") != 0 && l083_data.connected);

        // Center base coordinates for OK button
        int16_t ok_x = 136;
        int16_t ok_y = 146;

        // Button dimensions
        int16_t bw = 46;
        int16_t bh = 16;

        // Simple highlight logic if pin is active
        uint16_t col_up = COLOR_DARKGREY;
        uint16_t col_dn = COLOR_DARKGREY;
        uint16_t col_lt = COLOR_DARKGREY;
        uint16_t col_rt = COLOR_DARKGREY;
        uint16_t col_ok = COLOR_DARKGREY;

        uint16_t bg_up = COLOR_BLACK, bg_dn = COLOR_BLACK, bg_lt = COLOR_BLACK, bg_rt = COLOR_BLACK, bg_ok = COLOR_BLACK;

        if (is_pressed) {
            // Highlight general active button indicator
            col_ok = COLOR_GREEN;
            bg_ok = 0x03E0;
        }

        // Draw UP
        tft_draw_rect(ok_x, ok_y - 18, bw, bh, col_up);
        tft_print(ok_x + 16, ok_y - 14, "UP", (bg_up != COLOR_BLACK) ? COLOR_BLACK : COLOR_WHITE, bg_up, 1);

        // Draw DOWN
        tft_draw_rect(ok_x, ok_y + 18, bw, bh, col_dn);
        tft_print(ok_x + 10, ok_y + 22, "DOWN", (bg_dn != COLOR_BLACK) ? COLOR_BLACK : COLOR_WHITE, bg_dn, 1);

        // Draw LEFT
        tft_draw_rect(ok_x - 52, ok_y, bw, bh, col_lt);
        tft_print(ok_x - 44, ok_y + 4, "LEFT", (bg_lt != COLOR_BLACK) ? COLOR_BLACK : COLOR_WHITE, bg_lt, 1);

        // Draw RIGHT
        tft_draw_rect(ok_x + 52, ok_y, bw, bh, col_rt);
        tft_print(ok_x + 57, ok_y + 4, "RIGHT", (bg_rt != COLOR_BLACK) ? COLOR_BLACK : COLOR_WHITE, bg_rt, 1);

        // Draw OK
        tft_draw_rect(ok_x, ok_y, bw, bh, is_pressed ? COLOR_GREEN : COLOR_YELLOW);
        tft_fill_rect(ok_x + 1, ok_y + 1, bw - 2, bh - 2, bg_ok);
        tft_print(ok_x + 16, ok_y + 4, "OK", is_pressed ? COLOR_BLACK : COLOR_YELLOW, bg_ok, 1);

        // 4. Bottom Activity Banner
        char act_buf[64];
        if (l083_data.connected) {
            if (is_pressed) {
                snprintf(act_buf, sizeof(act_buf), "KEY DETECTED: PIN %-5s PULLED LOW!  ", l083_data.active_pin);
                tft_print(12, 200, act_buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                tft_print(12, 200, "KEYPAD IDLE - PRESS BUTTON TO TEST  ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
            }
            snprintf(act_buf, sizeof(act_buf), "PORTC: 0x%04X | SEQ: %lu      ", l083_data.reg_c, l083_data.seq);
            tft_print(12, 216, act_buf, COLOR_CYAN, COLOR_BLACK, 1);
        } else {
            tft_print(12, 200, "WAITING FOR SECOND MCU UART TRAFFIC...", COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(12, 216, "LISTENING ON PA10 (115200 BAUD)       ", COLOR_DARKGREY, COLOR_BLACK, 1);
        }
    }
}
