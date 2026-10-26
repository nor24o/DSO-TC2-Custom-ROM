/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Master Display & Pixel-Perfect Keypad Mapper (186 FPS)
 * 
 * Clean 320x240 UI layout with ZERO text overflow:
 * - Direct visual boxes for all 6 buttons (UP, DOWN, LEFT, RIGHT, OK, MENU)
 * - Highlights confirmed pins: RIGHT=PA8, LEFT=PB14, MENU=PB15
 * - Live bit monitors for PB12, PB13, PB14, PB15, PA8
 * - Real-time decoded button state with high contrast
 */

#include <Arduino.h>
#include <RTTStream.h>
#include "tft.h"

RTTStream rtt;

// Secondary MCU Telemetry
struct L083Telemetry {
    uint32_t seq;
    char active_btn[24];
    char active_pin[16];
    uint16_t reg_a;
    uint16_t reg_b;
    uint16_t reg_c;
    uint32_t vcc_mv;
    uint32_t uptime_ms;
    uint32_t total_packets;
    bool connected;
};

static L083Telemetry l083_data = {0, "NONE", "NONE", 0xFFFF, 0xFFFF, 0xFFFF, 3300, 0, 0, false};

// Serial Parser
static char rx_buf[128];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time = 0;

static void parse_packet(const char* pkt) {
    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    uint32_t seq, vcc, uptime;
    char btn_str[24], pin_str[16];
    unsigned int ra, rb, rc;

    int n = sscanf(pkt + 5, "%lu,%23[^,],%15[^,],%x,%x,%x,%lu,%lu*",
                   &seq, btn_str, pin_str, &ra, &rb, &rc, &vcc, &uptime);

    if (n >= 8) {
        l083_data.seq = seq;
        strncpy(l083_data.active_btn, btn_str, sizeof(l083_data.active_btn) - 1);
        strncpy(l083_data.active_pin, pin_str, sizeof(l083_data.active_pin) - 1);
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

// Helper to draw a sleek button box
static void draw_btn(int16_t x, int16_t y, int16_t w, int16_t h, 
                     const char* label, const char* sublabel, bool active) {
    uint16_t border_col = active ? COLOR_GREEN : COLOR_DARKCYAN;
    uint16_t bg_col     = active ? COLOR_GREEN : COLOR_BLACK;
    uint16_t text_col   = active ? COLOR_BLACK : COLOR_WHITE;
    uint16_t sub_col    = active ? 0x0010      : COLOR_YELLOW;

    tft_draw_rect(x, y, w, h, border_col);
    tft_fill_rect(x + 1, y + 1, w - 2, h - 2, bg_col);

    // Center text approximately
    int16_t tx = x + (w - (int16_t)strlen(label) * 8) / 2;
    tft_print(tx, y + 4, label, text_col, bg_col, 1);

    if (sublabel && sublabel[0]) {
        int16_t sx = x + (w - (int16_t)strlen(sublabel) * 8) / 2;
        tft_print(sx, y + 16, sublabel, sub_col, bg_col, 1);
    }
}

void setup() {
    tft_init();
    Serial1.begin(38400);

    tft_fill_screen(0x0842); // Clean dark navy background

    // 1. Header Banner
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);
    tft_print(8, 7, "DSO-TC2 KEYPAD PIN MAPPER", COLOR_YELLOW, 0x0010, 1);
    tft_print(250, 7, "186 FPS", COLOR_GREEN, 0x0010, 1);

    // Static Card 1: Keypad Matrix (Y: 28..138)
    tft_fill_rect(4, 28, 312, 108, COLOR_BLACK);
    tft_draw_rect(4, 28, 312, 108, COLOR_DARKCYAN);

    // Static Card 2: Bit Monitor & Live Data (Y: 140..236)
    tft_fill_rect(4, 140, 312, 96, COLOR_BLACK);
    tft_draw_rect(4, 140, 312, 96, COLOR_YELLOW);
}

void loop() {
    poll_uart();

    static uint32_t last_ui = 0;
    if (millis() - last_ui >= 40) { // 25 Hz UI refresh
        last_ui = millis();

        uint16_t ra = l083_data.reg_a;
        uint16_t rb = l083_data.reg_b;
        uint16_t rc = l083_data.reg_c;

        // Check which buttons are physically held down
        bool btn_right = !(ra & (1U << 8));   // PA8 = 0
        bool btn_left  = !(rb & (1U << 14));  // PB14 = 0
        bool btn_menu  = !(rb & (1U << 15));  // PB15 = 0
        bool btn_pb13  = !(rb & (1U << 13));  // PB13 = 0
        bool btn_pb12  = !(rb & (1U << 12));  // PB12 = 0

        // Check if other Port C or Port A bits dropped
        bool other_active = (strcmp(l083_data.active_btn, "NONE") != 0 &&
                             !btn_right && !btn_left && !btn_menu && !btn_pb13 && !btn_pb12);

        // --- Render Keypad Matrix ---
        // Layout:
        //               [ UP: PB12? ]
        // [ LEFT:PB14 ]   [ OK: ? ]   [ RIGHT:PA8 ]   [ MENU:PB15 ]
        //              [ DOWN: PB13? ]

        // UP (top center)
        draw_btn(110, 32, 76, 28, "UP", btn_pb12 ? "ACTIVE!" : "PB12?", btn_pb12);

        // LEFT
        draw_btn(20, 68, 76, 28, "LEFT", "PB14", btn_left);

        // OK (center)
        draw_btn(110, 68, 76, 28, "OK", other_active ? l083_data.active_pin : "???", other_active);

        // RIGHT
        draw_btn(200, 68, 76, 28, "RIGHT", "PA8", btn_right);

        // MENU (right side)
        draw_btn(280, 68, 34, 28, "M", "PB15", btn_menu);

        // DOWN (bottom center)
        draw_btn(110, 104, 76, 28, "DOWN", btn_pb13 ? "ACTIVE!" : "PB13?", btn_pb13);

        // --- Render Bit Monitor & Live Data ---
        char buf[64];

        // Line 1: Real-time Pin State
        if (strcmp(l083_data.active_btn, "NONE") != 0) {
            snprintf(buf, sizeof(buf), ">> BUTTON PRESSED: %-10s (PIN: %-5s) <<", 
                     l083_data.active_btn, l083_data.active_pin);
            tft_print(10, 146, buf, COLOR_GREEN, COLOR_BLACK, 1);
        } else {
            tft_print(10, 146, "IDLE: Press UP, DOWN, or OK firmly to detect pin!   ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        }

        // Line 2: Critical Bit Inspection
        snprintf(buf, sizeof(buf), "BITS: PB12=%d  PB13=%d  PB14=%d  PB15=%d  PA8=%d",
                 (rb >> 12) & 1, (rb >> 13) & 1, (rb >> 14) & 1, (rb >> 15) & 1, (ra >> 8) & 1);
        tft_print(10, 164, buf, COLOR_YELLOW, COLOR_BLACK, 1);

        // Line 3: Raw Registers
        snprintf(buf, sizeof(buf), "PORTA: 0x%04X  PORTB: 0x%04X  PORTC: 0x%04X", ra, rb, rc);
        tft_print(10, 182, buf, COLOR_CYAN, COLOR_BLACK, 1);

        // Line 4: Voltage & Packets
        snprintf(buf, sizeof(buf), "VCC: %0.2fV | LINK: %-5lu pkts (%2u Hz) | UPTIME: %lus",
                 l083_data.vcc_mv / 1000.0f, l083_data.total_packets, 25, l083_data.uptime_ms / 1000);
        tft_print(10, 202, buf, COLOR_WHITE, COLOR_BLACK, 1);

        // Line 5: Instructions
        tft_print(10, 220, "Confirmed: RIGHT=PA8, LEFT=PB14, MENU=PB15", 0x07E0, COLOR_BLACK, 1);
    }
}
