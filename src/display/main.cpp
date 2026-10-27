/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Master Display & Hardware Console (186 FPS)
 * 
 * Dual Mode Graphical Interface:
 * - Screen 0: Hardware Console & 6-Button Physical Matrix
 * - Screen 1: Dedicated Component Tester Menu & Measurement Console
 * Toggle between screens at any time by pressing MENU (PB15)!
 */

#include <Arduino.h>
#include <RTTStream.h>
#include "tft.h"

RTTStream rtt;

// Secondary MCU Telemetry & Component Tester Result
struct L083Telemetry {
    uint32_t seq;
    char active_btn[24];
    char active_pin[16];
    uint8_t btn_mask;
    uint8_t backlight;
    uint32_t vcc_mv;
    uint32_t uptime_ms;
    uint32_t total_packets;
    bool connected;

    // Component Tester Fields
    char comp_type[16];   // "NONE", "RESISTOR", "DIODE", "SHORT"
    char comp_val[24];    // "1.00 kOhm", "Vf=0.68V"
    char comp_pinout[20]; // "Pins 1 - 2", "1(A) -> 2(K)"
    uint16_t s1_mv;
    uint16_t s2_mv;
    uint16_t s3_mv;
};

static L083Telemetry l083_data = {
    0, "NONE", "NONE", 0, 255, 3300, 0, 0, false,
    "NONE", "No component", "Pins 1-2-3 Open", 0, 0, 0
};
static char last_raw_pkt[64] = "Waiting for UART...";

// Screen Mode: 0 = Hardware Console, 1 = Component Tester Menu
static uint8_t screen_mode = 1; // Default to Component Tester!
static bool need_full_redraw = true;

// Serial Parser
static char rx_buf[160];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time = 0;

static void parse_packet(const char* pkt) {
    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    strncpy(last_raw_pkt, pkt, sizeof(last_raw_pkt) - 1);
    last_raw_pkt[sizeof(last_raw_pkt) - 1] = '\0';

    uint32_t seq, vcc, uptime;
    char btn_str[24], pin_str[16];
    unsigned int mask_val, bl_val;
    char type_str[16] = "NONE", val_str[24] = "No component", pinout_str[20] = "---";
    unsigned int s1_v = 0, s2_v = 0, s3_v = 0;

    int n = sscanf(pkt + 5, "%lu,%23[^,],%15[^,],%x,%u,%lu,%lu,%15[^,],%23[^,],%19[^,],%u,%u,%u",
                   &seq, btn_str, pin_str, &mask_val, &bl_val, &vcc, &uptime,
                   type_str, val_str, pinout_str, &s1_v, &s2_v, &s3_v);

    if (n >= 7) {
        l083_data.seq = seq;
        strncpy(l083_data.active_btn, btn_str, sizeof(l083_data.active_btn) - 1);
        strncpy(l083_data.active_pin, pin_str, sizeof(l083_data.active_pin) - 1);
        l083_data.btn_mask = (uint8_t)mask_val;
        l083_data.backlight = (uint8_t)bl_val;
        l083_data.vcc_mv = vcc;
        l083_data.uptime_ms = uptime;
        l083_data.total_packets++;
        l083_data.connected = true;
        last_rx_time = millis();

        if (n >= 13) {
            strncpy(l083_data.comp_type, type_str, sizeof(l083_data.comp_type) - 1);
            strncpy(l083_data.comp_val, val_str, sizeof(l083_data.comp_val) - 1);
            strncpy(l083_data.comp_pinout, pinout_str, sizeof(l083_data.comp_pinout) - 1);
            l083_data.s1_mv = (uint16_t)s1_v;
            l083_data.s2_mv = (uint16_t)s2_v;
            l083_data.s3_mv = (uint16_t)s3_v;
        }
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

// Button Box Widget
static void draw_btn(int16_t x, int16_t y, int16_t w, int16_t h, 
                     const char* label, const char* sublabel, bool active) {
    uint16_t border_col = active ? COLOR_GREEN : COLOR_DARKCYAN;
    uint16_t bg_col     = active ? COLOR_GREEN : COLOR_BLACK;
    uint16_t text_col   = active ? COLOR_BLACK : COLOR_WHITE;
    uint16_t sub_col    = active ? 0x0010      : COLOR_YELLOW;

    tft_draw_rect(x, y, w, h, border_col);
    tft_fill_rect(x + 1, y + 1, w - 2, h - 2, bg_col);

    int16_t tx = x + (w - (int16_t)strlen(label) * 8) / 2;
    tft_print(tx, y + 4, label, text_col, bg_col, 1);

    if (sublabel && sublabel[0]) {
        int16_t sx = x + (w - (int16_t)strlen(sublabel) * 8) / 2;
        tft_print(sx, y + 16, sublabel, sub_col, bg_col, 1);
    }
}

// Redraw static screen background and frames
static void draw_screen_background() {
    tft_fill_screen(0x0842); // Clean dark navy

    // Header Banner (Y: 0..24)
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);

    if (screen_mode == 0) {
        // Mode 0: Hardware Console
        tft_print(8, 7, "DSO-TC2 HARDWARE CONSOLE", COLOR_YELLOW, 0x0010, 1);
        tft_print(232, 7, "[M] TESTER", COLOR_CYAN, 0x0010, 1);

        // Card 1: Keypad Matrix (Y: 28..138)
        tft_fill_rect(4, 28, 312, 108, COLOR_BLACK);
        tft_draw_rect(4, 28, 312, 108, COLOR_DARKCYAN);

        // Card 2: Diagnostics (Y: 140..236)
        tft_fill_rect(4, 140, 312, 96, COLOR_BLACK);
        tft_draw_rect(4, 140, 312, 96, COLOR_YELLOW);
    } else {
        // Mode 1: Component Tester Console
        tft_print(8, 7, "DSO-TC2 COMPONENT TESTER", COLOR_YELLOW, 0x0010, 1);
        tft_print(232, 7, "[M] CONSOLE", COLOR_CYAN, 0x0010, 1);

        // Card 1: Component Detection Result Window (Y: 28..126)
        tft_fill_rect(4, 28, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 28, 312, 98, COLOR_GREEN);

        // Card 2: Socket Terminal Readings & Probe Telemetry (Y: 130..236)
        tft_fill_rect(4, 130, 312, 106, COLOR_BLACK);
        tft_draw_rect(4, 130, 312, 106, COLOR_CYAN);
    }
}

void setup() {
    tft_init();
    Serial1.begin(38400);
    draw_screen_background();
}

void loop() {
    poll_uart();

    // Screen Switching: Press MENU button to toggle screens
    static bool prev_b_menu = false;
    uint8_t m = l083_data.btn_mask;
    bool b_menu = (m & 0x10) != 0;

    if (b_menu && !prev_b_menu) {
        screen_mode = (screen_mode == 0) ? 1 : 0;
        need_full_redraw = true;
    }
    prev_b_menu = b_menu;

    if (need_full_redraw) {
        draw_screen_background();
        need_full_redraw = false;
    }

    static uint32_t last_ui = 0;
    if (millis() - last_ui >= 40) { // 25 Hz UI refresh
        last_ui = millis();

        bool b_up    = (m & 0x01) != 0;
        bool b_down  = (m & 0x02) != 0;
        bool b_ok    = (m & 0x04) != 0;
        bool b_test  = (m & 0x08) != 0;
        bool b_pwr   = (m & 0x20) != 0;
        bool b_relay = (m & 0x40) != 0;
        bool b_ir    = (m & 0x80) != 0;
        bool b_buzz  = ((m & 0x3F) != 0) || b_ir;

        char buf[64];

        if (screen_mode == 0) {
            // ================================================================
            // SCREEN 0: HARDWARE CONSOLE & KEYPAD MATRIX
            // ================================================================
            // Row 0 (Y = 32)
            draw_btn(12, 32, 92, 28, "POWER", "PB13", b_pwr);
            draw_btn(114, 32, 92, 28, "UP ^", "PC8", b_up);
            draw_btn(216, 32, 92, 28, "MENU", "PB15", b_menu);

            // Row 1 (Y = 68)
            draw_btn(12, 68, 92, 28, "OK / <", "PB14", b_ok);
            draw_btn(114, 68, 92, 28, "RELAY", b_relay ? "ON:PB6" : "OFF:PB6", b_relay);
            draw_btn(216, 68, 92, 28, "TEST / >", "PA8", b_test);

            // Row 2 (Y = 104)
            draw_btn(12, 104, 92, 28, "BUZZER", b_buzz ? "ACTIVE" : "PC6", b_buzz);
            draw_btn(114, 104, 92, 28, "DOWN v", "PC9", b_down);
            draw_btn(216, 104, 92, 28, "IR RX", b_ir ? "SIGNAL!" : "PB9", b_ir);

            // Line 1: Active Event State (Y = 146)
            if (b_pwr) {
                tft_print(10, 146, ">> POWER: PB13 (Hold 1.5s=Off)  ", COLOR_RED, COLOR_BLACK, 1);
            } else if (b_ir) {
                tft_print(10, 146, ">> IR SIGNAL DETECTED ON PB9! <<", COLOR_CYAN, COLOR_BLACK, 1);
            } else if ((m & 0x3F) != 0) {
                snprintf(buf, sizeof(buf), ">> KEY: %-5s  PIN: %-4s (0x%02X)   ", 
                         l083_data.active_btn, l083_data.active_pin, m);
                tft_print(10, 146, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                tft_print(10, 146, "STATUS: All 6 Keys & HW Ready   ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
            }

            // Line 2: 1-2-3 ADC Sense Pins (Y = 164)
            tft_print(10, 164, "TESTER SENSE: 1=PA3  2=PA4  3=PA5", COLOR_YELLOW, COLOR_BLACK, 1);

            // Line 3: Probe Resistors (Y = 182)
            tft_print(10, 182, "RL:PA7,PB1,PB10 | RH:PB0,PB2,PB11", COLOR_CYAN, COLOR_BLACK, 1);

            // Line 4: Backlight, Relay & Buzzer (Y = 200)
            uint8_t bl_pct = (uint16_t)l083_data.backlight * 100 / 255;
            snprintf(buf, sizeof(buf), "BL:%2u%% | PWR:PB7 | RLY:%-3s | BUZ:PC6", 
                     bl_pct, b_relay ? "ON" : "OFF");
            tft_print(10, 200, buf, COLOR_WHITE, COLOR_BLACK, 1);

            // Line 5: System Telemetry (Y = 218)
            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%0.2fV | L083:ONLINE (%2uHz)     ",
                         l083_data.vcc_mv / 1000.0f, 25);
                tft_print(10, 218, buf, 0x07E0, COLOR_BLACK, 1);
            } else {
                tft_print(10, 218, "L083 STATUS: OFFLINE (Check UART)", COLOR_RED, COLOR_BLACK, 1);
            }
        } else {
            // ================================================================
            // SCREEN 1: DEDICATED COMPONENT TESTER CONSOLE
            // ================================================================
            // Top Card: Component Identification & Measurement (Y: 28..126)
            tft_print(14, 34, "COMPONENT TEST RESULT:", COLOR_DARKCYAN, COLOR_BLACK, 1);

            if (strcmp(l083_data.comp_type, "RESISTOR") == 0) {
                tft_print(14, 48, ">> RESISTOR <<            ", COLOR_GREEN, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "VALUE: %-18s ", l083_data.comp_val);
                tft_print(14, 72, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "TERMINALS: %-18s ", l083_data.comp_pinout);
                tft_print(14, 98, buf, COLOR_WHITE, COLOR_BLACK, 1);
            } else if (strcmp(l083_data.comp_type, "DIODE") == 0) {
                tft_print(14, 48, ">> DIODE / LED <<         ", COLOR_GREEN, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "%-20s      ", l083_data.comp_val);
                tft_print(14, 72, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "POLARITY: %-18s  ", l083_data.comp_pinout);
                tft_print(14, 98, buf, COLOR_CYAN, COLOR_BLACK, 1);
            } else if (strcmp(l083_data.comp_type, "SHORT") == 0) {
                tft_print(14, 48, ">> CONTINUITY / SHORT <<  ", COLOR_RED, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "R = %-18s    ", l083_data.comp_val);
                tft_print(14, 72, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "TERMINALS: %-18s ", l083_data.comp_pinout);
                tft_print(14, 98, buf, COLOR_WHITE, COLOR_BLACK, 1);
            } else {
                tft_print(14, 48, "[ NO COMPONENT DETECTED ] ", COLOR_LIGHTGREY, COLOR_BLACK, 2);
                tft_print(14, 72, "Insert component into 1-2-3   ", COLOR_YELLOW, COLOR_BLACK, 1);
                tft_print(14, 88, "and press TEST / > button     ", COLOR_WHITE, COLOR_BLACK, 1);
                tft_print(14, 106, "Auto-scans: Resistor, Diode, LED", COLOR_CYAN, COLOR_BLACK, 1);
            }

            // Bottom Card: Socket Terminals & Voltage Readings (Y: 130..236)
            // Pin 1 Box (X: 12, Y: 136, W: 92, H: 40)
            snprintf(buf, sizeof(buf), "%4u mV", l083_data.s1_mv);
            draw_btn(12, 136, 92, 40, "PIN 1 (PA3)", buf, (l083_data.s1_mv > 100));

            // Pin 2 Box (X: 114, Y: 136, W: 92, H: 40)
            snprintf(buf, sizeof(buf), "%4u mV", l083_data.s2_mv);
            draw_btn(114, 136, 92, 40, "PIN 2 (PA4)", buf, (l083_data.s2_mv > 100));

            // Pin 3 Box (X: 216, Y: 136, W: 92, H: 40)
            snprintf(buf, sizeof(buf), "%4u mV", l083_data.s3_mv);
            draw_btn(216, 136, 92, 40, "PIN 3 (PA5)", buf, (l083_data.s3_mv > 100));

            // Probe Lines (Y: 184)
            tft_print(12, 184, "RL (680):  PA7(1)  PB1(2)  PB10(3)", COLOR_CYAN, COLOR_BLACK, 1);
            tft_print(12, 200, "RH (470k): PB0(1)  PB2(2)  PB11(3)", COLOR_CYAN, COLOR_BLACK, 1);

            // Help line (Y: 218)
            tft_print(12, 218, "TEST=Measure | MENU=Hardware Console", 0x07E0, COLOR_BLACK, 1);
        }
    }
}
