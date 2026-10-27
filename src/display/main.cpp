/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Master Display & Hardware Console (186 FPS)
 * 
 * Graphical Component Tester & Hardware Console:
 * - Real-time schematic component graphics (Resistor, Capacitor, Diode, Short)
 * - Visual ZIF socket terminal representation with active pin glows
 * - Dual-screen navigation via MENU button (PB15)
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
    char comp_type[16];   // "NONE", "RESISTOR", "DIODE", "CAPACITOR", "SHORT"
    char comp_val[24];    // "1.00 kOhm", "100.0 nF", "Vf=0.68V"
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

// Screen Mode: 0 = Hardware Console, 1 = Graphical Component Tester
static uint8_t screen_mode = 1;
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

// Widget: Button Box
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

// Placeholder icon when open
static void tft_draw_circle_placeholder(int16_t cx, int16_t cy) {
    tft_draw_rect(cx - 14, cy - 14, 28, 28, COLOR_DARKGREY);
    tft_print(cx - 4, cy - 4, "?", COLOR_YELLOW, COLOR_BLACK, 1);
}

// Draw Component Schematic Symbol
static void draw_schematic_graphic(const char* type, const char* pinout) {
    // Graphic canvas: X: 10..220, Y: 32..74 (Center Y = 53)
    tft_fill_rect(10, 32, 216, 42, COLOR_BLACK);

    // Extract terminal indices from pinout string (e.g. "Pins 1 - 2", "1(A) -> 2(K)")
    char pA = '1', pB = '2';
    if (strstr(pinout, "2 - 3") || strstr(pinout, "2(A) -> 3(K)") || strstr(pinout, "3(A) -> 2(K)")) {
        pA = '2'; pB = '3';
    } else if (strstr(pinout, "1 - 3") || strstr(pinout, "1(A) -> 3(K)") || strstr(pinout, "3(A) -> 1(K)")) {
        pA = '1'; pB = '3';
    }

    if (strcmp(type, "RESISTOR") == 0) {
        // Left Terminal Badge
        tft_fill_rect(12, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(12, 42, 22, 22, COLOR_YELLOW);
        tft_draw_char(19, 46, pA, COLOR_YELLOW, COLOR_NAVY, 1);

        // Left Lead
        tft_draw_hline(34, 53, 44, COLOR_WHITE);

        // Resistor Body (X: 78..148, W: 70, H: 18)
        tft_fill_rect(78, 44, 70, 18, 0x5280); // Dark khaki / resistor body
        tft_draw_rect(78, 44, 70, 18, COLOR_YELLOW);

        // 4 Color Code Bands
        tft_fill_rect(90,  44, 5, 18, COLOR_MAROON); // Brown
        tft_fill_rect(104, 44, 5, 18, COLOR_BLACK);  // Black
        tft_fill_rect(118, 44, 5, 18, COLOR_RED);    // Red
        tft_fill_rect(134, 44, 5, 18, COLOR_YELLOW); // Gold tolerance

        // Right Lead
        tft_draw_hline(148, 53, 44, COLOR_WHITE);

        // Right Terminal Badge
        tft_fill_rect(192, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(192, 42, 22, 22, COLOR_YELLOW);
        tft_draw_char(199, 46, pB, COLOR_YELLOW, COLOR_NAVY, 1);

    } else if (strcmp(type, "CAPACITOR") == 0) {
        // Left Terminal Badge
        tft_fill_rect(12, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(12, 42, 22, 22, COLOR_CYAN);
        tft_draw_char(19, 46, pA, COLOR_CYAN, COLOR_NAVY, 1);

        // Left Lead
        tft_draw_hline(34, 53, 72, COLOR_WHITE);

        // Capacitor Plates (Parallel vertical bars)
        tft_fill_rect(106, 38, 4, 30, COLOR_CYAN);
        tft_fill_rect(118, 38, 4, 30, COLOR_CYAN);

        // Dielectric Gap Lines
        tft_draw_hline(110, 48, 8, COLOR_DARKCYAN);
        tft_draw_hline(110, 58, 8, COLOR_DARKCYAN);

        // Polarity indicator
        tft_print(96, 36, "+", COLOR_CYAN, COLOR_BLACK, 1);

        // Right Lead
        tft_draw_hline(122, 53, 70, COLOR_WHITE);

        // Right Terminal Badge
        tft_fill_rect(192, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(192, 42, 22, 22, COLOR_CYAN);
        tft_draw_char(199, 46, pB, COLOR_CYAN, COLOR_NAVY, 1);

    } else if (strcmp(type, "DIODE") == 0) {
        // Anode Terminal Badge
        tft_fill_rect(12, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(12, 42, 22, 22, COLOR_GREEN);
        tft_draw_char(19, 46, pA, COLOR_GREEN, COLOR_NAVY, 1);

        // Anode Lead
        tft_draw_hline(34, 53, 62, COLOR_WHITE);

        // Diode Triangle (Anode pointing right towards cathode)
        for (int i = 0; i <= 18; i++) {
            int16_t h = (i * 20) / 18;
            tft_draw_vline(96 + i, 53 - h / 2, h, COLOR_GREEN);
        }

        // Cathode Vertical Bar
        tft_fill_rect(116, 40, 4, 26, COLOR_GREEN);

        // LED Emission Rays
        tft_draw_line(110, 36, 120, 26, COLOR_YELLOW);
        tft_draw_line(116, 36, 126, 26, COLOR_YELLOW);

        // Cathode Lead
        tft_draw_hline(120, 53, 72, COLOR_WHITE);

        // Cathode Terminal Badge
        tft_fill_rect(192, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(192, 42, 22, 22, COLOR_GREEN);
        tft_draw_char(199, 46, pB, COLOR_GREEN, COLOR_NAVY, 1);

    } else if (strcmp(type, "SHORT") == 0) {
        // Left Badge
        tft_fill_rect(12, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(12, 42, 22, 22, COLOR_RED);
        tft_draw_char(19, 46, pA, COLOR_RED, COLOR_NAVY, 1);

        // Solid Glowing Continuity Wire
        tft_fill_rect(34, 51, 158, 5, COLOR_RED);
        tft_print(82, 38, "[ SHORT ]", COLOR_YELLOW, COLOR_BLACK, 1);

        // Right Badge
        tft_fill_rect(192, 42, 22, 22, COLOR_NAVY);
        tft_draw_rect(192, 42, 22, 22, COLOR_RED);
        tft_draw_char(199, 46, pB, COLOR_RED, COLOR_NAVY, 1);

    } else {
        // Idle: Open Leads with question mark
        tft_draw_hline(16, 53, 60, COLOR_DARKGREY);
        tft_draw_circle_placeholder(113, 53);
        tft_draw_hline(150, 53, 60, COLOR_DARKGREY);
        tft_print(74, 38, "[ OPEN LEADS ]", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    }

    // Mini Visual ZIF Socket Block (X: 230..308, Y: 32..120)
    tft_fill_rect(230, 32, 78, 88, 0x18C3); // Dark teal socket base
    tft_draw_rect(230, 32, 78, 88, COLOR_CYAN);
    tft_fill_rect(232, 34, 10, 84, 0x001F); // Blue locking lever

    // 3 Pin Holes
    bool s1_act = (pA == '1' || pB == '1');
    bool s2_act = (pA == '2' || pB == '2');
    bool s3_act = (pA == '3' || pB == '3');

    tft_fill_rect(250, 42, 52, 18, s1_act ? COLOR_GREEN : COLOR_BLACK);
    tft_print(254, 46, s1_act ? "*PIN 1*" : " PIN 1 ", s1_act ? COLOR_BLACK : COLOR_WHITE, s1_act ? COLOR_GREEN : COLOR_BLACK, 1);

    tft_fill_rect(250, 66, 52, 18, s2_act ? COLOR_GREEN : COLOR_BLACK);
    tft_print(254, 70, s2_act ? "*PIN 2*" : " PIN 2 ", s2_act ? COLOR_BLACK : COLOR_WHITE, s2_act ? COLOR_GREEN : COLOR_BLACK, 1);

    tft_fill_rect(250, 90, 52, 18, s3_act ? COLOR_GREEN : COLOR_BLACK);
    tft_print(254, 94, s3_act ? "*PIN 3*" : " PIN 3 ", s3_act ? COLOR_BLACK : COLOR_WHITE, s3_act ? COLOR_GREEN : COLOR_BLACK, 1);
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
        // Mode 1: Graphical Component Tester Console
        tft_print(8, 7, "DSO-TC2 COMPONENT TESTER", COLOR_YELLOW, 0x0010, 1);
        tft_print(232, 7, "[M] CONSOLE", COLOR_CYAN, 0x0010, 1);

        // Card 1: Schematic Graphic Window (Y: 28..126)
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
            draw_btn(12, 32, 92, 28, "POWER", "PB13", b_pwr);
            draw_btn(114, 32, 92, 28, "UP ^", "PC8", b_up);
            draw_btn(216, 32, 92, 28, "MENU", "PB15", b_menu);

            draw_btn(12, 68, 92, 28, "OK / <", "PB14", b_ok);
            draw_btn(114, 68, 92, 28, "RELAY", b_relay ? "ON:PB6" : "OFF:PB6", b_relay);
            draw_btn(216, 68, 92, 28, "TEST / >", "PA8", b_test);

            draw_btn(12, 104, 92, 28, "BUZZER", b_buzz ? "ACTIVE" : "PC6", b_buzz);
            draw_btn(114, 104, 92, 28, "DOWN v", "PC9", b_down);
            draw_btn(216, 104, 92, 28, "IR RX", b_ir ? "SIGNAL!" : "PB9", b_ir);

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

            tft_print(10, 164, "TESTER SENSE: 1=PA3  2=PA4  3=PA5", COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(10, 182, "RL:PA7,PB1,PB10 | RH:PB0,PB2,PB11", COLOR_CYAN, COLOR_BLACK, 1);

            uint8_t bl_pct = (uint16_t)l083_data.backlight * 100 / 255;
            snprintf(buf, sizeof(buf), "BL:%2u%% | PWR:PB7 | RLY:%-3s | BUZ:PC6", 
                     bl_pct, b_relay ? "ON" : "OFF");
            tft_print(10, 200, buf, COLOR_WHITE, COLOR_BLACK, 1);

            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%0.2fV | L083:ONLINE (%2uHz)     ",
                         l083_data.vcc_mv / 1000.0f, 25);
                tft_print(10, 218, buf, 0x07E0, COLOR_BLACK, 1);
            } else {
                tft_print(10, 218, "L083 STATUS: OFFLINE (Check UART)", COLOR_RED, COLOR_BLACK, 1);
            }
        } else {
            // ================================================================
            // SCREEN 1: DEDICATED GRAPHICAL COMPONENT TESTER
            // ================================================================
            // 1. Draw Visual Schematic Graphic
            static char prev_type[16] = "";
            static char prev_pinout[20] = "";
            if (strcmp(l083_data.comp_type, prev_type) != 0 || 
                strcmp(l083_data.comp_pinout, prev_pinout) != 0 || 
                need_full_redraw) {
                draw_schematic_graphic(l083_data.comp_type, l083_data.comp_pinout);
                strncpy(prev_type, l083_data.comp_type, sizeof(prev_type) - 1);
                strncpy(prev_pinout, l083_data.comp_pinout, sizeof(prev_pinout) - 1);
            }

            // 2. Component Measurement Text (Y: 78..124)
            if (strcmp(l083_data.comp_type, "RESISTOR") == 0) {
                snprintf(buf, sizeof(buf), "R = %-16s", l083_data.comp_val);
                tft_print(14, 80, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "Terminals: %-18s", l083_data.comp_pinout);
                tft_print(14, 106, buf, COLOR_WHITE, COLOR_BLACK, 1);
            } else if (strcmp(l083_data.comp_type, "CAPACITOR") == 0) {
                snprintf(buf, sizeof(buf), "C = %-16s", l083_data.comp_val);
                tft_print(14, 80, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "Terminals: %-18s", l083_data.comp_pinout);
                tft_print(14, 106, buf, COLOR_CYAN, COLOR_BLACK, 1);
            } else if (strcmp(l083_data.comp_type, "DIODE") == 0) {
                snprintf(buf, sizeof(buf), "%-18s", l083_data.comp_val);
                tft_print(14, 80, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "Diode: %-18s    ", l083_data.comp_pinout);
                tft_print(14, 106, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else if (strcmp(l083_data.comp_type, "SHORT") == 0) {
                snprintf(buf, sizeof(buf), "R = %-16s", l083_data.comp_val);
                tft_print(14, 80, buf, COLOR_YELLOW, COLOR_BLACK, 2);
                snprintf(buf, sizeof(buf), "Continuity: %-16s", l083_data.comp_pinout);
                tft_print(14, 106, buf, COLOR_RED, COLOR_BLACK, 1);
            } else {
                tft_print(14, 80, "[ READY FOR TEST ]  ", COLOR_LIGHTGREY, COLOR_BLACK, 2);
                tft_print(14, 106, "Insert component & press TEST", COLOR_YELLOW, COLOR_BLACK, 1);
            }

            // Bottom Card: Socket Terminals & Voltage Readings (Y: 130..236)
            snprintf(buf, sizeof(buf), "%4u mV", l083_data.s1_mv);
            draw_btn(12, 136, 92, 40, "PIN 1 (PA3)", buf, (l083_data.s1_mv > 100));

            snprintf(buf, sizeof(buf), "%4u mV", l083_data.s2_mv);
            draw_btn(114, 136, 92, 40, "PIN 2 (PA4)", buf, (l083_data.s2_mv > 100));

            snprintf(buf, sizeof(buf), "%4u mV", l083_data.s3_mv);
            draw_btn(216, 136, 92, 40, "PIN 3 (PA5)", buf, (l083_data.s3_mv > 100));

            // Probe Lines (Y: 184)
            tft_print(12, 184, "RL (680):  PA7(1)  PB1(2)  PB10(3)", COLOR_CYAN, COLOR_BLACK, 1);
            tft_print(12, 200, "RH (470k): PB0(1)  PB2(2)  PB11(3)", COLOR_CYAN, COLOR_BLACK, 1);

            // Help line (Y: 218)
            tft_print(12, 218, "TEST=Measure | MENU=Switch Screen", 0x07E0, COLOR_BLACK, 1);
        }
    }
}
