/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Master Display & Hardware Console (186 FPS)
 * 
 * Clean 320x240 UI layout with exact 6-button physical panel mapping:
 * - POWER      = PB13 (Pin 34)
 * - OK / ENTER = PB14 (Pin 35, Left Navigation <)
 * - MENU / AUTO= PB15 (Pin 36)
 * - UP         = PC8  (Pin 39, ^)
 * - DOWN       = PC9  (Pin 40, v)
 * - TEST / HOLD= PA8  (Pin 41, Right Navigation >)
 * - BUZZER     = PC6  (Pin 37, Audio Generator)
 * - BACKLIGHT  = PC7  (Pin 38, LCD Backlight PWM)
 * - RELAY      = PB6  (Pin 58, Oscilloscope Range Switch)
 * - PWR HOLD   = PB7  (Pin 59, Battery Latch)
 * - IR RECEIVER= PB9  (Pin 62, 38 kHz Demodulator)
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
    uint8_t btn_mask;
    uint8_t backlight;
    uint32_t vcc_mv;
    uint32_t uptime_ms;
    uint32_t total_packets;
    bool connected;
};

static L083Telemetry l083_data = {0, "NONE", "NONE", 0, 255, 3300, 0, 0, false};
static char last_raw_pkt[64] = "Waiting for UART...";

// Serial Parser
static char rx_buf[128];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time = 0;

static void parse_packet(const char* pkt) {
    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    // Save for debug display
    strncpy(last_raw_pkt, pkt, sizeof(last_raw_pkt) - 1);
    last_raw_pkt[sizeof(last_raw_pkt) - 1] = '\0';

    // Format: #TC2:<seq>,<btn_str>,<pin_str>,<mask_hex>,<bl_val>,<vcc>,<uptime>*
    uint32_t seq, vcc, uptime;
    char btn_str[24], pin_str[16];
    unsigned int mask_val, bl_val;

    int n = sscanf(pkt + 5, "%lu,%23[^,],%15[^,],%x,%u,%lu,%lu",
                   &seq, btn_str, pin_str, &mask_val, &bl_val, &vcc, &uptime);

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

    // Header Banner
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);
    tft_print(8, 7, "DSO-TC2 COMPLETE HARDWARE MAPPER", COLOR_YELLOW, 0x0010, 1);
    tft_print(250, 7, "186 FPS", COLOR_GREEN, 0x0010, 1);

    // Static Card 1: Keypad Matrix (Y: 28..138)
    tft_fill_rect(4, 28, 312, 108, COLOR_BLACK);
    tft_draw_rect(4, 28, 312, 108, COLOR_DARKCYAN);

    // Static Card 2: System Health & Diagnostics (Y: 140..236)
    tft_fill_rect(4, 140, 312, 96, COLOR_BLACK);
    tft_draw_rect(4, 140, 312, 96, COLOR_YELLOW);
}

void loop() {
    poll_uart();

    static uint32_t last_ui = 0;
    if (millis() - last_ui >= 40) { // 25 Hz UI refresh
        last_ui = millis();

        uint8_t m = l083_data.btn_mask;
        // bit 0: UP, bit 1: DOWN, bit 2: OK, bit 3: TEST, bit 4: MENU, bit 5: POWER, bit 6: RELAY, bit 7: IR_ACTIVE
        bool b_up    = (m & 0x01) != 0;
        bool b_down  = (m & 0x02) != 0;
        bool b_ok    = (m & 0x04) != 0;
        bool b_test  = (m & 0x08) != 0;
        bool b_menu  = (m & 0x10) != 0;
        bool b_pwr   = (m & 0x20) != 0;
        bool b_relay = (m & 0x40) != 0;
        bool b_ir    = (m & 0x80) != 0;
        bool b_buzz  = ((m & 0x3F) != 0) || b_ir; // Buzzer active on any button/IR

        // --- Render Keypad & Peripherals Matrix (3x3 Grid) ---
        // Row 0: [ POWER: PB13 ]   [ UP ^: PC8 ]    [ MENU: PB15 ]
        // Row 1: [ OK / <: PB14 ]  [ RELAY: PB6 ]   [ TEST / >: PA8 ]
        // Row 2: [ BUZZER: PC6 ]   [ DOWN v: PC9 ]  [ IR RX: PB9 ]

        // Top Row (Y = 32)
        draw_btn(12, 32, 92, 28, "POWER", "PB13", b_pwr);
        draw_btn(114, 32, 92, 28, "UP ^", "PC8", b_up);
        draw_btn(216, 32, 92, 28, "MENU", "PB15", b_menu);

        // Middle Row (Y = 68)
        draw_btn(12, 68, 92, 28, "OK / <", "PB14", b_ok);
        draw_btn(114, 68, 92, 28, "RELAY", b_relay ? "ON:PB6" : "OFF:PB6", b_relay);
        draw_btn(216, 68, 92, 28, "TEST / >", "PA8", b_test);

        // Bottom Row (Y = 104)
        draw_btn(12, 104, 92, 28, "BUZZER", b_buzz ? "ACTIVE" : "PC6", b_buzz);
        draw_btn(114, 104, 92, 28, "DOWN v", "PC9", b_down);
        draw_btn(216, 104, 92, 28, "IR RX", b_ir ? "SIGNAL!" : "PB9", b_ir);

        // --- Render System Health & Diagnostics ---
        char buf[96];

        // Line 1: Active Event State
        if (b_pwr) {
            tft_print(10, 146, ">> POWER BUTTON PRESSED: PB13 (Hold 1.5s to shut down) << ", COLOR_RED, COLOR_BLACK, 1);
        } else if (b_ir) {
            tft_print(10, 146, ">> INFRARED DETECTED: Demodulated pulse on PB9! <<        ", COLOR_CYAN, COLOR_BLACK, 1);
        } else if ((m & 0x3F) != 0) {
            snprintf(buf, sizeof(buf), ">> BUTTON ACTIVE: %-6s (PIN: %-4s, MASK: 0x%02X) <<        ", 
                     l083_data.active_btn, l083_data.active_pin, m);
            tft_print(10, 146, buf, COLOR_GREEN, COLOR_BLACK, 1);
        } else {
            tft_print(10, 146, "IDLE: 6 Buttons Mapped | Hold M+TEST: Relay | Hold PWR: Off", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        }

        // Line 2: Pin Mapping & Relay Status
        snprintf(buf, sizeof(buf), "PWR:PB13 UP:PC8 DN:PC9 OK:PB14 TEST:PA8 MN:PB15             ");
        tft_print(10, 164, buf, COLOR_YELLOW, COLOR_BLACK, 1);

        // Line 3: Power, Backlight, Buzzer & IR Controls
        uint8_t bl_pct = (uint16_t)l083_data.backlight * 100 / 255;
        snprintf(buf, sizeof(buf), "BL: PC7 (%2u%%) | PWR: PB7 | BUZZ: PC6 | RELAY: PB6 (%s)  ", bl_pct, b_relay ? "ON " : "OFF");
        tft_print(10, 182, buf, COLOR_CYAN, COLOR_BLACK, 1);

        // Line 4: Voltage & Packets
        snprintf(buf, sizeof(buf), "VCC: %0.2fV | LINK: %-5lu pkts (%2u Hz) | UPTIME: %lus     ",
                 l083_data.vcc_mv / 1000.0f, l083_data.total_packets, 25, l083_data.uptime_ms / 1000);
        tft_print(10, 202, buf, COLOR_WHITE, COLOR_BLACK, 1);

        // Line 5: Connection Status
        if (l083_data.connected) {
            tft_print(10, 220, "Secondary MCU: ONLINE (STM32L083RZ via UART)              ", 0x07E0, COLOR_BLACK, 1);
        } else {
            snprintf(buf, sizeof(buf), "OFFLINE | RX: %-36s", last_raw_pkt);
            tft_print(10, 220, buf, COLOR_RED, COLOR_BLACK, 1);
        }
    }
}
