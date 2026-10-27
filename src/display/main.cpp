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

// Secondary MCU Telemetry & Instruments
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
    char comp_pinout[28]; // "Pins 1 - 2", "1(A) -> 2(K)", "1:GND 2:DQ 3:VDD"
    uint16_t s1_mv;
    uint16_t s2_mv;
    uint16_t s3_mv;

    // Zener Diode Fields
    uint32_t zener_mv;
    char zener_status[24];

    // Real Li-Po Battery & External SMA Voltmeter (from STM32L083)
    uint16_t batt_mv;
    uint16_t sma_mv;

    // Signal Generator (PB8 PWM)
    uint32_t siggen_freq;
    uint8_t  siggen_on;

    // Active Instrument Mode (0 = Tester, 1 = IR Decoder, 2 = Zener, 3 = SigGen & Voltmeter)
    uint8_t  mode;
};

static L083Telemetry l083_data = {
    0, "NONE", "NONE", 0, 255, 3300, 0, 0, false,
    "NONE", "No component", "Pins 1-2-3 Open", 0, 0, 0,
    0, "STANDBY",
    3800, 0,
    1000, 0,
    0
};
static char last_raw_pkt[64] = "Waiting for UART...";

// Li-Po Battery Monitor (Updated live from STM32L083 PA2 2:1 divider telemetry)
static uint16_t batt_mv = 3800;
static uint8_t  batt_pct = 75;
static uint8_t  prev_batt_pct = 0xFF;
static uint16_t prev_batt_mv = 0;

static void draw_battery_indicator(bool force_redraw) {
    if (!force_redraw && batt_pct == prev_batt_pct && abs((int)batt_mv - (int)prev_batt_mv) < 30) {
        return;
    }
    prev_batt_pct = batt_pct;
    prev_batt_mv = batt_mv;

    // Percentage string (X: 240..272, Y: 7)
    char buf[8];
    snprintf(buf, sizeof(buf), "%3u%%", batt_pct);
    tft_print(240, 7, buf, COLOR_WHITE, 0x0010, 1);

    // Battery Shell (X: 278, Y: 5, W: 28, H: 14)
    tft_draw_rect(278, 5, 28, 14, COLOR_WHITE);
    // Positive Pip (X: 306, Y: 8, W: 3, H: 8)
    tft_fill_rect(306, 8, 3, 8, COLOR_WHITE);

    // Inner Fill (X: 280, Y: 7, max W: 24, H: 10)
    uint8_t fill_w = (batt_pct * 24) / 100;
    uint16_t col = (batt_pct > 50) ? COLOR_GREEN : (batt_pct > 20) ? COLOR_YELLOW : COLOR_RED;

    if (fill_w > 0) {
        tft_fill_rect(280, 7, fill_w, 10, col);
    }
    if (fill_w < 24) {
        tft_fill_rect(280 + fill_w, 7, 24 - fill_w, 10, 0x0010);
    }
}

// Decoded Infrared Receiver Telemetry
struct IrRxTelemetry {
    char protocol[12];
    uint16_t address;
    uint8_t command;
    uint32_t raw_code;
    uint32_t repeats;
    uint32_t last_rx_ms;
    bool received;
    // Decoded History
    uint32_t hist_raw[3];
    uint16_t hist_addr[3];
    uint8_t hist_cmd[3];
    uint8_t hist_cnt;
};

static IrRxTelemetry ir_rx = {
    "NONE", 0, 0, 0, 0, 0, false,
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0
};

// Screen Mode: 0 = Tester & Sensors, 1 = IR Decoder, 2 = Zener Diode, 3 = SigGen & SMA Volt
static uint8_t screen_mode = 0;
static bool need_full_redraw = true;

// Serial Parser
static char rx_buf[224];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time = 0;
static uint32_t raw_rx_bytes = 0;

static void parse_packet(const char* pkt) {
    if (strncmp(pkt, "#IR:", 4) == 0) {
        char proto[12] = "NEC";
        unsigned int addr = 0, cmd = 0;
        unsigned long raw = 0, rpt = 0;
        int n = sscanf(pkt + 4, "%11[^,],%x,%x,%lx,%lu", proto, &addr, &cmd, &raw, &rpt);
        if (n >= 4) {
            strncpy(ir_rx.protocol, proto, sizeof(ir_rx.protocol) - 1);
            ir_rx.address = (uint16_t)addr;
            ir_rx.command = (uint8_t)cmd;
            ir_rx.raw_code = (uint32_t)raw;
            ir_rx.repeats = (uint32_t)rpt;
            ir_rx.last_rx_ms = millis();
            ir_rx.received = true;

            // Push to history on new keypress (rpt == 0)
            if (rpt == 0) {
                ir_rx.hist_raw[2]  = ir_rx.hist_raw[1];
                ir_rx.hist_addr[2] = ir_rx.hist_addr[1];
                ir_rx.hist_cmd[2]  = ir_rx.hist_cmd[1];

                ir_rx.hist_raw[1]  = ir_rx.hist_raw[0];
                ir_rx.hist_addr[1] = ir_rx.hist_addr[0];
                ir_rx.hist_cmd[1]  = ir_rx.hist_cmd[0];

                ir_rx.hist_raw[0]  = (uint32_t)raw;
                ir_rx.hist_addr[0] = (uint16_t)addr;
                ir_rx.hist_cmd[0]  = (uint8_t)cmd;
                if (ir_rx.hist_cnt < 3) ir_rx.hist_cnt++;
            }
        }
        return;
    }

    if (strncmp(pkt, "#TC2:", 5) != 0) return;

    strncpy(last_raw_pkt, pkt, sizeof(last_raw_pkt) - 1);
    last_raw_pkt[sizeof(last_raw_pkt) - 1] = '\0';

    uint32_t seq, vcc, uptime;
    char btn_str[24], pin_str[16];
    unsigned int mask_val, bl_val;
    char type_str[16] = "NONE", val_str[24] = "No component", pinout_str[28] = "---";
    unsigned int s1_v = 0, s2_v = 0, s3_v = 0;
    unsigned long zener_v = 0;
    char zener_stat_str[24] = "STANDBY";
    unsigned int batt_v = 3800, sma_v = 0, sig_on = 0, sc_mode = 0;
    unsigned long sig_f = 1000;

    int n = sscanf(pkt + 5, "%lu,%23[^,],%15[^,],%x,%u,%lu,%lu,%15[^,],%23[^,],%27[^,],%u,%u,%u,%lu,%23[^,],%u,%u,%lu,%u,%u",
                   &seq, btn_str, pin_str, &mask_val, &bl_val, &vcc, &uptime,
                   type_str, val_str, pinout_str, &s1_v, &s2_v, &s3_v,
                   &zener_v, zener_stat_str,
                   &batt_v, &sma_v, &sig_f, &sig_on, &sc_mode);

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
        if (n >= 14) {
            l083_data.zener_mv = (uint32_t)zener_v;
            strncpy(l083_data.zener_status, zener_stat_str, sizeof(l083_data.zener_status) - 1);
        }
        if (n >= 19) {
            l083_data.batt_mv = (uint16_t)batt_v;
            l083_data.sma_mv  = (uint16_t)sma_v;
            l083_data.siggen_freq = (uint32_t)sig_f;
            l083_data.siggen_on   = (uint8_t)sig_on;
            if (sc_mode < 4 && sc_mode != screen_mode) {
                screen_mode = (uint8_t)sc_mode;
                need_full_redraw = true;
            }
            if (batt_v >= 2500 && batt_v <= 5000) {
                batt_mv = (uint16_t)batt_v;
                if (batt_mv <= 3300) batt_pct = 0;
                else if (batt_mv >= 4180) batt_pct = 100;
                else batt_pct = (uint8_t)(((uint32_t)(batt_mv - 3300) * 100UL) / (4180UL - 3300UL));
            }
        }
    }
}

static void poll_uart() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        raw_rx_bytes++;
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

    if (millis() - last_rx_time > 1500) {
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
    tft_draw_rect(cx - 12, cy - 12, 24, 24, COLOR_DARKGREY);
    tft_print(cx - 4, cy - 4, "?", COLOR_YELLOW, COLOR_BLACK, 1);
}

// Draw Component Schematic Symbol & Mini ZIF Socket
static void draw_schematic_graphic(const char* type, const char* pinout) {
    // 1. Left Schematic Canvas: X: 6..222, Y: 28..132
    tft_fill_rect(6, 28, 216, 104, COLOR_BLACK);

    // Extract terminal indices dynamically from pinout string (e.g. "Pins 1 - 2", "3(A) -> 2(K)", "B:1 C:2 E:3")
    char pA = '1', pB = '2', pC = '0';
    const char* ptr = pinout;
    while (*ptr && (*ptr < '1' || *ptr > '3')) ptr++;
    if (*ptr) {
        pA = *ptr++;
        while (*ptr && (*ptr < '1' || *ptr > '3')) ptr++;
        if (*ptr) {
            pB = *ptr++;
            while (*ptr && (*ptr < '1' || *ptr > '3')) ptr++;
            if (*ptr) pC = *ptr;
        }
    }

    bool is_open = (strcmp(type, "NONE") == 0 || strlen(type) == 0);

    if (strcmp(type, "RESISTOR") == 0) {
        // Left Terminal Badge
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_YELLOW);
        tft_draw_char(20, 46, pA, COLOR_YELLOW, COLOR_NAVY, 1);

        // Left Lead
        tft_draw_hline(34, 53, 40, COLOR_WHITE);

        // Resistor Body (X: 74..150, W: 76, H: 18)
        tft_fill_rect(74, 44, 76, 18, 0x5280); // Resistor body
        tft_draw_rect(74, 44, 76, 18, COLOR_YELLOW);

        // 4 Color Code Bands
        tft_fill_rect(86,  44, 5, 18, COLOR_MAROON); // Brown
        tft_fill_rect(100, 44, 5, 18, COLOR_BLACK);  // Black
        tft_fill_rect(116, 44, 5, 18, COLOR_RED);    // Red
        tft_fill_rect(134, 44, 5, 18, COLOR_YELLOW); // Gold

        // Right Lead
        tft_draw_hline(150, 53, 36, COLOR_WHITE);

        // Right Terminal Badge
        tft_fill_rect(186, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(186, 42, 20, 22, COLOR_YELLOW);
        tft_draw_char(192, 46, pB, COLOR_YELLOW, COLOR_NAVY, 1);

    } else if (strcmp(type, "CAPACITOR") == 0) {
        // Left Terminal Badge
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_CYAN);
        tft_draw_char(20, 46, pA, COLOR_CYAN, COLOR_NAVY, 1);

        // Left Lead
        tft_draw_hline(34, 53, 68, COLOR_WHITE);

        // Capacitor Plates (Parallel vertical bars)
        tft_fill_rect(102, 37, 5, 32, COLOR_CYAN);
        tft_fill_rect(115, 37, 5, 32, COLOR_CYAN);

        // Dielectric Gap Lines
        tft_draw_hline(107, 47, 8, COLOR_DARKCYAN);
        tft_draw_hline(107, 57, 8, COLOR_DARKCYAN);

        // Polarity indicator
        tft_print(92, 35, "+", COLOR_CYAN, COLOR_BLACK, 1);

        // Right Lead
        tft_draw_hline(120, 53, 66, COLOR_WHITE);

        // Right Terminal Badge
        tft_fill_rect(186, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(186, 42, 20, 22, COLOR_CYAN);
        tft_draw_char(192, 46, pB, COLOR_CYAN, COLOR_NAVY, 1);

    } else if (strcmp(type, "DIODE") == 0) {
        // Anode Terminal Badge (pA on Left)
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_GREEN);
        tft_draw_char(20, 46, pA, COLOR_GREEN, COLOR_NAVY, 1);
        tft_print(16, 32, "A", COLOR_GREEN, COLOR_BLACK, 1);

        // Anode Lead
        tft_draw_hline(34, 53, 58, COLOR_WHITE);

        // Standard Electronic Diode Symbol:
        for (int i = 0; i <= 18; i++) {
            int16_t h = 20 - (i * 20) / 18;
            tft_draw_vline(92 + i, 53 - h / 2, h, COLOR_GREEN);
        }

        // Cathode Vertical Bar (X: 111..114)
        tft_fill_rect(111, 41, 4, 24, COLOR_GREEN);

        // LED Emission Rays
        tft_draw_line(100, 37, 110, 27, COLOR_YELLOW);
        tft_draw_line(106, 37, 116, 27, COLOR_YELLOW);

        // Cathode Lead
        tft_draw_hline(115, 53, 71, COLOR_WHITE);

        // Cathode Terminal Badge (pB on Right)
        tft_fill_rect(186, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(186, 42, 20, 22, COLOR_GREEN);
        tft_draw_char(192, 46, pB, COLOR_GREEN, COLOR_NAVY, 1);
        tft_print(192, 32, "K", COLOR_GREEN, COLOR_BLACK, 1);

    } else if (strcmp(type, "NPN") == 0 || strcmp(type, "PNP") == 0) {
        bool is_npn = (strcmp(type, "NPN") == 0);
        // Base Terminal Badge (Left: X=14, Y=42)
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_YELLOW);
        tft_draw_char(20, 46, pA, COLOR_YELLOW, COLOR_NAVY, 1);
        tft_print(16, 32, "B", COLOR_YELLOW, COLOR_BLACK, 1);

        // Base lead & vertical bar
        tft_draw_hline(34, 53, 50, COLOR_WHITE);
        tft_fill_rect(84, 38, 4, 30, COLOR_WHITE);

        // Collector (top right)
        tft_draw_line(88, 44, 114, 35, COLOR_WHITE);
        tft_draw_hline(114, 35, 72, COLOR_WHITE);
        tft_fill_rect(186, 26, 20, 20, COLOR_NAVY);
        tft_draw_rect(186, 26, 20, 20, COLOR_CYAN);
        tft_draw_char(192, 30, pB, COLOR_CYAN, COLOR_NAVY, 1);
        tft_print(192, 16, "C", COLOR_CYAN, COLOR_BLACK, 1);

        // Emitter (bottom right)
        tft_draw_line(88, 62, 114, 71, COLOR_WHITE);
        tft_draw_hline(114, 71, 72, COLOR_WHITE);
        tft_fill_rect(186, 60, 20, 20, COLOR_NAVY);
        tft_draw_rect(186, 60, 20, 20, COLOR_GREEN);
        tft_draw_char(192, 64, pC, COLOR_GREEN, COLOR_NAVY, 1);
        tft_print(192, 82, "E", COLOR_GREEN, COLOR_BLACK, 1);

        tft_print(122, 50, is_npn ? "NPN" : "PNP", is_npn ? COLOR_GREEN : COLOR_YELLOW, COLOR_BLACK, 1);

    } else if (strcmp(type, "N-MOS") == 0 || strcmp(type, "P-MOS") == 0) {
        bool is_n = (strcmp(type, "N-MOS") == 0);
        // Gate Badge (Left: X=14, Y=42)
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_YELLOW);
        tft_draw_char(20, 46, pA, COLOR_YELLOW, COLOR_NAVY, 1);
        tft_print(16, 32, "G", COLOR_YELLOW, COLOR_BLACK, 1);

        // Gate lead & insulated bar
        tft_draw_hline(34, 53, 44, COLOR_WHITE);
        tft_fill_rect(78, 38, 3, 30, COLOR_YELLOW);

        // Channel segments
        tft_fill_rect(84, 38, 4, 8, COLOR_WHITE);
        tft_fill_rect(84, 49, 4, 8, COLOR_WHITE);
        tft_fill_rect(84, 60, 4, 8, COLOR_WHITE);

        // Drain (top right)
        tft_draw_hline(88, 42, 98, COLOR_WHITE);
        tft_fill_rect(186, 26, 20, 20, COLOR_NAVY);
        tft_draw_rect(186, 26, 20, 20, COLOR_CYAN);
        tft_draw_char(192, 30, pB, COLOR_CYAN, COLOR_NAVY, 1);
        tft_print(192, 16, "D", COLOR_CYAN, COLOR_BLACK, 1);

        // Source (bottom right)
        tft_draw_hline(88, 64, 98, COLOR_WHITE);
        tft_fill_rect(186, 60, 20, 20, COLOR_NAVY);
        tft_draw_rect(186, 60, 20, 20, COLOR_GREEN);
        tft_draw_char(192, 60, pC, COLOR_GREEN, COLOR_NAVY, 1);
        tft_print(192, 82, "S", COLOR_GREEN, COLOR_BLACK, 1);

        tft_print(112, 50, is_n ? "N-MOS" : "P-MOS", is_n ? COLOR_GREEN : COLOR_YELLOW, COLOR_BLACK, 1);

    } else if (strcmp(type, "SHORT") == 0) {
        // Left Badge
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_RED);
        tft_draw_char(20, 46, pA, COLOR_RED, COLOR_NAVY, 1);

        // Solid Glowing Continuity Wire
        tft_fill_rect(34, 51, 152, 5, COLOR_RED);
        tft_print(78, 37, "[ SHORT ]", COLOR_YELLOW, COLOR_BLACK, 1);

        // Right Badge
        tft_fill_rect(186, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(186, 42, 20, 22, COLOR_RED);
        tft_draw_char(192, 46, pB, COLOR_RED, COLOR_NAVY, 1);

    } else if (strcmp(type, "DS18B20") == 0 || strcmp(type, "DS18S20") == 0 || strcmp(type, "DS1822") == 0) {
        // TO-92 Digital Temp Sensor Body (X: 68..148, Y: 32..64)
        tft_fill_rect(68, 32, 80, 32, 0x2104);
        tft_draw_rect(68, 32, 80, 32, COLOR_LIGHTGREY);

        // Mercury Thermometer Graphic inside package
        tft_fill_rect(76, 36, 4, 16, COLOR_WHITE);
        tft_fill_rect(75, 50, 6, 6, COLOR_RED);
        tft_fill_rect(76, 42, 4, 9, COLOR_RED);
        tft_print(86, 37, "DS18B20", COLOR_CYAN, 0x2104, 1);
        tft_print(86, 49, "1-WIRE", COLOR_YELLOW, 0x2104, 1);

        // 3 Leads going down to badges:
        // Lead 1: GND
        tft_draw_vline(80, 64, 14, COLOR_WHITE);
        tft_fill_rect(62, 78, 36, 18, COLOR_NAVY);
        tft_draw_rect(62, 78, 36, 18, COLOR_CYAN);
        tft_print(64, 83, "1:GND", COLOR_CYAN, COLOR_NAVY, 1);

        // Lead 2: DQ
        tft_draw_vline(108, 64, 14, COLOR_WHITE);
        tft_fill_rect(100, 78, 30, 18, COLOR_NAVY);
        tft_draw_rect(100, 78, 30, 18, COLOR_YELLOW);
        tft_print(103, 83, "2:DQ", COLOR_YELLOW, COLOR_NAVY, 1);

        // Lead 3: VDD
        tft_draw_vline(136, 64, 14, COLOR_WHITE);
        tft_fill_rect(132, 78, 36, 18, COLOR_NAVY);
        tft_draw_rect(132, 78, 36, 18, COLOR_RED);
        tft_print(134, 83, "3:VDD", COLOR_RED, COLOR_NAVY, 1);

    } else if (strcmp(type, "DHT11") == 0 || strcmp(type, "DHT22") == 0) {
        // DHT Blue Sensor Case
        tft_fill_rect(68, 30, 80, 36, COLOR_BLUE);
        tft_draw_rect(68, 30, 80, 36, COLOR_CYAN);
        tft_draw_hline(74, 36, 68, COLOR_DARKCYAN);
        tft_draw_hline(74, 42, 68, COLOR_DARKCYAN);
        tft_print(74, 48, "DHT SENSOR", COLOR_WHITE, COLOR_BLUE, 1);

        // Leads: 1:VDD, 2:DAT, 3:GND
        tft_draw_vline(80, 66, 12, COLOR_WHITE);
        tft_fill_rect(62, 78, 36, 18, COLOR_NAVY);
        tft_draw_rect(62, 78, 36, 18, COLOR_RED);
        tft_print(64, 83, "1:VDD", COLOR_RED, COLOR_NAVY, 1);

        tft_draw_vline(108, 66, 12, COLOR_WHITE);
        tft_fill_rect(100, 78, 32, 18, COLOR_NAVY);
        tft_draw_rect(100, 78, 32, 18, COLOR_YELLOW);
        tft_print(102, 83, "2:DAT", COLOR_YELLOW, COLOR_NAVY, 1);

        tft_draw_vline(136, 66, 12, COLOR_WHITE);
        tft_fill_rect(134, 78, 36, 18, COLOR_NAVY);
        tft_draw_rect(134, 78, 36, 18, COLOR_CYAN);
        tft_print(136, 83, "3:GND", COLOR_CYAN, COLOR_NAVY, 1);

    } else if (strcmp(type, "CONTINUITY") == 0) {
        // Continuity Sound Waves
        tft_fill_rect(14, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(14, 42, 20, 22, COLOR_YELLOW);
        tft_draw_char(20, 46, pA, COLOR_YELLOW, COLOR_NAVY, 1);

        tft_fill_rect(34, 51, 152, 5, COLOR_YELLOW);

        // Sound waves: radiating bars and center dot
        tft_fill_rect(104, 50, 6, 7, COLOR_YELLOW);
        // Inner wave
        tft_draw_line(96, 44, 96, 62, COLOR_GREEN);
        tft_draw_line(118, 44, 118, 62, COLOR_GREEN);
        // Outer wave
        tft_draw_line(90, 40, 90, 66, COLOR_CYAN);
        tft_draw_line(124, 40, 124, 66, COLOR_CYAN);

        tft_print(64, 33, "(( BEEP! ))", COLOR_YELLOW, COLOR_BLACK, 1);

        tft_fill_rect(186, 42, 20, 22, COLOR_NAVY);
        tft_draw_rect(186, 42, 20, 22, COLOR_YELLOW);
        tft_draw_char(192, 46, pB, COLOR_YELLOW, COLOR_NAVY, 1);

    } else {
        // Idle: Open Leads with question mark
        tft_draw_hline(18, 53, 66, COLOR_DARKGREY);
        tft_draw_circle_placeholder(110, 53);
        tft_draw_hline(136, 53, 66, COLOR_DARKGREY);
        tft_print(68, 37, "[ OPEN LEADS ]", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    }

    // 2. Right Side: Mini Visual ZIF Socket Block (X: 224..310, Y: 30..132)
    tft_fill_rect(224, 30, 86, 102, 0x18C3); // Dark teal base
    tft_draw_rect(224, 30, 86, 102, COLOR_CYAN);
    tft_fill_rect(227, 34, 9, 94, 0x001F);  // Blue locking lever

    // 3 Pin Holes
    bool is_sensor = (strcmp(type, "DS18B20") == 0 || strcmp(type, "DS18S20") == 0 || 
                      strcmp(type, "DS1822") == 0 || strcmp(type, "DHT11") == 0 || strcmp(type, "DHT22") == 0);
    bool s1_act = is_sensor || (!is_open && (pA == '1' || pB == '1' || pC == '1'));
    bool s2_act = is_sensor || (!is_open && (pA == '2' || pB == '2' || pC == '2'));
    bool s3_act = is_sensor || (!is_open && (pA == '3' || pB == '3' || pC == '3'));

    tft_fill_rect(242, 36, 62, 26, s1_act ? COLOR_GREEN : COLOR_BLACK);
    tft_draw_rect(242, 36, 62, 26, s1_act ? COLOR_WHITE : COLOR_DARKGREY);
    tft_print(246, 44, s1_act ? "* PIN 1 *" : "  PIN 1  ", s1_act ? COLOR_BLACK : COLOR_WHITE, s1_act ? COLOR_GREEN : COLOR_BLACK, 1);

    tft_fill_rect(242, 68, 62, 26, s2_act ? COLOR_GREEN : COLOR_BLACK);
    tft_draw_rect(242, 68, 62, 26, s2_act ? COLOR_WHITE : COLOR_DARKGREY);
    tft_print(246, 76, s2_act ? "* PIN 2 *" : "  PIN 2  ", s2_act ? COLOR_BLACK : COLOR_WHITE, s2_act ? COLOR_GREEN : COLOR_BLACK, 1);

    tft_fill_rect(242, 100, 62, 26, s3_act ? COLOR_GREEN : COLOR_BLACK);
    tft_draw_rect(242, 100, 62, 26, s3_act ? COLOR_WHITE : COLOR_DARKGREY);
    tft_print(246, 108, s3_act ? "* PIN 3 *" : "  PIN 3  ", s3_act ? COLOR_BLACK : COLOR_WHITE, s3_act ? COLOR_GREEN : COLOR_BLACK, 1);
}

// Draw Dedicated Schematic Graphic for High-Voltage Zener Diode
static void draw_zener_graphic(uint32_t zener_mv, const char* status) {
    tft_fill_rect(8, 28, 304, 106, COLOR_BLACK);

    // Header inside Card 1
    tft_print(16, 32, "HIGH-VOLTAGE ZENER BREAKDOWN TEST", COLOR_YELLOW, COLOR_BLACK, 1);

    // Anode Terminal Box [ A ] (Left: X=18, Y=52, W=42, H=28)
    tft_fill_rect(18, 52, 42, 28, COLOR_NAVY);
    tft_draw_rect(18, 52, 42, 28, COLOR_WHITE);
    tft_print(24, 60, "[ A ]", COLOR_CYAN, COLOR_NAVY, 1);

    // Anode Lead line: (60, 66) to (120, 66)
    tft_draw_hline(60, 66, 60, COLOR_WHITE);

    // Zener Triangle (pointing right to X=150, Y=66):
    for (int i = 0; i < 30; i++) {
        int x = 120 + i;
        int half_h = 16 - (i * 16) / 30;
        tft_draw_vline(x, 66 - half_h, half_h * 2 + 1, COLOR_YELLOW);
    }

    // Zener Cathode Bar at X=150 (Y=48 to 85)
    tft_fill_rect(150, 48, 3, 37, COLOR_WHITE);

    // Zener Bent Wings:
    // Top wing: bends left from (150, 48) to (142, 48)
    tft_fill_rect(142, 48, 9, 3, COLOR_WHITE);
    // Bottom wing: bends right from (150, 82) to (158, 82)
    tft_fill_rect(150, 82, 9, 3, COLOR_WHITE);

    // Cathode Lead line: (153, 66) to (214, 66)
    tft_draw_hline(153, 66, 61, COLOR_WHITE);

    // Cathode Terminal Box [ K ] (Right: X=214, Y=52, W=42, H=28)
    tft_fill_rect(214, 52, 42, 28, COLOR_NAVY);
    tft_draw_rect(214, 52, 42, 28, COLOR_WHITE);
    tft_print(220, 60, "[ K ]", 0xFD20, COLOR_NAVY, 1);

    // Readout badge at Y=96
    char buf[36];
    if (zener_mv > 24000) {
        tft_print(16, 96, "Vz: >28.0V (OPEN / NO DIODE) ", COLOR_LIGHTGREY, COLOR_BLACK, 2);
    } else if (zener_mv >= 1000) {
        snprintf(buf, sizeof(buf), "Vz: %lu.%02lu V  (ZENER CLAMP)", zener_mv / 1000, (zener_mv % 1000) / 10);
        tft_print(16, 96, buf, COLOR_GREEN, COLOR_BLACK, 2);
    } else if (zener_mv >= 350) {
        snprintf(buf, sizeof(buf), "Vf: %lu.%02lu V  (FORWARD DIODE)", zener_mv / 1000, (zener_mv % 1000) / 10);
        tft_print(16, 96, buf, COLOR_CYAN, COLOR_BLACK, 2);
    } else {
        tft_print(16, 96, "SHORT CIRCUIT (< 0.2V)      ", COLOR_RED, COLOR_BLACK, 2);
    }
}

// Draw Dedicated UI Graphic for Programmable Signal Generator (PB8) & SMA Voltmeter (PA6)
static void draw_siggen_graphic(uint32_t freq_hz, bool is_on, uint16_t sma_mv, bool redraw_all) {
    // Card 1 Header inside Card 1 (Y: 26..136)
    if (redraw_all) {
        tft_print(12, 32, "PWM SIGNAL GENERATOR (PB8)", COLOR_CYAN, COLOR_BLACK, 1);
    }

    // Output State Badge at (X: 190, Y: 46)
    if (is_on) {
        tft_fill_rect(190, 46, 116, 22, COLOR_GREEN);
        tft_draw_rect(190, 46, 116, 22, COLOR_WHITE);
        tft_print(200, 52, "OUTPUT: ON ", COLOR_BLACK, COLOR_GREEN, 1);
    } else {
        tft_fill_rect(190, 46, 116, 22, 0x2104);
        tft_draw_rect(190, 46, 116, 22, COLOR_DARKGREY);
        tft_print(200, 52, "OUTPUT: OFF", COLOR_LIGHTGREY, 0x2104, 1);
    }

    // Large Frequency Readout (Size 2, X: 16, Y: 48)
    char fbuf[24];
    if (freq_hz >= 1000) {
        uint32_t khz = freq_hz / 1000;
        uint32_t rem = (freq_hz % 1000) / 10;
        snprintf(fbuf, sizeof(fbuf), "%2lu.%02lu kHz  ", khz, rem);
    } else {
        snprintf(fbuf, sizeof(fbuf), "%4lu Hz     ", freq_hz);
    }
    tft_print(16, 48, fbuf, COLOR_YELLOW, COLOR_BLACK, 2);

    // Square wave graphic across (X: 16..296, Y: 76..106)
    tft_fill_rect(16, 76, 284, 34, COLOR_BLACK);
    uint16_t wave_col = is_on ? COLOR_GREEN : COLOR_DARKGREY;
    int16_t y_low = 104;
    int16_t y_hi  = 78;
    int16_t x = 18;
    int16_t pw = 28; // pulse width
    while (x + pw * 2 < 296) {
        tft_draw_hline(x, y_hi, pw, wave_col);
        tft_draw_vline(x + pw, y_hi, y_low - y_hi, wave_col);
        tft_draw_hline(x + pw, y_low, pw, wave_col);
        tft_draw_vline(x + pw * 2, y_hi, y_low - y_hi, wave_col);
        x += pw * 2;
    }

    // Hint footer inside Card 1
    tft_print(16, 118, "UP/DN: Frequency | TEST: Output Toggle", COLOR_YELLOW, COLOR_BLACK, 1);

    // Card 2: SMA External Voltmeter (Y: 138..236)
    if (redraw_all) {
        tft_print(12, 144, "SMA EXTERNAL VOLTMETER (PA6 - Pin 22)", COLOR_YELLOW, COLOR_BLACK, 1);
    }

    // Large Voltage Readout (Size 2, X: 16, Y: 160)
    char vbuf[32];
    uint32_t v_int = sma_mv / 1000;
    uint32_t v_dec = (sma_mv % 1000) / 10;
    snprintf(vbuf, sizeof(vbuf), "V_in = %lu.%02lu V   ", v_int, v_dec);
    tft_print(16, 160, vbuf, COLOR_CYAN, COLOR_BLACK, 2);

    // Visual Analog Bargraph (X: 16..296, Y: 184, W: 280, H: 12)
    tft_draw_rect(16, 184, 280, 12, COLOR_WHITE);
    uint16_t bar_w = (sma_mv > 3300) ? 276 : (uint16_t)(((uint32_t)sma_mv * 276UL) / 3300UL);
    if (bar_w > 0) {
        uint16_t b_col = (sma_mv > 2800) ? COLOR_RED : (sma_mv > 1500) ? COLOR_YELLOW : COLOR_GREEN;
        tft_fill_rect(18, 186, bar_w, 8, b_col);
    }
    if (bar_w < 276) {
        tft_fill_rect(18 + bar_w, 186, 276 - bar_w, 8, COLOR_BLACK);
    }

    // Scale markers below bar
    tft_print(16, 198, "0.0V", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    tft_print(144, 198, "1.65V", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    tft_print(266, 198, "3.3V+", COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // Telemetry Footer
    char tbuf[48];
    snprintf(tbuf, sizeof(tbuf), "VCC:%lu.%02luV | BATT:%lu.%02luV (%u%%) ",
             l083_data.vcc_mv / 1000, (l083_data.vcc_mv % 1000) / 10,
             (uint32_t)batt_mv / 1000, ((uint32_t)batt_mv % 1000) / 10, batt_pct);
    tft_print(12, 222, tbuf, COLOR_GREEN, COLOR_BLACK, 1);
}

// Redraw static screen background and frames
static void draw_screen_background() {
    tft_fill_screen(0x0842); // Clean dark navy

    // Header Banner (Y: 0..24)
    tft_fill_rect(0, 0, TFT_WIDTH, 24, 0x0010);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);

    if (screen_mode == 0) {
        // Mode 0: Graphical Component Tester & Digital Sensors
        tft_print(8, 7, "TESTER", COLOR_YELLOW, 0x0010, 1);
        tft_print(110, 7, "[M] IR DEC", COLOR_CYAN, 0x0010, 1);

        // Card 1: Schematic Graphic Window (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, COLOR_GREEN);

        // Card 2: Socket Terminal Readings & Telemetry (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_CYAN);

    } else if (screen_mode == 1) {
        // Mode 1: Infrared Remote Decoder Console
        tft_print(8, 7, "IR DECODER", COLOR_YELLOW, 0x0010, 1);
        tft_print(110, 7, "[M] ZENER ", COLOR_CYAN, 0x0010, 1);

        // Card 1: IR Decoder Dashboard (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, 0xF81F); // Magenta

        // Card 2: Decoded History Log (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_CYAN);

    } else if (screen_mode == 2) {
        // Mode 2: High-Voltage Zener Diode Tester (0-30V)
        tft_print(8, 7, "ZENER(30V)", COLOR_YELLOW, 0x0010, 1);
        tft_print(110, 7, "[M] SIGGEN", COLOR_CYAN, 0x0010, 1);

        // Card 1: Zener Schematic Window (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, 0xFD20); // Orange / Gold

        // Card 2: Breakdown Analysis & Parameters (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_CYAN);

    } else {
        // Mode 3: Signal Generator & SMA Voltmeter
        tft_print(8, 7, "SIG GEN&VOLT", COLOR_YELLOW, 0x0010, 1);
        tft_print(110, 7, "[M] TESTER", COLOR_CYAN, 0x0010, 1);

        // Card 1: PWM Signal Generator (PB8) (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, COLOR_GREEN);

        // Card 2: SMA External Voltmeter (PA6) (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_YELLOW);
    }

    draw_battery_indicator(true);
}

void setup() {
    tft_init();
    Serial1.setRx(PA10);
    Serial1.setTx(PA9);
    Serial1.begin(38400);
    draw_screen_background();
}

void loop() {
    poll_uart();

    // Screen Switching: Press MENU button to toggle between 4 screens
    static bool prev_b_menu = false;
    uint8_t m = l083_data.btn_mask;
    bool b_menu = (m & 0x10) != 0;

    if (b_menu && !prev_b_menu) {
        screen_mode = (screen_mode + 1) % 4;
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
        poll_uart();
        draw_battery_indicator(need_full_redraw);

        bool b_test = (m & 0x08) != 0;
        char buf[64];

        if (screen_mode == 0) {
            // ================================================================
            // SCREEN 0: COMPONENT TESTER & 1-WIRE / DIGITAL SENSORS
            // ================================================================
            static char prev_type[16] = "";
            static char prev_pinout[28] = "";
            if (strcmp(l083_data.comp_type, prev_type) != 0 || 
                strcmp(l083_data.comp_pinout, prev_pinout) != 0 || 
                need_full_redraw) {
                draw_schematic_graphic(l083_data.comp_type, l083_data.comp_pinout);
                strncpy(prev_type, l083_data.comp_type, sizeof(prev_type) - 1);
                strncpy(prev_pinout, l083_data.comp_pinout, sizeof(prev_pinout) - 1);
            }

            static char prev_val_display[24] = "";
            static char prev_pin_display[28] = "";
            static char prev_type_display[28] = "";

            bool is_empty = (strcmp(l083_data.comp_type, "NONE") == 0 || strlen(l083_data.comp_type) == 0);
            if (is_empty) {
                if (need_full_redraw || strcmp(prev_val_display, "READY") != 0) {
                    tft_print(16, 81, "READY               ", COLOR_LIGHTGREY, COLOR_BLACK, 2);
                    tft_print(16, 104, "Socket 1-2-3 (Auto-detect) ", COLOR_YELLOW, COLOR_BLACK, 1);
                    tft_print(16, 120, "DS18B20, DHT, RLC, BJT, MOS", COLOR_CYAN, COLOR_BLACK, 1);
                    strcpy(prev_val_display, "READY");
                    prev_pin_display[0] = '\0';
                    prev_type_display[0] = '\0';
                }
            } else {
                char val_pad[24];
                snprintf(val_pad, sizeof(val_pad), "%-12s", l083_data.comp_val);
                if (need_full_redraw || strcmp(prev_val_display, val_pad) != 0) {
                    tft_print(16, 81, val_pad, COLOR_YELLOW, COLOR_BLACK, 2);
                    strncpy(prev_val_display, val_pad, sizeof(prev_val_display) - 1);
                }

                char pin_pad[28];
                snprintf(pin_pad, sizeof(pin_pad), "%-25s", l083_data.comp_pinout);
                if (need_full_redraw || strcmp(prev_pin_display, pin_pad) != 0) {
                    tft_print(16, 104, pin_pad, COLOR_CYAN, COLOR_BLACK, 1);
                    strncpy(prev_pin_display, pin_pad, sizeof(prev_pin_display) - 1);
                }

                char type_pad[28];
                snprintf(type_pad, sizeof(type_pad), "Type: %-19s", l083_data.comp_type);
                if (need_full_redraw || strcmp(prev_type_display, type_pad) != 0) {
                    tft_print(16, 120, type_pad, COLOR_WHITE, COLOR_BLACK, 1);
                    strncpy(prev_type_display, type_pad, sizeof(prev_type_display) - 1);
                }
            }

            // Bottom Card: Socket Terminals & Voltage Readings
            static uint16_t prev_s1 = 9999, prev_s2 = 9999, prev_s3 = 9999;
            if (need_full_redraw || abs((int)l083_data.s1_mv - (int)prev_s1) > 50) {
                snprintf(buf, sizeof(buf), "%4u mV", l083_data.s1_mv);
                draw_btn(10, 144, 94, 34, "PIN 1: PA3", buf, (l083_data.s1_mv > 100));
                prev_s1 = l083_data.s1_mv;
            }
            if (need_full_redraw || abs((int)l083_data.s2_mv - (int)prev_s2) > 50) {
                snprintf(buf, sizeof(buf), "%4u mV", l083_data.s2_mv);
                draw_btn(112, 144, 94, 34, "PIN 2: PA4", buf, (l083_data.s2_mv > 100));
                prev_s2 = l083_data.s2_mv;
            }
            if (need_full_redraw || abs((int)l083_data.s3_mv - (int)prev_s3) > 50) {
                snprintf(buf, sizeof(buf), "%4u mV", l083_data.s3_mv);
                draw_btn(214, 144, 94, 34, "PIN 3: PA5", buf, (l083_data.s3_mv > 100));
                prev_s3 = l083_data.s3_mv;
            }

            tft_print(12, 184, "RL: 680R (PA7,PB1,PB10) | RH: 470k", COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(12, 198, "Sense: PA3(1), PA4(2), PA5(3)     ", COLOR_CYAN, COLOR_BLACK, 1);
            tft_print(12, 212, "TEST=Measure | MENU=Switch IR Dec ", 0x07E0, COLOR_BLACK, 1);

            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%lu.%02luV | BATT:%lu.%02luV (%u%%)  ",
                         l083_data.vcc_mv / 1000, (l083_data.vcc_mv % 1000) / 10,
                         (uint32_t)batt_mv / 1000, ((uint32_t)batt_mv % 1000) / 10, batt_pct);
                tft_print(12, 224, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                snprintf(buf, sizeof(buf), "LINK:WAITING | BATT:%lu.%02luV (%u%%)",
                         (uint32_t)batt_mv / 1000, ((uint32_t)batt_mv % 1000) / 10, batt_pct);
                tft_print(12, 224, buf, COLOR_RED, COLOR_BLACK, 1);
            }
            poll_uart();

        } else if (screen_mode == 1) {
            // ================================================================
            // SCREEN 1: DEDICATED INFRARED REMOTE DECODER
            // ================================================================
            static bool prev_received = false;
            static uint8_t prev_hist_cnt = 255;

            if (b_test && !b_menu) {
                ir_rx.received = false;
                ir_rx.hist_cnt = 0;
                ir_rx.raw_code = 0;
            }

            if (ir_rx.received != prev_received || need_full_redraw) {
                tft_fill_rect(5, 27, 310, 108, COLOR_BLACK);
                prev_received = ir_rx.received;
            }

            if (!ir_rx.received) {
                tft_print(16, 36, "[ WAITING FOR IR REMOTE... ]    ", COLOR_YELLOW, COLOR_BLACK, 1);
                tft_print(16, 54, "Point remote at front sensor PB9", COLOR_WHITE, COLOR_BLACK, 1);
                tft_print(16, 72, "Supported: NEC / Samsung / Sony ", COLOR_CYAN, COLOR_BLACK, 1);
                tft_print(16, 90, "Press any button on remote...   ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
                tft_print(16, 110, "Sensor: PB9 Demodulator (Active)", 0x07E0, COLOR_BLACK, 1);
            } else {
                bool is_recent = (millis() - ir_rx.last_rx_ms < 600);
                if (is_recent) {
                    tft_print(16, 30, ">> IR FRAME DECODED! <<         ", COLOR_GREEN, COLOR_BLACK, 1);
                } else {
                    tft_print(16, 30, "-- LAST RECEIVED IR CODE --     ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
                }

                snprintf(buf, sizeof(buf), "CMD:  0x%02X (%3u)", ir_rx.command, ir_rx.command);
                tft_print(16, 44, buf, COLOR_YELLOW, COLOR_BLACK, 2);

                snprintf(buf, sizeof(buf), "ADDR: 0x%04X | Proto: %-5s", ir_rx.address, ir_rx.protocol);
                tft_print(16, 66, buf, COLOR_CYAN, COLOR_BLACK, 1);

                snprintf(buf, sizeof(buf), "RAW:  0x%08lX | Rpt: %-3lu", ir_rx.raw_code, ir_rx.repeats);
                tft_print(16, 82, buf, COLOR_WHITE, COLOR_BLACK, 1);

                tft_print(16, 98, "Bits: [MSB]", COLOR_DARKGREY, COLOR_BLACK, 1);
                tft_print(250, 98, "[LSB]", COLOR_DARKGREY, COLOR_BLACK, 1);
                for (int b = 0; b < 32; b++) {
                    bool bit_val = (ir_rx.raw_code & (1UL << (31 - b))) != 0;
                    uint16_t b_col = bit_val ? COLOR_GREEN : 0x001F;
                    tft_fill_rect(16 + b * 7, 108, 6, 8, b_col);
                }

                uint32_t ago = (millis() - ir_rx.last_rx_ms) / 1000;
                snprintf(buf, sizeof(buf), "Rx: %lu sec ago (%s)            ",
                         ago, (ago < 1) ? "ACTIVE" : "IDLE");
                tft_print(16, 120, buf, (ago < 1) ? COLOR_GREEN : COLOR_LIGHTGREY, COLOR_BLACK, 1);
            }

            if (ir_rx.hist_cnt != prev_hist_cnt || need_full_redraw) {
                tft_fill_rect(5, 154, 310, 56, COLOR_BLACK);
                prev_hist_cnt = ir_rx.hist_cnt;
            }

            tft_print(12, 142, "DECODED IR HISTORY (Recent Keys):", COLOR_CYAN, COLOR_BLACK, 1);

            if (ir_rx.hist_cnt == 0) {
                tft_print(16, 160, "No recent codes in buffer.      ", COLOR_DARKGREY, COLOR_BLACK, 1);
                tft_print(16, 176, "Point remote & press button.    ", COLOR_DARKGREY, COLOR_BLACK, 1);
            } else {
                for (int h = 0; h < 3; h++) {
                    int16_t hy = 158 + (h * 16);
                    if (h < ir_rx.hist_cnt) {
                        snprintf(buf, sizeof(buf), "#%d: Cmd:0x%02X Addr:0x%04X Raw:%08lX",
                                 h + 1, ir_rx.hist_cmd[h], ir_rx.hist_addr[h], ir_rx.hist_raw[h]);
                        tft_print(16, hy, buf, (h == 0) ? COLOR_YELLOW : COLOR_WHITE, COLOR_BLACK, 1);
                    }
                }
            }

            tft_print(12, 212, "Sensor: PB9 Demodulator (Active-L)", COLOR_LIGHTGREY, COLOR_BLACK, 1);
            if (l083_data.connected) {
                tft_print(12, 224, "MENU=Next(Zener) | TEST=Clear | OK", 0x07E0, COLOR_BLACK, 1);
            } else {
                snprintf(buf, sizeof(buf), "MENU=Next | LINK:OFFLINE (RX:%luB)", raw_rx_bytes);
                tft_print(12, 224, buf, COLOR_RED, COLOR_BLACK, 1);
            }
            poll_uart();

        } else if (screen_mode == 2) {
            // ================================================================
            // SCREEN 2: HIGH-VOLTAGE ZENER DIODE TESTER (0-30V)
            // ================================================================
            static uint32_t prev_zm = 999999;
            static char prev_zstat[24] = "";

            if (abs((long)l083_data.zener_mv - (long)prev_zm) > 200 || 
                strcmp(l083_data.zener_status, prev_zstat) != 0 || 
                need_full_redraw) {
                draw_zener_graphic(l083_data.zener_mv, l083_data.zener_status);
                prev_zm = l083_data.zener_mv;
                strncpy(prev_zstat, l083_data.zener_status, sizeof(prev_zstat) - 1);
            }

            tft_print(12, 144, "ZENER BREAKDOWN ANALYSIS:", COLOR_CYAN, COLOR_BLACK, 1);

            char zbuf[40];
            if (l083_data.zener_mv > 24000) {
                snprintf(zbuf, sizeof(zbuf), "Voltage: >28.0 V (Open Circuit)  ");
            } else if (l083_data.zener_mv >= 1000) {
                snprintf(zbuf, sizeof(zbuf), "Voltage: %lu.%02lu V (Zener Breakdown)", 
                         l083_data.zener_mv / 1000, (l083_data.zener_mv % 1000) / 10);
            } else if (l083_data.zener_mv >= 350) {
                snprintf(zbuf, sizeof(zbuf), "Voltage: %lu.%02lu V (Forward Diode) ", 
                         l083_data.zener_mv / 1000, (l083_data.zener_mv % 1000) / 10);
            } else {
                snprintf(zbuf, sizeof(zbuf), "Voltage: <0.20 V (Short Circuit) ");
            }
            tft_print(12, 160, zbuf, COLOR_YELLOW, COLOR_BLACK, 1);

            tft_print(12, 176, "Socket:  K (Cathode) | A (Anode) ", COLOR_WHITE, COLOR_BLACK, 1);
            tft_print(12, 192, "Boost:   MT3608 2A Step-Up (~28V)", 0xFD20, COLOR_BLACK, 1);
            tft_print(12, 208, "TEST=Sample | MENU=Next Screen   ", 0x07E0, COLOR_BLACK, 1);

            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%lu.%02luV | BATT:%lu.%02luV (%u%%)  ",
                         l083_data.vcc_mv / 1000, (l083_data.vcc_mv % 1000) / 10,
                         (uint32_t)batt_mv / 1000, ((uint32_t)batt_mv % 1000) / 10, batt_pct);
                tft_print(12, 224, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                snprintf(buf, sizeof(buf), "LINK:WAITING | BATT:%lu.%02luV (%u%%)",
                         (uint32_t)batt_mv / 1000, ((uint32_t)batt_mv % 1000) / 10, batt_pct);
                tft_print(12, 224, buf, COLOR_RED, COLOR_BLACK, 1);
            }
            poll_uart();

        } else if (screen_mode == 3) {
            // ================================================================
            // SCREEN 3: PROGRAMMABLE SIGNAL GENERATOR & SMA VOLTMETER
            // ================================================================
            static uint32_t prev_sig_freq = 0xFFFFFFFF;
            static uint8_t  prev_sig_on   = 0xFF;
            static uint16_t prev_sma_mv   = 0xFFFF;

            bool freq_changed = (l083_data.siggen_freq != prev_sig_freq);
            bool state_changed = (l083_data.siggen_on != prev_sig_on);
            bool sma_changed = (abs((int)l083_data.sma_mv - (int)prev_sma_mv) > 20);

            if (need_full_redraw || freq_changed || state_changed || sma_changed) {
                draw_siggen_graphic(l083_data.siggen_freq, l083_data.siggen_on != 0,
                                    l083_data.sma_mv, need_full_redraw);
                prev_sig_freq = l083_data.siggen_freq;
                prev_sig_on   = l083_data.siggen_on;
                prev_sma_mv   = l083_data.sma_mv;
            }
            poll_uart();
        }
    }
}
