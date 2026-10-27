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

    // Zener Diode Fields
    uint32_t zener_mv;
    char zener_status[24];
};

static L083Telemetry l083_data = {
    0, "NONE", "NONE", 0, 255, 3300, 0, 0, false,
    "NONE", "No component", "Pins 1-2-3 Open", 0, 0, 0,
    0, "STANDBY"
};
static char last_raw_pkt[64] = "Waiting for UART...";

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

// Screen Mode: 0 = Hardware Console, 1 = Component Tester, 2 = IR Remote Decoder
static uint8_t screen_mode = 1;
static bool need_full_redraw = true;

// Serial Parser
static char rx_buf[160];
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
    char type_str[16] = "NONE", val_str[24] = "No component", pinout_str[20] = "---";
    unsigned int s1_v = 0, s2_v = 0, s3_v = 0;
    unsigned long zener_v = 0;
    char zener_stat_str[24] = "STANDBY";

    int n = sscanf(pkt + 5, "%lu,%23[^,],%15[^,],%x,%u,%lu,%lu,%15[^,],%23[^,],%19[^,],%u,%u,%u,%lu,%23[^\n\r*]",
                   &seq, btn_str, pin_str, &mask_val, &bl_val, &vcc, &uptime,
                   type_str, val_str, pinout_str, &s1_v, &s2_v, &s3_v,
                   &zener_v, zener_stat_str);

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
    // 1. Left Schematic Canvas: X: 10..220, Y: 30..76 (Center Y = 53)
    tft_fill_rect(10, 30, 210, 46, COLOR_BLACK);

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
        // Flat Base on Left at X=92, Pointing Right to tip at X=110 touching Cathode Bar at X=111
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
    bool s1_act = !is_open && (pA == '1' || pB == '1' || pC == '1');
    bool s2_act = !is_open && (pA == '2' || pB == '2' || pC == '2');
    bool s3_act = !is_open && (pA == '3' || pB == '3' || pC == '3');

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
        snprintf(buf, sizeof(buf), "Vz: %4.2f V  (ZENER CLAMP)", zener_mv / 1000.0f);
        tft_print(16, 96, buf, COLOR_GREEN, COLOR_BLACK, 2);
    } else if (zener_mv >= 350) {
        snprintf(buf, sizeof(buf), "Vf: %4.2f V  (FORWARD DIODE)", zener_mv / 1000.0f);
        tft_print(16, 96, buf, COLOR_CYAN, COLOR_BLACK, 2);
    } else {
        tft_print(16, 96, "SHORT CIRCUIT (< 0.2V)      ", COLOR_RED, COLOR_BLACK, 2);
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
        tft_print(232, 7, "[M] TESTER ", COLOR_CYAN, 0x0010, 1);

        // Card 1: Keypad Matrix (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, COLOR_DARKCYAN);

        // Card 2: Diagnostics (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_YELLOW);

    } else if (screen_mode == 1) {
        // Mode 1: Graphical Component Tester Console
        tft_print(8, 7, "DSO-TC2 COMPONENT TESTER", COLOR_YELLOW, 0x0010, 1);
        tft_print(232, 7, "[M] IR DEC ", COLOR_CYAN, 0x0010, 1);

        // Card 1: Schematic Graphic Window (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, COLOR_GREEN);

        // Card 2: Socket Terminal Readings & Telemetry (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_CYAN);

    } else if (screen_mode == 2) {
        // Mode 2: Infrared Remote Decoder Console
        tft_print(8, 7, "DSO-TC2 IR DECODER (38k)", COLOR_YELLOW, 0x0010, 1);
        tft_print(232, 7, "[M] ZENER  ", COLOR_CYAN, 0x0010, 1);

        // Card 1: IR Decoder Dashboard (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, 0xF81F); // Magenta

        // Card 2: Decoded History Log (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_CYAN);

    } else {
        // Mode 3: High-Voltage Zener Diode Tester
        tft_print(8, 7, "DSO-TC2 ZENER TESTER(30V)", COLOR_YELLOW, 0x0010, 1);
        tft_print(232, 7, "[M] CONSOLE", COLOR_CYAN, 0x0010, 1);

        // Card 1: Zener Schematic Window (Y: 26..136)
        tft_fill_rect(4, 26, 312, 110, COLOR_BLACK);
        tft_draw_rect(4, 26, 312, 110, 0xFD20); // Orange / Gold

        // Card 2: Breakdown Analysis & Parameters (Y: 138..236)
        tft_fill_rect(4, 138, 312, 98, COLOR_BLACK);
        tft_draw_rect(4, 138, 312, 98, COLOR_CYAN);
    }
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
    if (millis() - last_ui >= 50) { // 20 Hz UI refresh
        last_ui = millis();
        poll_uart();

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
            static uint8_t prev_m_drawn = 0xFF;
            if (need_full_redraw || m != prev_m_drawn) {
                prev_m_drawn = m;
                draw_btn(12, 30, 92, 28, "POWER", "PB13", b_pwr);
                draw_btn(114, 30, 92, 28, "UP ^", "PC8", b_up);
                draw_btn(216, 30, 92, 28, "MENU", "PB15", b_menu);

                draw_btn(12, 64, 92, 28, "OK / <", "PB14", b_ok);
                draw_btn(114, 64, 92, 28, "RELAY", b_relay ? "ON:PB6" : "OFF:PB6", b_relay);
                draw_btn(216, 64, 92, 28, "TEST / >", "PA8", b_test);

                draw_btn(12, 98, 92, 28, "BUZZER", b_buzz ? "ACTIVE" : "PC6", b_buzz);
                draw_btn(114, 98, 92, 28, "DOWN v", "PC9", b_down);
                draw_btn(216, 98, 92, 28, "IR RX", b_ir ? "SIGNAL!" : "PB9", b_ir);
            }

            if (b_pwr) {
                tft_print(10, 144, ">> POWER: PB13 (Hold 1.5s=Off)  ", COLOR_RED, COLOR_BLACK, 1);
            } else if (b_ir) {
                tft_print(10, 144, ">> IR SIGNAL DETECTED ON PB9! <<", COLOR_CYAN, COLOR_BLACK, 1);
            } else if ((m & 0x3F) != 0) {
                snprintf(buf, sizeof(buf), ">> KEY: %-5s  PIN: %-4s (0x%02X)   ", 
                         l083_data.active_btn, l083_data.active_pin, m);
                tft_print(10, 144, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                tft_print(10, 144, "STATUS: All 6 Keys & HW Ready   ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
            }

            tft_print(10, 160, "TESTER SENSE: 1=PA3  2=PA4  3=PA5", COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(10, 176, "RL:PA7,PB1,PB10 | RH:PB0,PB2,PB11", COLOR_CYAN, COLOR_BLACK, 1);

            uint8_t bl_pct = (uint16_t)l083_data.backlight * 100 / 255;
            snprintf(buf, sizeof(buf), "BL:%2u%% | PWR:PB7 | RLY:%-3s | BUZ:PC6", 
                     bl_pct, b_relay ? "ON" : "OFF");
            tft_print(10, 192, buf, COLOR_WHITE, COLOR_BLACK, 1);

            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%0.2fV | L083:ONLINE (%lu pkts)  ",
                         l083_data.vcc_mv / 1000.0f, l083_data.total_packets);
                tft_print(10, 208, buf, 0x07E0, COLOR_BLACK, 1);
            } else {
                snprintf(buf, sizeof(buf), "L083 OFFLINE (RX:%lu B, Pkt:%lu) ",
                         raw_rx_bytes, l083_data.total_packets);
                tft_print(10, 208, buf, COLOR_RED, COLOR_BLACK, 1);
            }

            tft_print(10, 222, "Press MENU to switch to Tester  ", COLOR_LIGHTGREY, COLOR_BLACK, 1);
            poll_uart();

        } else if (screen_mode == 1) {
            // ================================================================
            // SCREEN 1: DEDICATED GRAPHICAL COMPONENT TESTER
            // ================================================================
            // 1. Draw Visual Schematic Graphic & ZIF Socket
            static char prev_type[16] = "";
            static char prev_pinout[20] = "";
            if (strcmp(l083_data.comp_type, prev_type) != 0 || 
                strcmp(l083_data.comp_pinout, prev_pinout) != 0 || 
                need_full_redraw) {
                draw_schematic_graphic(l083_data.comp_type, l083_data.comp_pinout);
                strncpy(prev_type, l083_data.comp_type, sizeof(prev_type) - 1);
                strncpy(prev_pinout, l083_data.comp_pinout, sizeof(prev_pinout) - 1);
            }

            // 2. Component Measurement Text (Flicker-free padded printing)
            static char prev_val_display[24] = "";
            static char prev_pin_display[28] = "";
            static char prev_type_display[28] = "";

            bool is_empty = (strcmp(l083_data.comp_type, "NONE") == 0 || strlen(l083_data.comp_type) == 0);
            if (is_empty) {
                if (need_full_redraw || strcmp(prev_val_display, "READY") != 0) {
                    tft_print(16, 81, "READY               ", COLOR_LIGHTGREY, COLOR_BLACK, 2);
                    tft_print(16, 104, "Insert component in socket", COLOR_YELLOW, COLOR_BLACK, 1);
                    tft_print(16, 120, "Press TEST to measure     ", COLOR_GREEN, COLOR_BLACK, 1);
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

            // Bottom Card: Socket Terminals & Voltage Readings (Differential redraw)
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

            // Status lines inside Card 2
            tft_print(12, 184, "RL: 680R (PA7,PB1,PB10) | RH: 470k", COLOR_YELLOW, COLOR_BLACK, 1);
            tft_print(12, 198, "Sense: PA3(1), PA4(2), PA5(3)     ", COLOR_CYAN, COLOR_BLACK, 1);
            tft_print(12, 212, "TEST=Measure | MENU=Switch IR Dec ", 0x07E0, COLOR_BLACK, 1);

            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%0.2fV | LINK:OK (%lu pkts)   ",
                         l083_data.vcc_mv / 1000.0f, l083_data.total_packets);
                tft_print(12, 224, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                snprintf(buf, sizeof(buf), "LINK:WAITING (RX:%lu B, Pkt:%lu)  ",
                         raw_rx_bytes, l083_data.total_packets);
                tft_print(12, 224, buf, COLOR_RED, COLOR_BLACK, 1);
            }
            poll_uart();

        } else if (screen_mode == 2) {
            // ================================================================
            // SCREEN 2: DEDICATED INFRARED REMOTE DECODER
            // ================================================================
            static bool prev_received = false;
            static uint8_t prev_hist_cnt = 255;

            // Clear test log if TEST pressed without holding MENU
            if (b_test && !b_menu) {
                ir_rx.received = false;
                ir_rx.hist_cnt = 0;
                ir_rx.raw_code = 0;
            }

            // Cleanly clear Card 1 on state change or full redraw
            if (ir_rx.received != prev_received || need_full_redraw) {
                tft_fill_rect(5, 27, 310, 108, COLOR_BLACK);
                prev_received = ir_rx.received;
            }

            // Card 1: Decoder Dashboard (Y: 26..136)
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

                // Line 2: Large Command (Y: 44, Size 2)
                snprintf(buf, sizeof(buf), "CMD:  0x%02X (%3u)", ir_rx.command, ir_rx.command);
                tft_print(16, 44, buf, COLOR_YELLOW, COLOR_BLACK, 2);

                // Line 3: Address & Protocol (Y: 66, Size 1)
                snprintf(buf, sizeof(buf), "ADDR: 0x%04X | Proto: %-5s", ir_rx.address, ir_rx.protocol);
                tft_print(16, 66, buf, COLOR_CYAN, COLOR_BLACK, 1);

                // Line 4: Raw 32-bit Code & Repeats (Y: 82, Size 1)
                snprintf(buf, sizeof(buf), "RAW:  0x%08lX | Rpt: %-3lu", ir_rx.raw_code, ir_rx.repeats);
                tft_print(16, 82, buf, COLOR_WHITE, COLOR_BLACK, 1);

                // Line 5: 32-Bit Visual Pulse Bar (Y: 98, Size 1)
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

            // Card 2: History Log (Y: 138..236)
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

        } else if (screen_mode == 3) {
            // ================================================================
            // SCREEN 3: HIGH-VOLTAGE ZENER DIODE TESTER (0-30V)
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

            // Card 2: Zener Breakdown Details (Y: 138..236)
            tft_print(12, 144, "ZENER BREAKDOWN ANALYSIS:", COLOR_CYAN, COLOR_BLACK, 1);

            char zbuf[40];
            if (l083_data.zener_mv > 24000) {
                snprintf(zbuf, sizeof(zbuf), "Voltage: >28.0 V (Open Circuit)  ");
            } else if (l083_data.zener_mv >= 1000) {
                snprintf(zbuf, sizeof(zbuf), "Voltage: %4.2f V (Zener Breakdown)", l083_data.zener_mv / 1000.0f);
            } else if (l083_data.zener_mv >= 350) {
                snprintf(zbuf, sizeof(zbuf), "Voltage: %4.2f V (Forward Diode) ", l083_data.zener_mv / 1000.0f);
            } else {
                snprintf(zbuf, sizeof(zbuf), "Voltage: <0.20 V (Short Circuit) ");
            }
            tft_print(12, 160, zbuf, COLOR_YELLOW, COLOR_BLACK, 1);

            tft_print(12, 176, "Socket:  K (Cathode) | A (Anode) ", COLOR_WHITE, COLOR_BLACK, 1);
            tft_print(12, 192, "Boost:   MT3608 2A Step-Up (~28V)", 0xFD20, COLOR_BLACK, 1);
            tft_print(12, 208, "TEST=Sample | MENU=Next Screen   ", 0x07E0, COLOR_BLACK, 1);

            if (l083_data.connected) {
                snprintf(buf, sizeof(buf), "VCC:%0.2fV | LINK:OK (%lu pkts)   ",
                         l083_data.vcc_mv / 1000.0f, l083_data.total_packets);
                tft_print(12, 224, buf, COLOR_GREEN, COLOR_BLACK, 1);
            } else {
                snprintf(buf, sizeof(buf), "LINK:WAITING (RX:%lu B, Pkt:%lu)  ",
                         raw_rx_bytes, l083_data.total_packets);
                tft_print(12, 224, buf, COLOR_RED, COLOR_BLACK, 1);
            }
            poll_uart();
        }
    }
}
