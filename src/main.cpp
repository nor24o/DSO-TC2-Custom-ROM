/**
 * @file main.cpp
 * @brief FNIRSI DSO-TC2 Custom ROM - Component Tester Hardware Mapper & Diagnostic Engine
 */

#include <Arduino.h>
#include <RTTStream.h>
#include "tft.h"

RTTStream rtt;

// Baseline values for change detection
static uint16_t base_adc[7] = {0};
static uint16_t last_adc[7] = {0};
static uint16_t base_pb = 0xFFFF;
static uint8_t  last_d[4]   = {1, 1, 1, 1};

static char last_event_str[48] = "System Ready. Press any button!";
static uint16_t event_color = COLOR_YELLOW;

void setup() {
    tft_init();

    // Configure free Port A pins as inputs with pullup
    pinMode(PA9,  INPUT_PULLUP);
    pinMode(PA10, INPUT_PULLUP);
    pinMode(PA11, INPUT_PULLUP);
    pinMode(PA12, INPUT_PULLUP);

    rtt.println("\n\n========================================================");
    rtt.println("   FNIRSI DSO-TC2 COMPONENT TESTER HARDWARE MAPPER       ");
    rtt.println("========================================================");

    // Initial background
    tft_fill_screen(COLOR_NAVY);

    // Top Header Banner
    tft_fill_rect(0, 0, TFT_WIDTH, 26, 0x0010);
    tft_draw_hline(0, 26, TFT_WIDTH, COLOR_CYAN);
    tft_print(10, 8, "FNIRSI DSO-TC2 :: COMPONENT TESTER", COLOR_YELLOW, 0x0010, 1);
    tft_print(270, 8, "v2.0", COLOR_GREEN, 0x0010, 1);

    // Card 1: Analog Channels (PA0 .. PA6)
    tft_fill_rect(8, 32, 148, 140, COLOR_BLACK);
    tft_draw_rect(8, 32, 148, 140, COLOR_DARKGREY);
    tft_fill_rect(9, 33, 146, 16, 0x18E3);
    tft_print(14, 37, "ADC CHANNELS", COLOR_WHITE, 0x18E3, 1);

    // Card 2: Digital Pins & Port B
    tft_fill_rect(164, 32, 148, 140, COLOR_BLACK);
    tft_draw_rect(164, 32, 148, 140, COLOR_DARKGREY);
    tft_fill_rect(165, 33, 146, 16, 0x18E3);
    tft_print(170, 37, "DIGITAL / BUS", COLOR_WHITE, 0x18E3, 1);

    // Card 3: Live Event Log Box
    tft_fill_rect(8, 178, 304, 56, COLOR_BLACK);
    tft_draw_rect(8, 178, 304, 56, COLOR_DARKCYAN);
    tft_fill_rect(9, 179, 302, 16, 0x0186);
    tft_print(14, 183, "LIVE HARDWARE EVENT DETECTOR", COLOR_CYAN, 0x0186, 1);

    tft_print(14, 204, last_event_str, COLOR_YELLOW, COLOR_BLACK, 1);
    tft_print(14, 218, "Press OK, UP, DOWN, LEFT, RIGHT, MENU", COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // Initial baseline calibration (average of 8 readings)
    for (int ch = 0; ch < 7; ch++) {
        uint32_t sum = 0;
        uint32_t pin = PA0 + ch;
        for (int i = 0; i < 8; i++) {
            sum += analogRead(pin);
            delay(2);
        }
        base_adc[ch] = sum / 8;
        last_adc[ch] = base_adc[ch];
    }

    rtt.print("[INIT] Baseline ADC: ");
    for (int ch = 0; ch < 7; ch++) {
        rtt.print("A"); rtt.print(ch); rtt.print("="); rtt.print(base_adc[ch]); rtt.print(" ");
    }
    rtt.println();
}

void loop() {
    static uint32_t last_ui_time = 0;

    // Read all 7 ADC channels
    uint16_t cur_adc[7];
    cur_adc[0] = analogRead(PA0);
    cur_adc[1] = analogRead(PA1);
    cur_adc[2] = analogRead(PA2);
    cur_adc[3] = analogRead(PA3);
    cur_adc[4] = analogRead(PA4);
    cur_adc[5] = analogRead(PA5);
    cur_adc[6] = analogRead(PA6);

    // Read Digital Port A pins
    uint8_t cur_d[4];
    cur_d[0] = digitalRead(PA9);
    cur_d[1] = digitalRead(PA10);
    cur_d[2] = digitalRead(PA11);
    cur_d[3] = digitalRead(PA12);

    // Probe Port B while TFT CS is HIGH
    GPIOA->BSRR = (1U << 15); // CS_HIGH
    __asm__ volatile("nop; nop; nop; nop");
    // Switch PB0..PB15 to input with pullup
    GPIOB->CRL = 0x88888888;
    GPIOB->CRH = 0x88888888;
    GPIOB->ODR = 0xFFFF;
    for (volatile int d = 0; d < 10; d++) __asm__ volatile("nop");
    uint16_t cur_pb = GPIOB->IDR;
    // Restore PB0..PB15 to 50MHz push-pull output
    GPIOB->CRL = 0x33333333;
    GPIOB->CRH = 0x33333333;

    // Check for ADC changes (> 60 counts from baseline)
    for (int ch = 0; ch < 7; ch++) {
        int32_t diff = (int32_t)cur_adc[ch] - (int32_t)base_adc[ch];
        if (abs(diff) > 80) {
            float volts = (float)cur_adc[ch] * 3.3f / 4095.0f;
            snprintf(last_event_str, sizeof(last_event_str), 
                     ">> PA%d TRIGGER: %d (%0.2fV) diff:%ld", ch, cur_adc[ch], volts, (long)diff);
            event_color = COLOR_GREEN;
            rtt.print("[EVENT] ");
            rtt.println(last_event_str);
        }
    }

    // Check for Digital Pin presses (LOW state)
    for (int i = 0; i < 4; i++) {
        if (cur_d[i] == LOW && last_d[i] == HIGH) {
            snprintf(last_event_str, sizeof(last_event_str), 
                     ">> PA%d PRESSED (LOW)!", (i == 0 ? 9 : (i == 1 ? 10 : (i == 2 ? 11 : 12))));
            event_color = COLOR_CYAN;
            rtt.print("[EVENT] ");
            rtt.println(last_event_str);
        }
        last_d[i] = cur_d[i];
    }

    // Check for Port B bus bits pulled LOW
    if (cur_pb != 0xFFFF) {
        snprintf(last_event_str, sizeof(last_event_str), 
                 ">> BUS PB LOW: 0x%04X (Bits changed!)", cur_pb);
        event_color = COLOR_MAGENTA;
        rtt.print("[EVENT] ");
        rtt.println(last_event_str);
    }

    // Update UI every 50ms (20 Hz, rock-solid, crisp)
    if (millis() - last_ui_time >= 50) {
        last_ui_time = millis();

        // 1. Render Analog Card
        for (int ch = 0; ch < 7; ch++) {
            int16_t y = 54 + (ch * 17);
            float volts = (float)cur_adc[ch] * 3.3f / 4095.0f;
            char buf[24];
            snprintf(buf, sizeof(buf), "A%d: %4d  %0.2fV", ch, cur_adc[ch], volts);

            bool changed = (abs((int32_t)cur_adc[ch] - (int32_t)base_adc[ch]) > 80);
            uint16_t fg = changed ? COLOR_GREEN : COLOR_WHITE;
            uint16_t bg = changed ? 0x0180 : COLOR_BLACK;

            tft_print(14, y, buf, fg, bg, 1);
        }

        // 2. Render Digital Card
        char dbuf[24];
        snprintf(dbuf, sizeof(dbuf), "PA9 : %s", cur_d[0] ? "HIGH" : "LOW ");
        tft_print(172, 54, dbuf, cur_d[0] ? COLOR_WHITE : COLOR_RED, COLOR_BLACK, 1);

        snprintf(dbuf, sizeof(dbuf), "PA10: %s", cur_d[1] ? "HIGH" : "LOW ");
        tft_print(172, 71, dbuf, cur_d[1] ? COLOR_WHITE : COLOR_RED, COLOR_BLACK, 1);

        snprintf(dbuf, sizeof(dbuf), "PA11: %s", cur_d[2] ? "HIGH" : "LOW ");
        tft_print(172, 88, dbuf, cur_d[2] ? COLOR_WHITE : COLOR_RED, COLOR_BLACK, 1);

        snprintf(dbuf, sizeof(dbuf), "PA12: %s", cur_d[3] ? "HIGH" : "LOW ");
        tft_print(172, 105, dbuf, cur_d[3] ? COLOR_WHITE : COLOR_RED, COLOR_BLACK, 1);

        snprintf(dbuf, sizeof(dbuf), "PB BUS: 0x%04X", cur_pb);
        tft_print(172, 130, dbuf, (cur_pb == 0xFFFF) ? COLOR_LIGHTGREY : COLOR_YELLOW, COLOR_BLACK, 1);

        // 3. Render Event Log Box
        // Clear old event text line
        tft_fill_rect(12, 200, 296, 14, COLOR_BLACK);
        tft_print(14, 204, last_event_str, event_color, COLOR_BLACK, 1);
    }
}
