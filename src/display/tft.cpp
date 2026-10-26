#include "tft.h"
#include "font8x8.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

// Pin control macros
#define CS_LOW()    (GPIOA->BRR  = (1U << 15))
#define CS_HIGH()   (GPIOA->BSRR = (1U << 15))

#define DC_LOW()    (GPIOC->BRR  = (1U << 13))
#define DC_HIGH()   (GPIOC->BSRR = (1U << 13))

#define WR_LOW()    (GPIOC->BRR  = (1U << 14))
#define WR_HIGH()   (GPIOC->BSRR = (1U << 14))

#define RD_LOW()    (GPIOC->BRR  = (1U << 15))
#define RD_HIGH()   (GPIOC->BSRR = (1U << 15))

#define RST_LOW()   (GPIOA->BRR  = (1U << 8))
#define RST_HIGH()  (GPIOA->BSRR = (1U << 8))

static inline void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 12; i++) {
        __asm__ volatile("nop");
    }
}

// High-speed write strobe (direct inline NOPs, safe for PC14 slew rate)
static inline void pulse_wr(void) {
    WR_LOW();
    __asm__ volatile("nop; nop; nop; nop; nop; nop");
    WR_HIGH();
    __asm__ volatile("nop; nop; nop; nop; nop; nop");
}

// Ultra-fast 16-bit bus write (drives PB0..PB15 and PA7)
static inline void write_bus_16(uint16_t val) {
    if (val & 0x0001) {
        GPIOA->BSRR = (1U << 7);
    } else {
        GPIOA->BRR  = (1U << 7);
    }
    GPIOB->ODR = val;
    __asm__ volatile("nop; nop");
    pulse_wr();
}

static inline void write_cmd(uint8_t c) {
    DC_LOW();
    __asm__ volatile("nop; nop");
    write_bus_16(((uint16_t)c << 8) | c);
}

static inline void write_data(uint8_t d) {
    DC_HIGH();
    __asm__ volatile("nop; nop");
    write_bus_16(((uint16_t)d << 8) | d);
}

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    write_cmd(0x2A); // CASET
    write_data(x0 >> 8);
    write_data(x0 & 0xFF);
    write_data(x1 >> 8);
    write_data(x1 & 0xFF);

    write_cmd(0x2B); // RASET
    write_data(y0 >> 8);
    write_data(y0 & 0xFF);
    write_data(y1 >> 8);
    write_data(y1 & 0xFF);

    write_cmd(0x2C); // RAMWR
}

void tft_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;

    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    PWR->CR |= PWR_CR_DBP;
    BKP->CR &= ~BKP_CR_TPE;
    BKP->CSR &= ~BKP_CSR_TPIE;
    RCC->BDCR &= ~(RCC_BDCR_RTCEN | RCC_BDCR_LSEON | RCC_BDCR_LSEBYP);

    GPIOC->CRH = (GPIOC->CRH & 0x000FFFFF) | 0x22200000;
    GPIOC->BSRR = (1U << 13) | (1U << 14) | (1U << 15);

    GPIOA->CRH = (GPIOA->CRH & 0x0FFFFFF0) | 0x30000003;
    GPIOA->BSRR = (1U << 8) | (1U << 15);

    GPIOA->CRL = (GPIOA->CRL & 0x0FFFFFFF) | 0x30000000;

    GPIOB->CRL = 0x33333333;
    GPIOB->CRH = 0x33333333;
    GPIOB->ODR = 0x0000;

    RST_HIGH();
    delay(20);
    RST_LOW();
    delay(50);
    RST_HIGH();
    delay(150);

    CS_LOW();
    RD_HIGH();
    WR_HIGH();
    delay(10);

    write_cmd(0x01);
    delay(150);

    write_cmd(0x11);
    delay(150);

    write_cmd(0x3A);
    write_data(0x55);
    delay(10);

    // 0xA0 = Flipped Landscape (180 deg) MV=1, MY=1, MX=0, RGB color order
    write_cmd(0x36);
    write_data(0xA0);
    delay(10);

    write_cmd(0xB2);
    write_data(0x0C); write_data(0x0C); write_data(0x00); write_data(0x33); write_data(0x33);

    write_cmd(0xB7);
    write_data(0x35);

    write_cmd(0xBB);
    write_data(0x28);

    write_cmd(0xC0);
    write_data(0x2C);

    write_cmd(0xC2);
    write_data(0x01);

    write_cmd(0xC3);
    write_data(0x12);

    write_cmd(0xC4);
    write_data(0x20);

    write_cmd(0xC6);
    write_data(0x0F);

    write_cmd(0xD0);
    write_data(0xA4); write_data(0xA1);

    write_cmd(0xE0);
    write_data(0xD0); write_data(0x04); write_data(0x0D); write_data(0x11);
    write_data(0x13); write_data(0x2B); write_data(0x3F); write_data(0x54);
    write_data(0x4C); write_data(0x18); write_data(0x0D); write_data(0x0B);
    write_data(0x1F); write_data(0x23);

    write_cmd(0xE1);
    write_data(0xD0); write_data(0x04); write_data(0x0C); write_data(0x11);
    write_data(0x13); write_data(0x2C); write_data(0x3F); write_data(0x44);
    write_data(0x51); write_data(0x2F); write_data(0x1F); write_data(0x1F);
    write_data(0x20); write_data(0x23);

    // CRITICAL FIX: Display Inversion OFF (0x20) for TN Panel
    // 0x21 inverted all colors (Black became White, Yellow became Blue!)
    write_cmd(0x20);
    delay(10);

    write_cmd(0x13);
    delay(10);

    write_cmd(0x29);
    delay(150);

    CS_HIGH();
}

void tft_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > TFT_WIDTH)  w = TFT_WIDTH - x;
    if (y + h > TFT_HEIGHT) h = TFT_HEIGHT - y;
    if (w <= 0 || h <= 0) return;

    CS_LOW();
    set_window(x, y, x + w - 1, y + h - 1);
    DC_HIGH();
    __asm__ volatile("nop; nop");

    uint32_t total = (uint32_t)w * h;
    for (uint32_t i = 0; i < total; i++) {
        write_bus_16(color);
    }
    CS_HIGH();
}

void tft_fill_screen(uint16_t color) {
    tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

void tft_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    tft_draw_hline(x, y, w, color);
    tft_draw_hline(x, y + h - 1, w, color);
    tft_draw_vline(x, y, h, color);
    tft_draw_vline(x + w - 1, y, h, color);
}

void tft_draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= TFT_WIDTH || y < 0 || y >= TFT_HEIGHT) return;
    CS_LOW();
    set_window(x, y, x, y);
    DC_HIGH();
    __asm__ volatile("nop; nop");
    write_bus_16(color);
    CS_HIGH();
}

void tft_draw_hline(int16_t x, int16_t y, int16_t len, uint16_t color) {
    tft_fill_rect(x, y, len, 1, color);
}

void tft_draw_vline(int16_t x, int16_t y, int16_t len, uint16_t color) {
    tft_fill_rect(x, y, 1, len, color);
}

void tft_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy, e2;

    while (1) {
        tft_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void tft_draw_char(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg, uint8_t size) {
    if (c < 32 || c > 126) c = '?';
    uint8_t glyph_idx = c - 32;

    if (size == 1) {
        if (x + 8 > TFT_WIDTH || y + 8 > TFT_HEIGHT || x < 0 || y < 0) {
            for (uint8_t r = 0; r < 8; r++) {
                uint8_t row_bits = font8x8_basic[glyph_idx][r];
                for (uint8_t col = 0; col < 8; col++) {
                    if (row_bits & (1 << (7 - col))) {
                        tft_draw_pixel(x + col, y + r, fg);
                    } else if (bg != fg) {
                        tft_draw_pixel(x + col, y + r, bg);
                    }
                }
            }
            return;
        }

        CS_LOW();
        set_window(x, y, x + 7, y + 7);
        DC_HIGH();
        __asm__ volatile("nop; nop");
        for (uint8_t r = 0; r < 8; r++) {
            uint8_t row_bits = font8x8_basic[glyph_idx][r];
            for (uint8_t col = 0; col < 8; col++) {
                write_bus_16((row_bits & (1 << (7 - col))) ? fg : bg);
            }
        }
        CS_HIGH();
    } else {
        for (uint8_t r = 0; r < 8; r++) {
            uint8_t row_bits = font8x8_basic[glyph_idx][r];
            for (uint8_t col = 0; col < 8; col++) {
                uint16_t c_color = (row_bits & (1 << (7 - col))) ? fg : bg;
                tft_fill_rect(x + col * size, y + r * size, size, size, c_color);
            }
        }
    }
}

void tft_print(int16_t x, int16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t size) {
    int16_t cur_x = x;
    int16_t cur_y = y;
    while (*str) {
        if (*str == '\n') {
            cur_x = x;
            cur_y += 8 * size + 2;
        } else {
            tft_draw_char(cur_x, cur_y, *str, fg, bg, size);
            cur_x += 8 * size;
        }
        str++;
    }
}

void tft_printf(int16_t x, int16_t y, uint16_t fg, uint16_t bg, uint8_t size, const char *format, ...) {
    char buf[64];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    tft_print(x, y, buf, fg, bg, size);
}

void tft_draw_scope_grid(int16_t gx, int16_t gy, int16_t gw, int16_t gh) {
    tft_fill_rect(gx, gy, gw, gh, COLOR_BLACK);
    tft_draw_rect(gx, gy, gw, gh, COLOR_DARKGREY);

    int16_t dx = gw / 10;
    int16_t dy = gh / 6;

    for (int i = 1; i < 10; i++) {
        int16_t x = gx + i * dx;
        for (int16_t y = gy; y < gy + gh; y += 4) {
            tft_draw_pixel(x, y, COLOR_GRID);
        }
    }

    for (int j = 1; j < 6; j++) {
        int16_t y = gy + j * dy;
        for (int16_t x = gx; x < gx + gw; x += 4) {
            tft_draw_pixel(x, y, COLOR_GRID);
        }
    }

    int16_t cx = gx + gw / 2;
    int16_t cy = gy + gh / 2;
    for (int16_t x = gx; x < gx + gw; x += 6) {
        tft_draw_pixel(x, cy, COLOR_DARKGREY);
    }
    for (int16_t y = gy; y < gy + gh; y += 6) {
        tft_draw_pixel(cx, y, COLOR_DARKGREY);
    }
}

void tft_draw_status_bar(const char *mode_str, const char *vdiv_str, const char *tdiv_str, bool running) {
    tft_fill_rect(0, 0, TFT_WIDTH, 24, COLOR_NAVY);
    tft_draw_hline(0, 24, TFT_WIDTH, COLOR_CYAN);

    tft_print(8, 8, mode_str, COLOR_WHITE, COLOR_NAVY, 1);

    if (running) {
        tft_fill_rect(130, 4, 45, 16, COLOR_DARKGREEN);
        tft_print(138, 8, "RUN", COLOR_WHITE, COLOR_DARKGREEN, 1);
    } else {
        tft_fill_rect(130, 4, 45, 16, COLOR_MAROON);
        tft_print(138, 8, "STOP", COLOR_WHITE, COLOR_MAROON, 1);
    }

    tft_print(190, 4, vdiv_str, COLOR_YELLOW, COLOR_NAVY, 1);
    tft_print(190, 14, tdiv_str, COLOR_CYAN, COLOR_NAVY, 1);

    tft_draw_rect(285, 6, 22, 12, COLOR_WHITE);
    tft_fill_rect(307, 9, 3, 6, COLOR_WHITE);
    tft_fill_rect(287, 8, 14, 8, COLOR_GREEN);
}

// Fast LUT for oscilloscope background row attributes
// Bit 0: Horizontal grid line (rel_y % 26 == 0)
// Bit 1: Vertical dot pattern (rel_y % 4 == 0)
// Bit 2: Center horizontal axis (rel_y == gh / 2)
static uint8_t scope_row_flags[240];
static bool scope_lut_init = false;

static void init_scope_lut(int16_t gh) {
    int16_t mid = gh / 2;
    for (int16_t y = 0; y < gh && y < 240; y++) {
        uint8_t f = 0;
        if (y > 0 && y < gh && (y % 26 == 0)) f |= 0x01;
        if (y % 4 == 0)                       f |= 0x02;
        if (y == mid)                         f |= 0x04;
        scope_row_flags[y] = f;
    }
    scope_lut_init = true;
}

// Blazingly fast, zero-ghosting differential bounding column oscilloscope trace renderer
void tft_draw_scope_trace(int16_t gx, int16_t gy, int16_t gw, int16_t gh,
                          const int16_t *current_y, int16_t *previous_y,
                          uint16_t trace_color) {
    if (!scope_lut_init) init_scope_lut(gh);

    CS_LOW();

    int16_t gy_max = gy + gh - 1;

    for (int16_t x = 0; x < gw; x++) {
        // Current segment bounds
        int16_t yc0 = current_y[x];
        int16_t yc1 = (x > 0) ? current_y[x - 1] : yc0;
        int16_t c_low  = (yc0 < yc1) ? yc0 : yc1;
        int16_t c_high = (yc0 > yc1) ? yc0 : yc1;

        // Previous segment bounds
        int16_t yp0 = previous_y[x];
        int16_t yp1 = (x > 0) ? previous_y[x - 1] : yp0;
        int16_t p_low  = (yp0 < yp1) ? yp0 : yp1;
        int16_t p_high = (yp0 > yp1) ? yp0 : yp1;

        // Combined bounding span (encompasses both old to erase and new to draw)
        int16_t y_min = (c_low < p_low) ? c_low : p_low;
        int16_t y_max = (c_high > p_high) ? c_high : p_high;

        if (y_min < gy) y_min = gy;
        if (y_max > gy_max) y_max = gy_max;

        if (y_min <= y_max) {
            set_window(gx + x, y_min, gx + x, y_max);
            DC_HIGH();
            __asm__ volatile("nop; nop");

            bool is_vgrid = (x > 0 && x < gw && (x % 30 == 0));
            bool is_dot_h = (x % 4 == 0);
            bool is_caxis = (x % 6 == 0);

            for (int16_t actual_y = y_min; actual_y <= y_max; actual_y++) {
                if (actual_y >= c_low && actual_y <= c_high) {
                    write_bus_16(trace_color);
                } else {
                    int16_t rel_y = actual_y - gy;
                    uint8_t f = scope_row_flags[rel_y];
                    if ((is_vgrid && (f & 0x02)) || ((f & 0x01) && is_dot_h)) {
                        write_bus_16(COLOR_GRID);
                    } else if ((f & 0x04) && is_caxis) {
                        write_bus_16(COLOR_DARKGREY);
                    } else {
                        write_bus_16(COLOR_BLACK);
                    }
                }
            }
        }

        previous_y[x] = yc0;
    }

    CS_HIGH();
}
