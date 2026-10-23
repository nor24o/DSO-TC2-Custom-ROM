#ifndef TFT_H
#define TFT_H

#include <stdint.h>
#include <stddef.h>

#define TFT_WIDTH   320
#define TFT_HEIGHT  240

// Standard RGB565 Colors
#define COLOR_BLACK       0x0000
#define COLOR_NAVY        0x000F
#define COLOR_DARKGREEN   0x03E0
#define COLOR_DARKCYAN    0x03EF
#define COLOR_MAROON      0x7800
#define COLOR_PURPLE      0x780F
#define COLOR_OLIVE       0x7BE0
#define COLOR_LIGHTGREY   0xC618
#define COLOR_DARKGREY    0x39E7
#define COLOR_BLUE        0x001F
#define COLOR_GREEN       0x07E0
#define COLOR_CYAN        0x07FF
#define COLOR_RED         0xF800
#define COLOR_MAGENTA     0xF81F
#define COLOR_YELLOW      0xFFE0
#define COLOR_WHITE       0xFFFF
#define COLOR_ORANGE      0xFD20
#define COLOR_GRID        0x18E3  // Dark slate blue-gray for scope grid

void tft_init(void);
void tft_fill_screen(uint16_t color);
void tft_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void tft_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void tft_draw_pixel(int16_t x, int16_t y, uint16_t color);
void tft_draw_hline(int16_t x, int16_t y, int16_t len, uint16_t color);
void tft_draw_vline(int16_t x, int16_t y, int16_t len, uint16_t color);
void tft_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

// Text rendering
void tft_draw_char(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg, uint8_t size = 1);
void tft_print(int16_t x, int16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t size = 1);
void tft_printf(int16_t x, int16_t y, uint16_t fg, uint16_t bg, uint8_t size, const char *format, ...);

// High-Speed Oscilloscope UI & Waveform Rendering
void tft_draw_scope_grid(int16_t gx = 10, int16_t gy = 28, int16_t gw = 300, int16_t gh = 160);
void tft_draw_status_bar(const char *mode_str, const char *vdiv_str, const char *tdiv_str, bool running);
void tft_draw_scope_trace(int16_t gx, int16_t gy, int16_t gw, int16_t gh,
                          const int16_t *current_y, int16_t *previous_y,
                          uint16_t trace_color);

#endif // TFT_H
