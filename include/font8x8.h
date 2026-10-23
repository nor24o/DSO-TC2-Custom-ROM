#ifndef FONT8X8_H
#define FONT8X8_H

#include <stdint.h>

// Standard 8x8 monochrome bitmap font (ASCII 32 ' ' to 126 '~')
// Row-major: each byte represents one row of 8 horizontal pixels (bit 7 = leftmost)
extern const uint8_t font8x8_basic[95][8];

#endif // FONT8X8_H
