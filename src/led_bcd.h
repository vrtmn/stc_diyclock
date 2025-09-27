/*
  LED functions & data structs for BCD Clock (Binary Coded Decimal clock)
*/

#ifndef LED_BCD_H
#define LED_BCD_H

#include <stdint.h>

/*
 Symbol indices in the ledSymbols array
*/

#define LED_BLANK   (LED_a + ('F' - 'A') + 1)       // All segments are off
#define LED_DASH    (LED_BLANK + 1)                 // '-'
#define LED_dp      (LED_DASH + 1)                  // '.'
#define LED_dp_mask 0b01101111                      // '.' - actual bit mask

#define LED_a       10
#define LED_b       (LED_a + ('B' - 'A'))
#define LED_c       (LED_a + ('C' - 'A'))
#define LED_d       (LED_a + ('D' - 'A'))
#define LED_e       (LED_a + ('E' - 'A'))
#define LED_f       (LED_a + ('F' - 'A'))

const uint8_t
#if !defined(WITHOUT_LEDTABLE_RELOC)
__at (0x1000)
#endif
ledSymbols[]
 = {
    // dp,g,f,e,d,c,b,a
    0b11111111, // 0
    0b11110111, // 1
    0b11111011, // 2
    0b11110011, // 3
    0b11111101, // 4
    0b11110101, // 5
    0b11111001, // 6
    0b11110001, // 7
    0b11111110, // 8
    0b11110110, // 9
    0b11111010, // 10 (hex A)
    0b11110010, // 11 (hex B)
    0b11111100, // 12 (hex C)
    0b11110100, // 13 (hex D)
    0b11111000, // 14 (hex E)
    0b11110000, // 15 (hex F)
    0b11111111, // ' ' - blank
    LED_dp_mask, // '-' - dash
    LED_dp_mask, // '.' - dot
};

// Each bit determines the visibility of the corresponding dot
uint8_t dotsBuffer;
// Each item contains an index of a symbol in the ledSymbols(Rev) table
uint8_t frameBuffer[NUMBER_OF_DIGITS];
// Each item contains a symbol to display
uint8_t displayBuffer[NUMBER_OF_DIGITS];

inline uint8_t isDotVisible(uint8_t pos) { return dotsBuffer & (1 << pos); }

inline void fillDot(uint8_t pos, uint8_t isVisible) {
  if (isVisible)
    dotsBuffer |= 1 << pos;
}

inline void fillDigit(uint8_t pos, uint8_t value) {
  frameBuffer[pos] = value;
}

void clearFrameBuffer() {
  dotsBuffer = 0;
  for (uint8_t n = 0; n != NUMBER_OF_DIGITS; n++) {
    frameBuffer[n] = LED_BLANK;
  }
}

inline void updateDisplayBuffer() {
  for (uint8_t n = 0; n != NUMBER_OF_DIGITS; n++) {
    uint8_t tmp = ledSymbols[frameBuffer[n]];
    if (isDotVisible(n)) {
      tmp &= LED_dp_mask;
    }

    displayBuffer[n] = tmp;
  }
}

#endif // #define LED_BCD_H