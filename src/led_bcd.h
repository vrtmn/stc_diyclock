/*
  LED functions & data structs for BCD Clock (Binary Coded Decimal clock)
*/

#ifndef LED_BCD_H
#define LED_BCD_H

#include <stdint.h>

/*
 Symbol indices in the ledSymbols array

 Note:
 BCD clock is not supposed to display letters, but they are still defined
 in the table for backward compatibility of the codebase.
 All symbols except the digits ('0'-'9') and the dot are defined as 'blank' (all LEDs off).
*/

#define LED_BLANK   (LED_a + ('Z' - 'A') + 1)       // All segments are off
#define LED_DASH    (LED_BLANK + 1)                 // '-'
#define LED_dp      (LED_DASH + 1)                  // '.'
#define LED_dp_mask 0b01111111                      // '.' - actual bit mask

#define LED_a       10
#define LED_b       (LED_a + ('B' - 'A'))
#define LED_c       (LED_a + ('C' - 'A'))
#define LED_d       (LED_a + ('D' - 'A'))
#define LED_e       (LED_a + ('E' - 'A'))
#define LED_f       (LED_a + ('F' - 'A'))
#define LED_h       (LED_a + ('H' - 'A'))
#define LED_l       (LED_a + ('L' - 'A'))
#define LED_n       (LED_a + ('N' - 'A'))
#define LED_o       (LED_a + ('O' - 'A'))
#define LED_p       (LED_a + ('P' - 'A'))
#define LED_s       (LED_a + ('S' - 'A'))
#define LED_t       (LED_a + ('T' - 'A'))
#define LED_u       (LED_a + ('U' - 'A'))

const uint8_t
#if !defined(WITHOUT_LEDTABLE_RELOC)
__at (0x1000)
#endif
ledSymbols[]
 = {
    // dp,g,f,e,d,c,b,a
    0b11111111, // 0
    0b11111110, // 1
    0b11111101, // 2
    0b11111100, // 3
    0b11110011, // 4
    0b11111010, // 5
    0b11111001, // 6
    0b11111000, // 7
    0b11110111, // 8
    0b11110110, // 9
    0b11111111, // A
    0b11111111, // b
    0b11111111, // C
    0b11111111, // d
    0b11111111, // E
    0b11111111, // F
    0b11111111, // G
    0b11111111, // H
    0b11111111, // I
    0b11111111, // J
    0b11111111, // K
    0b11111111, // L
    0b11111111, // M
    0b11111111, // N
    0b11111111, // O
    0b11111111, // P
    0b11111111, // Q
    0b11111111, // R
    0b11111111, // S
    0b11111111, // T
    0b11111111, // U
    0b11111111, // V
    0b11111111, // W
    0b11111111, // X
    0b11111111, // Y
    0b11111111, // Z
    0b11111111, // ' ' - blank
    0b11111111, // '-' - dash
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