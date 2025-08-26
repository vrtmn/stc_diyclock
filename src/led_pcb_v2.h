/*
  LED functions & data structs for custom PCB
*/

#ifndef LED_PCB_V2_H
#define LED_PCB_V2_H

#include <stdint.h>

// Symbol indices in the ledSymbols and ledSymbolsRev arrays

#define LED_BLANK       (LED_a + ('Z' - 'A') + 1) // All segments are off
#define LED_DASH        (LED_BLANK + 1)           // '-'
#define LED_dp          (LED_DASH + 1)            // '.'
#define LED_dp_mask     0b01111111                // '.' - actual bit mask
#define LED_dp_mask_rev 0b10111111                // '.' - actual bit mask

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
    0b11000000, // 0
    0b11111001, // 1
    0b10100100, // 2
    0b10110000, // 3
    0b10011001, // 4
    0b10010010, // 5
    0b10000010, // 6
    0b11111000, // 7
    0b10000000, // 8
    0b10010000, // 9
    0b10001000, // A
    0b10000011, // b
    0b11000110, // C
    0b10100001, // d
    0b10000110, // E
    0b10001110, // F
    0b11000010, // G
    0b10001011, // H
    0b11111011, // I
    0b11100001, // J
    0b10001010, // K
    0b11000111, // L
    0b11001000, // M
    0b10101011, // N
    0b10100011, // O
    0b10001100, // P
    0b10011000, // Q
    0b10101111, // R
    0b10010010, // S
    0b10000111, // T
    0b11000001, // U
    0b11100011, // V
    0b10000001, // W
    0b10001001, // X
    0b10010001, // Y
    0b10110110, // Z
    0b11111111, // ' ' - blank
    0b10111111, // '-' - dash
    LED_dp_mask, // '.' - dot
};

// Same but with abc <-> def

const uint8_t
#if !defined(WITHOUT_LEDTABLE_RELOC)
__at (0x1100)
#endif
ledSymbolsRev[]
 ={
    // dp,g,f,e,d,c,b,a
    0b11000000, // 0
    0b11111001, // 1
    0b01100100, // 2
    0b01110000, // 3
    0b01011001, // 4
    0b01010010, // 5
    0b01000010, // 6
    0b11111000, // 7
    0b01000000, // 8
    0b01010000, // 9
    0b01001000, // A
    0b01000011, // b
    0b11000110, // C
    0b01100001, // d
    0b01000110, // E
    0b01001110, // F
    0b11000010, // G
    0b01001011, // H
    0b11111011, // I
    0b11100001, // J
    0b01001010, // K
    0b11000111, // L
    0b11001000, // M
    0b01101011, // N
    0b01100011, // O
    0b01001100, // P
    0b01011000, // Q
    0b01101111, // R
    0b01010010, // S
    0b01000111, // T
    0b11000001, // U
    0b11100011, // V
    0b01000001, // W
    0b01001001, // X
    0b01010001, // Y
    0b01110110, // Z
    0b11111111, // ' ' - blank
    0b01111111, // '-' - dash
    LED_dp_mask_rev, // '.' - dot
};

#ifndef WITHOUT_WEEKDAY
const char weekDay[][4] = {
    "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT",
};
#endif

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
    uint8_t tmp;
    if (2 == n || n == 4) {
      tmp = ledSymbolsRev[frameBuffer[n]];
      if (isDotVisible(n)) {
        tmp &= LED_dp_mask_rev;
      }
    } else {
      tmp = ledSymbols[frameBuffer[n]];
      if (isDotVisible(n)) {
        tmp &= LED_dp_mask;
      }
    }

    displayBuffer[n] = tmp;
  }
}

#endif // #define LED_PCB_V2_H