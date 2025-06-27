/*
  LED functions & data structs
*/

#ifndef LED_H
#define LED_H

#include <stdint.h>

// Indexes of symbols in the ledSymbols(Rev)

#define LED_BLANK   (LED_a + ('Z' - 'A') + 1)       // All segments are off
#define LED_DASH    (LED_BLANK + 1)                 // '-'
#define LED_dp      (LED_DASH + 1)                  // '.'

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
    0b11000000, // was 0b00111111, // 0
    0b11111001, //     0b00000110, // 1
    0b10100100, //     0b01011011, // 2
    0b10110000, //     0b01001111, // 3
    0b10011001, //     0b01100110, // 4
    0b10010010, //     0b01101101, // 5
    0b10000010, //     0b01111101, // 6
    0b11111000, //     0b00000111, // 7
    0b10000000, //     0b01111111, // 8
    0b10010000, //     0b01101111, // 9 with d segment
    0b10001000, //     0b01110111, // A
    0b10000011, //     0b01111100, // b
    0b11000110, //     0b00111001, // C
    0b10100001, //     0b01011110, // d
    0b10000110, //     0b01111001, // E
    0b10001110, //     0b01110001, // F
    0b11000010, //     G  0x14
    0b10001011, //     H  
    0b11111011, //     I
    0b11100001, //     J
    0b10001010, //     K  0x18
    0b11000111, //     L
    0b11001000, //     M
    0b10101011, //     N
    0b10100011, //     O  0x1c
    0b10001100, //     P
    0b10011000, //     Q
    0b10101111, //     R
    0b10010010, //     S  0x20
    0b10000111, //     T
    0b11000001, //     U
    0b11100011, //     V  0x23
    0b10000001, //     W  0x24
    0b10001001, //     X
    0b10010001, //     Y
    0b10110110, //     Z  0x27
    0b11111111, //     0b00000000, // ' ' - blank
    0b10111111, //     0b01000000, // '-' - dash
    0b01111111, //     0b10000000, // '.' - dot
};

// Same but with abc <-> def

const uint8_t
#if !defined(WITHOUT_LEDTABLE_RELOC)
__at (0x1100)
#endif
ledSymbolsRev[]
 ={
    0b11000000, // 0
    0b11001111, // 1
    0b10100100, // 2
    0b10000110, // 3
    0b10001011, // 4
    0b10010010, // 5
    0b10010000, // 6
    0b11000111, // 7
    0b10000000, // 8
    0b10000010, // 9
    0b10000001, // A
    0b10011000, // B
    0b11110000, // C
    0b10001100, // D
    0b10110000, // E
    0b10110001, // F
    0b11010000, // G
    0b10011001, // H
    0b11011111, // I
    0b11001100, // J
    0b10010001, // K
    0b11111000, // L
    0b11000001, // M
    0b10011101, // N
    0b10011100, // O
    0b10100001, // P
    0b10000011, // Q
    0b10111101, // R
    0b10010010, // S
    0b10111000, // T
    0b11001000, // U
    0b11011100, // V
    0b10001000, // W
    0b10001001, // X
    0b10001010, // Y
    0b10110110, // Z
    0b11111111, // ' ' - blank
    0b10111111, // '-' - dash
    0b01111111, // '.' - dot
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
    if (2 == n || n > 3) {
      tmp = ledSymbolsRev[frameBuffer[n]];
    } else {
      tmp = ledSymbols[frameBuffer[n]];
    }

    if (isDotVisible(n)) {
      tmp &= 0x7F;
    }

    displayBuffer[n] = tmp;
  }
}

#endif // #define LED_H