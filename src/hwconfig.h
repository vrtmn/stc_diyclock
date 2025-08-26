// hardware definitions

// as we start to discover different hw variants with various features, chipsets, pinouts, etc
// we can capture things like HW_REVISION_C to describe this, in absence of any more descriptive naming

#ifndef HWCONFIG_H
#define HWCONFIG_H

#include "models.h"

#if defined(HW_REVISION_A)
 #define BUZZER     P1_5
 #define BUZZER_ON  (BUZZER = 0)
 #define BUZZER_OFF (BUZZER = 1)
 #define ADC_LIGHT  6
 #define ADC_TEMP   7
 // offset where the digits start on LED_DIGITS_PORT
 #define LED_DIGITS_PORT_BASE   2
 #define NUM_SW     2
#elif defined(HW_REVISION_CUSTOM_PCB)
 #define BUZZER     P1_5
 #define BUZZER_ON  (BUZZER = 0)
 #define BUZZER_OFF (BUZZER = 1)
 #define ADC_LIGHT  6
 #define ADC_TEMP   7
 // offset where the digits start on LED_DIGITS_PORT
 #define LED_DIGITS_PORT_BASE   2
 #define NUM_SW     2
 #elif defined(HW_REVISION_WITH_VOICE_CHIP)
 // revision with stc15w408as (with voice chip)
 #define LED            P1_5
 #define BUZZER_ON
 #define BUZZER_OFF
 #define ADC_LIGHT      6
 #define ADC_TEMP       7
 #define LED_DIGITS_PORT_BASE   2
 #define NUM_SW         2
#elif defined(HW_REVISION_C)
/*
 Another model with STC15F204, but diff pinouts, described here:
 https://github.com/zerog2k/stc_diyclock/issues/20
*/
 #define BUZZER     P3_3
 #define BUZZER_ON  BUZZER = 0
 #define BUZZER_OFF BUZZER = 1
 #define ADC_LIGHT  3
 #define ADC_TEMP   6
 #define LED_DIGITS_PORT_BASE   4
 #define SW3        P1_4
 #define NUM_SW     3
#endif

//
// Buttons
// 

#ifdef MAP_SW1_TO_P1_3
#define SW1     P1_3
#else
#define SW1     P3_1
#endif

#ifdef MAP_SW2_TO_P1_4
#define SW2     P1_4
#else
#define SW2     P3_0
#endif

//
// NMEA
// 

#if defined(WITH_NMEA) && defined(WITH_NMEA_DEVICE_SWITCH)
#if !defined(NMEA_DEVICE_SWITCH_PORT)
    #define NMEA_DEVICE_SWITCH_PORT P1_3
#endif
#define NMEA_DEVICE_ON (NMEA_DEVICE_SWITCH_PORT=0)
#define NMEA_DEVICE_OFF (NMEA_DEVICE_SWITCH_PORT=1)
#endif 

//
// 7-seg LEDs
//

// Which port the segments are connected to
#define LED_SEGMENT_PORT P2
// Which port controls the digits
#define LED_DIGITS_PORT  P3

// Number of digits
#ifdef SIX_DIGITS
#define NUMBER_OF_DIGITS 6
#else
#define NUMBER_OF_DIGITS 4
#endif

// Macro mask to turn off digits
#ifdef SIX_DIGITS
#define DIGITS_MASK 0b111111
#else
#define DIGITS_MASK 0b1111
#endif

#define LED_DIGITS_OFF()    ( LED_DIGITS_PORT |= (DIGITS_MASK << LED_DIGITS_PORT_BASE))

// Macro to turn on single digit
#define LED_DIGIT_ON(digit) (LED_DIGITS_PORT &= ~((1<<LED_DIGITS_PORT_BASE) << digit))

//
// DS1302 pins
//

#ifdef HW_REVISION_C
 #define DS_CE    P0_0
 #define DS_IO    P0_1
 #define DS_SCLK  P3_2
 // needed for asm optimizations
 #define _DS_IO   _P0_1
 #define _DS_SCLK _P3_2
#else
 #define DS_CE    P1_0
 #define DS_IO    P1_1
 #define DS_SCLK  P1_2
 // needed for asm optimizations
 #define _DS_IO   _P1_1
 #define _DS_SCLK _P1_2
#endif

#endif // #define HWCONFIG_H
