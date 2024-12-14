/*
Models.h

This file defines configurations for clock models.
*/

#ifndef MODELS_H
#define MODELS_H

#ifdef GREEN_4
// Greek clock, 4 digits, HH:mm

#define HW_REVISION_A
#define FOUR_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_DATE
#define WITHOUT_CHIME
#define WITHOUT_WEEKDAY
#define WITHOUT_H12_24_SWITCH
#define WITH_NMEA_DEVICE_SWITCH
#define MOVE_UART_PINS_TO_P3_6
// #define WITH_DEBUG_SCREENS

#elif GREEN_6
// Greek clock, 6 digits, HH:mm:ss

#define HW_REVISION_A
#define SIX_DIGITS
#define WITHOUT_CHIME
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_WEEKDAY
#define MAP_SW2_TO_P1_4
#define WITH_NMEA_DEVICE_SWITCH
#define WITHOUT_H12_24_SWITCH
#define LIGHTVAL_LOWEST_VALUE 1
#define WITH_DEBUG_SCREENS

#endif  // #elif GREEN_6

#endif // #ifndef MODELS_H
