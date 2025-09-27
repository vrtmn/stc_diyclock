/*
Models.h

This file defines configurations for clock models.
*/

#ifndef MODELS_H
#define MODELS_H

#if defined(MOD4)
// 4 digits (HH:mm), v1

#define HW_REVISION_A
#define FOUR_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_DATE
#define WITHOUT_WEEKDAY
#define WITHOUT_H12_24_SWITCH
#define WITH_NMEA_DEVICE_SWITCH
#define MOVE_UART_PINS_TO_P3_6
// #define WITH_DEBUG_SCREENS

#elif defined(MOD4_MIN)
// 4 digits (HH:mm), all features are off, v1

#define HW_REVISION_A
#define FOUR_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_DATE
#define WITHOUT_CHIME
#define WITHOUT_ALARM
#define WITHOUT_WEEKDAY
#define WITHOUT_H12_24_SWITCH
#define WITHOUT_INACTIVITY_TIMER
#define WITH_NMEA_DEVICE_SWITCH
#define MOVE_UART_PINS_TO_P3_6

#elif defined(MOD6)
// Greek clock, 6 digits (HH:mm:ss), v1

#define HW_REVISION_A
#define SIX_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_WEEKDAY
#define MAP_SW2_TO_P1_4
#define WITH_NMEA_DEVICE_SWITCH
#define WITHOUT_H12_24_SWITCH
#define WITH_DEBUG_SCREENS

#elif defined(MOD6_V2)
// Greek clock, 6 digits (HH:mm:ss), v2

#define HW_REVISION_CUSTOM_PCB
#define SIX_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_WEEKDAY
#define MAP_SW1_TO_P1_3
#define MAP_SW2_TO_P1_4
#define NMEA_DEVICE_SWITCH_PORT P3_1
#define WITH_NMEA_DEVICE_SWITCH
#define WITHOUT_H12_24_SWITCH
#define NUMBER_OF_DEBUG_SCREENS 1
#define WITH_DEBUG_SCREENS

#elif defined(MOD6_MIN)
// 6 digits (HH:mm:ss), all features are off, v1

#define HW_REVISION_A
#define SIX_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_DATE
#define WITHOUT_CHIME
#define WITHOUT_ALARM
#define WITHOUT_WEEKDAY
#define WITHOUT_H12_24_SWITCH
#define WITHOUT_INACTIVITY_TIMER
#define WITH_NMEA_DEVICE_SWITCH
#define MAP_SW2_TO_P1_4

#elif defined(MOD4_V2)
// Purple, 4 digits (HH:mm), v2

#define HW_REVISION_A
#define FOUR_DIGITS
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_WEEKDAY
#define MAP_SW1_TO_P1_3
#define MAP_SW2_TO_P1_4
#define NMEA_DEVICE_SWITCH_PORT P3_1
#define WITH_NMEA_DEVICE_SWITCH
#define WITHOUT_H12_24_SWITCH
#define WITH_DEBUG_SCREENS

#elif defined(MOD_BCD)
// BCD display

#define BCD_DISPLAY
#define HW_REVISION_A
#define SIX_DIGITS
#define WITHOUT_DATE
#define WITHOUT_CHIME
#define WITHOUT_ALARM
#define WITHOUT_WEEKDAY
#define WITHOUT_H12_24_SWITCH
#define MAP_SW1_TO_P1_3
#define MAP_SW2_TO_P1_4
#define NMEA_DEVICE_SWITCH_PORT P3_1
#define WITH_NMEA_DEVICE_SWITCH
#define WITH_DEBUG_SCREENS
#define NUMBER_OF_DEBUG_SCREENS 1

#endif  

#endif // #ifndef MODELS_H
