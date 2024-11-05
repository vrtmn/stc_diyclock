/*
Models.h

This file defines configurations for clock models.
*/

#ifndef MODELS_H
#define MODELS_H

#ifdef GREEN_4
// Greek clock, 4 digits, HH:mm

#define FOUR_DIGITS

#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_DATE
#define WITHOUT_CHIME
#define SHOW_MINUTES_WITH_SECONDS
#define WITHOUT_WEEKDAY
#define DEBUG

#elif GREEN_6
// Greek clock, 6 digits, HH:mm:ss

#define SIX_DIGITS

// #define WITHOUT_CHIME
#define AUTO_SHOW_TEMPERATURE 
#define WITHOUT_WEEKDAY
#define MAP_SW2_TO_P1_4
// #define DEBUG

#endif

#endif // #ifndef MODELS_H
