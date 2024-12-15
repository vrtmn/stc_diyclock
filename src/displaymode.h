#ifndef DISPLAYMODE_H
#define DISPLAYMODE_H

//
// Display modes
//

enum DisplayMode {
  DM_NORMAL,

#if !defined(WITHOUT_H12_24_SWITCH)
  DM_SET_HOUR_12_24,
#endif

#ifdef WITH_NMEA
  DM_NMEA_TIMEZONE,
  DM_NMEA_DST,
  DM_NMEA_AUTOUPDATE,
#endif

#if !defined(SIX_DIGITS)
  DM_SECONDS,
#endif

  DM_TEMPERATURE,

#if !defined(WITHOUT_DATE)
  DM_DATE,
#if !defined(WITHOUT_WEEKDAY)
  DM_WEEKDAY,
#endif
#if !defined(SIX_DIGITS)
  DM_YEAR,
#endif
#endif

#if !defined(WITHOUT_ALARM)
  DM_ALARM,
#endif

#if !defined(WITHOUT_CHIME)
  DM_CHIME,
#endif

#ifdef WITH_DEBUG_SCREENS
  DM_DEBUG_SCREEN_1,
  DM_DEBUG_SCREEN_2,
  DM_DEBUG_SCREEN_3,
#endif
};

#endif // #ifndef DISPLAYMODE_H