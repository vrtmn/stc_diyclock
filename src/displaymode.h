#ifndef DISPLAYMODE_H
#define DISPLAYMODE_H

//
// Display modes
//

enum DisplayMode {
  M_NORMAL,

#if !defined(WITHOUT_H12_24_SWITCH)
  M_SET_HOUR_12_24,
#endif

#ifdef WITH_NMEA
  M_NMEA_TIMEZONE,
  M_NMEA_DST,
  M_NMEA_AUTOUPDATE,
#endif

#if !defined(SIX_DIGITS)
  M_SECONDS_DISP,
#endif

  M_TEMPERATURE_DISP,

#if !defined(WITHOUT_DATE)
  M_DATE_DISP,
#if !defined(WITHOUT_WEEKDAY)
  M_WEEKDAY_DISP,
#endif
#if !defined(SIX_DIGITS)
  M_YEAR_DISP,
#endif
#endif

#if !defined(WITHOUT_ALARM)
  M_ALARM,
#endif

#if !defined(WITHOUT_CHIME)
  M_CHIME,
#endif

#ifdef DEBUG
  M_DEBUG,
  M_DEBUG2,
  M_DEBUG3,
#endif
};

#endif // #ifndef DISPLAYMODE_H