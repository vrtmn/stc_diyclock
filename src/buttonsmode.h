#ifndef BUTTONSMODE_H
#define BUTTONSMODE_H

//
// Buttons modes
//

enum ButtonsMode {
  K_NORMAL,
  K_SET_HOUR,
  K_SET_MINUTE,
#ifdef SIX_DIGITS
  K_SET_SECOND,
#endif

#if !defined(WITHOUT_H12_24_SWITCH)
  K_SET_HOUR_12_24,
#endif

#ifdef WITH_NMEA
  K_NMEA_SET_TZ_HOUR,
  K_NMEA_SET_TZ_MINUTE,
  K_NMEA_SET_DST,
  K_NMEA_SET_AUTOUPDATE,
#endif

#if !defined(SIX_DIGITS)
  K_SEC_DISP,
#endif
  K_TEMP_DISP,

#if !defined(WITHOUT_DATE)
  K_DATE_DISP,
  K_SET_MONTH,
  K_SET_DAY,
#ifdef SIX_DIGITS
  K_SET_YEAR,
#else
  K_YEAR_DISP,
#endif

#if !defined(WITHOUT_WEEKDAY)
  K_WEEKDAY_DISP,
#endif
#endif

#if !defined(WITHOUT_ALARM)
  K_ALARM,
  K_ALARM_SET_HOUR,
  K_ALARM_SET_MINUTE,
#endif

#if !defined(WITHOUT_CHIME)
  K_CHIME,
  K_CHIME_SET_SINCE,
  K_CHIME_SET_UNTIL,
#endif

#ifdef WITH_DEBUG_SCREENS
  K_DEBUG_SCREEN_1,
  K_DEBUG_SCREEN_2,
  K_DEBUG_SCREEN_3,
#endif
};

#endif // #ifndef BUTTONSMODE_H
