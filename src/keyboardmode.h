#ifndef KEYBOARDMODE_H
#define KEYBOARDMODE_H

//
// Keyboard mode states
//

enum keyboard_mode {
    K_NORMAL,
    K_SET_HOUR,
    K_SET_MINUTE,
    K_SET_SECOND,
    K_SET_HOUR_12_24,

#ifdef WITH_NMEA
    K_TZ_SET_HOUR,
    K_TZ_SET_MINUTE,
    K_TZ_SET_DST,
#endif

    K_SEC_DISP,
    K_TEMP_DISP,

#ifndef WITHOUT_DATE
    K_DATE_DISP,
    K_SET_MONTH,
    K_SET_DAY,
#ifdef SIX_DIGITS
    K_SET_YEAR,
#else
    K_YEAR_DISP,
#endif
    K_WEEKDAY_DISP,
#endif

#ifndef WITHOUT_ALARM
    K_ALARM,
    K_ALARM_SET_HOUR,
    K_ALARM_SET_MINUTE,
#endif

#ifndef WITHOUT_CHIME
    K_CHIME,
    K_CHIME_SET_SINCE,
    K_CHIME_SET_UNTIL,
#endif

#ifdef DEBUG
    K_DEBUG,
    K_DEBUG2,
    K_DEBUG3,
#endif
};

#endif // #ifndef KEYBOARDMODE_H
