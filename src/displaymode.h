#ifndef DISPLAYMODE_H
#define DISPLAYMODE_H

//
// display mode states
//

enum display_mode {
    M_NORMAL,
    M_SET_HOUR_12_24,
#ifdef WITH_NMEA
    M_TZ_SET_TIME,
    M_TZ_SET_DST,
#endif
#ifndef SIX_DIGITS
    M_SEC_DISP,
#endif
    M_TEMP_DISP,
#ifndef WITHOUT_DATE
    M_DATE_DISP,
#ifndef WITHOUT_WEEKDAY
    M_WEEKDAY_DISP,
#endif
    M_YEAR_DISP,
#endif
#ifndef WITHOUT_ALARM
    M_ALARM,
#endif
#ifndef WITHOUT_CHIME
    M_CHIME,
#endif
#ifdef DEBUG
    M_DEBUG,
    M_DEBUG2,
    M_DEBUG3,
#endif
};

#endif // #ifndef DISPLAYMODE_H