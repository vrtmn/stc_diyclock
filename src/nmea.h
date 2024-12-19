#include "eeprom.h"
#include <time.h>

#define BAUDRATE 9600 // serial port speed (4800/9600 - standard for GPS)

#define IAP_TZ_ADDRESS 0x0000
#define IAP_TZ_HR      0x0000
#define IAP_TZ_MIN     0x0001
#define IAP_TZ_DST     0x0002
#define IAP_TZ_AUTOSYNC 0x0003

#define OSCILLATOR_FREQUENCY    11059200  // 11.0592MHz

volatile uint32_t nmea_seconds_to_sync = 0;
volatile uint32_t nmea_progress_seconds = 0;
volatile int8_t nmea_tz_hr;
volatile uint8_t nmea_tz_min;
volatile uint8_t nmea_tz_dst;
int8_t nmea_prev_tz_hr;
uint8_t nmea_prev_tz_min;
uint8_t nmea_prev_tz_dst;
uint8_t nmea_prev_autosync;

#define NMEA_LINE_MAX_LEN 84
#define NMEA_LINE_MIN_LEN 39
#define NMEA_COMMA_TIME 1
#define NMEA_COMMA_STATUS 2
#define NMEA_COMMA_DATE 9

// Contains the received NMEA sentence
__pdata char nmeaBuffer[NMEA_LINE_MAX_LEN];
volatile int8_t nmeaSentenceLength = 0;

/*
RMC heades:
https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm

Example 1 (GPS):
$GPRMC,203522.00,A,5109.0262308,N,11401.8407342,W,0.004,133.4,130522,0.0,E,D*2B

Example 2 (Multi-constellation):
$GNRMC,204520.00,A,5109.0262239,N,11401.8407338,W,0.004,102.3,130522,0.0,E,D*3B

Example 3 (NTP):
$GPRMC,232231.00,A,,,,,,,170420,,,*27
*/

#define RMC_HEADER_LENGTH 6
__code const char *RMC_HEADERS[] = {"$GPRMC", "$GNRMC"};

volatile enum {
    NMEA_NONE = 0,
    NMEA_RECEIVING,
    NMEA_VALIDATE,
    NMEA_READY
} nmea_state = NMEA_NONE;

// Auto sync interval, hours
volatile enum {
    NMEA_AUTOSYNC_OFF = 0,
    NMEA_AUTOSYNC_3H = 3,
    NMEA_AUTOSYNC_6H = 6,
    NMEA_AUTOSYNC_12H = 12,
    NMEA_AUTOSYNC_24H = 24
} nmea_autosync = NMEA_AUTOSYNC_OFF;

#define IS_NMEA_AUTOSYNC_ON (nmea_autosync != NMEA_AUTOSYNC_OFF)
#define SECONDS_IN_ONE_HOUR 3600
#define NMEA_AUTOSYNC_DELAY (nmea_autosync * SECONDS_IN_ONE_HOUR)
#define NMEA_MAX_SYNC_DURATION ((3600 / 2) * 10)  // Number of 100-ms intervals (1000/100)

void uart1_init()
{
#ifdef MOVE_UART_PINS_TO_P3_6
    P_SW1 |= (1 << 6);          // move UART1 pins -> P3_6:rxd, P3_7:txd
#endif
    
    // UART1 use Timer2
    T2L = (65536 - (OSCILLATOR_FREQUENCY / 4 / BAUDRATE)) & 0xFF;
    T2H = (65536 - (OSCILLATOR_FREQUENCY / 4 / BAUDRATE)) >> 8;
    SM1 = 1;                    // serial mode 1: 8-bit async
    AUXR |= 0x14;               // T2R: run T2, T2x12: T2 clk src sysclk/1
    AUXR |= 0x01;               // S1ST2: T2 is baudrate generator
    ES = 1;                     // enable uart1 interrupt
    EA = 1;                     // enable interrupts
    REN = 1;
}

// Returns a pointer to a comma number N in nmeaBuffer
char *nmeaGetComma(uint8_t num)
{
    for (int8_t i = 0; i != nmeaSentenceLength; i++) {
        if (nmeaBuffer[i] == ',' && !--num) {
            return nmeaBuffer + i;
        }
    }
    return NULL;
}

// Compares the first n characters of two given strings. Returns 0 if they are equal, otherwise - 1.
inline int8_t nmea_strcmp(const char *dst, const char *src, uint8_t n)
{
    while (n--) {
        if (*dst++ != *src++)
            return 1;
    }
    return 0;
}

// Converts a single hex char into an integer
uint8_t hexChar2Int(const char *ch) 
{
    if (*ch >= 'a')
        return *ch - 'a' + 10;
    else if (*ch >= 'A')
        return *ch - 'A' + 10;
    else
        return *ch - '0';
}

//
// Validation
// 

// Returns 'true' (1) if the received string has a valid length
inline __bit isNmeaLengthValid() 
{
    return nmeaSentenceLength >= NMEA_LINE_MIN_LEN;
}

// Returns 'true' (1) if the received string starts with a correct RMC header
inline __bit isNmeaHeaderValid()
{
    return 0 == nmea_strcmp(nmeaBuffer, RMC_HEADERS[0], RMC_HEADER_LENGTH) ||
           0 == nmea_strcmp(nmeaBuffer, RMC_HEADERS[1], RMC_HEADER_LENGTH);
}

// Returns 'true' (1) if the received string contains a valid status
inline __bit isNmeaStatusValid() 
{    
    char *p;
    return (p = nmeaGetComma(NMEA_COMMA_STATUS)) && (*(p + 1) == 'A');
}

// Returns 'true' (1) if the received NMEA string has a correct CRC
__bit isNmeaCRCValid()
{
    char *p = nmeaBuffer + 1;
    uint8_t calculatedCRC = 0;
    for (int8_t i = 0; *p != '*' && i != nmeaSentenceLength; i++) {
        calculatedCRC ^= *p++;
    }
    uint8_t receivedCRC = ((hexChar2Int(p + 1) & 0xF) << 4) | (hexChar2Int(p + 2) & 0xF);
    return receivedCRC == calculatedCRC;
}

// Returns 'true' (1) if the received NMEA string has a correct trail
inline __bit isNmeaTrailValid()
{
    return *(nmeaBuffer + nmeaSentenceLength - 1) == '\n' &&
           *(nmeaBuffer + nmeaSentenceLength - 2) == '\r';
}

// Returns 'true' (1) if the received NMEA string contains a valid time (date) substring
__bit isNmeaTimeDateValid(uint8_t comma) {
    char *p = nmeaGetComma(comma);
    if (NULL == p) {
        return 0;
    }

    for (int8_t n = 1; n <= 6; n++) {
        if (*(p + n) < '0' || *(p + n) > '9')  {
            return 0;
        }
    }

    return 1;
}

// Returns 'true' (1) if the received string is a valid NMEA sentence
__bit isNmeaSentenceValid()
{
    return isNmeaLengthValid() &&
           isNmeaHeaderValid() &&
           isNmeaStatusValid() &&
           isNmeaTimeDateValid(NMEA_COMMA_TIME) &&
           isNmeaTimeDateValid(NMEA_COMMA_DATE) &&
           isNmeaTrailValid() &&
           isNmeaCRCValid();
}

//
// Receiving
//

void uart1_isr() __interrupt(4) __using(2)
{
    if (!RI) {
        return;
    }
    
    RI = 0;
    if (nmea_state != NMEA_READY && nmea_state != NMEA_VALIDATE) {
        char ch = SBUF;
        switch (ch) {
        case '$':
            nmeaSentenceLength = 0;
            nmea_state = NMEA_RECEIVING;
            break;
        case '\n':
            if (nmea_state == NMEA_RECEIVING)
                nmea_state = NMEA_VALIDATE;
            else
                nmea_state = NMEA_NONE;
            break;
        default:
            if (nmea_state != NMEA_RECEIVING)
                nmea_state = NMEA_NONE;
            break;
        }

        if (nmea_state == NMEA_NONE) {
            return;
        }

        nmeaBuffer[nmeaSentenceLength++] = ch;

        if (nmea_state == NMEA_VALIDATE) {
            if (isNmeaSentenceValid()) {
                nmea_state = NMEA_READY;
                loop_gate = 1; // force main loop
            } else {
                // bad line, reset
                nmeaSentenceLength = 0;
                nmea_state = NMEA_NONE;
            }
        } else if (nmeaSentenceLength >= NMEA_LINE_MAX_LEN) {
            nmeaSentenceLength = 0;
            nmea_state = NMEA_NONE;
        }
    }
}

//
// Parsing
// 

inline void backupNmeaValues() {
  nmea_prev_tz_hr = nmea_tz_hr;
  nmea_prev_tz_min = nmea_tz_min;
  nmea_prev_tz_dst = nmea_tz_dst;
  nmea_prev_autosync = nmea_autosync;
}

#if !defined(WITHOUT_INACTIVITY_TIMER)
inline void restoreNmeaValues() {
  nmea_tz_hr = nmea_prev_tz_hr;
  nmea_tz_min = nmea_prev_tz_min;
  nmea_tz_dst = nmea_prev_tz_dst;
  nmea_autosync = nmea_prev_autosync;
}
#endif

// Converts two decimal characters into an integer
uint8_t decChars2int(char *p) { return (*p - '0') * 10 + (*(p + 1) - '0'); }

#if !defined(WITHOUT_WEEKDAY)
__code static uint8_t doft[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

// method by Tomohiko Sakamoto, 1992, accurate for any Gregorian date
// returns 0 = Sunday, 1 = Monday, etc.
uint8_t dayofweek(uint16_t y, uint8_t m, uint8_t d) {
  // 1 <= m <= 12,  y > 1752 (in the U.K.)
  y -= m < 3;
  return (y + y / 4 - y / 100 + y / 400 + doft[m - 1] + d) % 7;
}
#endif

inline uint8_t isleap(uint16_t year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

__code const uint8_t monlen[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// month 1..12, year with century
uint8_t days_per_month(uint16_t year, uint8_t month) {
  if (month == 1) {
    return isleap(year) ? 29 : 28;
  }
  
  return monlen[month];
}

void nmea_apply_tz(struct tm *t) {
  int8_t hdiff = nmea_tz_hr;
  if (nmea_tz_dst) {
    hdiff++;
  }

  if (nmea_tz_min) {
    if (hdiff >= 0) {
      // positive TZ
      if ((t->tm_min = t->tm_min + nmea_tz_min) >= 60) {
        t->tm_min -= 60;
        hdiff++;
      }
    } else {
      // negative TZ
      if (nmea_tz_min > t->tm_min) {
        t->tm_min = t->tm_min + 60 - nmea_tz_min;
        hdiff--;
      } else
        t->tm_min = t->tm_min - nmea_tz_min;
    }
  }

  if (hdiff > 0) {
    // positive TZ
    if ((t->tm_hour = t->tm_hour + hdiff) >= 24) {
      // next day
      t->tm_hour -= 24;
      if (++t->tm_mday > days_per_month(t->tm_year + 1900, t->tm_mon)) {
        // next month
        t->tm_mday = 1;
        if (++t->tm_mon >= 12) {
          // next year
          t->tm_mon = 0; // jan
          t->tm_year++;
        }
      }
    }
  } else if (hdiff < 0) {
    // negative TZ
    if (-hdiff > t->tm_hour) {
      // prev day
      t->tm_hour += 24 + hdiff;
      if (!--t->tm_mday) {
        // prev month
        if (!t->tm_mon) { // jan
          // prev year
          t->tm_mon = 11; // dec
          t->tm_year--;
        } else {
          t->tm_mon--;
        }
        t->tm_mday = days_per_month(t->tm_year + 1900, t->tm_mon);
      }
    } else {
      // same day
      t->tm_hour += hdiff;
    }
  }
}

void nmea2localtime() {
  char *p;
  struct tm t;
  // $GPRMC,232231.00,A,,,,,,,170420,,,*27
  p = nmeaGetComma(NMEA_COMMA_DATE) + 1;
  t.tm_mday = decChars2int(p);           // day
  t.tm_mon = decChars2int(p + 2) - 1;    // month, 0 = jan
  t.tm_year = decChars2int(p + 4) + 100; // year - 1900
  p = nmeaGetComma(NMEA_COMMA_TIME) + 1;
  t.tm_hour = decChars2int(p);
  t.tm_min = decChars2int(p + 2);
  t.tm_sec = decChars2int(p + 4);
  nmea_apply_tz(&t);

  ds_writebyte(DS_ADDR_DAY, ds_int2bcd(t.tm_mday));
  ds_writebyte(DS_ADDR_MONTH, ds_int2bcd(t.tm_mon + 1));
  ds_writebyte(DS_ADDR_YEAR, ds_int2bcd(t.tm_year - 100));
  #if !defined(WITHOUT_WEEKDAY)
  ds_writebyte(DS_ADDR_WEEKDAY, dayofweek(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday) + 1);
  #endif

#if !defined(WITHOUT_H12_24_SWITCH)
  if (H12_12) {
    uint8_t hmode = DS_MASK_1224_MODE;

    if (t.tm_hour >= 12) {
      t.tm_hour -= 12;
      hmode |= DS_MASK_PM;
    }

    if (!t.tm_hour) {
      t.tm_hour = 12;
    }

    ds_writebyte(DS_ADDR_HOUR, ds_int2bcd(t.tm_hour) | hmode);
  } else 
#endif  
  {
    ds_writebyte(DS_ADDR_HOUR, ds_int2bcd(t.tm_hour));
  }
  ds_writebyte(DS_ADDR_MINUTES, ds_int2bcd(t.tm_min));
  ds_writebyte(DS_ADDR_SECONDS, 0b10000000);           // set CH, stop clock
  ds_writebyte(DS_ADDR_SECONDS, ds_int2bcd(t.tm_sec)); // clear CH, start clock
}

void nmea_save_tz(void) {
  Delay(10);
  IapEraseSector(IAP_TZ_ADDRESS);
  Delay(10);
  IapProgramByte(IAP_TZ_HR, (uint8_t)nmea_tz_hr);
  IapProgramByte(IAP_TZ_MIN, nmea_tz_min);
  IapProgramByte(IAP_TZ_DST, nmea_tz_dst);
  IapProgramByte(IAP_TZ_AUTOSYNC, nmea_autosync);
}

void nmea_load_tz(void) {
  nmea_tz_hr = (int8_t)IapReadByte(IAP_TZ_HR);
  nmea_tz_min = IapReadByte(IAP_TZ_MIN);
  nmea_tz_dst = IapReadByte(IAP_TZ_DST);
  nmea_autosync = IapReadByte(IAP_TZ_AUTOSYNC);
  
  // HR after reflash will be == 0xff == -1 by default
  if (nmea_tz_hr < -12 || nmea_tz_hr > 12) {
    nmea_tz_hr = 0;
  }

  if (nmea_tz_min == 0xff || nmea_tz_min && nmea_tz_min != 30 && nmea_tz_min != 45) {
    nmea_tz_min = 0;
  }

  if (nmea_tz_dst == 0xff || nmea_tz_dst > 1) {
    nmea_tz_dst = 0;
  }
}
