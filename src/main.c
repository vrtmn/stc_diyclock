//
// STC15F204EA DIY LED Clock
// Copyright 2016, Jens Jensen
//

#include <stdint.h>
#include <stdio.h>
#include "stc15.h"
#include "models.h"
#include "adc.h"
#include "ds1302.h"
#include "eeprom_consts.h"
#ifdef BCD_DISPLAY
#include "led_bcd.h"

#define BCD_DISPLAY_SETTINGS_1 1
#define BCD_DISPLAY_SETTINGS_2 2
#define BCD_DISPLAY_SETTINGS_3 3
#define BCD_DISPLAY_SETTINGS_4 4
#define BCD_DISPLAY_SETTINGS_5 5
#define BCD_DISPLAY_SETTINGS_6 6
#define BCD_DISPLAY_SETTINGS_7 7

#elif defined(HW_REVISION_CUSTOM_PCB)
#include "led_pcb_v2.h"
#else
#include "led.h"
#endif
#include "hwconfig.h"
#include "buttonsmode.h"
#include "displaymode.h"
#include "event.h"
#include "buttonmonitor.h"

// When defined, all symbols are shown in a loop instead of time
// #define SYMBOLS_TEST

#ifdef SYMBOLS_TEST
#define WITHOUT_INACTIVITY_TIMER
#endif

#define BRIGHTNESS_HIGH_DEFAULT_VALUE   0x01
#define BRIGHTNESS_LOW_DEFAULT_VALUE    0x24
#define BRIGHTNESS_LOW_MAX_VALUE        0x46
#define BRIGHTNESS_NIGHT_DEFAULT_VALUE  0x34
#define BRIGHTNESS_NIGHT_MAX_VALUE      0x71
#define LIGHT_SENSOR_NIGHT_TH           245         // Night mode threshold
#define LIGHT_SENSOR_MAX_VALUE          (255 - (255 - LIGHT_SENSOR_NIGHT_TH))
#define LIGHT_SENSOR_DEFAULT_CORRECTION 0
#define LIGHT_SENSOR_MAX_CORRECTION     50

uint8_t lightSensorCorrection = LIGHT_SENSOR_DEFAULT_CORRECTION;
uint8_t brightnessHigh = BRIGHTNESS_HIGH_DEFAULT_VALUE;
uint8_t brightnessLow = BRIGHTNESS_LOW_DEFAULT_VALUE;
uint8_t brightnessNight = BRIGHTNESS_NIGHT_DEFAULT_VALUE;

uint8_t count;     // main loop counter
uint8_t temp;      // temperature sensor value
uint8_t lightval;  // light sensor value
uint8_t previous_second = 0;

volatile uint8_t display_counter;
volatile int8_t counter_10ms;
volatile int8_t counter_100ms;
volatile int8_t counter_500ms;
volatile int8_t counter_1sec;
volatile int8_t counter_blinker_slow;
#if !defined(WITHOUT_INACTIVITY_TIMER)
// Contains the number of seconds that have passed since any button was pressed
volatile int8_t counter_inactivity;
#endif

volatile __bit blinker_slow;
volatile __bit blinker_fast;
volatile __bit loop_gate;

#ifdef SYMBOLS_TEST
#define SYMBOLS_TEST_BCD_MAX_VALUE 15
#define SYMBOLS_TEST_BCD_COLUMNS 6

volatile int8_t counter_symbols_test = 0;
#endif

enum DisplayMode display_mode = DM_NORMAL;
enum ButtonsMode buttons_mode = K_NORMAL;

__bit flash_01;
__bit flash_23;
#ifdef SIX_DIGITS
__bit flash_45;
#endif

uint8_t rtc_hh_bcd;
uint8_t rtc_mm_bcd;

#if !defined(WITHOUT_H12_24_SWITCH)
__bit rtc_pm;
#endif

#if !defined(WITHOUT_ALARM) || !defined(WITHOUT_CHIME)
#ifndef WITHOUT_ALARM
uint8_t alarm_hh_bcd;
uint8_t alarm_mm_bcd;
__bit alarm_pm;
__bit alarm_trigger;
__bit alarm_reset;

uint8_t snooze_time;	            // snooze(min)
uint8_t alarm_mm_snooze;	        // Next alarm time (min)

volatile int16_t counter_2sec;
volatile __bit blinker_slowest;
#endif

#ifndef WITHOUT_CHIME
uint8_t chime_ss_bcd; // hour since
uint8_t chime_uu_bcd; // hour until
__bit chime_ss_pm;
__bit chime_uu_pm;

enum ChimeState {
    CHIME_IDLE = 0,
    CHIME_RUNNING,
    CHIME_WAITING
};

volatile enum ChimeState chime_state = CHIME_IDLE;
volatile int16_t chime_ticks;   // 10ms inc
#endif

__bit cfg_changed = 1;
#endif

volatile __bit S1_LONG;
volatile __bit S1_PRESSED;
volatile __bit S2_LONG;
volatile __bit S2_PRESSED;
#ifdef HW_REVISION_WITH_VOICE_CHIP
volatile __bit S3_LONG;
volatile __bit S3_PRESSED;
#endif

volatile uint8_t debounce[NUM_SW];      // switch debounce buffer
volatile uint8_t switchcount[NUM_SW];

#ifdef WITH_NMEA
#include "nmea.h"

__bit is_nmea_receiving_on = 0;

void enable_nmea_receiving();
void disable_nmea_receiving();
#endif

void saveSettings();

volatile enum Event event;

/*
Align the blinker slow counter with the actual time (seconds).
This is essential for the HH:MM:SS model - the blinking dots must be
in sync with the changing seconds to look nice.
*/
inline void timerBlinkerSlow() {
  if (0 == previous_second) { // initial value
    previous_second = rtc_table[DS_ADDR_SECONDS];
  } else if (rtc_table[DS_ADDR_SECONDS] != previous_second) {
    counter_blinker_slow = 1;
    blinker_slow = 1;
    previous_second = rtc_table[DS_ADDR_SECONDS];
  }

  if (counter_blinker_slow == 5) { // 500 ms
    counter_blinker_slow = 0;
    blinker_slow = !blinker_slow; // blink every 500ms
  }
}

inline void timer500ms() {
#ifndef WITHOUT_ALARM
  // 1/ 2sec: 20000 ms
  if (counter_2sec == 20) {
    counter_2sec = 0;
  }
  // 500 ms on=true=1, 1500 ms off=false=0
  blinker_slowest = counter_2sec < 5;
#endif
}

inline void timer1sec() {
#if defined(WITH_NMEA)
  if (nmea_seconds_to_sync > 0) {
    nmea_seconds_to_sync--;
  }

  if (IS_NMEA_AUTOSYNC_ON && 0 == nmea_seconds_to_sync) {
    enable_nmea_receiving();
  }
#endif

#ifdef SYMBOLS_TEST
  counter_symbols_test++;
#ifdef BCD_DISPLAY
  if (counter_symbols_test > (SYMBOLS_TEST_BCD_MAX_VALUE + SYMBOLS_TEST_BCD_COLUMNS - 1)) {
    counter_symbols_test = 0;
  }
#else
  if (counter_symbols_test > sizeof(ledSymbolsRev) / sizeof(uint8_t) - 1) {
    counter_symbols_test = 0;
  }
#endif
#endif
}

inline void timer100ms() {
  blinker_fast = !blinker_fast; // blink every 100ms
  loop_gate = 1;                // every 100ms

  counter_500ms++; // increment every 100ms
  counter_1sec++;
  counter_blinker_slow++;

#ifndef WITHOUT_ALARM
  counter_2sec++; // increment every 100ms
#endif

  timerBlinkerSlow();

  if (counter_500ms == 5) {
    counter_500ms = 0;
    timer500ms();
  }

  if (10 == counter_1sec) {
#if !defined(WITHOUT_INACTIVITY_TIMER)
    counter_inactivity++; // 1 sec increment
#endif
    counter_1sec = 0;
    timer1sec();
  }
}

inline void displayDigits() {
  uint8_t digit = display_counter % (uint8_t)NUMBER_OF_DIGITS;

  // turn off all digits, set high
  LED_DIGITS_OFF();

  // auto dimming, skip lighting for some cycles
  if (display_counter % lightval < NUMBER_OF_DIGITS) {
    // fill digits
    LED_SEGMENT_PORT = displayBuffer[digit];
    // issue #32, fix for newer sdcc versions which are using non-atomic port
    // access
    LED_DIGITS_PORT &= ~((1 << LED_DIGITS_PORT_BASE) << digit);
  }

  display_counter++;
}

inline void checkButtons() {
  enum Event ev = EV_NONE;

  // Check SW status and chattering control
  MONITOR_S(1);
  MONITOR_S(2);
#ifdef HW_REVISION_WITH_VOICE_CHIP
  MONITOR_S(3);
#endif

  if (ev == EV_S1_LONG && S2_PRESSED) {
    S2_LONG = 1;
    switchcount[1] = 0;
    ev = EV_S1S2_LONG;
  } else if (ev == EV_S2_LONG && S1_PRESSED) {
    S1_LONG = 1;
    switchcount[0] = 0;
    ev = EV_S1S2_LONG;
  }
  if (event == EV_NONE) {
    event = ev;
  }
}

inline void timer10ms() {
  counter_100ms++;

  // 10/sec: 100 ms
  if (counter_100ms == 10) {
    counter_100ms = 0;
    timer100ms();
  }

#ifndef WITHOUT_CHIME
  if (chime_state != CHIME_IDLE) {
    chime_ticks++; // increment every 10ms
  }
#endif

  checkButtons();
}

/*
  interrupt: every 0.1ms=100us come here

  Check button status
  Dynamically LED turn on
 */
void timer0_isr() __interrupt(1) __using(1) {
  displayDigits();

  // 100/sec: 10 ms
  if (counter_10ms == 100) {
    counter_10ms = 0;
    timer10ms();
  }

  counter_10ms++; // increment every 0.1ms
}

// Call timer0_isr() 10000/sec: 0.0001 sec
// Initialize the timer count so that it overflows after 0.0001 sec
// THTL = 0x10000 - FOSC / 12 / 10000 = 0x10000 - 92.16 = 65444 = 0xFFA4
// When 11.0592MHz clock case, set every 100us interruption
void timer0Init(void) // 100us @ 11.0592MHz
{
  // refer to section 7 of datasheet: STC15F2K60S2-en2.pdf
  // TMOD = 0;    // default: 16-bit auto-reload
  // AUXR = 0;    // default: traditional 8051 timer frequency of FOSC / 12
  // Initial values of TL0 and TH0 are stored in hidden reload registers: RL_TL0
  // and RL_TH0
  TL0 = 0xA4; // Initial timer value
  TH0 = 0xFF; // Initial timer value
  TF0 = 0;    // Clear overflow flag
  TR0 = 1;    // Timer0 start run
  ET0 = 1;    // Enable timer0 interrupt
  EA = 1;     // Enable global interrupt
}

// Formula was : 76-raw*64/637 - which makes use of integer mult/div routines
// Getting degF from degC using integer was not good as values were sometimes jumping by 2
// The floating point one is even worse in term of code size generated (>1024bytes...)
// Approximation for slope is 1/10 (64/637) - valid for a normal 20 degrees range
// & let's find some other trick (80 bytes - See also docs\Temp.ods file)
int8_t getTemperature(uint16_t raw) {
    uint16_t val=raw;
    uint8_t temp;

    raw<<=2;
    if (CONF_C_F) raw<<=1;  // raw*5 (4+1) if Celcius, raw*9 (4*2+1) if Farenheit
    raw+=val;

    if (CONF_C_F) {val=6835; temp=32;}  // equiv. to temp=xxxx-(9/5)*raw/10 i.e. 9*raw/50
                                        // see next - same for degF
             else {val=5*757; temp=0;}  // equiv. to temp=xxxx-raw/10 or which is same 5*raw/50  
                                        // at 25degC, raw is 512, thus 24 is 522 and limit between 24 and 25 is 517
                                        // so between 0deg and 1deg, limit is 517+24*10 = 757 
                                        // (*5 due to previous adjustment of raw value)
    while (raw<val) {temp++; val-=50;}

    return temp + (cfg_table[CFG_TEMP_BYTE] & CFG_TEMP_MASK) - 4;
}

inline uint8_t adjustDotVisibility(uint8_t pm) {
#if !defined(WITHOUT_ALARM)
#ifdef WITHOUT_H12_24_SWITCH
  return CONF_ALARM_ON && blinker_slowest && blinker_fast;
#else
  // dot 3: If alarm is on, blink for 500 ms every 2000 ms
  //        If 12h: on if pm when not blinking
  if (!H12_12) { // 24h mode
    pm = CONF_ALARM_ON && blinker_slowest && blinker_fast;
  } else if (CONF_ALARM_ON && blinker_slowest) {
    // 12h mode case: blink 500ms, AM/PM=Off/On in another 500ms
    pm = blinker_fast;
  }
#endif
#endif

  return pm;
}

#ifndef WITHOUT_ALARM
// Set next alarm_min by adding snooze
uint8_t addBCD(uint8_t snooze) {
  snooze;	//dpl
  __asm
    mov a, dpl
    add a, _alarm_mm_bcd
    da a
    mov dpl, a
    ret
  __endasm;
	
}
#endif

//
// Display functions
//

inline void displayTime() {
  uint8_t hh = rtc_hh_bcd;
  uint8_t mm = rtc_mm_bcd;
  __bit pm = 0;
#if !defined(WITHOUT_H12_24_SWITCH)
  pm = rtc_pm;
#endif

#if !defined(WITHOUT_ALARM)
  if (display_mode == DM_ALARM) {
    hh = alarm_hh_bcd;
    mm = alarm_mm_bcd;
    pm = alarm_pm;
  }
#endif

  // Digits 0, 1
  if (!flash_01 || blinker_fast || S1_LONG) {
    uint8_t h0 = hh >> 4;
#if !defined(WITHOUT_H12_24_SWITCH)
    if (H12_12 && h0 == 0) {
      h0 = LED_BLANK;
    }
#endif
    fillDigit(0, h0);
    fillDigit(1, hh & 0x0F);
  }

  // Digits 2, 3
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(2, mm >> 4);

#ifdef SIX_DIGITS
    fillDigit(3, mm & 0x0F);
#else
    fillDigit(3, mm & 0x0F);
#endif
  }

#ifdef SIX_DIGITS
  fillDot(3, display_mode == DM_NORMAL ? blinker_slow : 0);
#endif

  // Dots 1, 2
#if !defined(WITHOUT_ALARM)
  if (display_mode == DM_ALARM) {
    fillDot(1, 1);
    fillDot(2, 1);
  } else
#endif
      if (blinker_slow) {
    fillDot(1, 1);
    fillDot(2, 1);
  }

  // Digits 4, 5
#ifdef SIX_DIGITS
  if (!flash_45 || blinker_fast || S1_LONG) {
    if (display_mode == DM_NORMAL) {
      fillDigit(4, (rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4));      
      fillDigit(5, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS);
    }
#if !defined(WITHOUT_ALARM)
    if (display_mode == DM_ALARM) {
      // Show letter 'A' for the alarm mode
      fillDigit(5, LED_a);
    }
#endif
  }
  fillDot(4, display_mode == DM_NORMAL ? blinker_slow : 0);
#endif

#ifdef SIX_DIGITS
  fillDot(5, adjustDotVisibility(pm));
#else
  fillDot(3, adjustDotVisibility(pm));
#endif
}

#if !defined(WITHOUT_CHIME)
inline void displayChime() {
  // Digits 0, 1
  if (!flash_01 || blinker_fast || S1_LONG) {
    uint8_t h0 = chime_ss_bcd >> 4;
#if !defined(WITHOUT_H12_24_SWITCH)
    if (H12_12 && h0 == 0) {
      h0 = LED_BLANK;
    }
#endif
    fillDigit(0, h0);
    fillDigit(1, chime_ss_bcd & 0x0F);
  }
  fillDot(1, chime_ss_pm);

  // Digits 2, 3
  if (!flash_23 || blinker_fast || S1_LONG) {
    uint8_t m0 = chime_uu_bcd >> 4;
#if !defined(WITHOUT_H12_24_SWITCH)
    if (H12_12 && m0 == 0) {
      m0 = LED_BLANK;
    }
#endif
    fillDigit(2, m0);
    fillDigit(3, chime_uu_bcd & 0x0F);
  }

#ifdef SIX_DIGITS
  fillDot(3, chime_uu_pm);
  fillDigit(5, LED_c);
  fillDot(5, CONF_CHIME_ON);
#else
  fillDot(2, chime_uu_pm);
  fillDot(3, CONF_CHIME_ON);
#endif
}
#endif

#if !defined(WITHOUT_H12_24_SWITCH)
inline void display12h24h() {
  if (!H12_12) {
    fillDigit(1, 2);
    fillDigit(2, 4);
  } else {
    fillDigit(1, 1);
    fillDigit(2, 2);
  }
  fillDigit(3, LED_h);
}
#endif

#ifdef WITH_NMEA
#ifdef BCD_DISPLAY
inline void displayNmeaTimeZone() {
  fillDigit(1, BCD_DISPLAY_SETTINGS_1);
  int8_t hh = nmea_tz_hr;
  if (hh < 0) {
    hh = -hh;
    fillDot(3, 1);
  }
  fillDot(3, 1);
  fillDot(4, 1);

  if (!flash_01 || blinker_fast || S1_LONG) {
    if (hh >= 10) {
      fillDigit(2, 1);
    }
    fillDigit(3, hh % 10);
  }
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(4, nmea_tz_min / 10);
    fillDigit(5, nmea_tz_min % 10);
  }
}
#elif defined(SIX_DIGITS)
inline void displayNmeaTimeZone() {
  fillDigit(0, LED_t);
  int8_t hh = nmea_tz_hr;
  if (hh < 0) {
    hh = -hh;
    if (hh < 10) {
      fillDigit(2, LED_DASH);
    } else {
      fillDigit(1, LED_DASH);
    }
  } else {
    fillDigit(1, LED_BLANK);
    fillDigit(2, LED_BLANK);
  }
  fillDot(3, 1);
  fillDot(4, 1);

  if (!flash_01 || blinker_fast || S1_LONG) {
    if (hh >= 10) {
      fillDigit(2, 1);
    }
    fillDigit(3, hh % 10);
  }
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(4, nmea_tz_min / 10);
    fillDigit(5, nmea_tz_min % 10);
  }
}
#else
inline void displayNmeaTimeZone() {
  int8_t hh = nmea_tz_hr;
  if (hh < 0) {
    hh = -hh;
    fillDot(3, 1);
  }
  fillDot(1, 1);
  fillDot(2, 1);

  if (!flash_01 || blinker_fast || S1_LONG) {
    if (hh >= 10) {
      fillDigit(0, 1);
    } else {
      fillDigit(0, LED_BLANK);
    }
    fillDigit(1, hh % 10);
  }

  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(2, nmea_tz_min / 10);
    fillDigit(3, nmea_tz_min % 10);
  }
}
#endif

inline void displayNmeaDST() {
#ifdef BCD_DISPLAY
  fillDigit(1, BCD_DISPLAY_SETTINGS_2);
#else
  fillDigit(0, LED_d);
  fillDigit(1, LED_s);
  fillDigit(2, LED_t);
#endif

#ifdef BCD_DISPLAY
  if (!flash_45 || blinker_fast || S1_LONG) {
    if (nmea_tz_dst) {
      fillDigit(5, LED_f);
    } else {
      fillDigit(5, LED_BLANK);
    }
  }
#elif defined(SIX_DIGITS) && !defined(BCD_DISPLAY)
  if ((!flash_23 && !flash_45) || blinker_fast || S1_LONG) {
    if (nmea_tz_dst) {
      fillDigit(4, LED_o);
      fillDigit(5, LED_n);
    } else {
      fillDigit(3, LED_o);
      fillDigit(4, LED_f);
      fillDigit(5, LED_f);
    }
  }
#else
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(3, nmea_tz_dst);
  }
#endif
}

#ifdef BCD_DISPLAY
inline void displayNmeaAutoupdate() {
  fillDigit(1, BCD_DISPLAY_SETTINGS_3);

  if (NMEA_AUTOSYNC_OFF != nmea_autosync) {
    if (!flash_23 || blinker_fast || S1_LONG) {
      fillDigit(2, nmea_autosync / 10);
      fillDigit(3, nmea_autosync % 10);
    }
  }
}
#elif defined(SIX_DIGITS) && !defined(BCD_DISPLAY)
inline void displayNmeaAutoupdate() {
  fillDigit(0, LED_u);
  fillDigit(1, LED_p);
  fillDigit(2, LED_d);

  if ((!flash_23 && !flash_45) || blinker_fast || S1_LONG) {
    if (NMEA_AUTOSYNC_OFF == nmea_autosync) {
      fillDigit(3, 0);
      fillDigit(4, LED_f);
      fillDigit(5, LED_f);
    } else {
      fillDigit(3, nmea_autosync / 10);
      fillDigit(4, nmea_autosync % 10);
      fillDigit(5, LED_h);
    }
  }
}
#else
inline void displayNmeaAutoupdate() {
  if (NMEA_AUTOSYNC_OFF == nmea_autosync) {
    fillDigit(0, 0);
    fillDigit(1, LED_f);
    fillDigit(2, LED_f);
  } else {
    fillDigit(0, nmea_autosync / 10);
    fillDigit(1, nmea_autosync % 10);
    fillDigit(2, LED_h);
  }
}
#endif
#endif

inline void displayLightSensorCorrection() {
#ifdef BCD_DISPLAY
  fillDigit(1, BCD_DISPLAY_SETTINGS_4);
#else
  fillDigit(0, LED_s);
  fillDigit(1, LED_c);
#endif

#ifdef SIX_DIGITS
  if (!flash_45 || blinker_fast || S1_LONG) {
    fillDigit(4, lightSensorCorrection  >> 4);
    fillDigit(5, lightSensorCorrection & 0x0F);
  }
#else
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(2, lightSensorCorrection  >> 4);
    fillDigit(3, lightSensorCorrection & 0x0F);
  }
#endif
}

inline void displayBrightnessHigh() {
#ifdef BCD_DISPLAY
  fillDigit(1, BCD_DISPLAY_SETTINGS_5);
#else
  fillDigit(0, LED_b);
  fillDigit(1, LED_h);
#endif
  
  #ifdef SIX_DIGITS
  if (!flash_45 || blinker_fast || S1_LONG) {
    fillDigit(4, brightnessHigh  >> 4);
    fillDigit(5, brightnessHigh & 0x0F);
  }
  #else
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(2, brightnessHigh  >> 4);
    fillDigit(3, brightnessHigh & 0x0F);
  }
  #endif
}

inline void displayBrightnessLow() {
#ifdef BCD_DISPLAY
  fillDigit(1, BCD_DISPLAY_SETTINGS_6);
#else
  fillDigit(0, LED_b);
  fillDigit(1, LED_l);
#endif

  #ifdef SIX_DIGITS
  if (!flash_45 || blinker_fast || S1_LONG) {
    fillDigit(4, brightnessLow  >> 4);
    fillDigit(5, brightnessLow & 0x0F);
  }
  #else
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(2, brightnessLow  >> 4);
    fillDigit(3, brightnessLow & 0x0F);
  }
  #endif
}

inline void displayBrightnessNight() {
#ifdef BCD_DISPLAY
  fillDigit(1, BCD_DISPLAY_SETTINGS_7);
#else
  fillDigit(0, LED_b);
  fillDigit(1, LED_n);
#endif

  #ifdef SIX_DIGITS
  if (!flash_45 || blinker_fast || S1_LONG) {
    fillDigit(4, brightnessNight  >> 4);
    fillDigit(5, brightnessNight & 0x0F);
  }
  #else
  if (!flash_23 || blinker_fast || S1_LONG) {
    fillDigit(2, brightnessNight  >> 4);
    fillDigit(3, brightnessNight & 0x0F);
  }
  #endif
}

#if !defined(SIX_DIGITS)
inline void displaySeconds() {
#ifdef SHOW_MINUTES_WITH_SECONDS
  uint8_t mm = rtc_mm_bcd;
  fillDigit(0, mm >> 4);
  fillDigit(1, mm & 0x0F);
#endif
  fillDot(1, blinker_slow);

  fillDigit(2, (rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4));
  fillDot(2, blinker_slow);
  fillDigit(3, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS);
}
#endif

#if !defined(WITHOUT_DATE)
inline void displayDate() {
  if (!flash_01 || blinker_fast || S1_LONG) {
    if (!CONF_SW_MMDD) {
      // tenmonth ( &MASK_TENS useless, as MSB bits are read as '0')
      fillDigit(0, rtc_table[DS_ADDR_MONTH] >> 4); 
      fillDigit(1, rtc_table[DS_ADDR_MONTH] & DS_MASK_MONTH_UNITS);
    } else {
      // tenmonth ( &MASK_TENS useless, as MSB bits are read as '0')
      fillDigit(2, rtc_table[DS_ADDR_MONTH] >> 4); 
      fillDigit(3, rtc_table[DS_ADDR_MONTH] & DS_MASK_MONTH_UNITS);
    }
  }

  if (!flash_23 || blinker_fast || S1_LONG) {
    if (!CONF_SW_MMDD) {
      // tenday   ( &MASK_TENS useless)
      fillDigit(2, rtc_table[DS_ADDR_DAY] >> 4); 
      fillDigit(3, rtc_table[DS_ADDR_DAY] & DS_MASK_DAY_UNITS);
    } else {
      // tenday   ( &MASK_TENS useless)
      fillDigit(0, rtc_table[DS_ADDR_DAY] >> 4); 
      fillDigit(1, rtc_table[DS_ADDR_DAY] & DS_MASK_DAY_UNITS);
    }
  }
  fillDot(1, 1);
#ifdef SIX_DIGITS
  fillDot(3, 1);
  if (!flash_45 || blinker_fast || S1_LONG) {
    fillDigit(4, (rtc_table[DS_ADDR_YEAR] >> 4) & (DS_MASK_YEAR_TENS >> 4));
    fillDigit(5, rtc_table[DS_ADDR_YEAR] & DS_MASK_YEAR_UNITS);
  }
#endif
}

#if !defined(WITHOUT_WEEKDAY)
inline void displayWeekday() {
  uint8_t wd = rtc_table[DS_ADDR_WEEKDAY] - 1;
  fillDigit(1, weekDay[wd][0] - 'A' + LED_a);
  fillDigit(2, weekDay[wd][1] - 'A' + LED_a);
  fillDigit(3, weekDay[wd][2] - 'A' + LED_a);
}
#endif

#if !defined(SIX_DIGITS)
inline void displayYear() {
  fillDigit(0, 2);
  fillDigit(1, 0);

  fillDigit(2, (rtc_table[DS_ADDR_YEAR] >> 4) & (DS_MASK_YEAR_TENS >> 4));
  fillDigit(3, rtc_table[DS_ADDR_YEAR] & DS_MASK_YEAR_UNITS);
}
#endif
#endif

inline void displayTemperature() {
#ifdef SIX_DIGITS
  fillDigit(2, ds_int2bcd_tens(temp));
  fillDigit(3, ds_int2bcd_ones(temp));
#ifdef BCD_DISPLAY
  if (CONF_C_F) {
    fillDigit(4, LED_f);
    fillDigit(5, 9);
  } else {
    fillDigit(4, LED_f);
    fillDigit(5, LED_c);
  }
  fillDot(4, 1);
#else
  fillDigit(4, CONF_C_F ? LED_f : LED_c);
  fillDot(4, 1);
#endif
#else
  fillDigit(0, ds_int2bcd_tens(temp));
  fillDigit(1, ds_int2bcd_ones(temp));
  fillDigit(2, CONF_C_F ? LED_f : LED_c);
  fillDot(2, 1);
#endif
}

#ifdef WITH_DEBUG_SCREENS
inline void displayDebugLightSensorADCValue() {
  // photoresistor adc and lightval
  uint8_t adc = getADCResult8(ADC_LIGHT);
  uint8_t lv = lightval;
  fillDigit(0, adc >> 4);
  fillDigit(1, adc & 0x0F);
  fillDigit(2, lv >> 4);
  fillDigit(3, lv & 0x0F);
#ifdef SIX_DIGITS
  fillDigit(5, 1);
#endif
}

inline void displayDebugThermistorADCValue() {
  // thermistor adc
  uint16_t rt = getADCResult(ADC_TEMP);
  fillDigit(0, rt >> 12);
  fillDigit(1, rt >> 8 & 0x0F);
  fillDigit(2, rt >> 4 & 0x0F);
  fillDigit(3, rt & 0x0F);
#ifdef SIX_DIGITS
  fillDigit(5, 2);
#endif
}

inline void displayDebugLoopCounter() {
  // seconds, loop counter, blinkers, S1/S2, keypress events
  uint8_t cc = count;
  if (S1_PRESSED || S2_PRESSED) {
    fillDigit(0, S1_PRESSED || S2_PRESSED ? LED_DASH : LED_BLANK);
    fillDigit(1, S1_LONG || S2_LONG ? LED_DASH : LED_BLANK);
  } else {
    fillDigit(0, (rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4));
    fillDigit(1, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS);
    fillDot(1, blinker_slow);
  }
  fillDigit(2, cc >> 4 & 0x0F);
  fillDigit(3, cc & 0x0F);
  fillDot(3, blinker_slow & blinker_fast);
#ifdef SIX_DIGITS
  fillDigit(5, 3);
#endif
}
#endif

#ifdef SYMBOLS_TEST
inline void displaySymbolsTest() {
#ifdef BCD_DISPLAY
  if (counter_symbols_test > SYMBOLS_TEST_BCD_MAX_VALUE) {
    uint8_t column = counter_symbols_test - SYMBOLS_TEST_BCD_MAX_VALUE - 1;
    fillDigit(column, LED_f);
    fillDot(column, blinker_slow);
  } else {
    for (int n = 0; n < NUMBER_OF_DIGITS; n++) {
      fillDigit(n, counter_symbols_test);
      fillDot(n, blinker_slow);
    }
  }
#else
  for (int n = 0; n < NUMBER_OF_DIGITS; n++) {
    fillDigit(n, counter_symbols_test);
    fillDot(n, blinker_slow);
  }
#endif
}
#endif

inline void displayScreen() {
#ifdef SYMBOLS_TEST
  displaySymbolsTest();
  return;
#endif

  switch (display_mode) {
  case DM_NORMAL:
#if !defined(WITHOUT_ALARM)
  case DM_ALARM:
#endif
    displayTime();
    break;

#if !defined(WITHOUT_CHIME)
  case DM_CHIME:
    displayChime();
    break;
#endif

#if !defined(WITHOUT_H12_24_SWITCH)
  case DM_SET_HOUR_12_24:
    display12h24h();
    break;
#endif

#ifdef WITH_NMEA
  case DM_NMEA_TIMEZONE:
    displayNmeaTimeZone();
    break;

  case DM_NMEA_DST:
    displayNmeaDST();
    break;

  case DM_NMEA_AUTOUPDATE:
    displayNmeaAutoupdate();
    break;
#endif

  case DM_LIGHT_SENSOR_CORRECTION:
    displayLightSensorCorrection();
    break;

    case DM_BRIGHTNESS_HIGH:
    displayBrightnessHigh();
    break;

  case DM_BRIGHTNESS_LOW:
    displayBrightnessLow();
    break;

  case DM_BRIGHTNESS_NIGHT:
    displayBrightnessNight();
    break;

#if !defined(SIX_DIGITS)
  case DM_SECONDS:
    displaySeconds();
    break;
#endif

#if !defined(WITHOUT_DATE)
  case DM_DATE:
    displayDate();
    break;
#endif

#if !defined(WITHOUT_DATE)
#if !defined(WITHOUT_WEEKDAY)
  case DM_WEEKDAY:
    displayWeekday();
    break;
#endif

#if !defined(SIX_DIGITS)
  case DM_YEAR:
    displayYear();
    break;
#endif
#endif

  case DM_TEMPERATURE:
    displayTemperature();
    break;

#ifdef WITH_DEBUG_SCREENS
  case DM_DEBUG_SCREEN_1:
    displayDebugLightSensorADCValue();
    break;
#if NUMBER_OFDEBUG_SCREENS > 1
  case DM_DEBUG_SCREEN_2:
    displayDebugThermistorADCValue();
    break;
#if NUMBER_OFDEBUG_SCREENS > 2
  case DM_DEBUG_SCREEN_3:
    displayDebugLoopCounter();
    break;
#endif
#endif
#endif
  }
}

#if !defined(WITHOUT_INACTIVITY_TIMER)
inline void handleInactivityTimer(enum Event ev) {
  if (ev != EV_NONE) {
    counter_inactivity = 0;
  }

  uint8_t debugScreensFlag = 0;
#ifdef WITH_DEBUG_SCREENS
  debugScreensFlag = (display_mode == DM_DEBUG_SCREEN_1) ||
                     (display_mode == DM_DEBUG_SCREEN_2) ||
                     (display_mode == DM_DEBUG_SCREEN_3);
#endif

  if (counter_inactivity > 10 && !debugScreensFlag) {
    counter_inactivity = 0;
#ifdef WITH_NMEA
    if (display_mode == DM_NMEA_TIMEZONE || display_mode == DM_NMEA_DST ||
        display_mode == DM_NMEA_AUTOUPDATE) {
      restoreNmeaValues();
    }
#endif
    buttons_mode = K_NORMAL;
  }
}
#endif

inline void adjustDisplayMode() {
    if (display_mode == DM_NORMAL && buttons_mode == K_NORMAL) {
      uint8_t ss = rtc_table[DS_ADDR_SECONDS];
      if (ss < 0x20) {
        return;
      }
#ifdef AUTO_SHOW_TEMPERATURE
      else if (ss < 0x25) {
      display_mode = DM_TEMPERATURE;
    }
#endif

#if !defined(WITHOUT_DATE) && defined(AUTO_SHOW_DATE)
    else if (ss < 0x30) {
      display_mode = DM_DATE;
    }
#endif

#if !defined(WITHOUT_DATE) && !defined(WITHOUT_WEEKDAY) && defined(AUTO_SHOW_WEEKDAY)
    else if (ss < 0x35) {
      display_mode = DM_WEEKDAY;
    }
#endif
  }
}

//
// ADC functions
//

inline void processADCValues() {
  if (count % (uint8_t)4 == 0) {
    temp = getTemperature(getADCResult(ADC_TEMP));

    /*
    Auto-dimming
     The light sensor (getADCResult8(ADC_LIGHT) function)
     returns 0 at maximum light and 255 in darkness.

    The lightval must must be in the following range:
     LIGHTVAL_LOWEST_VALUE - highest brightness (default value: 4)
     LIGHTVAL_DARKEST_VALUE - lowest brightness (it's currently set to 32,
     may be lower, but the digits will flicker)
    */

    uint8_t adcLight = getADCResult8(ADC_LIGHT);
    uint8_t lightvalMax = brightnessLow;

    if (adcLight > lightSensorCorrection) {
      adcLight -= lightSensorCorrection;
    } else {
      adcLight = 0;
    }

    // Check for the night mode
    if (adcLight >= LIGHT_SENSOR_NIGHT_TH - lightSensorCorrection) {
      lightvalMax = brightnessNight;
    }

    // Calculate the lightval
    lightval = adcLight * lightvalMax / LIGHT_SENSOR_MAX_VALUE;
    if (lightval < brightnessHigh) {
      lightval = brightnessHigh;
    }
  }
}

//
// Alarm & Chime functions
//

#if !defined(WITHOUT_ALARM) || !defined(WITHOUT_CHIME)
inline void prepareAlarmAndChimeBCD() {
  if (cfg_changed) {
#ifndef WITHOUT_CHIME
    chime_ss_bcd = cfg_table[CFG_CHIME_SINCE_BYTE] >> CFG_CHIME_SINCE_SHIFT;
    chime_uu_bcd = cfg_table[CFG_CHIME_UNTIL_BYTE] & CFG_CHIME_UNTIL_MASK;
    chime_ss_pm = chime_uu_pm = 0;
    
#if !defined(WITHOUT_H12_24_SWITCH)    
    if (H12_12) {
      if (chime_ss_bcd >= 12) {
        chime_ss_pm = 1;
        chime_ss_bcd -= 12;
      }
      if (chime_ss_bcd == 0)
        chime_ss_bcd = 12;
      if (chime_uu_bcd >= 12) {
        chime_uu_pm = 1;
        chime_uu_bcd -= 12;
      }
      if (chime_uu_bcd == 0)
        chime_uu_bcd = 12;
    }
#endif    

    // convert to BCD
    chime_ss_bcd = ds_int2bcd(chime_ss_bcd);
    chime_uu_bcd = ds_int2bcd(chime_uu_bcd);
#endif

#ifndef WITHOUT_ALARM
    alarm_pm = 0;
    alarm_hh_bcd = cfg_table[CFG_ALARM_HOURS_BYTE] >> CFG_ALARM_HOURS_SHIFT;
#if !defined(WITHOUT_H12_24_SWITCH)    
    if (H12_12) {
      if (alarm_hh_bcd >= 12) {
        alarm_pm = 1;
        alarm_hh_bcd -= 12;
      }
      if (alarm_hh_bcd == 0) {
        alarm_hh_bcd = 12;
      }
    }
#endif    

    alarm_mm_bcd = cfg_table[CFG_ALARM_MINUTES_BYTE] & CFG_ALARM_MINUTES_MASK;
    // convert to BCD
    alarm_hh_bcd = ds_int2bcd(alarm_hh_bcd);
    alarm_mm_bcd = ds_int2bcd(alarm_mm_bcd);

    snooze_time = 0;
#endif
    cfg_changed = 0;
  }
}
#endif // !defined(WITHOUT_ALARM) || !defined(WITHOUT_CHIME)

#if !defined(WITHOUT_CHIME)
inline void prepareChimeTrigger() {
  // xx:00:00
  if (CONF_CHIME_ON && chime_state == CHIME_IDLE && !rtc_mm_bcd &&
      !rtc_table[DS_ADDR_SECONDS]) {
    uint8_t hh = rtc_hh_bcd;
    uint8_t ss = chime_ss_bcd;
    uint8_t uu = chime_uu_bcd;
    // convert all to 24h for comparision
#if !defined(WITHOUT_H12_24_SWITCH)
    if (H12_12) {
      if (hh == 0x12 && !rtc_pm)
        hh == 0x00;
      else if (hh != 0x12 && rtc_pm)
        hh += 0x12;
      if (ss == 0x12 && !chime_ss_pm)
        ss == 0x00;
      else if (ss != 0x12 && chime_ss_pm)
        ss += 0x12;
      if (uu == 0x12 && !chime_uu_pm)
        uu == 0x00;
      else if (uu != 0x12 && chime_uu_pm)
        uu += 0x12;
    }
#endif
    if ((ss <= uu && hh >= ss && hh <= uu) ||
        (ss > uu && (hh >= ss || hh <= uu))) {
      chime_state = CHIME_RUNNING;
    }
  }
}
#endif

#if !defined(WITHOUT_CHIME)
inline void turnOnBuzzerForChime() {
  switch (chime_state) {
  case CHIME_RUNNING: // ~100ms chime
    BUZZER_ON;
    for (chime_ticks = 0; chime_ticks < 10;)
      ;
    BUZZER_OFF;
    chime_state = CHIME_WAITING;
  case CHIME_WAITING: // wait > 1sec until rtc sec changed
    if (chime_ticks > 150)
      chime_state = CHIME_IDLE; // stop chime
    break;
  default:
    break;
  }
}
#endif

#if !defined(WITHOUT_ALARM)
enum Event prepareAlarmTrigger(enum Event ev) {
  // check for alarm trigger
  // when snooze_time>0, just compare min portion
  if ((snooze_time == 0 &&
       (alarm_hh_bcd == rtc_hh_bcd && alarm_mm_bcd == rtc_mm_bcd
#if !defined(WITHOUT_H12_24_SWITCH)
        && alarm_pm == rtc_pm
#endif
        )) ||
      (snooze_time > 0 && (alarm_mm_snooze == rtc_mm_bcd))) {
    if (CONF_ALARM_ON && !alarm_trigger && !alarm_reset) {
      alarm_trigger = 1;
    }
  } else {
    alarm_trigger = 0;
    alarm_reset = 0;
  }

  // S1 or S2 to stop alarm
  // Snooze by pushing S1
  if (alarm_trigger && !alarm_reset) {
    if (ev == EV_S1_SHORT) {
      // set snooze
      alarm_trigger = 0;
      snooze_time += 5; // next alarm as 5min later
      // stop snooze after 1hour passing from the first alarm
      if (snooze_time > 60)
        snooze_time = 0;

      // need BCD calculation
      alarm_mm_snooze = addBCD(ds_int2bcd(snooze_time));

      if (alarm_mm_snooze > 0x59) {
        alarm_mm_snooze = alarm_mm_snooze - 0x60;
      }

      return EV_NONE;
    } else if (ev == EV_S2_SHORT) {
      alarm_reset = 1;
      alarm_trigger = 0;
      snooze_time = 0;
      return EV_NONE;
    }
    return ev;
  }
}
#endif

#if !defined(WITHOUT_ALARM)
inline void turnOnBuzzerForAlarm() {
  if (alarm_trigger && !alarm_reset) {
    if (blinker_slow && blinker_fast) {
      clearFrameBuffer();
      BUZZER_ON;
    } else {
      BUZZER_OFF;
    }
  } else {
    BUZZER_OFF;
  }
}
#endif

//
// Button handling functions
//

inline void handleButtonsSetHour(enum Event ev) {
  flash_01 = 1;

  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
    ds_hours_incr();
  else if (ev == EV_S2_SHORT)
    buttons_mode = K_SET_MINUTE;
}

inline void handleButtonsSetMinute(enum Event ev) {
  flash_01 = 0;
  flash_23 = 1;
  
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_minutes_incr();
  } else if (ev == EV_S2_SHORT) {
#ifdef SIX_DIGITS
    buttons_mode = K_SET_SECOND;
#else

#ifdef WITHOUT_H12_24_SWITCH
#ifdef WITH_NMEA
    backupNmeaValues();
    buttons_mode = K_NMEA_SET_TZ_HOUR;
#else
    buttons_mode = K_LIGHT_SENSOR_CORRECTION;
#endif
#else
    buttons_mode = K_SET_HOUR_12_24;
#endif

#endif
  }
}

#ifdef SIX_DIGITS
inline void handleButtonsSetSecond6d(enum Event ev) {
  flash_01 = 0;
  flash_23 = 0;
  flash_45 = 1;

  if (ev == EV_S1_SHORT) {
    ds_sec_zero();
  } else if (ev == EV_S2_SHORT) {
#ifdef WITHOUT_H12_24_SWITCH
#ifdef WITH_NMEA
    backupNmeaValues();
    buttons_mode = K_NMEA_SET_TZ_HOUR;
#else
    buttons_mode = K_LIGHT_SENSOR_CORRECTION;
#endif
#else
    buttons_mode = K_SET_HOUR_12_24;
#endif
  }
}
#endif

#if !defined(WITHOUT_H12_24_SWITCH)
inline void handleButtonsSet12h24(enum Event ev) {
  display_mode = DM_SET_HOUR_12_24;
  if (ev == EV_S1_SHORT) {
    ds_hours_12_24_toggle();
    cfg_changed = 1;
  } else if (ev == EV_S2_SHORT) {
#ifdef WITH_NMEA
    backupNmeaValues();
    buttons_mode = K_NMEA_SET_TZ_HOUR;
#else
    buttons_mode = K_LIGHT_SENSOR_CORRECTION;
#endif
  }
}
#endif

#ifdef WITH_NMEA
inline void handleButtonsNmeaSetTimeZoneHour(enum Event ev) {
  display_mode = DM_NMEA_TIMEZONE;
  flash_01 = 1;
  flash_23 = 0;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
    nmea_tz_hr = nmea_tz_hr < 12 ? nmea_tz_hr + 1 : -12;
  else if (ev == EV_S2_SHORT)
    buttons_mode = K_NMEA_SET_TZ_MINUTE;
}

inline void handleButtonsNmeaSetTimeZoneMinute(enum Event ev) {
  flash_01 = 0;
  flash_23 = 1;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
    switch (nmea_tz_min) {
    case 0:
      nmea_tz_min = 30;
      break;
    case 30:
      nmea_tz_min = 45;
      break;
    default:
      nmea_tz_min = 0;
      break;
    }
  else if (ev == EV_S2_SHORT)
    buttons_mode = K_NMEA_SET_DST;
}

inline void handleButtonsNmeaSetDST(enum Event ev) {
  display_mode = DM_NMEA_DST;
  flash_01 = flash_23 = 1;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
    nmea_tz_dst ^= 0x01;
  else if (ev == EV_S2_SHORT) {
    flash_01 = flash_23 = 0;
    buttons_mode = K_NMEA_SET_AUTOUPDATE;
  }
}

inline void handleButtonsNmeaSetAutoupdate(enum Event ev) {
  display_mode = DM_NMEA_AUTOUPDATE;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    switch (nmea_autosync) {
    case NMEA_AUTOSYNC_OFF:
      nmea_autosync = NMEA_AUTOSYNC_3H;
      break;
    case NMEA_AUTOSYNC_3H:
      nmea_autosync = NMEA_AUTOSYNC_6H;
      break;
    case NMEA_AUTOSYNC_6H:
      nmea_autosync = NMEA_AUTOSYNC_12H;
      break;
    case NMEA_AUTOSYNC_12H:
      nmea_autosync = NMEA_AUTOSYNC_24H;
      break;
    default:
      nmea_autosync = NMEA_AUTOSYNC_OFF;
      break;
    }
  } else if (ev == EV_S2_SHORT) {
    nmea_seconds_to_sync = NMEA_AUTOSYNC_DELAY;
    buttons_mode = K_LIGHT_SENSOR_CORRECTION;
  }
}
#endif

inline void handleButtonsLightSensorCorrection(enum Event ev) {  
  flash_23 = 1;
  display_mode = DM_LIGHT_SENSOR_CORRECTION;
  
  if ((ev == EV_S1_SHORT) || (S1_LONG && blinker_fast)) {    
    if (lightSensorCorrection < LIGHT_SENSOR_MAX_CORRECTION) {
      lightSensorCorrection++;
    } else {
      lightSensorCorrection = 0;
    }    
  } else if (S2_LONG && blinker_fast) {
    if (lightSensorCorrection > 0) {
      lightSensorCorrection--;
    } else {
      lightSensorCorrection = LIGHT_SENSOR_MAX_CORRECTION;
    }
  } else if (ev == EV_S2_SHORT) {
    buttons_mode = K_BRIGHTNESS_HIGH;
  }
}

inline void handleButtonsBrightnessHigh(enum Event ev) {  
  flash_23 = 1;
  display_mode = DM_BRIGHTNESS_HIGH;
  
  if ((ev == EV_S1_SHORT) || (S1_LONG && blinker_fast)) {
    if (brightnessHigh + 1 > brightnessLow) {
      brightnessHigh = BRIGHTNESS_HIGH_DEFAULT_VALUE;
    } else {
      brightnessHigh++;
    }
  } else if (S2_LONG && blinker_fast) {
    if (brightnessHigh > BRIGHTNESS_HIGH_DEFAULT_VALUE) {
      brightnessHigh--;
    } else {
      brightnessHigh = brightnessLow - 1;
    }    
  } else if (ev == EV_S2_SHORT) {
    buttons_mode = K_BRIGHTNESS_LOW;
  }
}

inline void handleButtonsBrightnessLow(enum Event ev) {  
  flash_23 = 1;
  display_mode = DM_BRIGHTNESS_LOW;
  
  if ((ev == EV_S1_SHORT) || (S1_LONG && blinker_fast)) {
    if (brightnessLow + 1 > BRIGHTNESS_LOW_MAX_VALUE) {
      brightnessLow = brightnessHigh;
    } else {
      brightnessLow++;
    }    
  } else if (S2_LONG && blinker_fast) {
    if (brightnessLow + 1 > brightnessHigh) {
      brightnessLow--;
    } else {
      brightnessLow = brightnessHigh + 1;
    }    
  } else if (ev == EV_S2_SHORT) {
    buttons_mode = K_BRIGHTNESS_NIGHT;
  }
}

inline void handleButtonsBrightnessNight(enum Event ev) {  
  flash_23 = 1;
  display_mode = DM_BRIGHTNESS_NIGHT;
  
  if ((ev == EV_S1_SHORT) || (S1_LONG && blinker_fast)) {
    if (brightnessNight + 1 > BRIGHTNESS_NIGHT_MAX_VALUE) {
      brightnessNight = brightnessHigh;
    } else {
      brightnessNight++;
    }    
  } else if (ev == EV_S2_SHORT) {
    saveSettings();
    if (nmea_prev_tz_hr != nmea_tz_hr || nmea_prev_tz_min != nmea_tz_min ||
        nmea_prev_tz_dst != nmea_tz_dst ||
        nmea_prev_autosync != nmea_autosync) {
      enable_nmea_receiving();
    }
    buttons_mode = K_NORMAL;
  }
}

inline void handleButtonsTemperature(enum Event ev) {
  display_mode = DM_TEMPERATURE;
  if (ev == EV_S1_SHORT) {
    ds_temperature_offset_incr();
  }
#ifdef WITH_DEBUG_SCREENS
  else if (ev == EV_S1S2_LONG) {
    buttons_mode = K_DEBUG_SCREEN_1;
  }
#endif
  else if (ev == EV_S1_LONG) {
    ds_temperature_cf_toggle();
  } else if (ev == EV_S2_SHORT) {
#ifdef WITHOUT_DATE
    buttons_mode = K_NORMAL;
#else
    buttons_mode = K_DATE_DISP;
#endif
  }
}

#if !defined(WITHOUT_DATE)
inline void handleButtonsDate(enum Event ev) {
  display_mode = DM_DATE;
  if (ev == EV_S1_SHORT)
    ds_date_mmdd_toggle();
  else if (ev == EV_S2_LONG)
    buttons_mode = CONF_SW_MMDD ? K_SET_DAY : K_SET_MONTH;
  else if (ev == EV_S2_SHORT) {
#ifdef WITHOUT_WEEKDAY
#ifdef SIX_DIGITS
    buttons_mode = K_NORMAL;
#else
    buttons_mode = K_YEAR_DISP;
#endif
#else
    buttons_mode = K_WEEKDAY_DISP;
#endif
  }
}

inline void handleButtonsSetMonth(enum Event ev) {
  flash_01 = 1;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_month_incr();
  } else if (ev == EV_S2_SHORT) {
    flash_01 = 0;
#ifdef SIX_DIGITS
    buttons_mode = CONF_SW_MMDD ? K_SET_YEAR : K_SET_DAY;
#else
    buttons_mode = CONF_SW_MMDD ? K_DATE_DISP : K_SET_DAY;
#endif
  }
}

inline void handleButtonsSetDay(enum Event ev) {
  flash_23 = 1;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_day_incr();
  } else if (ev == EV_S2_SHORT) {
    flash_23 = 0;
#ifdef SIX_DIGITS
    buttons_mode = CONF_SW_MMDD ? K_SET_MONTH : K_SET_YEAR;
#else
    buttons_mode = CONF_SW_MMDD ? K_SET_MONTH : K_DATE_DISP;
#endif
  }
}

#ifdef SIX_DIGITS
inline void handleButtonsSetYear(enum Event ev) {
  flash_45 = 1;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_year_incr();
  } else if (ev == EV_S2_SHORT) {
    flash_45 = 0;
    buttons_mode = K_NORMAL;
  }
}
#endif

#if !defined(WITHOUT_WEEKDAY)
inline void handleButtonsWeekday(enum Event ev) {
  display_mode = DM_WEEKDAY;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_weekday_incr();
  } else if (ev == EV_S2_SHORT) {
#ifdef SIX_DIGITS
    buttons_mode = K_NORMAL;
#else
    buttons_mode = K_YEAR_DISP;
#endif
  }
}
#endif

#ifndef SIX_DIGITS
inline void handleButtonsYear(enum Event ev) {
  display_mode = DM_YEAR;
  if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_year_incr();
  } else if (ev == EV_S2_SHORT) {
    buttons_mode = K_NORMAL;
  }
}
#endif
#endif

#if !defined(SIX_DIGITS)
inline void handleButtonsSeconds(enum Event ev) {
  display_mode = DM_SECONDS;
  if (ev == EV_S1_SHORT) {
    buttons_mode = K_NORMAL;
  } else if (ev == EV_S2_LONG) {
    ds_sec_zero();
  }
}
#endif

#if !defined(WITHOUT_ALARM)
inline void handleButtonsAlarm(enum Event ev) {
  flash_01 = 0;
  flash_23 = 0;
#ifdef SIX_DIGITS
  flash_45 = 0;
#endif
  display_mode = DM_ALARM;
  if (ev == EV_S1_SHORT) {
#if !defined(WITHOUT_CHIME)
    buttons_mode = K_CHIME;
#else
    buttons_mode = K_NORMAL;
#endif
  } else if (ev == EV_S2_SHORT) {
    ds_alarm_on_toggle();
    cfg_changed = 1;
  } else if (ev == EV_S2_LONG) {
    buttons_mode = K_ALARM_SET_HOUR;
  }
}

inline void handleButtonsAlarmSetHour(enum Event ev) {
  flash_01 = 1;
  alarm_reset = 1; // don't trigger while setting
  if (ev == EV_S2_SHORT) {
    buttons_mode = K_ALARM_SET_MINUTE;
  } else if (ev == EV_S1_SHORT || S1_LONG && blinker_fast) {
    ds_alarm_hours_incr();
    cfg_changed = 1;
  }
}

inline void handleButtonsAlarmSetMinute(enum Event ev) {
  flash_01 = 0;
  flash_23 = 1;
  alarm_reset = 1;
  if (ev == EV_S2_SHORT) {
    buttons_mode = K_ALARM;
  } else if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_alarm_minutes_incr();
    cfg_changed = 1;
  }
}
#endif

#if !defined(WITHOUT_CHIME)
inline void handleButtonsChime(enum Event ev) {
  flash_01 = 0;
  flash_23 = 0;
#ifdef SIX_DIGITS
  flash_45 = 0;
#endif
  display_mode = DM_CHIME;
  if (ev == EV_S1_SHORT) {
    buttons_mode = K_NORMAL;
  } else if (ev == EV_S2_SHORT) {
    ds_chime_on_toggle();
    cfg_changed = 1;
  } else if (ev == EV_S2_LONG) {
    buttons_mode = K_CHIME_SET_SINCE;
  }
}

inline void handleButtonsChimeSetSince(enum Event ev) {
  flash_01 = 1;
  if (ev == EV_S2_SHORT) {
    buttons_mode = K_CHIME_SET_UNTIL;
  } else if (ev == EV_S1_SHORT || S1_LONG && blinker_fast) {
    ds_chime_since_incr();
    cfg_changed = 1;
  }
}

inline void handleButtonsChimeSetUntil(enum Event ev) {
  flash_01 = 0;
  flash_23 = 1;
  if (ev == EV_S2_SHORT) {
    buttons_mode = K_CHIME;
  } else if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
    ds_chime_until_incr();
    cfg_changed = 1;
  }
}
#endif

inline void handleButtonsNormal(enum Event ev) {
  flash_01 = 0;
  flash_23 = 0;
#ifdef SIX_DIGITS
  flash_45 = 0;
#endif

  display_mode = DM_NORMAL;

  if (ev == EV_S1_SHORT) {
#ifdef SIX_DIGITS
#ifndef WITHOUT_ALARM
    buttons_mode = K_ALARM;
#endif
#else
    buttons_mode = K_SEC_DISP;
#endif
  }
#ifdef WITH_NMEA
  else if (ev == EV_S1S2_LONG) {
    if (is_nmea_receiving_on) {
      disable_nmea_receiving();
    } else {
      enable_nmea_receiving();
    }
  }
#endif
  else if (ev == EV_S1_LONG) {
#if !defined(SIX_DIGITS) && !defined(WITHOUT_ALARM)
    buttons_mode = K_ALARM;
#endif
  } else if (ev == EV_S2_LONG) {
    buttons_mode = K_SET_HOUR;
  } else if (ev == EV_S2_SHORT) {
    buttons_mode = K_TEMP_DISP;
  }
#ifdef HW_REVISION_WITH_VOICE_CHIP
  else if (ev == EV_S3_LONG) {
    LED = !LED;
  }
#endif
}

#ifdef WITH_DEBUG_SCREENS
/*
To enter DEBUG mode, go to the Temperature screen,
then hold S1 and S2 simultaneously.
S1 cycles through the DEBUG modes.
To exit DEBUG mode, hold S1 and S2 again.
*/

#if !defined(NUMBER_OF_DEBUG_SCREENS)
#define NUMBER_OF_DEBUG_SCREENS 3
#endif

inline void handleDebugScreens(enum Event ev) {
  display_mode = DM_DEBUG_SCREEN_1 + buttons_mode - K_DEBUG_SCREEN_1;
  if (ev == EV_S1_SHORT) {
    buttons_mode = (buttons_mode - K_DEBUG_SCREEN_1 + 1) % NUMBER_OF_DEBUG_SCREENS + K_DEBUG_SCREEN_1;
  } else if (ev == EV_S1S2_LONG) {
    buttons_mode = K_TEMP_DISP;
  }
}
#endif

void handleButtonEvents(enum Event ev) {
  switch (buttons_mode) {
  case K_SET_HOUR:
    handleButtonsSetHour(ev);
    break;

  case K_SET_MINUTE:
    handleButtonsSetMinute(ev);
    break;

#ifdef SIX_DIGITS
  case K_SET_SECOND:
    handleButtonsSetSecond6d(ev);
    break;
#endif

#if !defined(WITHOUT_H12_24_SWITCH)
  case K_SET_HOUR_12_24:
    handleButtonsSet12h24(ev);
    break;
#endif

#ifdef WITH_NMEA
  case K_NMEA_SET_TZ_HOUR:
    handleButtonsNmeaSetTimeZoneHour(ev);
    break;

  case K_NMEA_SET_TZ_MINUTE:
    handleButtonsNmeaSetTimeZoneMinute(ev);
    break;

  case K_NMEA_SET_DST:
    handleButtonsNmeaSetDST(ev);
    break;

  case K_NMEA_SET_AUTOUPDATE:
    handleButtonsNmeaSetAutoupdate(ev);
    break;
#endif

  case K_LIGHT_SENSOR_CORRECTION:
    handleButtonsLightSensorCorrection(ev);
    break;

  case K_BRIGHTNESS_HIGH:
    handleButtonsBrightnessHigh(ev);
    break;

  case K_BRIGHTNESS_LOW:
    handleButtonsBrightnessLow(ev);
    break;

  case K_BRIGHTNESS_NIGHT:
    handleButtonsBrightnessNight(ev);
    break;

  case K_TEMP_DISP:
    handleButtonsTemperature(ev);
    break;

#ifndef WITHOUT_DATE
  case K_DATE_DISP:
    handleButtonsDate(ev);
    break;

  case K_SET_MONTH:
    handleButtonsSetMonth(ev);
    break;

  case K_SET_DAY:
    handleButtonsSetDay(ev);
    break;

#ifdef SIX_DIGITS
  case K_SET_YEAR:
    handleButtonsSetYear(ev);
    break;
#endif

#ifndef WITHOUT_WEEKDAY
  case K_WEEKDAY_DISP:
    handleButtonsWeekday(ev);
    break;
#endif

#ifndef SIX_DIGITS
  case K_YEAR_DISP:
    handleButtonsYear(ev);
    break;
#endif
#endif

#ifdef WITH_DEBUG_SCREENS
  case K_DEBUG_SCREEN_1:
  case K_DEBUG_SCREEN_2:
  case K_DEBUG_SCREEN_3:
    handleDebugScreens(ev);
    break;
#endif

#if !defined(SIX_DIGITS)
  case K_SEC_DISP:
    handleButtonsSeconds(ev);
    break;
#endif

#if !defined(WITHOUT_ALARM)
  case K_ALARM:
    handleButtonsAlarm(ev);
    break;

  case K_ALARM_SET_HOUR:
    handleButtonsAlarmSetHour(ev);
    break;

  case K_ALARM_SET_MINUTE:
    handleButtonsAlarmSetMinute(ev);
    break;
#endif

#if !defined(WITHOUT_CHIME)
  case K_CHIME:
    handleButtonsChime(ev);
    break;

  case K_CHIME_SET_SINCE:
    handleButtonsChimeSetSince(ev);
    break;

  case K_CHIME_SET_UNTIL:
    handleButtonsChimeSetUntil(ev);
    break;
#endif

  case K_NORMAL:
  default:
    handleButtonsNormal(ev);
    break;
  };
}

//
// NMEA functions
//

#ifdef WITH_NMEA
void enable_nmea_receiving() {
#ifdef WITH_NMEA_DEVICE_SWITCH
  NMEA_DEVICE_ON;
#endif
  REN = 1; // enable uart receiving
  is_nmea_receiving_on = 1;
  nmea_progress_seconds = 0;
}

void disable_nmea_receiving() {
#ifdef WITH_NMEA_DEVICE_SWITCH
  NMEA_DEVICE_OFF;
#endif
  REN = 0; // disable uart receiving
  is_nmea_receiving_on = 0;
  nmea_progress_seconds = 0;
}

void processNmeaUpdate() {
  if (is_nmea_receiving_on) {
    if (nmea_state == NMEA_READY ||
        ++nmea_progress_seconds >= NMEA_MAX_SYNC_DURATION) {
      disable_nmea_receiving();

      if (nmea_state == NMEA_READY) {
        nmea2localtime();
      }

      nmea_state = NMEA_NONE;
      nmea_seconds_to_sync = NMEA_AUTOSYNC_DELAY;
    }
  }
}
#endif

//
// Brightness settings
//

inline void loadBrightnessSettings() {
  brightnessHigh = (int8_t)IapReadByte(IAP_BRIGHTNESS_HIGH);
  brightnessLow = IapReadByte(IAP_BRIGHTNESS_LOW);
  brightnessNight = IapReadByte(IAP_BRIGHTNESS_NIGHT);
  lightSensorCorrection = IapReadByte(IAP_LIGHT_SENSOR_CORR);
  
  if (brightnessHigh == 0xff) {
    brightnessHigh = BRIGHTNESS_HIGH_DEFAULT_VALUE;
  }

  if (brightnessLow == 0xff) {
    brightnessLow = BRIGHTNESS_LOW_DEFAULT_VALUE;
  }

  if (brightnessNight == 0xff) {
    brightnessNight = BRIGHTNESS_NIGHT_DEFAULT_VALUE;
  }

  if (lightSensorCorrection == 0xff) {
    lightSensorCorrection = LIGHT_SENSOR_DEFAULT_CORRECTION;
  }
}

inline void saveBrightnessSettings() {
  IapProgramByte(IAP_BRIGHTNESS_HIGH, brightnessHigh);
  IapProgramByte(IAP_BRIGHTNESS_LOW, brightnessLow);
  IapProgramByte(IAP_BRIGHTNESS_NIGHT, brightnessNight);
  IapProgramByte(IAP_LIGHT_SENSOR_CORR, lightSensorCorrection);
}

inline void saveSettings() {
  Delay(10);
  IapEraseSector(IAP_ADDRESS);
  Delay(10);

#ifdef WITH_NMEA
    nmea_save_tz();
#endif

  saveBrightnessSettings();
}

//
// main
//

int main() {
  // set photoresistor & ntc pins to open-drain output
  P1M1 |= (1 << ADC_LIGHT) | (1 << ADC_TEMP);
  P1M0 |= (1 << ADC_LIGHT) | (1 << ADC_TEMP);

  // init rtc
  ds_init();
  // init/read ram config
  ds_ram_config_init();
#ifdef WITH_CAPACITOR
  // enable capacitor trickle charge ~1mA
  ds_writebyte(DS_ADDR_TCSDS, DS_TCS_TCON | DS_TC_D1_4KO);
#endif

  // uncomment in order to reset minutes and hours to zero.
  // ds_reset_clock();

  timer0Init(); // display refresh & switch read

#ifdef WITH_NMEA
  uart1_init();   // setup uart
  nmea_load_tz(); // read TZ/DST from eeprom
  nmea_seconds_to_sync = NMEA_AUTOSYNC_DELAY;
#endif

  loadBrightnessSettings();

  // LOOP
  while (1) {
    while (!loop_gate)
      ;            // wait for open every 100ms
    loop_gate = 0; // close gate

    enum Event ev = event;
    event = EV_NONE;

    // sample adc, run frequently
    processADCValues();

    // Read RTC
    ds_readburst();

    // parse RTC
    rtc_hh_bcd = rtc_table[DS_ADDR_HOUR];

#ifdef WITHOUT_H12_24_SWITCH
    rtc_hh_bcd &= DS_MASK_HOUR24;
#else
    if (H12_12) {
      rtc_hh_bcd &= DS_MASK_HOUR12;
    } else {
      rtc_hh_bcd &= DS_MASK_HOUR24;
    }
    rtc_pm = H12_12 && H12_PM;
#endif

    rtc_mm_bcd = rtc_table[DS_ADDR_MINUTES] & DS_MASK_MINUTES;

#if !defined(WITHOUT_ALARM) || !defined(WITHOUT_CHIME)
    prepareAlarmAndChimeBCD();
#if !defined(WITHOUT_CHIME)
    prepareChimeTrigger();
#endif
#if !defined(WITHOUT_ALARM)
    ev = prepareAlarmTrigger(ev);
#endif
#endif

    handleButtonEvents(ev);

    clearFrameBuffer();
#if !defined(WITHOUT_INACTIVITY_TIMER)
    handleInactivityTimer(ev);
#endif
    adjustDisplayMode();
    displayScreen();

#if !defined(WITHOUT_ALARM)
    turnOnBuzzerForAlarm();
#endif

#if !defined(WITHOUT_CHIME)
    turnOnBuzzerForChime();
#endif

    __critical { updateDisplayBuffer(); }

    count++;
    WDT_CLEAR();

#ifdef WITH_NMEA
    processNmeaUpdate();
#endif
  }
}

/*
void _delay_ms(uint8_t ms)
{
    // delay function, tuned for 11.092 MHz clock
    // optimized to assembler
    ms; // keep compiler from complaining?
    __asm;
        ; dpl contains ms param value
    delay$:
        mov	b, #8   ; i
    outer$:
        mov	a, #243    ; j
    inner$:
        djnz acc, inner$
        djnz b, outer$
        djnz dpl, delay$
    __endasm;
}
*/