//
// STC15F204EA DIY LED Clock
// Copyright 2016, Jens Jensen
//

#include "stc15.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "ds1302.h"
#include "led.h"

#include "hwconfig.h"
#include "models.h"
#include "keyboardmode.h"
#include "displaymode.h"
#include "event.h"
#include "buttonmonitor.h"

// clear wdt
#define WDT_CLEAR()    (WDT_CONTR |= 1 << 4)

#define NUM_DEBUG 3

#ifdef DEBUG
uint8_t hex[] = {0,1,2,3,4,5,6,7,8,9,14,15,16,17,18,19};
#endif

#define LIGHTVAL_LOWEST_VALUE   4           // Max. brightness
#define LIGHTVAL_HIGHEST_VALUE  36          // Min. brightness

#define LIGHT_SENSOR_NIGHT_TH   245         // Night mode threshold
#define LIGHTVAL_NIGHT_VALUE    52          // Night brightness
#define LIGHT_SENSOR_MAX_VALUE  (255 - (255 - LIGHT_SENSOR_NIGHT_TH))

/* ------------------------------------------------------------------------- */
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

uint8_t  count;     // main loop counter
uint8_t  temp;      // temperature sensor value
uint8_t  lightval;  // light sensor value

volatile uint8_t displaycounter;
volatile int8_t count_100;	//0.01s=10ms
volatile int8_t count_1000;	//0.1s=100ms
volatile int8_t count_5000;	//0.5s=500ms
#ifndef WITHOUT_ALARM
volatile int16_t count_20000;	//2s
volatile __bit blinker_slowest;
#endif
#ifndef WITHOUT_CHIME
volatile int16_t chime_ticks;   //10ms inc
#endif

volatile uint16_t count_timeout; // max 6.5536 sec
#define TIMEOUT_LONG 0xFFFF
volatile __bit blinker_slow;
volatile __bit blinker_fast;
volatile __bit loop_gate;

uint8_t dmode = M_NORMAL;     // display mode state
uint8_t kmode = K_NORMAL;
uint8_t dmode_bak = M_NORMAL;

__bit  flash_01;
__bit  flash_23;
#ifdef SIX_DIGITS
__bit  flash_45;
#endif

uint8_t rtc_hh_bcd;
uint8_t rtc_mm_bcd;
__bit rtc_pm;

#ifndef WITHOUT_ALARM
uint8_t alarm_hh_bcd;
uint8_t alarm_mm_bcd;
__bit alarm_pm;
__bit alarm_trigger;
__bit alarm_reset;
#endif

#ifndef WITHOUT_CHIME
uint8_t chime_ss_bcd; // hour since
uint8_t chime_uu_bcd; // hour until
__bit chime_ss_pm;
__bit chime_uu_pm;
enum chime_state {
    CHIME_IDLE = 0,
    CHIME_RUNNING,
    CHIME_WAITING
};
volatile uint8_t chime_trigger = CHIME_IDLE;
#endif

__bit cfg_changed = 1;
uint8_t snooze_time;	//snooze(min)
uint8_t alarm_mm_snooze;	//next alarm time (min)

#if defined(WITH_MONTHLY_CORR) && WITH_MONTHLY_CORR != 0
#define SEC_PER_MONTH 2592000
#if WITH_MONTHLY_CORR > 0
#define CORR_VALUE 1 // +1 sec every ~RUNTIME_PER_SEC seconds
#else
#define CORR_VALUE -1 // -1 sec every ~RUNTIME_PER_SEC seconds
#endif
#define RUNTIME_PER_SEC (SEC_PER_MONTH / (WITH_MONTHLY_CORR * CORR_VALUE))
volatile uint32_t corr_remaining = RUNTIME_PER_SEC;
#endif

volatile __bit S1_LONG;
volatile __bit S1_PRESSED;
volatile __bit S2_LONG;
volatile __bit S2_PRESSED;
#ifdef stc15w408as
volatile __bit S3_LONG;
volatile __bit S3_PRESSED;
#endif

volatile uint8_t debounce[NUM_SW];      // switch debounce buffer
volatile uint8_t switchcount[NUM_SW];

#ifdef WITH_NMEA
#define FOSC    11059200

#include "nmea.h"

__bit is_nmea_receiving_on = 0;

void enable_nmea_receiving();
void disable_nmea_receiving();
#endif

volatile enum Event event;

/*
  interrupt: every 0.1ms=100us come here

  Check button status
  Dynamically LED turn on
 */
void timer0_isr() __interrupt(1) __using(1)
{
    uint8_t tmp;
    enum Event ev = EV_NONE;
    // display refresh ISR
    // cycle thru digits one at a time
    uint8_t digit = displaycounter % (uint8_t) NUMBER_OF_DIGITS;

    // turn off all digits, set high
    LED_DIGITS_OFF();

    // auto dimming, skip lighting for some cycles
    if (displaycounter % lightval < NUMBER_OF_DIGITS) {
        // fill digits
        LED_SEGMENT_PORT = dbuf[digit];
        // turn on selected digit, set low
        //LED_DIGIT_ON(digit);
        // issue #32, fix for newer sdcc versions which are using non-atomic port access
        tmp = ~((1<<LED_DIGITS_PORT_BASE) << digit);
        LED_DIGITS_PORT &= tmp;
    }
    displaycounter++;

    // 100/sec: 10 ms
    if (count_100 == 100) {
        count_100 = 0;

	count_1000++;	//increment every 10ms

#ifndef WITHOUT_CHIME
        if (chime_trigger != CHIME_IDLE)
            chime_ticks ++;     //increment every 10ms
#endif

        // 10/sec: 100 ms
        if (count_1000 == 10) {
            count_1000 = 0;
            blinker_fast = !blinker_fast;	//blink every 100ms
            loop_gate = 1;	//every 100ms

            count_5000++; // increment every 100ms
#ifndef WITHOUT_ALARM
            count_20000++; // increment every 100ms
#endif
            // 2/sec: 500 ms
            if (count_5000 == 5) {
                count_5000 = 0;
                blinker_slow = !blinker_slow; // blink every 500ms
#if defined(WITH_MONTHLY_CORR) && WITH_MONTHLY_CORR != 0
                if (!blinker_slow && corr_remaining)
                    corr_remaining--;
#endif
#if defined(WITH_NMEA)
                if (IS_NMEA_AUTOSYNC_ON && !blinker_slow && 
                    nmea_seconds_to_sync && !--nmea_seconds_to_sync) {
                    enable_nmea_receiving();
                }
#endif
#ifndef WITHOUT_ALARM
                // 1/ 2sec: 20000 ms
                if (count_20000 == 20) {
                    count_20000 = 0;
                }
                // 500 ms on=true=1, 1500 ms off=false=0
                blinker_slowest = count_20000 < 5;
#endif
            }
        }

        // Check SW status and chattering control
        MONITOR_S(1);
        MONITOR_S(2);
#ifdef stc15w408as
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
    count_100++;	//increment every 0.1ms
    
#ifndef WITHOUT_ALARM
    if (count_timeout != 0) {
        count_timeout--;
        if (count_timeout == 0) {
            if (event == EV_NONE) {
                event = EV_TIMEOUT;
            }
        }
    }
#endif
}

// Call timer0_isr() 10000/sec: 0.0001 sec
// Initialize the timer count so that it overflows after 0.0001 sec
// THTL = 0x10000 - FOSC / 12 / 10000 = 0x10000 - 92.16 = 65444 = 0xFFA4
// When 11.0592MHz clock case, set every 100us interruption 
void Timer0Init(void)		//100us @ 11.0592MHz
{
    // refer to section 7 of datasheet: STC15F2K60S2-en2.pdf
    // TMOD = 0;    // default: 16-bit auto-reload
    // AUXR = 0;    // default: traditional 8051 timer frequency of FOSC / 12
    // Initial values of TL0 and TH0 are stored in hidden reload registers: RL_TL0 and RL_TH0
    TL0 = 0xA4;		// Initial timer value
    TH0 = 0xFF;		// Initial timer value
    TF0 = 0;		// Clear overflow flag
    TR0 = 1;		// Timer0 start run
    ET0 = 1;        // Enable timer0 interrupt
    EA = 1;         // Enable global interrupt
}

// Formula was : 76-raw*64/637 - which makes use of integer mult/div routines
// Getting degF from degC using integer was not good as values were sometimes jumping by 2
// The floating point one is even worse in term of code size generated (>1024bytes...)
// Approximation for slope is 1/10 (64/637) - valid for a normal 20 degrees range
// & let's find some other trick (80 bytes - See also docs\Temp.ods file)
int8_t gettemp(uint16_t raw) {
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

uint8_t preparepm(uint8_t pm)
{    
#ifndef WITHOUT_ALARM
    // dot 3: If alarm is on, blink for 500 ms every 2000 ms
    //        If 12h: on if pm when not blinking
    if (!H12_12) { // 24h mode
        pm = CONF_ALARM_ON && blinker_slowest && blinker_fast;
    } else if (CONF_ALARM_ON && blinker_slowest) {
      //12h mode case: blink 500ms, AM/PM=Off/On in another 500ms 
        pm = blinker_fast;
    }
#endif
return pm;
}

#ifndef WITHOUT_ALARM
//set next alarm_min by adding snooze
uint8_t add_BCD(uint8_t snooze) {
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

/*********************************************/
int main()
{
    // SETUP
    // set photoresistor & ntc pins to open-drain output
    P1M1 |= (1<<ADC_LIGHT) | (1<<ADC_TEMP);
    P1M0 |= (1<<ADC_LIGHT) | (1<<ADC_TEMP);

    // init rtc
    ds_init();
    // init/read ram config
    ds_ram_config_init();
#ifdef WITH_CAPACITOR
    // enable capacitor trickle charge ~1mA
    ds_writebyte(DS_ADDR_TCSDS, DS_TCS_TCON | DS_TC_D1_4KO);
#endif

    // uncomment in order to reset minutes and hours to zero.. Should not need this.
    //ds_reset_clock();

    Timer0Init(); // display refresh & switch read

#ifdef WITH_NMEA
    uart1_init();   // setup uart
    nmea_load_tz(); // read TZ/DST from eeprom
#endif

    // LOOP
    while (1)
    {
        enum Event ev;

        while (!loop_gate); // wait for open every 100ms
        loop_gate = 0; // close gate

        ev = event;
        event = EV_NONE;

        // sample adc, run frequently
        if (count % (uint8_t) 4 == 0) {
            temp = gettemp(getADCResult(ADC_TEMP));
            
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
            
            uint8_t lightvalMax = LIGHTVAL_HIGHEST_VALUE;

            // Check for the night mode
            if (adcLight >= LIGHT_SENSOR_NIGHT_TH) {
                lightvalMax = LIGHTVAL_NIGHT_VALUE;
            }

            // Calculate the lightval
            lightval = adcLight * lightvalMax / LIGHT_SENSOR_MAX_VALUE;
            if (lightval < LIGHTVAL_LOWEST_VALUE) {
                lightval = LIGHTVAL_LOWEST_VALUE;
            }
        }

        // Read RTC
        ds_readburst();
        // parse RTC
        {
            rtc_hh_bcd = rtc_table[DS_ADDR_HOUR];
            if (H12_12) {
                rtc_hh_bcd &= DS_MASK_HOUR12;
            } else {
                rtc_hh_bcd &= DS_MASK_HOUR24;
            }
            rtc_pm = H12_12 && H12_PM;
            rtc_mm_bcd = rtc_table[DS_ADDR_MINUTES] & DS_MASK_MINUTES;
        }

#if !defined(WITHOUT_ALARM) || !defined(WITHOUT_CHIME)
        if (cfg_changed) {
#ifndef WITHOUT_CHIME
            chime_ss_bcd = cfg_table[CFG_CHIME_SINCE_BYTE] >> CFG_CHIME_SINCE_SHIFT;
            chime_uu_bcd = cfg_table[CFG_CHIME_UNTIL_BYTE] & CFG_CHIME_UNTIL_MASK;
            chime_ss_pm = chime_uu_pm = 0;
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
            // convert to BCD
            chime_ss_bcd = ds_int2bcd(chime_ss_bcd);
            chime_uu_bcd = ds_int2bcd(chime_uu_bcd);
#endif
#ifndef WITHOUT_ALARM
            alarm_pm = 0;
            alarm_hh_bcd = cfg_table[CFG_ALARM_HOURS_BYTE] >> CFG_ALARM_HOURS_SHIFT;
            if (H12_12) {
                if (alarm_hh_bcd >= 12) {
                    alarm_pm = 1;
                    alarm_hh_bcd -= 12;
                }
                if (alarm_hh_bcd == 0) {
                    alarm_hh_bcd = 12;
                }
            }
            alarm_mm_bcd = cfg_table[CFG_ALARM_MINUTES_BYTE] & CFG_ALARM_MINUTES_MASK;
            // convert to BCD
            alarm_hh_bcd = ds_int2bcd(alarm_hh_bcd);
            alarm_mm_bcd = ds_int2bcd(alarm_mm_bcd);

            snooze_time = 0;
#endif
            cfg_changed = 0;
        }
#endif // !defined(WITHOUT_ALARM) || !defined(WITHOUT_CHIME)

#ifndef WITHOUT_CHIME
        // xx:00:00
        if (CONF_CHIME_ON && chime_trigger == CHIME_IDLE && !rtc_mm_bcd && !rtc_table[DS_ADDR_SECONDS]) {
            uint8_t hh = rtc_hh_bcd;
            uint8_t ss = chime_ss_bcd;
            uint8_t uu = chime_uu_bcd;
            // convert all to 24h for comparision
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
            if((ss <= uu && hh >= ss && hh <= uu) || (ss > uu && (hh >= ss || hh <= uu)))
                chime_trigger = CHIME_RUNNING;
            }
#endif

#ifndef WITHOUT_ALARM
        // check for alarm trigger
	    // when snooze_time>0, just compare min portion
        if ((snooze_time == 0 && (alarm_hh_bcd == rtc_hh_bcd && alarm_mm_bcd == rtc_mm_bcd && alarm_pm == rtc_pm))
	     || (snooze_time>0 && (alarm_mm_snooze == rtc_mm_bcd)) ) {
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
                alarm_mm_snooze = add_BCD(ds_int2bcd(snooze_time));

                if (alarm_mm_snooze > 0x59)
                    alarm_mm_snooze = alarm_mm_snooze - 0x60;

                ev = EV_NONE;
            }
            else if (ev == EV_S2_SHORT) {
                alarm_reset = 1;
                alarm_trigger = 0;
                ev = EV_NONE;
                snooze_time = 0;
            }
        }
#endif

        /////////////////////////////////////////////////////////////////////
        // keyboard decision tree
        switch (kmode) {
            case K_SET_HOUR:
                flash_01 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_hours_incr();
                else if (ev == EV_S2_SHORT)
                    kmode = K_SET_MINUTE;
                break;

            case K_SET_MINUTE:
                flash_01 = 0;
                flash_23 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_minutes_incr();
                else if (ev == EV_S2_SHORT)
                #ifdef SIX_DIGITS
                    kmode = K_SET_SECOND;
                #else
                    kmode = K_SET_HOUR_12_24;
                #endif
                break;

            case K_SET_SECOND:
                flash_01 = 0;
                flash_23 = 0;
#ifdef SIX_DIGITS                
                flash_45 = 1;
#endif                
                if (ev == EV_S1_SHORT)
                    ds_sec_zero();
                else if (ev == EV_S2_SHORT)
                    kmode = K_SET_HOUR_12_24;
                break;

            case K_SET_HOUR_12_24:
                dmode = M_SET_HOUR_12_24;
                if (ev == EV_S1_SHORT) {
                    ds_hours_12_24_toggle();
                    cfg_changed = 1;
                } else if (ev == EV_S2_SHORT) {
#ifdef WITH_NMEA
                    nmea_prev_tz_hr = nmea_tz_hr;
                    nmea_prev_tz_min = nmea_tz_min;
                    nmea_prev_tz_dst = nmea_tz_dst;
                    nmea_prev_autosync = nmea_autosync;
                    kmode = K_TZ_SET_HOUR;
#else
                    kmode = K_NORMAL;
#endif
                }
                break;
#ifdef WITH_NMEA
            case K_TZ_SET_HOUR:
                dmode = M_TZ_SET_TIME;
                flash_01 = 1;
                flash_23 = 0;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    nmea_tz_hr = nmea_tz_hr < 12 ? nmea_tz_hr + 1 : -12;
                else if (ev == EV_S2_SHORT)
                    kmode = K_TZ_SET_MINUTE;
                break;
            case K_TZ_SET_MINUTE:
                flash_01 = 0;
                flash_23 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    switch(nmea_tz_min) {
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
                    kmode = K_TZ_SET_DST;
                break;
            case K_TZ_SET_DST:
                dmode = M_TZ_SET_DST;
                flash_01 = flash_23 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    nmea_tz_dst ^= 0x01;
                else if (ev == EV_S2_SHORT) {
                    flash_01 = flash_23 = 0;
                    kmode = K_TZ_AUTOUPDATE;
                }
                break;
            case K_TZ_AUTOUPDATE:
                dmode = M_TZ_AUTOUPDATE;
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
                    if (nmea_prev_tz_hr != nmea_tz_hr ||
                        nmea_prev_tz_min != nmea_tz_min ||
                        nmea_prev_tz_dst != nmea_tz_dst ||
                        nmea_prev_autosync != nmea_autosync) {
                        nmea_save_tz();
                        enable_nmea_receiving();
                    }
                    kmode = K_NORMAL;
                }
                break;
#endif
            case K_TEMP_DISP:
                dmode = M_TEMP_DISP;
                if (ev == EV_S1_SHORT) 
                    ds_temperature_offset_incr();
#ifdef DEBUG
                else if (ev == EV_S1S2_LONG) {
                    kmode = K_DEBUG;
                }
#endif
                else if (ev == EV_S1_LONG)
                    ds_temperature_cf_toggle();
                else if (ev == EV_S2_SHORT) {
#ifdef WITHOUT_DATE
                    kmode = K_NORMAL;
#else
                    kmode = K_DATE_DISP;
#endif
                }
                break;

#ifndef WITHOUT_DATE
            case K_DATE_DISP:
                dmode = M_DATE_DISP;
                if (ev == EV_S1_SHORT)
                    ds_date_mmdd_toggle();
                else if (ev == EV_S2_LONG)
                    kmode = CONF_SW_MMDD ? K_SET_DAY : K_SET_MONTH;
                else if (ev == EV_S2_SHORT)
#ifdef WITHOUT_WEEKDAY
#ifdef SIX_DIGITS
                    kmode = K_NORMAL;
#else
                    kmode = K_YEAR_DISP;
#endif
#else
                    kmode = K_WEEKDAY_DISP;
#endif
                break;

            case K_SET_MONTH:
                flash_01 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_month_incr();
                else if (ev == EV_S2_SHORT) {
                    flash_01 = 0;
#ifdef SIX_DIGITS
                    kmode = CONF_SW_MMDD ? K_SET_YEAR : K_SET_DAY;
#else                    
                    kmode = CONF_SW_MMDD ? K_DATE_DISP : K_SET_DAY;
#endif
                }
                break;

            case K_SET_DAY:
                flash_23 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_day_incr();
                else if (ev == EV_S2_SHORT) {
                    flash_23 = 0;
#ifdef SIX_DIGITS
                    kmode = CONF_SW_MMDD ? K_SET_MONTH : K_SET_YEAR;
#else                    
                    kmode = CONF_SW_MMDD ? K_SET_MONTH : K_DATE_DISP;
#endif
                }
                break;

#ifdef SIX_DIGITS
            case K_SET_YEAR:
                flash_45 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_year_incr();
                else if (ev == EV_S2_SHORT) {
                    flash_45 = 0;
                    kmode = K_NORMAL;
                }
                break;
#endif

#ifndef WITHOUT_WEEKDAY
            case K_WEEKDAY_DISP:
                dmode = M_WEEKDAY_DISP;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_weekday_incr();
                else if (ev == EV_S2_SHORT)
#ifdef SIX_DIGITS
                    kmode = K_NORMAL;
#else
                    kmode = K_YEAR_DISP;
#endif
                break;
#endif                

#ifndef SIX_DIGITS
            case K_YEAR_DISP:
                dmode = M_YEAR_DISP;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast))
                    ds_year_incr();
                else if (ev == EV_S2_SHORT)
                    kmode = K_NORMAL;
                break;
#endif
#endif

#ifdef DEBUG
            /*
            To enter DEBUG mode, go to the Temperature screen,
            then hold S1 and S2 simultaneously.
            S1 cycles through the DEBUG modes.
            To exit DEBUG mode, hold S1 and S2 again.
            */
            case K_DEBUG:
            case K_DEBUG2:
            case K_DEBUG3:
                dmode = M_DEBUG + kmode - K_DEBUG;
                if (ev == EV_S1_SHORT) {
                    kmode = (kmode - K_DEBUG + 1) % NUM_DEBUG + K_DEBUG;
                }
                else if (ev == EV_S1S2_LONG) {
                    kmode = K_TEMP_DISP;
                }
                break;
#endif

#ifndef SIX_DIGITS
            case K_SEC_DISP:
                dmode = M_SEC_DISP;
                if (ev == EV_S1_SHORT) {
                    kmode = K_NORMAL;
                } else if (ev == EV_S2_LONG) {
                    ds_sec_zero();
#if defined(WITH_MONTHLY_CORR) && WITH_MONTHLY_CORR != 0
                    corr_remaining = RUNTIME_PER_SEC;
#endif
                }
                break;
#endif

#ifndef WITHOUT_ALARM
            case K_ALARM:
                flash_01 = 0;
                flash_23 = 0;
#ifdef SIX_DIGITS                                
                flash_45 = 0;
#endif                
                dmode = M_ALARM;
                if (ev == EV_TIMEOUT)
                    kmode = K_NORMAL;
                else if (ev == EV_S1_SHORT) {
#if !defined(WITHOUT_CHIME)
                    count_timeout = TIMEOUT_LONG; // timeout for chime disp
                    kmode = K_CHIME;
#else
                    kmode = K_NORMAL;
#endif
                } else if (ev == EV_S2_SHORT) {
                    count_timeout = TIMEOUT_LONG; // reset timeout on toggle
                    ds_alarm_on_toggle();
                    cfg_changed = 1;
                } else if (ev == EV_S2_LONG) {
                    count_timeout = 0; // infinite adjusting
                    kmode = K_ALARM_SET_HOUR;
                }
                break;

            case K_ALARM_SET_HOUR:
                flash_01 = 1;
                alarm_reset = 1; // don't trigger while setting
                if (ev == EV_S2_SHORT)
                    kmode = K_ALARM_SET_MINUTE;
                else if (ev == EV_S1_SHORT || S1_LONG && blinker_fast) {
                    ds_alarm_hours_incr();
                    cfg_changed = 1;
                }
                break;

            case K_ALARM_SET_MINUTE:
                flash_01 = 0;
                flash_23 = 1;
                alarm_reset = 1;
                if (ev == EV_S2_SHORT) {
                    count_timeout = TIMEOUT_LONG; // timeout for alarm disp
                    kmode = K_ALARM;
                } else if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
                    ds_alarm_minutes_incr();
                    cfg_changed = 1;
                }
                break;
#endif

#ifndef WITHOUT_CHIME
            case K_CHIME:
                flash_01 = 0;
                flash_23 = 0;
#ifdef SIX_DIGITS                
                flash_45 = 0;
#endif                
                dmode = M_CHIME;
                if (ev == EV_S1_SHORT || ev == EV_TIMEOUT) {
                    kmode = K_NORMAL;
                } else if (ev == EV_S2_SHORT) {
                    count_timeout = TIMEOUT_LONG; // reset timeout on toggle
                    ds_chime_on_toggle();
                    cfg_changed = 1;
                } else if (ev == EV_S2_LONG) {
                    count_timeout = 0; // infinite adjusting
                    kmode = K_CHIME_SET_SINCE;
                }
                break;

            case K_CHIME_SET_SINCE:
                flash_01 = 1;
                if (ev == EV_S2_SHORT)
                    kmode = K_CHIME_SET_UNTIL;
                else if (ev == EV_S1_SHORT || S1_LONG && blinker_fast) {
                    ds_chime_since_incr();
                    cfg_changed = 1;
                }
                break;

            case K_CHIME_SET_UNTIL:
                flash_01 = 0;
                flash_23 = 1;
                if (ev == EV_S2_SHORT) {
                    count_timeout = TIMEOUT_LONG; // timeout for chime disp
                    kmode = K_CHIME;
                } else if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
                    ds_chime_until_incr();
                    cfg_changed = 1;
                }
                break;
#endif

            case K_NORMAL:
            default:
                flash_01 = 0;
                flash_23 = 0;
#ifdef SIX_DIGITS                                
                flash_45 = 0;
#endif                

                dmode = M_NORMAL;
                if (count_timeout)
                    count_timeout = 0; // no timeout for normal (time display) mode

                if (ev == EV_S1_SHORT)
                {
#ifdef SIX_DIGITS
#ifndef WITHOUT_ALARM
                    kmode = K_ALARM;
#endif
#else
                    kmode = K_SEC_DISP;
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
                else if (ev == EV_S1_LONG)
                {
#if !defined(SIX_DIGITS) && !defined(WITHOUT_ALARM)
                    kmode = K_ALARM;
#endif
                }
                else if (ev == EV_S2_LONG)
                    kmode = K_SET_HOUR;
                else if (ev == EV_S2_SHORT)
                    kmode = K_TEMP_DISP;
#ifdef stc15w408as
                else if (ev == EV_S3_LONG) {
                    LED = !LED;
                }
#endif
        };

        /////////////////////////////////////////////////////////////////////
        // display execution tree

        clearTmpDisplay();

        dmode_bak = dmode;

        if (dmode == M_NORMAL && kmode == K_NORMAL) {
            uint8_t ss = rtc_table[DS_ADDR_SECONDS];
            if (ss < 0x20) 
            {
                dmode = M_NORMAL;
            }
#ifdef AUTO_SHOW_TEMPERATURE
            else if (ss < 0x25) 
            {
                dmode = M_TEMP_DISP;
            }
#endif
#if !defined(WITHOUT_DATE) && defined(AUTO_SHOW_DATE)
            else if (ss < 0x30)
            {
                dmode = M_DATE_DISP;
            }
#endif
#if !defined(WITHOUT_DATE) && !defined(WITHOUT_WEEKDAY) && defined(AUTO_SHOW_WEEKDAY)
            else if (ss < 0x35)
            {
                dmode = M_WEEKDAY_DISP;
            }
#endif
        }

#if defined(WITH_MONTHLY_CORR) && WITH_MONTHLY_CORR != 0
        if (!corr_remaining && rtc_table[DS_ADDR_SECONDS] == 0x51) {
            ds_writebyte(DS_ADDR_SECONDS, rtc_table[DS_ADDR_SECONDS] + CORR_VALUE);
            corr_remaining = RUNTIME_PER_SEC;
        }
#endif

        switch (dmode) {
            case M_NORMAL:
#ifndef WITHOUT_ALARM
            case M_ALARM:
#endif
#ifndef WITHOUT_CHIME
            case M_CHIME:
#endif
            {
                uint8_t hh = rtc_hh_bcd;
                uint8_t mm = rtc_mm_bcd;
                __bit pm = rtc_pm;

#ifndef WITHOUT_ALARM
                if (dmode == M_ALARM) {
                    hh = alarm_hh_bcd;
                    mm = alarm_mm_bcd;
                    pm = alarm_pm;
                }
#endif

#ifndef WITHOUT_CHIME
                if (dmode == M_CHIME) {
                    hh = chime_ss_bcd;
                    mm = chime_uu_bcd;
                    pm = chime_uu_pm;
                }
#endif

                if (!flash_01 || blinker_fast || S1_LONG) {
                    uint8_t h0 = hh >> 4;
                    if (H12_12 && h0 == 0) {
                        h0 = LED_BLANK;
                    }
                    filldisplay(0, h0, 0);
                    filldisplay(1, hh & 0x0F, 0);
                }

                if (!flash_23 || blinker_fast || S1_LONG) {
#ifndef WITHOUT_CHIME
                    if (dmode == M_CHIME) {
                        // remove leading zero in chime stop hr
                        uint8_t m0 = mm >> 4;
                        if (H12_12 && m0 == 0) {
                            m0 = LED_BLANK;
                        }
                        filldisplay(2, m0, 0);
                    } else
#endif
                    filldisplay(2, mm >> 4, 0);
#ifdef SIX_DIGITS
                    filldisplay(3, mm & 0x0F, dmode == M_NORMAL ? blinker_slow : 0);
#else
                    filldisplay(3, mm & 0x0F, 0);
#endif
                }

#ifdef SIX_DIGITS
                if (!flash_45 || blinker_fast || S1_LONG) {                    
                    if (dmode == M_NORMAL)
                    {
                        filldisplay(4, (rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4), blinker_slow);
                        filldisplay(5, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS, 0);
                    }
#ifndef WITHOUT_ALARM
                    else if (dmode == M_ALARM)
                    {
                        // Show letter 'A' for the alarm mode
                        filldisplay(5, LED_a, 0);
                    }
#endif                    
#ifndef WITHOUT_CHIME
                    else if (dmode == M_CHIME)
                    {
                        // Show letter 'A' for the alarm mode
                        filldisplay(5, LED_c, 0);
                    }
#endif
                }
#endif

                if (blinker_slow || dmode != M_NORMAL) {
#ifndef WITHOUT_CHIME
                    if (dmode != M_CHIME) {
#endif
                    dotdisplay(1, 1);
                    dotdisplay(2, 1);
#ifndef WITHOUT_CHIME
                }
#endif
                }
#ifndef WITHOUT_CHIME
                if (dmode == M_CHIME) {
#ifdef SIX_DIGITS
                    dotdisplay(4, CONF_CHIME_ON);
#else
                    dotdisplay(2, CONF_CHIME_ON);
#endif                    
                    dotdisplay(1, chime_ss_pm);
                }
#endif
#ifdef SIX_DIGITS
                dotdisplay(5, preparepm(pm));
#else
                dotdisplay(3, preparepm(pm));
#endif
                break;
            }
            case M_SET_HOUR_12_24:
                if (!H12_12) {
                    filldisplay(1, 2, 0);
                    filldisplay(2, 4, 0);
                } else {
                    filldisplay(1, 1, 0);
                    filldisplay(2, 2, 0);
                }
                filldisplay(3, LED_h, 0);
                break;

#ifdef WITH_NMEA
            case M_TZ_SET_TIME:
            {
                int8_t hh = nmea_tz_hr;
                if (hh < 0) {
                    hh = -hh;
                    dotdisplay(3, 1);
                } else {
                    dotdisplay(3, 0);
                }
                dotdisplay(1, 1);
                dotdisplay(2, 1);

                if (!flash_01 || blinker_fast || S1_LONG) {
                    if (hh >= 10) {
                        filldisplay(0, 1, 0);
                    } else {
                        filldisplay(0, LED_BLANK, 0);
                    }
                    filldisplay(1, hh % 10, 0);
                }
                if (!flash_23 || blinker_fast || S1_LONG) {
                    filldisplay(2, nmea_tz_min/10, 0);
                    filldisplay(3, nmea_tz_min%10, 0);
                }
                break;
            }
            case M_TZ_SET_DST:
                filldisplay(0, 'D'-'A'+LED_a, 0);
                filldisplay(1, 'S'-'A'+LED_a, 0);
                filldisplay(2, 'T'-'A'+LED_a, 0);
#ifdef SIX_DIGITS                
                filldisplay(4, nmea_tz_dst, 0);
#else
                filldisplay(3, nmea_tz_dst, 0);
#endif                
                break;
            case M_TZ_AUTOUPDATE:
#ifdef SIX_DIGITS                
                filldisplay(0, 'U'-'A'+LED_a, 0);
                filldisplay(1, 'P'-'A'+LED_a, 0);
                filldisplay(2, 'D'-'A'+LED_a, 0);

                if (NMEA_AUTOSYNC_OFF == nmea_autosync) {
                    filldisplay(3, 0, 0);
                    filldisplay(4, LED_f, 0);
                    filldisplay(5, LED_f, 0);
                } else {
                    filldisplay(3, nmea_autosync / 10, 0);
                    filldisplay(4, nmea_autosync % 10, 0);
                    filldisplay(5, LED_h, 0);
                }
#else
                if (NMEA_AUTOSYNC_OFF == nmea_autosync) {
                    filldisplay(0, 0, 0);
                    filldisplay(1, LED_f, 0);
                    filldisplay(2, LED_f, 0);
                } else {
                    filldisplay(0, nmea_autosync / 10, 0);
                    filldisplay(1, nmea_autosync % 10, 0);
                    filldisplay(2, LED_h, 0);
                }
#endif                
                break;
#endif

#ifndef SIX_DIGITS
            case M_SEC_DISP:
            #ifdef SHOW_MINUTES_WITH_SECONDS
                uint8_t mm = rtc_mm_bcd;
                filldisplay(0, mm >> 4, 0);
                filldisplay(1, mm & 0x0F, blinker_slow);
            #else
                dotdisplay(0, 0);
                dotdisplay(1, blinker_slow);
            #endif
            
                filldisplay(2, (rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4), blinker_slow);
                filldisplay(3, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS, 0);
                dotdisplay(3, 0);
                break;
#endif

#ifndef WITHOUT_DATE
            case M_DATE_DISP:
                if (!flash_01 || blinker_fast || S1_LONG) {
                    if (!CONF_SW_MMDD) {
                        filldisplay( 0, rtc_table[DS_ADDR_MONTH] >> 4, 0);// tenmonth ( &MASK_TENS useless, as MSB bits are read as '0')
                        filldisplay( 1, rtc_table[DS_ADDR_MONTH] & DS_MASK_MONTH_UNITS, 0);
                    }
                    else {
                        filldisplay( 2, rtc_table[DS_ADDR_MONTH] >> 4, 0);// tenmonth ( &MASK_TENS useless, as MSB bits are read as '0')
                        filldisplay( 3, rtc_table[DS_ADDR_MONTH] & DS_MASK_MONTH_UNITS, 0);
                    }
                }

                if (!flash_23 || blinker_fast || S1_LONG) {
                    if (!CONF_SW_MMDD) {
                        filldisplay( 2, rtc_table[DS_ADDR_DAY] >> 4, 0); // tenday   ( &MASK_TENS useless)
                        filldisplay( 3, rtc_table[DS_ADDR_DAY] & DS_MASK_DAY_UNITS, 0);     // day
                    }
                    else {
                        filldisplay( 0, rtc_table[DS_ADDR_DAY] >> 4, 0); // tenday   ( &MASK_TENS useless)
                        filldisplay( 1, rtc_table[DS_ADDR_DAY] & DS_MASK_DAY_UNITS, 0);     // day
                    }
                }
                dotdisplay(1, 1);
#ifdef SIX_DIGITS
                dotdisplay(3, 1);
                if (!flash_45 || blinker_fast || S1_LONG) {
                    filldisplay(4, (rtc_table[DS_ADDR_YEAR] >> 4) & (DS_MASK_YEAR_TENS >> 4), 0);
                    filldisplay(5, rtc_table[DS_ADDR_YEAR] & DS_MASK_YEAR_UNITS, 0);
                }
#else
                dotdisplay(3, 0);
#endif
                break;
#endif

#ifndef WITHOUT_DATE
#ifndef WITHOUT_WEEKDAY
            case M_WEEKDAY_DISP:
            {
                uint8_t wd;

                wd = rtc_table[DS_ADDR_WEEKDAY] - 1;

                filldisplay(1, weekDay[wd][0] - 'A' + LED_a, 0);
                filldisplay(2, weekDay[wd][1] - 'A' + LED_a, 0);
                filldisplay(3, weekDay[wd][2] - 'A' + LED_a, 0);

                dotdisplay(3, 0);
	      }
	      break;
#endif          

          case M_YEAR_DISP:
              // fix upper 2 digit as 20
              filldisplay(0, 2, 0);
              filldisplay(1, 0, 0);

              filldisplay(2, (rtc_table[DS_ADDR_YEAR] >> 4) & (DS_MASK_YEAR_TENS >> 4), 0);
              filldisplay(3, rtc_table[DS_ADDR_YEAR] & DS_MASK_YEAR_UNITS, 0);
              break;
#endif

          case M_TEMP_DISP:
#ifdef SIX_DIGITS
              filldisplay(2, ds_int2bcd_tens(temp), 0);
              filldisplay(3, ds_int2bcd_ones(temp), 0);
              filldisplay(4, CONF_C_F ? LED_f : LED_c, 1);
              // if (temp<0) filldisplay( 3, LED_DASH, 0);  -- temp defined as uint16, cannot be <0
              dotdisplay(5, 0);
#else
              filldisplay(0, ds_int2bcd_tens(temp), 0);
              filldisplay(1, ds_int2bcd_ones(temp), 0);
              filldisplay(2, CONF_C_F ? LED_f : LED_c, 1);
              dotdisplay(3, 0);
#endif
              break;

#ifdef DEBUG
            case M_DEBUG:
            {
                // seconds, loop counter, blinkers, S1/S2, keypress events
                uint8_t cc = count;
                if (S1_PRESSED || S2_PRESSED) {
                    filldisplay( 0, S1_PRESSED || S2_PRESSED ? LED_DASH : LED_BLANK, ev == EV_S1_SHORT || ev == EV_S2_SHORT);
                    filldisplay( 1, S1_LONG || S2_LONG ? LED_DASH : LED_BLANK, ev == EV_S1_LONG || ev == EV_S2_LONG);
                } else {
                    filldisplay(0, (rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4), 0);
                    filldisplay(1, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS, blinker_slow );
                }
                filldisplay(2, hex[cc >> 4 & 0x0F], ev != EV_NONE);
                filldisplay(3, hex[cc & 0x0F], blinker_slow & blinker_fast);
#ifdef SIX_DIGITS                
                filldisplay(5, 1, 0);
#endif    
                break;
            }
            case M_DEBUG2:
            {
                // photoresistor adc and lightval
                uint8_t adc = getADCResult8(ADC_LIGHT);
                uint8_t lv = lightval;
                filldisplay( 0, hex[adc>>4], 0);
                filldisplay( 1, hex[adc & 0x0F], 0);
                filldisplay( 2, hex[lv>>4], 0);
                filldisplay( 3, hex[lv & 0x0F], 0);
#ifdef SIX_DIGITS                
                filldisplay(5, 2, 0);
#endif    
                break;
            }
            case M_DEBUG3:
            {
                // thermistor adc
                uint16_t rt = getADCResult(ADC_TEMP);
                filldisplay( 0, hex[rt >> 12], 0);
                filldisplay( 1, hex[rt >> 8 & 0x0F], 0);
                filldisplay( 2, hex[rt >> 4 & 0x0F], 0);
                filldisplay( 3, hex[rt & 0x0F], 0);
#ifdef SIX_DIGITS                
                filldisplay(5, 3, 0);
#endif    
                break;
            }
#endif
        }

        dmode = dmode_bak; // back to original dmode

#ifndef WITHOUT_ALARM
        if (alarm_trigger && !alarm_reset) {
            if (blinker_slow && blinker_fast) {
                clearTmpDisplay();
                BUZZER_ON;
            } else {
                BUZZER_OFF;
            }
        } else {
            BUZZER_OFF;
        }
#endif

#ifndef WITHOUT_CHIME
        switch (chime_trigger) {
        case CHIME_RUNNING: // ~100ms chime
                BUZZER_ON;
            for (chime_ticks = 0; chime_ticks < 10; );
                BUZZER_OFF;
            chime_trigger = CHIME_WAITING;
        case CHIME_WAITING: // wait > 1sec until rtc sec changed
            if (chime_ticks > 150)
                chime_trigger = CHIME_IDLE; // stop chime
            break;
        default:
            break;
        }
#endif

        __critical {
            updateTmpDisplay();
        }

        count++;
        WDT_CLEAR();

#ifdef WITH_NMEA
        if (is_nmea_receiving_on) {
            if (nmea_state == NMEA_SET || ++nmea_progress_seconds >= NMEA_MAX_SYNC_DURATION) {
                disable_nmea_receiving();
                
                if (nmea_state == NMEA_SET) {
                    nmea2localtime();
                }

                nmea_state = NMEA_NONE;
                nmea_seconds_to_sync = NMEA_AUTOSYNC_DELAY;
            }
        }
#endif
    }
}

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
#endif

/* ------------------------------------------------------------------------- */
