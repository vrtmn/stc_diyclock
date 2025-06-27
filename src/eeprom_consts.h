#ifndef EEPROM_CONSTS_H
#define EEPROM_CONSTS_H

#define IAP_ADDRESS  0x0000

// NMEA settings

#define IAP_TZ_HR       (IAP_ADDRESS + 0)
#define IAP_TZ_MIN      (IAP_ADDRESS + 1)
#define IAP_TZ_DST      (IAP_ADDRESS + 2)
#define IAP_TZ_AUTOSYNC (IAP_ADDRESS + 3)

// Brightness settings

#define IAP_BRIGHTNESS_LOW      (IAP_ADDRESS + 4)
#define IAP_BRIGHTNESS_HIGH     (IAP_ADDRESS + 5)
#define IAP_BRIGHTNESS_NIGHT    (IAP_ADDRESS + 6)

#endif // #define EEPROM_CONSTS_H
