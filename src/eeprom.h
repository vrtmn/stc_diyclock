#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

/*----------------------------
 Software delay function
 ----------------------------*/
void Delay(uint8_t n);

/*----------------------------
 Reads one byte from ISP/IAP/EEPROM area
 Input: addr (ISP/IAP/EEPROM address)
 Output: Flash data
----------------------------*/
uint8_t IapReadByte(uint16_t addr);

/*----------------------------
 Programs one byte to ISP/IAP/EEPROM area
 Input: addr (ISP/IAP/EEPROM address)
 dat (ISP/IAP/EEPROM data)
 Output:-
 ----------------------------*/
void IapProgramByte(uint16_t addr, uint8_t dat);

/*----------------------------
 Erases one sector area
 Input: addr (ISP/IAP/EEPROM address)
 Output:-
 ----------------------------*/
void IapEraseSector(uint16_t addr);

#endif // #define EEPROM_H