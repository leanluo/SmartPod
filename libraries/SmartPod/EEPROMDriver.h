#ifndef EEPROM_DRIVER_H
#define EEPROM_DRIVER_H

#include <EEPROM.h>


void EEPROM_writeFloat(int address, float value);
float EEPROM_readFloat(int address);

#endif
