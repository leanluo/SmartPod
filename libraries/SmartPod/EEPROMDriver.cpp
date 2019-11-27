#include "EEPROMDriver.h"

// ****************
// Write floating point values to EEPROM
// ****************
void EEPROM_writeFloat(int address, float value)
{
uint8_t* p = (uint8_t*)(void*)&value;
for (int i = 0; i < sizeof(value); i++)
{
EEPROM.write(address++, *p++);
}
}
// ****************
// Read floating point values from EEPROM
// ****************
float EEPROM_readFloat(int address)
{
float value = 0.0;
uint8_t* p = (uint8_t*)(void*)&value;
for (int i = 0; i < sizeof(value); i++)
{
*p++ = EEPROM.read(address++);
}
return value;
}