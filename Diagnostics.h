#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "Arduino.h"

#define LED1_PIN A0
#define LED2_PIN A1
#define LED3_PIN A2
#define BLINK_TIME 250 //On-off time in milliseconds

// Init pin configurations
void initDiagnostics();

// Update LEDs states
void updateDiagnostics(uint8_t state);

#endif // DIAGNOSTICS_H