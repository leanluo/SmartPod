#ifndef STEPPER_H
#define STEPPER_H

#include "Arduino.h"

#define PI 3.1415926535897932384626433832795
#define STEP_ANGLE 1.8
#define MIN_STEP_DIF 3
#define MICRO_DELAY 800

// Initialize stepper pins
void initStepper(uint8_t stepPin, uint8_t dirPin);

// Add steps to the step counter
void addSteps(float XT, float YT);

// Do the remaining steps
void moveStepper();

#endif // STEPPER_H