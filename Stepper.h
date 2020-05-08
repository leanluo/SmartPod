#ifndef STEPPER_H
#define STEPPER_H

#include "Arduino.h"

#define PI 3.1415926535897932384626433832795
#define STEP_ANGLE (1.8/16)     // 16th step mode
#define MIN_ANGLE_DIF 3.0
#define MIN_MICRO_DELAY 600
#define MAX_MICRO_DELAY 2000

// Initialize stepper pins
void initStepper(uint8_t stepPin, uint8_t dirPin);

// Add steps to the step counter
void addSteps(float XT, float YT);

// Do the remaining steps
void moveStepper();

// Calculate step thresholds to switch speeds
void calculateThresholds();

// Calculate micro delays to adjust speeds
void calculateDelays();

#endif // STEPPER_H