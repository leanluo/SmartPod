#ifndef STEPPER_H
#define STEPPER_H

#include "Arduino.h"

#define PI 3.1415926535897932384626433832795
#define STEP_ANGLE (1.8/16)     // 16th step mode
#define MIN_ANGLE_DIF 2.0
#define MIN_MICRO_DELAY 900
#define MAX_MICRO_DELAY 8500

#define MS_STILL 0
#define MS_ACCELERATING 1
#define MS_CRUISING 2
#define MS_DECELERATING 3
#define MS_TURNING 4

#define ACCEL_FACTOR 0.9
#define DECEL_FACTOR 0.5
#define BASE_DELAY 390000
#define PHASE_II_FACTOR 3
#define PHASE_III_FACTOR 9

// Initialize stepper pins
void initStepper(uint8_t stepPin, uint8_t dirPin);

// Add steps to the step counter
void addSteps(float XT, float YT);

// Do the remaining steps
void moveStepper();

// Calculate the FSM for the motor state
void calculateMotorState(int16_t stepDif);

// Calculate step thresholds to switch speeds
void calculateThresholds();

// Calculate micro delays to adjust speeds
void calculateDelays();

#endif // STEPPER_H