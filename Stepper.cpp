#include "Stepper.h"
#include "Arduino.h"

uint16_t stepPosition = 0;
int16_t stepsRemaining = 0;
uint8_t _stepPin;
uint8_t _dirPin;
uint16_t stepTH1 = 0;
uint16_t stepTH2 = 0;
uint16_t shortDelay = 0;
uint16_t longDelay = 0;
int sd1 = 0;
int sd2 = 0;
int sd3 = 0;
int sd4 = 0;

// Initialize stepper pins
void initStepper(uint8_t stepPin, uint8_t dirPin)
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    _stepPin = stepPin;
    _dirPin = dirPin;
}

// Add steps to the step counter
void addSteps(float XT, float YT)
{
    float newAngle = float(180*atan2(YT,XT)/PI);
    if (newAngle < 0) {
        newAngle = newAngle+360;
    }
    uint16_t newStepPos = uint16_t(newAngle/STEP_ANGLE);
    int stepDif = newStepPos - stepPosition;
    if (stepDif < -180/STEP_ANGLE) {
        stepDif += 360/STEP_ANGLE;
    } else if ( stepDif > 180/STEP_ANGLE) {
        stepDif -= 360/STEP_ANGLE;
    }

    if (abs(stepDif) > MIN_ANGLE_DIF/STEP_ANGLE) {
        stepsRemaining += stepDif;
        stepPosition = newStepPos;
        calculateDelays();
        calculateThresholds();
        if (stepPosition < 0) {
            stepPosition += 360/STEP_ANGLE;
        }
    }
}

// Do the remaining steps
void moveStepper()
{
    unsigned int delay;

    if (stepsRemaining > 0) {
        //if (true) {
        if (stepsRemaining > stepTH1 && stepsRemaining < stepTH2) {
            delay = shortDelay;     // Fast
        } else {
            delay = longDelay;      // Slow
        }
        digitalWrite(_dirPin, LOW);
        delayMicroseconds(delay);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(delay);
        digitalWrite(_stepPin, LOW);
        stepsRemaining--;
    } else if (stepsRemaining < 0) {
        //if (true) {
        if (stepsRemaining < stepTH1 && stepsRemaining > stepTH2) {
            delay = shortDelay;     // Fast
        } else {
            delay = longDelay;      // Slow
        }
        digitalWrite(_dirPin, HIGH);
        delayMicroseconds(delay);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(delay);
        digitalWrite(_stepPin, LOW);
        stepsRemaining++;
    }
}

void calculateDelays()
{
    // Asumiendo que un ciclo entero de comunicación toma 160 ms
    shortDelay = 53333/abs(stepsRemaining);
    longDelay = 160000/abs(stepsRemaining);
    if (shortDelay < MIN_MICRO_DELAY)
        shortDelay = MIN_MICRO_DELAY;
    else if (shortDelay > MAX_MICRO_DELAY)
        shortDelay = MAX_MICRO_DELAY;
    if (longDelay < MIN_MICRO_DELAY)
        longDelay = MIN_MICRO_DELAY;
    else if (longDelay > MAX_MICRO_DELAY)
        longDelay = MAX_MICRO_DELAY;
}

void calculateThresholds()
{
    stepTH1 = stepsRemaining/8;
    stepTH2 = stepsRemaining*7/8;
}