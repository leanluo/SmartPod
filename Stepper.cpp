#include "Stepper.h"
#include "Arduino.h"

uint8_t stepPosition = 0;
int8_t stepsRemaining = 0;
uint8_t _stepPin;
uint8_t _dirPin;
unsigned long lastAction;

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
    uint8_t newStepPos = uint8_t(newAngle/STEP_ANGLE);
    int stepDif = newStepPos - stepPosition;
    if (stepDif < -100) {
        stepDif += 200;
    } else if ( stepDif > 100) {
        stepDif -= 200;
    }

    if (abs(stepDif) > MIN_STEP_DIF) {
        stepsRemaining += stepDif;
        stepPosition = newStepPos;
        if (stepPosition < 0) {
            stepPosition += 200;
        }
        Serial.print("Dif:");
        Serial.print(stepDif);
        Serial.print("\tPos:");
        Serial.println(stepPosition);
    }
}

// Do the remaining steps
void moveStepper()
{
    if (stepsRemaining > 0) {
        digitalWrite(_dirPin, LOW);
        delayMicroseconds(MICRO_DELAY);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(MICRO_DELAY);
        digitalWrite(_stepPin, LOW);
        stepsRemaining--;
    } else if (stepsRemaining < 0) {
        digitalWrite(_dirPin, HIGH);
        delayMicroseconds(MICRO_DELAY);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(MICRO_DELAY);
        digitalWrite(_stepPin, LOW);
        stepsRemaining++;
    } else {
        delayMicroseconds(MICRO_DELAY);
        delayMicroseconds(MICRO_DELAY);

    }
}