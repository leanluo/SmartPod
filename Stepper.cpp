#include "Stepper.h"
#include "Arduino.h"

int16_t stepPosition = 0;
int16_t stepsRemaining = 0;

uint8_t _stepPin;
uint8_t _dirPin;

uint8_t motorState = MS_STILL;
int16_t lastStepDif = 0;

uint16_t stepTH1 = 0;
uint16_t stepTH2 = 0;

uint16_t delay1 = 0;
uint16_t delay2 = 0;
uint16_t delay3 = 0;


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
    int16_t stepDif = newStepPos - stepPosition;
    if (stepDif < -180/STEP_ANGLE) {
        stepDif += 360/STEP_ANGLE;
    } else if (stepDif > 180/STEP_ANGLE) {
        stepDif -= 360/STEP_ANGLE;
    }

    if (stepDif) {
        calculateMotorState(stepDif);
        calculateDelays();
        calculateThresholds();
    }

    // if (abs(stepDif) > MIN_ANGLE_DIF/STEP_ANGLE) {
    //     stepsRemaining += stepDif;
    //     stepPosition = newStepPos;
    //     calculateDelays();
    //     calculateThresholds();
    //     if (stepPosition < 0) {
    //         stepPosition += 360/STEP_ANGLE;
    //     }
    // }
}

// Do the remaining steps
void moveStepper()
{
    unsigned int delay;

    if (stepsRemaining > 0) {
        if (stepsRemaining > stepTH1) {
            delay = delay1;
        } else if (stepsRemaining > stepTH2) {
            delay = delay2;
        } else {
            delay = delay3;
        }
        delayMicroseconds(delay/3);
        digitalWrite(_dirPin, LOW);
        delayMicroseconds(delay/3);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(delay/3);
        digitalWrite(_stepPin, LOW);
        stepsRemaining--;
    } else if (stepsRemaining < 0) {
        if (stepsRemaining < stepTH1) {
            delay = delay1;     // Fast
        } else if (stepsRemaining < stepTH2) {
            delay = delay2;      // Slow
        } else {
            delay = delay3;
        }
        delayMicroseconds(delay/3);
        digitalWrite(_dirPin, HIGH);
        delayMicroseconds(delay/3);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(delay/3);
        digitalWrite(_stepPin, LOW);
        stepsRemaining++;
    }
}

// Calculate motor state
void calculateMotorState(int16_t stepDif)
{
    // Still motor state
    if(motorState == MS_STILL) {
        if (abs(stepDif) > MIN_ANGLE_DIF/STEP_ANGLE) {
            motorState = MS_ACCELERATING;
            stepsRemaining += stepDif*ACCEL_FACTOR;     // Have some space to decelerate after
            stepPosition += stepDif*ACCEL_FACTOR;
            lastStepDif = stepDif;
        }
        else {
            // motorState = MS_STILL;
            stepsRemaining = 0;
            lastStepDif = 0;
        }
    }
    // Accelerating motor state
    else if (motorState == MS_ACCELERATING) {
        if (abs(stepDif) > DECEL_FACTOR*abs(lastStepDif)) {
            motorState = MS_CRUISING;
            stepsRemaining +=  stepDif*ACCEL_FACTOR;
            stepPosition += stepDif*ACCEL_FACTOR;
            lastStepDif = stepDif;
        }
        else {
            motorState = MS_DECELERATING;
            stepsRemaining += stepDif;
            stepPosition += stepDif;
            lastStepDif = stepDif;
        }
    }
    // Cruising motor state
    else if (motorState == MS_CRUISING) {
        if (abs(stepDif) > DECEL_FACTOR*abs(lastStepDif)) {
            motorState = MS_CRUISING;
            stepsRemaining +=  stepDif*ACCEL_FACTOR;
            stepPosition += stepDif*ACCEL_FACTOR;
            lastStepDif = stepDif;
        }
        else {
            motorState = MS_DECELERATING;
            stepsRemaining += stepDif;
            stepPosition += stepDif;
            lastStepDif = stepDif;
        }
    }
    // Accelerating motor state
    else if (motorState == MS_DECELERATING) {
        if (abs(stepDif) > MIN_ANGLE_DIF/STEP_ANGLE) {
            motorState = MS_ACCELERATING;
            stepsRemaining += stepDif*ACCEL_FACTOR;     // Have some space to decelerate after
            stepPosition += stepDif*ACCEL_FACTOR;
            lastStepDif = stepDif;
        }
        else {
            motorState = MS_STILL;
            stepsRemaining += stepDif;
            stepPosition += stepDif;
            //stepsRemaining = 0;
            lastStepDif = stepDif;
        }
    }
    
    // Serial.print(stepPosition);
    // Serial.print("\t");

    if (stepPosition < 0) {
        stepPosition += 360/STEP_ANGLE;
    }
    else if (stepPosition > 360/STEP_ANGLE) {
        stepPosition -= 360/STEP_ANGLE;
    }
    // Serial.println(stepPosition);
}

// Calculate delays for the three phases
void calculateDelays()
{
    // Asumiendo que un ciclo entero de comunicación toma 160 ms
    if (motorState == MS_STILL) {
        delay1 = MAX_MICRO_DELAY;
        delay2 = MAX_MICRO_DELAY;
        delay3 = MAX_MICRO_DELAY;
    }
    else if (motorState == MS_ACCELERATING) {
        delay1 = BASE_DELAY/abs(stepsRemaining)*(1+PHASE_II_FACTOR+PHASE_III_FACTOR)/3;
        delay2 = BASE_DELAY/abs(stepsRemaining)*(1+PHASE_II_FACTOR+PHASE_III_FACTOR)/3/PHASE_II_FACTOR;
        delay3 = BASE_DELAY/abs(stepsRemaining)*(1+PHASE_II_FACTOR+PHASE_III_FACTOR)/3/PHASE_III_FACTOR;
    }
    else if (motorState == MS_CRUISING) {
        delay1 = BASE_DELAY/abs(stepsRemaining);
        delay2 = BASE_DELAY/abs(stepsRemaining);
        delay3 = BASE_DELAY/abs(stepsRemaining);
    }
    else if (motorState == MS_DECELERATING) {
        delay1 = BASE_DELAY/abs(stepsRemaining)*(1+PHASE_II_FACTOR+PHASE_III_FACTOR)/3/PHASE_III_FACTOR;
        delay2 = BASE_DELAY/abs(stepsRemaining)*(1+PHASE_II_FACTOR+PHASE_III_FACTOR)/3/PHASE_II_FACTOR;
        delay3 = BASE_DELAY/abs(stepsRemaining)*(1+PHASE_II_FACTOR+PHASE_III_FACTOR)/3;
    }
    if (delay1 < MIN_MICRO_DELAY)
        delay1 = MIN_MICRO_DELAY;
    else if (delay1 > MAX_MICRO_DELAY)
        delay1 = MAX_MICRO_DELAY;
    if (delay2 < MIN_MICRO_DELAY)
        delay2 = MIN_MICRO_DELAY;
    else if (delay2 > MAX_MICRO_DELAY)
        delay2 = MAX_MICRO_DELAY;
    if (delay3 < MIN_MICRO_DELAY)
        delay3 = MIN_MICRO_DELAY;
    else if (delay3 > MAX_MICRO_DELAY)
        delay3 = MAX_MICRO_DELAY;
}

// Calculate thresholds to separate phases
void calculateThresholds()
{
    if (motorState == MS_STILL) {
        stepTH1 = 0;
        stepTH2 = 0;
    }
    else if (motorState == MS_ACCELERATING) {
        stepTH1 = stepsRemaining*(PHASE_III_FACTOR+PHASE_II_FACTOR)/(1+PHASE_II_FACTOR+PHASE_III_FACTOR);
        stepTH2 = stepsRemaining*PHASE_III_FACTOR/(1+PHASE_II_FACTOR+PHASE_III_FACTOR);
    }
    else if (motorState == MS_CRUISING) {
        stepTH1 = stepsRemaining/3;
        stepTH2 = stepsRemaining*2/3;
    }
    else if (motorState == MS_DECELERATING) {
        stepTH1 = stepsRemaining*(1+PHASE_II_FACTOR)/(1+PHASE_II_FACTOR+PHASE_III_FACTOR);
        stepTH2 = stepsRemaining/(1+PHASE_II_FACTOR+PHASE_III_FACTOR);
    }
}


// --------------------- ANTES ---------------

// #include "Stepper.h"
// #include "Arduino.h"

// uint16_t stepPosition = 0;
// int16_t stepsRemaining = 0;
// uint8_t _stepPin;
// uint8_t _dirPin;
// uint16_t stepTH1 = 0;
// uint16_t stepTH2 = 0;
// uint16_t shortDelay = 0;
// uint16_t longDelay = 0;
// int sd1 = 0;
// int sd2 = 0;
// int sd3 = 0;
// int sd4 = 0;

// // Initialize stepper pins
// void initStepper(uint8_t stepPin, uint8_t dirPin)
// {
//     pinMode(stepPin, OUTPUT);
//     pinMode(dirPin, OUTPUT);
//     _stepPin = stepPin;
//     _dirPin = dirPin;
// }

// // Add steps to the step counter
// void addSteps(float XT, float YT)
// {
//     float newAngle = float(180*atan2(YT,XT)/PI);
//     if (newAngle < 0) {
//         newAngle = newAngle+360;
//     }
//     uint16_t newStepPos = uint16_t(newAngle/STEP_ANGLE);
//     int stepDif = newStepPos - stepPosition;
//     if (stepDif < -180/STEP_ANGLE) {
//         stepDif += 360/STEP_ANGLE;
//     } else if ( stepDif > 180/STEP_ANGLE) {
//         stepDif -= 360/STEP_ANGLE;
//     }

//     if (abs(stepDif) > MIN_ANGLE_DIF/STEP_ANGLE) {
//         stepsRemaining += stepDif;
//         stepPosition = newStepPos;
//         calculateDelays();
//         calculateThresholds();
//         if (stepPosition < 0) {
//             stepPosition += 360/STEP_ANGLE;
//         }
//     }
// }

// // Do the remaining steps
// void moveStepper()
// {
//     unsigned int delay;

//     if (stepsRemaining > 0) {
//         //if (true) {
//         if (stepsRemaining > stepTH1 && stepsRemaining < stepTH2) {
//             delay = shortDelay;     // Fast
//         } else {
//             delay = longDelay;      // Slow
//         }
//         digitalWrite(_dirPin, LOW);
//         delayMicroseconds(delay);
//         digitalWrite(_stepPin, HIGH);
//         delayMicroseconds(delay);
//         digitalWrite(_stepPin, LOW);
//         stepsRemaining--;
//     } else if (stepsRemaining < 0) {
//         //if (true) {
//         if (stepsRemaining < stepTH1 && stepsRemaining > stepTH2) {
//             delay = shortDelay;     // Fast
//         } else {
//             delay = longDelay;      // Slow
//         }
//         digitalWrite(_dirPin, HIGH);
//         delayMicroseconds(delay);
//         digitalWrite(_stepPin, HIGH);
//         delayMicroseconds(delay);
//         digitalWrite(_stepPin, LOW);
//         stepsRemaining++;
//     }
// }

// void calculateDelays()
// {
//     // Asumiendo que un ciclo entero de comunicación toma 160 ms
//     shortDelay = 53333/abs(stepsRemaining);
//     longDelay = 160000/abs(stepsRemaining);
//     if (shortDelay < MIN_MICRO_DELAY)
//         shortDelay = MIN_MICRO_DELAY;
//     else if (shortDelay > MAX_MICRO_DELAY)
//         shortDelay = MAX_MICRO_DELAY;
//     if (longDelay < MIN_MICRO_DELAY)
//         longDelay = MIN_MICRO_DELAY;
//     else if (longDelay > MAX_MICRO_DELAY)
//         longDelay = MAX_MICRO_DELAY;
// }

// void calculateThresholds()
// {
//     stepTH1 = stepsRemaining/8;
//     stepTH2 = stepsRemaining*7/8;
// }