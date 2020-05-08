#include "Diagnostics.h"
#include "pins_arduino.h"
#include "DW1000Ranging.h"

uint8_t diagnosticsState;
unsigned long led1timer = 0;
unsigned long led2timer = 0;
unsigned long led3timer = 0;
bool led1state;
bool led2state;
bool led3state;


// Init pin configurations
void initDiagnostics()
{
	pinMode(LED1_PIN, OUTPUT);
	pinMode(LED2_PIN, OUTPUT);
	pinMode(LED3_PIN, OUTPUT);
    diagnosticsState = STATE_IDLE;
    led1state = false;
    led2state = false;
    led3state = false;
}

void updateDiagnostics(uint8_t state)
{
    // Idle state
    if (state == STATE_IDLE) {
        // led 1 off
        if (led1state) {
            led1state = false;
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 off
        if (led2state) {
            led2state = false;
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 off
        if (led3state) {
            led3state = false;
            digitalWrite(LED3_PIN, led3state);
        }
    // Calibration 1 state
    } else if (state == STATE_CALIB1) {
        // led 1 blinking
        if (millis()-led1timer > BLINK_TIME) {
            led1state = !led1state;
            led1timer = millis();
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 off
        if (led2state) {
            led2state = false;
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 off
        if (led3state) {
            led3state = false;
            digitalWrite(LED3_PIN, led3state);
        }
    // Ready 1 state
    } else if (state == STATE_READY1) {
        // led 1 on
        if (!led1state) {
            led1state = true;
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 off
        if (led2state) {
            led2state = false;
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 off
        if (led3state) {
            led3state = false;
            digitalWrite(LED3_PIN, led3state);
        }
    // Calibration 2 state
    } else if (state == STATE_CALIB2) {
        // led 1 on
        if (!led1state) {
            led1state = true;
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 blinking
        if (millis()-led2timer > BLINK_TIME) {
            led2state = !led2state;
            led2timer = millis();
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 off
        if (led3state) {
            led3state = false;
            digitalWrite(LED3_PIN, led3state);
        }
    // Ready 2 state
    } else if (state == STATE_READY2 || state == STATE_PRECALIB) {
        // led 1 on
        if (!led1state) {
            led1state = true;
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 on
        if (!led2state) {   // Era !led2state en vez de true
            led2state = true;
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 off
        if (led3state) {
            led3state = false;
            digitalWrite(LED3_PIN, led3state);
        }
    // Trillateration state
    } else if (state == STATE_TRILAT) {
        // led 1 on
        if (!led1state) {
            led1state = true;
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 on
        if (!led2state) {
            led2state = true;
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 blinking
        if (millis()-led3timer > BLINK_TIME) {
            led3state = !led3state;
            led3timer = millis();
            digitalWrite(LED3_PIN, led3state);
        }
    // Error state
    } else if (state == STATE_ERROR) {
        // led 1 on
        if (!led1state) {
            led1state = true;
            digitalWrite(LED1_PIN, led1state);
        }
        // led 2 on
        if (!led2state) {
            led2state = true;
            digitalWrite(LED2_PIN, led2state);
        }
        // led 3 on
        if (!led3state) {
            led3state = true;
            digitalWrite(LED3_PIN, led3state);
        }
    }
}