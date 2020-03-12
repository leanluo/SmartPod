#include "Stepper.h"
#include <avr/wdt.h>
#include "Diagnostics.h"
#include <SPI.h>
#include "DW1000Ranging.h"

#define STEP_PIN 3
#define DIR_PIN 4
#define MS1_PIN 5
#define MS2_PIN 6
#define MS3_PIN 7
#define BTN1_PIN A3
#define BTN2_PIN A4

#define BTN_CD 500


// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
float XT, YT;

unsigned long CalibrationTimer = 0;

void setup()
{
	Serial.begin(115200);
	delay(1000);

	//init the configuration
	DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

	/*********************		 	TAG						**************************************/
	//DW1000Ranging.startAsTag("33:33:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

	/*********************			ANCHOR 1				**************************************/
	//DW1000Ranging.startAsTanchor("11:11:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

	/*********************			ANCHOR 2				**************************************/
	//DW1000Ranging.startAsTanchor("22:22:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

	/*********************			MAIN ANCHOR				**************************************/
	DW1000Ranging.startAsManchor("00:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);	
	//init stepper
	initStepper(STEP_PIN, DIR_PIN);
	pinMode(MS1_PIN, OUTPUT);
	pinMode(MS2_PIN, OUTPUT);
	pinMode(MS3_PIN, OUTPUT);
	digitalWrite(MS1_PIN, HIGH);
	digitalWrite(MS2_PIN, HIGH);
	digitalWrite(MS3_PIN, HIGH);
	//init LED pins
	initDiagnostics();
	//init button pins
	pinMode(BTN1_PIN, INPUT);
	pinMode(BTN2_PIN, INPUT);

	Serial.println(EEPROM_readFloat(EEPROM_AB));
	Serial.println(EEPROM_readFloat(EEPROM_CX));
	Serial.println(EEPROM_readFloat(EEPROM_CY));
}

void loop()
{
	DW1000Ranging.loop();

	// // ------------- Only for Main Anchor --------------
	//  -- Stepper --
	XT = DW1000Ranging.getXT();
	YT = DW1000Ranging.getYT();
	// XT = 0.0;
	// YT = 2.5;
	addSteps(XT, YT);
	moveStepper();
	//  -- Diagnostics --
	updateDiagnostics(DW1000Ranging.getState());
	pollCalibrationButton();
}

void newRange()
{
	// Serial.print("from: ");
	// Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
	// Serial.print("\t Range: ");
	// Serial.print(DW1000Ranging.getDistantDevice()->getRange());
	// Serial.print(" m");
	// Serial.print("\t RX power: ");
	// Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
	// Serial.println(" dBm");
}

void newBlink(DW1000Device *device)
{
	// Serial.print("blink; 1 device added ! -> ");
	// Serial.print(" short:");
	// Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
	// Serial.print("delete inactive device: ");
	// Serial.println(device->getShortAddress(), HEX);
}

void pollCalibrationButton()
{
	if (digitalRead(BTN1_PIN) == HIGH && millis()-CalibrationTimer > BTN_CD)
	{
		EEPROM_writeFloat(CALIBRATION_FLAG,0.0);
		CalibrationTimer = millis();
		// Serial.print("Calibration button pressed. Timer: ");
		// Serial.println(CalibrationTimer);
	}
}
