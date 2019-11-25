/**
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsAnchor
 *  - give example description
 */
#include <SPI.h>
#include "DW1000Ranging.h"
#include "Stepper.h"

#define STEP_PIN 3
#define DIR_PIN 4

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
float XT, YT;

void setup()
{
	Serial.begin(115200);
	delay(1000);
	//init stepper
	initStepper(STEP_PIN, DIR_PIN);
	//init the configuration
	DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
	//define the sketch as anchor. It will be great to dynamically change the type of module
	DW1000Ranging.attachNewRange(newRange);
	DW1000Ranging.attachBlinkDevice(newBlink);
	DW1000Ranging.attachInactiveDevice(inactiveDevice);
	//Enable the filter to smooth the distance
	//DW1000Ranging.useRangeFilter(true);

	/*********************		 	TAG						**************************************/
	//DW1000Ranging.startAsTag("33:33:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

	/*********************			ANCHOR 1				**************************************/
	//DW1000Ranging.startAsTanchor("11:11:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

	/*********************			ANCHOR 2				**************************************/
	//DW1000Ranging.startAsTanchor("22:22:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

	/*********************			MAIN ANCHOR				**************************************/
	DW1000Ranging.startAsManchor("00:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}

void loop()
{
	DW1000Ranging.loop();

	// Only for Main Anchor
	XT = DW1000Ranging.getXT();
	YT = DW1000Ranging.getYT();
	//XT = 1.0;
	//YT = 1.0;
	addSteps(XT, YT);
	moveStepper();
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
	Serial.print("blink; 1 device added ! -> ");
	Serial.print(" short:");
	Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
	Serial.print("delete inactive device: ");
	Serial.println(device->getShortAddress(), HEX);
}
