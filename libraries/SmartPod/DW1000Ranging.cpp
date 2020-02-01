/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Ranging.h
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */

#include "DW1000Ranging.h"
#include "DW1000Device.h"


DW1000RangingClass DW1000Ranging;

//other devices we are going to communicate with which are on our network:
DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];
byte DW1000RangingClass::_currentAddress[8];
byte DW1000RangingClass::_currentShortAddress[2];
byte DW1000RangingClass::_lastSentToShortAddress[2];
volatile uint8_t DW1000RangingClass::_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t DW1000RangingClass::_lastDistantDevice = 0;				// TODO short, 8bit?
DW1000Mac DW1000RangingClass::_globalMac;

//module type (anchor or tag)
int16_t DW1000RangingClass::_type; // TODO enum??

// message flow state
volatile byte DW1000RangingClass::_expectedMsgId;

//kalman filter
KalmanFilter DW1000RangingClass::kalmanfilter;

// range filter
volatile boolean DW1000RangingClass::_useRangeFilter = false;
uint16_t DW1000RangingClass::_rangeFilterValue = 15;

// message sent/received state
volatile boolean DW1000RangingClass::_sentAck = false;
volatile boolean DW1000RangingClass::_receivedAck = false;

// protocol error state
boolean DW1000RangingClass::_protocolFailed = false;

// timestamps to remember
int32_t DW1000RangingClass::timer = 0;
int16_t DW1000RangingClass::counterForBlink = 0; // TODO 8 bit?
int16_t DW1000RangingClass::counterForSetup = 0; // TODO 8 bit?

// data buffer
byte DW1000RangingClass::data[LEN_DATA];
// reset line to the chip
uint8_t DW1000RangingClass::_RST;
uint8_t DW1000RangingClass::_SS;
// watchdog and reset period
uint32_t DW1000RangingClass::_lastActivity;
uint32_t DW1000RangingClass::_resetPeriod;
// reply times (same on both sides for symm. ranging)
uint16_t DW1000RangingClass::_replyDelayTimeUS;
//timer delay
uint16_t DW1000RangingClass::_timerDelay;
// ranging counter (per second)
uint16_t DW1000RangingClass::_successRangingCount = 0;
uint32_t DW1000RangingClass::_rangingCountPeriod = 0;
//Here our handlers
void (*DW1000RangingClass::_handleNewRange)(void) = 0;
void (*DW1000RangingClass::_handleBlinkDevice)(DW1000Device *) = 0;
void (*DW1000RangingClass::_handleNewDevice)(DW1000Device *) = 0;
void (*DW1000RangingClass::_handleInactiveDevice)(DW1000Device *) = 0;
// setup phase
byte DW1000RangingClass::_firstTanchorShortAddress[2];
byte DW1000RangingClass::_secondTanchorShortAddress[2];
float DW1000RangingClass::_distanceAB = 0;
float DW1000RangingClass::_distanceAC = 0;
float DW1000RangingClass::_distanceBC = 0;
float DW1000RangingClass::_x2 = 0;
float DW1000RangingClass::_y2 = 0;
float DW1000RangingClass::_XT = 2;
float DW1000RangingClass::_YT = 0;
float DW1000RangingClass::_angle = 0;
uint8_t DW1000RangingClass::_state = STATE_IDLE;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

void DW1000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ)
{
	// reset line to the chip
	_RST = myRST;
	_SS = mySS;
	_resetPeriod = DEFAULT_RESET_PERIOD;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;
	//we set our timer delay
	_timerDelay = DEFAULT_TIMER_DELAY;

	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}

void DW1000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[])
{
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.enableMode(mode);
	DW1000.commitConfiguration();
}

void DW1000RangingClass::generalStart()
{
	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message

	kalmanfilter.setState( 0 );

	if (DEBUG)
	{
		// DEBUG monitoring
		Serial.println("DW1000-arduino");
		// initialize the driver

		Serial.println("configuration..");
		// DEBUG chip info and registers pretty printed
		char msg[90];
		DW1000.getPrintableDeviceIdentifier(msg);
		Serial.print("Device ID: ");
		Serial.println(msg);
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		Serial.print("Unique ID: ");
		Serial.print(msg);
		char string[6];
		sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
		Serial.print(" short: ");
		Serial.println(string);

		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		Serial.print("Network ID & Device Address: ");
		Serial.println(msg);
		DW1000.getPrintableDeviceMode(msg);
		Serial.print("Device mode: ");
		Serial.println(msg);
	}

	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
	// for first time ranging frequency computation
	_rangingCountPeriod = millis();
}

void DW1000RangingClass::startAsAnchor(char address[], const byte mode[], const bool randomShortAddress)
{
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	if (randomShortAddress)
	{
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else
	{
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0] * 256 + _currentShortAddress[1], 0xDECA, mode);

	//general start:
	generalStart();

	//defined type as anchor
	_type = ANCHOR;

	Serial.println("### ANCHOR ###");
}

void DW1000RangingClass::startAsManchor(char address[], const byte mode[], const bool randomShortAddress)
{
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	if (randomShortAddress)
	{
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else
	{
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0] * 256 + _currentShortAddress[1], 0xDECA, mode);

	//general start:
	generalStart();

	if (EEPROM_readFloat(CALIBRATION_FLAG) == 1) {
		_distanceAB = EEPROM_readFloat(EEPROM_AB);
		_x2 = EEPROM_readFloat(EEPROM_CX);
		_y2 = EEPROM_readFloat(EEPROM_CY);
		_firstTanchorShortAddress[0] = EEPROM.read(EEPROM_ADDRESS_B);
		_firstTanchorShortAddress[1] = EEPROM.read(EEPROM_ADDRESS_B+1);
		_secondTanchorShortAddress[0] = EEPROM.read(EEPROM_ADDRESS_C);
		_secondTanchorShortAddress[1] = EEPROM.read(EEPROM_ADDRESS_C+1);
	}

	//defined type as anchor
	_type = MANCHOR;

	Serial.println("### MAIN ANCHOR ###");
}

void DW1000RangingClass::startAsTag(char address[], const byte mode[], const bool randomShortAddress)
{
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	if (randomShortAddress)
	{
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else
	{
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0] * 256 + _currentShortAddress[1], 0xDECA, mode);

	generalStart();
	//defined type as tag
	_type = TAG;

	Serial.println("### TAG ###");
}

void DW1000RangingClass::startAsTanchor(char address[], const byte mode[], const bool randomShortAddress)
{
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	if (randomShortAddress)
	{
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else
	{
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0] * 256 + _currentShortAddress[1], 0xDECA, mode);

	generalStart();
	//defined type as tanchor (anchor that starts as tag)
	_type = TANCHOR;

	Serial.println("### ANCHOR (TEMPORARY TAG) ###");
}

boolean DW1000RangingClass::addNetworkDevices(DW1000Device *device, boolean shortAddress)
{
	boolean addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		if (_networkDevices[i].isAddressEqual(device) && !shortAddress)
		{
			//the device already exists
			addDevice = false;
			return false;
		}
		else if (_networkDevices[i].isShortAddressEqual(device) && shortAddress)
		{
			//the device already exists
			addDevice = false;
			return false;
		}
	}

	if (addDevice)
	{
		device->setRange(0);
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}

	return false;
}

boolean DW1000RangingClass::addNetworkDevices(DW1000Device *device)
{
	boolean addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		if (_networkDevices[i].isAddressEqual(device) && _networkDevices[i].isShortAddressEqual(device))
		{
			//the device already exists
			addDevice = false;
			return false;
		}
	}

	if (addDevice)
	{
		if (_type == ANCHOR || _type == MANCHOR) //for now let's start with 1 TAG
		{
			_networkDevicesNumber = 0;
		}
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}

	return false;
}

void DW1000RangingClass::removeNetworkDevices(int16_t index)
{
	//if we have just 1 element
	if (_networkDevicesNumber == 1)
	{
		_networkDevicesNumber = 0;
		if (_state == STATE_CALIB1)
			_state = STATE_READY1;
		else if (_state == STATE_CALIB2)
			_state = STATE_READY2;
	}
	else if (index == _networkDevicesNumber - 1) //if we delete the last element
	{
		_networkDevicesNumber--;
	}
	else
	{
		//we translate all the element wich are after the one we want to delete.
		for (int16_t i = index; i < _networkDevicesNumber - 1; i++)
		{ // TODO 8bit?
			memcpy(&_networkDevices[i], &_networkDevices[i + 1], sizeof(DW1000Device));
			_networkDevices[i].setIndex(i);
		}
		_networkDevicesNumber--;
	}
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */

//setters
void DW1000RangingClass::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }

void DW1000RangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }

DW1000Device *DW1000RangingClass::searchDistantDevice(byte shortAddress[])
{
	//we compare the 2 bytes address with the others
	for (uint16_t i = 0; i < _networkDevicesNumber; i++)
	{ // TODO 8bit?
		if (memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0)
		{
			//we have found our device !
			return &_networkDevices[i];
		}
	}

	return nullptr;
}

DW1000Device *DW1000RangingClass::getDistantDevice()
{
	//we get the device which correspond to the message which was sent (need to be filtered by MAC address)

	return &_networkDevices[_lastDistantDevice];
}

/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

void DW1000RangingClass::checkForReset()
{
	uint32_t curMillis = millis();
	if (!_sentAck && !_receivedAck)
	{
		// check if inactive
		if (curMillis - _lastActivity > _resetPeriod)
		{
			resetInactive();
		}
		return; // TODO cc
	}
}

void DW1000RangingClass::checkForInactiveDevices()
{
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		if (_networkDevices[i].isInactive())
		{
			if (_handleInactiveDevice != 0)
			{
				(*_handleInactiveDevice)(&_networkDevices[i]);
			}
			//we need to delete the device from the array:
			removeNetworkDevices(i);
		}
	}
}

// TODO check return type
int16_t DW1000RangingClass::detectMessageType(byte datas[])
{
	if (datas[0] == FC_1_BLINK)
	{
		return BLINK;
	}
	else if (datas[0] == FC_1 && datas[1] == FC_2)
	{
		//we have a long MAC frame message (ranging init)
		return datas[LONG_MAC_LEN];
	}
	else if (datas[0] == FC_1 && datas[1] == FC_2_SHORT)
	{
		//we have a short mac frame message (poll, range, range report, etc..)
		return datas[SHORT_MAC_LEN];
	}
}

void DW1000RangingClass::loop()
{
	//we check if needed to reset !
	checkForReset();
	uint32_t time = millis(); // TODO other name - too close to "timer"
	if (time - timer > _timerDelay)
	{
		timer = time;
		timerTick();
	}

	if (_sentAck)
	{
		_sentAck = false;

		// TODO cc
		int messageType = detectMessageType(data);

		if (messageType != POLL_ACK && messageType != POLL && messageType != RANGE)
			return;

		//A msg was sent. We launch the ranging protocole when a message was sent
		if (_type == ANCHOR || _type == MANCHOR)
		{
			if (messageType == POLL_ACK)
			{
				DW1000Device *myDistantDevice = searchDistantDevice(_lastSentToShortAddress);

				if (myDistantDevice)
				{
					DW1000.getTransmitTimestamp(myDistantDevice->timePollAckSent);
				}
			}
		}
		else if (_type == TAG || _type == TANCHOR)
		{
			if (messageType == POLL)
			{
				DW1000Time timePollSent;
				DW1000.getTransmitTimestamp(timePollSent);
				//if the last device we send the POLL is broadcast:
				if (_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF)
				{
					//we save the value for all the devices !
					for (uint16_t i = 0; i < _networkDevicesNumber; i++)
					{
						_networkDevices[i].timePollSent = timePollSent;
					}
				}
				else
				{
					//we search the device associated with the last send address
					DW1000Device *myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					if (myDistantDevice)
					{
						myDistantDevice->timePollSent = timePollSent;
					}
				}
			}
			else if (messageType == RANGE)
			{
				DW1000Time timeRangeSent;
				DW1000.getTransmitTimestamp(timeRangeSent);
				//if the last device we send the POLL is broadcast:
				if (_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF)
				{
					//we save the value for all the devices !
					for (uint16_t i = 0; i < _networkDevicesNumber; i++)
					{
						_networkDevices[i].timeRangeSent = timeRangeSent;
					}
				}
				else
				{
					//we search the device associated with the last send address
					DW1000Device *myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					if (myDistantDevice)
					{
						myDistantDevice->timeRangeSent = timeRangeSent;
					}
				}
			}
		}
	}

	//check for new received message
	if (_receivedAck)
	{
		_receivedAck = false;

		//we read the datas from the modules:
		// get message and parse
		DW1000.getData(data, LEN_DATA);

		int messageType = detectMessageType(data);

		//we have just received a BLINK message from tag
		if (messageType == BLINK && (_type == ANCHOR || _type == MANCHOR))
		{
			byte address[8];
			byte shortAddress[2];
			_globalMac.decodeBlinkFrame(data, address, shortAddress);
			//we crate a new device with the tag
			DW1000Device myTag(address, shortAddress);

			if (addNetworkDevices(&myTag))
			{
				if (_handleBlinkDevice != 0)
				{
					(*_handleBlinkDevice)(&myTag);
				}
				//we reply by the transmit ranging init message
				transmitRangingInit(&myTag);
				noteActivity();
			}
			_expectedMsgId = POLL;
		}
		else if (messageType == RANGING_INIT && (_type == TAG || _type == TANCHOR))
		{

			byte address[2];
			_globalMac.decodeLongMACFrame(data, address);
			//we crate a new device with the anchor
			DW1000Device myAnchor(address, true);

			if (addNetworkDevices(&myAnchor, true))
			{
				if (_handleNewDevice != 0)
				{
					(*_handleNewDevice)(&myAnchor);
				}
			}

			noteActivity();
		}
		else
		{
			//we have a short mac layer frame !
			byte address[2];
			_globalMac.decodeShortMACFrame(data, address);

			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			DW1000Device *myDistantDevice = searchDistantDevice(address);

			if ((_networkDevicesNumber == 0) || (myDistantDevice == nullptr))
			{
				//we don't have the short address of the device in memory
				if (DEBUG)
				{
					Serial.println("Not found");
					/*
					Serial.print("unknown: ");
					Serial.print(address[0], HEX);
					Serial.print(":");
					Serial.println(address[1], HEX);
					*/
				}
				return;
			}

			//then we proceed to range protocole
			if (_type == ANCHOR || _type == MANCHOR)
			{
				if (messageType != _expectedMsgId)
				{
					// unexpected message, start over again (except if already POLL)
					_protocolFailed = true;
				}
				if (messageType == POLL)
				{
					//we receive a POLL which is a broacast message
					//we need to grab info about it
					float distanceAT, distanceBT, distanceCT;	// distances between main anchor/tanchors and tag
					float AC, BC;
					uint16_t numberDevices = 0;
					memcpy(&numberDevices, data + SHORT_MAC_LEN + 1, 1);

					for (uint16_t i = 0; i < numberDevices; i++)
					{
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data + SHORT_MAC_LEN + 2 + 8 * i, 2);

						float range = 0;
						memcpy(&range, data + SHORT_MAC_LEN + 2 + 2 + 2 + i * 8, 4);

						// Serial.print("Dirección: ");
						// Serial.print(shortAddress[1] * 256 + shortAddress[0], HEX);
						// Serial.print("\t Rango: ");
						// Serial.print(range);
						// Serial.println(" m");

						//we test if the short address is our address
						if (shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1])
						{
							//we grab the replytime wich is for us
							uint16_t replyTime;
							memcpy(&replyTime, data + SHORT_MAC_LEN + 2 + 2 + i * 8, 2);
							//we configure our replyTime;
							_replyDelayTimeUS = replyTime;

							// on POLL we (re-)start, so no protocol failure
							_protocolFailed = false;

							DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
							//we note activity for our device:
							myDistantDevice->noteActivity();
							//we indicate our next receive message for our ranging protocole
							_expectedMsgId = RANGE;
							transmitPollAck(myDistantDevice);
							noteActivity();
						}

						// If it is the main anchor, do the setup phase calculations
						if (_type == MANCHOR) {
							//If there is only 1 device, we are on the AB setup phase (main anchor + first tanchor)
							if (numberDevices == 1) {
								if (!_firstTanchorShortAddress[0] && !_firstTanchorShortAddress[1]) {
									_firstTanchorShortAddress[0] = myDistantDevice->getByteShortAddress()[0];
									_firstTanchorShortAddress[1] = myDistantDevice->getByteShortAddress()[1];
								}
								if (_firstTanchorShortAddress[0] == myDistantDevice->getByteShortAddress()[0]
								&& _firstTanchorShortAddress[1] == myDistantDevice->getByteShortAddress()[1]) {
									_distanceAB += range/SETUP_ROUNDS;
								}
								Serial.print("AB:");Serial.print(_distanceAB);Serial.print("\t");
								Serial.print("AC:");Serial.print(_distanceAC);Serial.print("\t");
								Serial.print("BC:");Serial.println(_distanceBC);
								Serial.print("AT:");Serial.print(distanceAT);Serial.print("\t");
								Serial.print("BT:");Serial.print(distanceBT);Serial.print("\t");
								Serial.print("CT:");Serial.println(distanceCT);

								//State handling
								if(_state == STATE_IDLE)
									_state = STATE_CALIB1;
								else if (_state == STATE_TRILAT)
									_state = STATE_ERROR;
							//If there are 2 devices, we are on the ABC setup phase (main anchor + both tanchors)
							} else if (numberDevices == 2) {
								if (!_secondTanchorShortAddress[0] && !_secondTanchorShortAddress[1]) {
									_secondTanchorShortAddress[0] = myDistantDevice->getByteShortAddress()[0];
									_secondTanchorShortAddress[1] = myDistantDevice->getByteShortAddress()[1];
								}
								if (_secondTanchorShortAddress[0] == myDistantDevice->getByteShortAddress()[0]
								&& _secondTanchorShortAddress[1] == myDistantDevice->getByteShortAddress()[1]) {
									if (shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
										AC = range;
									} else if (shortAddress[0] == _firstTanchorShortAddress[0] && shortAddress[1] == _firstTanchorShortAddress[1]) {
										BC = range;
									}
									if (i == numberDevices-1) {
										if ( AC && BC ) {
											counterForSetup++;
											_distanceAC = (_distanceAC * (counterForSetup-1) + AC)/counterForSetup;
											_distanceBC = (_distanceBC * (counterForSetup-1) + BC)/counterForSetup;
										}
										_x2 = (_distanceAB*_distanceAB+_distanceAC*_distanceAC-_distanceBC*_distanceBC)/_distanceAB/2;
										_y2 = sqrt(_distanceAC*_distanceAC-_x2*_x2);
									}
								}
								Serial.print("newAC:");Serial.print(AC);Serial.print("\t");
								Serial.print("newBC:");Serial.print(BC);Serial.print("\t");
								Serial.print("AB:");Serial.print(_distanceAB);Serial.print("\t");
								Serial.print("AC:");Serial.print(_distanceAC);Serial.print("\t");
								Serial.print("BC:");Serial.println(_distanceBC);

								//State handling
								if(_state == STATE_READY1)
									_state = STATE_CALIB2;
								else if (_state == STATE_TRILAT)
									_state = STATE_ERROR;
							}
							//If there are 3 devices, we are on the ABCT trillateration phase (3 anchors + tag)
							else if (numberDevices == 3) {
								//No nos importa nada de lo calculado porque la calibracion ya está hecha
								if (EEPROM_readFloat(CALIBRATION_FLAG) == 1) {
									_distanceAB = EEPROM_readFloat(EEPROM_AB);
									_x2 = EEPROM_readFloat(EEPROM_CX);
									_y2 = EEPROM_readFloat(EEPROM_CY);
									_firstTanchorShortAddress[0] = EEPROM.read(EEPROM_ADDRESS_B);
									_firstTanchorShortAddress[1] = EEPROM.read(EEPROM_ADDRESS_B+1);
									_secondTanchorShortAddress[0] = EEPROM.read(EEPROM_ADDRESS_C);
									_secondTanchorShortAddress[1] = EEPROM.read(EEPROM_ADDRESS_C+1);
								} else if (EEPROM_readFloat(CALIBRATION_FLAG) == 0) {
									EEPROM_writeFloat(EEPROM_AB, _distanceAB);
									EEPROM_writeFloat(EEPROM_CX, _x2);
									EEPROM_writeFloat(EEPROM_CY, _y2);
									EEPROM.write(EEPROM_ADDRESS_B, _firstTanchorShortAddress[0]);
									EEPROM.write(EEPROM_ADDRESS_B + 1, _firstTanchorShortAddress[1]);
									EEPROM.write(EEPROM_ADDRESS_C, _secondTanchorShortAddress[0]);
									EEPROM.write(EEPROM_ADDRESS_C + 1, _secondTanchorShortAddress[1]);
									EEPROM_writeFloat(CALIBRATION_FLAG, 1);
								}
								
								if (shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
									distanceAT = range;
								} else if (shortAddress[0] == _firstTanchorShortAddress[0] && shortAddress[1] == _firstTanchorShortAddress[1]) {
									distanceBT = range;
								} else {
									distanceCT = range;
								}

								//State handling
								if(_state == STATE_READY2)
									_state = STATE_TRILAT;
							}
						}
					}
					if (_type == MANCHOR && numberDevices == 3) {
						trillaterate(distanceAT, distanceBT, distanceCT);
					}
					return;
				}
				else if (messageType == RANGE)
				{
					//we receive a RANGE which is a broacast message
					//we need to grab info about it
					uint8_t numberDevices = 0;
					memcpy(&numberDevices, data + SHORT_MAC_LEN + 1, 1);

					for (uint8_t i = 0; i < numberDevices; i++)
					{
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data + SHORT_MAC_LEN + 2 + i * 17, 2);

						//we test if the short address is our address
						if (shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1])
						{
							//we grab the replytime wich is for us
							DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
							noteActivity();
							_expectedMsgId = POLL;

							if (!_protocolFailed)
							{
								myDistantDevice->timePollSent.setTimestamp(data + SHORT_MAC_LEN + 4 + 17 * i);
								myDistantDevice->timePollAckReceived.setTimestamp(data + SHORT_MAC_LEN + 9 + 17 * i);
								myDistantDevice->timeRangeSent.setTimestamp(data + SHORT_MAC_LEN + 14 + 17 * i);

								// (re-)compute range as two-way ranging is done
								DW1000Time myTOF;
								computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM

								float distance = myTOF.getAsMeters();
								float pRx = myDistantDevice->getRXPower();
								if (pRx > -86) {
									distance = (distance-0.1291)/1.0535;		// linea directa
								} else {
									distance = (distance-0.6876)/1.1023;		// tapando con el pecho
								}
								kalmanfilter.correct( distance );
								distance = kalmanfilter.getState();
								//Serial.println();
								//Serial.print(distance);Serial.print(" ");
								//Serial.println(distance);

								if (_useRangeFilter)
								{
									//Skip first range
									if (myDistantDevice->getRange() != 0.0f)
									{
										distance = filterValue(distance, myDistantDevice->getRange(), _rangeFilterValue);
									}
								}

								myDistantDevice->setRXPower(DW1000.getReceivePower());
								myDistantDevice->setRange(distance);

								myDistantDevice->setFPPower(DW1000.getFirstPathPower());
								myDistantDevice->setQuality(DW1000.getReceiveQuality());

								//we send the range to TAG
								transmitRangeReport(myDistantDevice);

								//we have finished our range computation. We send the corresponding handler
								_lastDistantDevice = myDistantDevice->getIndex();
								if (_handleNewRange != 0)
								{
									(*_handleNewRange)();
								}
							}
							else
							{
								transmitRangeFailed(myDistantDevice);
							}

							return;
						}
					}
				}
			}
			else if (_type == TAG || _type == TANCHOR)
			{
				// get message and parse
				if (messageType != _expectedMsgId)
				{
					// unexpected message, start over again
					//not needed ?
					return;
					_expectedMsgId = POLL_ACK;
					return;
				}
				if (messageType == POLL_ACK)
				{
					DW1000.getReceiveTimestamp(myDistantDevice->timePollAckReceived);
					//we note activity for our device:
					myDistantDevice->noteActivity();

					//in the case the message come from our last device:
					if (myDistantDevice->getIndex() == _networkDevicesNumber - 1)
					{
						_expectedMsgId = RANGE_REPORT;
						//and transmit the next message (range) of the ranging protocole (in broadcast)
						transmitRange(nullptr);
					}
				}
				else if (messageType == RANGE_REPORT)
				{

					float curRange;
					memcpy(&curRange, data + 1 + SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, data + 5 + SHORT_MAC_LEN, 4);

					if (_useRangeFilter)
					{
						//Skip first range
						if (myDistantDevice->getRange() != 0.0f)
						{
							curRange = filterValue(curRange, myDistantDevice->getRange(), _rangeFilterValue);
						}
					}

					//we have a new range to save !
					myDistantDevice->setRange(curRange);
					myDistantDevice->setRXPower(curRXPower);

					//setup phase
					if (_type == TANCHOR) {
						counterForSetup++;
						if (counterForSetup == SETUP_ROUNDS) {
							_type = ANCHOR;
							Serial.println("Tag (tanchor) to anchor transition");
						}
					}

					//We can call our handler !
					//we have finished our range computation. We send the corresponding handler
					_lastDistantDevice = myDistantDevice->getIndex();
					if (_handleNewRange != 0)
					{
						(*_handleNewRange)();
					}
				}
				else if (messageType == RANGE_FAILED)
				{
					//not needed as we have a timer;
					return;
					_expectedMsgId = POLL_ACK;
				}
			}
		}
	}
}

void DW1000RangingClass::trillaterate(float AT, float BT, float CT) {
	float C1 = -AT*AT;
	float A2 = -2*_distanceAB;
	float C2 = _distanceAB*_distanceAB-BT*BT;
	float A3 = -2*_x2;
	float B3 = -2*_y2;
	float C3 = _x2*_x2+_y2*_y2-CT*CT;
	float newAngle = 0;

	float XT = (C1-C2)/A2;
	float YT = ((C2-C3)*A2-(C2-C1)*(A2-A3))/(A2*B3);


	_XT = XT;
	_YT = YT;

	
   	newAngle = float(180*atan2(YT,XT)/PI);

	if ( abs(_angle - newAngle) > 3) {
		_angle = newAngle;
	}
	// Serial.print("BX:"); Serial.print(_distanceAB); Serial.print(",");
	// Serial.print("CX:"); Serial.print(_x2); Serial.print(",");
	// Serial.print("CY:"); Serial.print(_y2); Serial.print(",");
	// Serial.print("AT:"); Serial.print(AT); Serial.print(",");
	// Serial.print("BT:"); Serial.print(BT); Serial.print(",");
	// Serial.print("CT:"); Serial.print(CT); Serial.print(",");
	// Serial.print("XT:"); Serial.print(XT); Serial.print(",");
	// Serial.print("YT:"); Serial.print(YT); Serial.print(",");
	// Serial.print("angle:"); Serial.println(_angle);


}

float DW1000RangingClass::getXT()
{
	return _XT;
}

float DW1000RangingClass::getYT()
{
	return _YT;
}

uint8_t DW1000RangingClass::getState()
{
	return _state;
}

void DW1000RangingClass::useRangeFilter(boolean enabled)
{
	_useRangeFilter = enabled;
}

void DW1000RangingClass::setRangeFilterValue(uint16_t newValue)
{
	if (newValue < 2)
	{
		_rangeFilterValue = 2;
	}
	else
	{
		_rangeFilterValue = newValue;
	}
}

/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */

void DW1000RangingClass::handleSent()
{
	// status change on sent success
	_sentAck = true;
}

void DW1000RangingClass::handleReceived()
{
	// status change on received success
	_receivedAck = true;
}

void DW1000RangingClass::noteActivity()
{
	// update activity timestamp, so that we do not reach "resetPeriod"
	_lastActivity = millis();
}

void DW1000RangingClass::resetInactive()
{
	//if inactive
	if (_type == ANCHOR || _type == MANCHOR)
	{
		_expectedMsgId = POLL;
		receiver();
	}
	noteActivity();
}

void DW1000RangingClass::timerTick()
{
	if (_networkDevicesNumber > 0 && counterForBlink != 0)
	{
		if (_type == TAG || _type == TANCHOR)
		{
			_expectedMsgId = POLL_ACK;
			//send a prodcast poll
			transmitPoll(nullptr);
		}
	}
	else if (counterForBlink == 0)
	{
		if (_type == TAG || _type == TANCHOR)
		{
			transmitBlink();
		}
		//check for inactive devices if we are a TAG or ANCHOR
		checkForInactiveDevices();
	}
	counterForBlink++;
	if (counterForBlink > 20)
	{
		counterForBlink = 0;
	}
}

void DW1000RangingClass::copyShortAddress(byte address1[], byte address2[])
{
	*address1 = *address2;
	*(address1 + 1) = *(address2 + 1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

void DW1000RangingClass::transmitInit()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
}

void DW1000RangingClass::transmit(byte datas[])
{
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000RangingClass::transmit(byte datas[], DW1000Time time)
{
	DW1000.setDelay(time);
	DW1000.setData(data, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000RangingClass::transmitBlink()
{
	transmitInit();
	_globalMac.generateBlinkFrame(data, _currentAddress, _currentShortAddress);
	transmit(data);
}

void DW1000RangingClass::transmitRangingInit(DW1000Device *myDistantDevice)
{
	transmitInit();
	//we generate the mac frame for a ranging init message
	_globalMac.generateLongMACFrame(data, _currentShortAddress, myDistantDevice->getByteAddress());
	//we define the function code
	data[LONG_MAC_LEN] = RANGING_INIT;

	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());

	transmit(data);
}

void DW1000RangingClass::transmitPoll(DW1000Device *myDistantDevice)
{

	transmitInit();

	if (myDistantDevice == nullptr)
	{
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY + (uint16_t)(_networkDevicesNumber * 3 * DEFAULT_REPLY_DELAY_TIME / 1000);

		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN] = POLL;
		//we enter the number of devices
		data[SHORT_MAC_LEN + 1] = _networkDevicesNumber;

		for (uint8_t i = 0; i < _networkDevicesNumber; i++)
		{
			//each devices have a different reply delay time.
			_networkDevices[i].setReplyTime((2 * i + 1) * DEFAULT_REPLY_DELAY_TIME);
			//we write the short address of our device:
			memcpy(data + SHORT_MAC_LEN + 2 + 8 * i, _networkDevices[i].getByteShortAddress(), 2);

			//we add the replyTime
			uint16_t replyTime = _networkDevices[i].getReplyTime();
			memcpy(data + SHORT_MAC_LEN + 2 + 2 + 8 * i, &replyTime, 2);

			//agregamos la distancia
			float range = _networkDevices[i].getRange();
			memcpy(data + SHORT_MAC_LEN + 2 + 2 + 2 + 8 * i, &range, 4);
		}

		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
	}
	else
	{
		//we redefine our default_timer_delay for just 1 device;
		_timerDelay = DEFAULT_TIMER_DELAY;

		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());

		data[SHORT_MAC_LEN] = POLL;
		data[SHORT_MAC_LEN + 1] = 1;
		uint16_t replyTime = myDistantDevice->getReplyTime();
		memcpy(data + SHORT_MAC_LEN + 2, &replyTime, sizeof(uint16_t)); // todo is code correct?

		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}

	transmit(data);
}

void DW1000RangingClass::transmitPollAck(DW1000Device *myDistantDevice)
{
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = POLL_ACK;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, deltaTime);
}

void DW1000RangingClass::transmitRange(DW1000Device *myDistantDevice)
{
	//transmit range need to accept broadcast for multiple anchor
	transmitInit();

	if (myDistantDevice == nullptr)
	{
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY + (uint16_t)(_networkDevicesNumber * 3 * DEFAULT_REPLY_DELAY_TIME / 1000);

		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN] = RANGE;
		//we enter the number of devices
		data[SHORT_MAC_LEN + 1] = _networkDevicesNumber;

		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
		DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);

		for (uint8_t i = 0; i < _networkDevicesNumber; i++)
		{
			//we write the short address of our device:
			memcpy(data + SHORT_MAC_LEN + 2 + 17 * i, _networkDevices[i].getByteShortAddress(), 2);

			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			_networkDevices[i].timeRangeSent = timeRangeSent;
			_networkDevices[i].timePollSent.getTimestamp(data + SHORT_MAC_LEN + 4 + 17 * i);
			_networkDevices[i].timePollAckReceived.getTimestamp(data + SHORT_MAC_LEN + 9 + 17 * i);
			_networkDevices[i].timeRangeSent.getTimestamp(data + SHORT_MAC_LEN + 14 + 17 * i);
		}

		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
	}
	else
	{
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		data[SHORT_MAC_LEN] = RANGE;
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
		//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
		myDistantDevice->timeRangeSent = DW1000.setDelay(deltaTime);
		myDistantDevice->timePollSent.getTimestamp(data + 1 + SHORT_MAC_LEN);
		myDistantDevice->timePollAckReceived.getTimestamp(data + 6 + SHORT_MAC_LEN);
		myDistantDevice->timeRangeSent.getTimestamp(data + 11 + SHORT_MAC_LEN);
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}

	transmit(data);
}

void DW1000RangingClass::transmitRangeReport(DW1000Device *myDistantDevice)
{
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_REPORT;
	// write final ranging result
	float curRange = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	//We add the Range and then the RXPower
	memcpy(data + 1 + SHORT_MAC_LEN, &curRange, 4);
	memcpy(data + 5 + SHORT_MAC_LEN, &curRXPower, 4);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS));
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device *myDistantDevice)
{
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_FAILED;

	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data);
}

void DW1000RangingClass::receiver()
{
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}

/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */

void DW1000RangingClass::computeRangeAsymmetric(DW1000Device *myDistantDevice, DW1000Time *myTOF)
{
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceived - myDistantDevice->timePollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent - myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived - myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSent - myDistantDevice->timePollAckReceived).wrap();

	myTOF->setTimestamp((round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2));
	/*
	Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
	Serial.print("timePollSent ");myDistantDevice->timePollSent.print();
	Serial.print("round1 "); Serial.println((long)round1.getTimestamp());
	
	Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
	Serial.print("timePollReceived ");myDistantDevice->timePollReceived.print();
	Serial.print("reply1 "); Serial.println((long)reply1.getTimestamp());
	
	Serial.print("timeRangeReceived ");myDistantDevice->timeRangeReceived.print();
	Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
	Serial.print("round2 "); Serial.println((long)round2.getTimestamp());
	
	Serial.print("timeRangeSent ");myDistantDevice->timeRangeSent.print();
	Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
	Serial.print("reply2 "); Serial.println((long)reply2.getTimestamp());
	 */
}

/* FOR DEBUGGING*/
void DW1000RangingClass::visualizeDatas(byte datas[])
{
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
			datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7], datas[8], datas[9], datas[10], datas[11], datas[12], datas[13], datas[14], datas[15]);
	Serial.println(string);
}

/* ###########################################################################
 * #### Utils  ###############################################################
 * ######################################################################### */

float DW1000RangingClass::filterValue(float value, float previousValue, uint16_t numberOfElements)
{

	float k = 2.0f / ((float)numberOfElements + 1.0f);
	return (value * k) + previousValue * (1.0f - k);
}
