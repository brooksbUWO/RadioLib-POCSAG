// Include Files
// ****************************************************************************
#include <Arduino.h>
#include <SPI.h>							// SPI library
#include <RadioLib.h>						// Universal wireless communication library

#define LORA_SCK        5
#define LORA_MISO       19
#define LORA_MOSI       27
#define LORA_SS         18
#define LORA_DIO0       26
#define LORA_DIO1       33
#define LORA_DIO2       32
#define LORA_RST        23


// Globals
// ****************************************************************************
SX1276 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, LORA_DIO1);
PagerClient pager(&radio);					// Pager client instance using the FSK module

bool flagSetTimePager = false;				// Flag set time using pager
bool flagSetTimeTest = false;				// Flag for testing set time


// Begin Code
// ****************************************************************************
void setup()
{
  	// Initialize SX1276
  	Serial.print(F("[SX1276] Initializing ... "));
  	// carrier frequency:           462.45 MHz
  	// bit rate:                    1.2 kbps
  	// frequency deviation:         4.5 kHz
  	// Rx bandwidth:                12.5 kHz
  	// output power:                17 dBm
  	// preambleLength				576 bits
	// enableOOK					false = use FSK instead of OOK
  	int state = radio.beginFSK(462.45, 1.2, 4.5, 12.5, 17, 576, false);

  	if(state == 0)
  	{
	    Serial.println(F("success!"));
  	}
	else
	{
	    Serial.print(F("failed, code "));
    	Serial.println(state);
  	}

  	// initalize Pager client
  	Serial.print(F("[Pager] Initializing ... "));
  	// base (center) frequency:     462.45 MHz
  	// speed:                       512 bps
  	state = pager.begin(462.45, 512);
  	if(state == 0)
  	{
	    Serial.println(F("success!"));
  	}
	else
	{
    	Serial.print(F("failed, code "));
    	Serial.println(state);
	}

}


// Main program
// ****************************************************************************
void loop()
{
	if ( flagSetTimePager )
	{
		flagSetTimePager = false;

		// address = 0800828 or 0xC383C
		// command = 000001FF002WA202209021707025303944
		int state = pager.transmit(command, 0xC383C);
	}

	if ( flagSetTimeTest )
	{
		flagSetTimeTest = false;

		// Set time using POCSAG messaged captured from functional RS232 data radio modem
		uint32_t msg[] = {0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x5555553E, 0x09DD7891, 0x30F74530, 0xF74530F7, 0x4530F745, 0x30F74530, 0xF74530F7, 0x4530671C, 0xBF501856, 0x9C18EE99, 0x18809F18, 0x809B18B6, 0xE6D8E8CC, 0x1A0A7D09, 0xDD8430ED, 0x9876710D, 0x65C81F06, 0xB1D0B2D6, 0x4B8B382F, 0x6C8BB76D, 0xF35DD806, 0xA156B41A, 0x4D18B31A, 0x00364507, 0xBA4507BA, 0x4507BA45, 0x07FA};
		pager.testXmit(msg, sizeof(msg));
	}

}