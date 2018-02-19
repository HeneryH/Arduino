/**************************************************************************/
/*! 
    @file     readMifare.pde
    @author   Adafruit Industries
	@license  BSD (see license.txt)

    This example will wait for any ISO14443A card or tag, and
    depending on the size of the UID will attempt to read from it.
   
    If the card has a 4-byte UID it is probably a Mifare
    Classic card, and the following steps are taken:
   
    - Authenticate block 4 (the first block of Sector 1) using
      the default KEYA of 0XFF 0XFF 0XFF 0XFF 0XFF 0XFF
    - If authentication succeeds, we can then read any of the
      4 blocks in that sector (though only block 4 is read here)
	 
    If the card has a 7-byte UID it is probably a Mifare
    Ultralight card, and the 4 byte pages can be read directly.
    Page 4 is read by default since this is the first 'general-
    purpose' page on the tags.


This is an example sketch for the Adafruit PN532 NFC/RFID breakout boards
This library works with the Adafruit NFC breakout 
  ----> https://www.adafruit.com/products/364
 
Check out the links above for our tutorials and wiring diagrams 
These chips use SPI or I2C to communicate.

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

*/


// Sample RFM69 sender/node sketch for the MotionMote
// http://lowpowerlab.com/motionmote
// Event sensor connected to D3 (INT1)
// When FALL happens on D3, the sketch transmits a "Event" msg to receiver Moteino and goes back to sleep
// In sleep mode, Moteino + PIR motion sensor use about ~60uA
// IMPORTANT: adjust the settings in the configuration section below !!!

// **********************************************************************************
// Copyright Felix Rusu of LowPowerLab.com, 2016
// RFM69 library and sample code by Felix Rusu - lowpowerlab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************




/**************************************************************************/
#include <Adafruit_PN532.h>
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
//#include <SparkFunBME280.h> //get it here: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/tree/master/Libraries/Arduino/src
#include <Wire.h>     //comes with Arduino


// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (2)
#define PN532_MOSI (3)
#define PN532_SS   (4)
#define PN532_MISO (5)

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
//#define PN532_IRQ   (2)
//#define PN532_RESET (3)  // Not connected by default on the NFC Shield
// Moteino uses pin 2 for the transceiver, we need to use 3 instead.  Is I2C inherently interupt driven??
#define PN532_IRQ   (3)  // Note that 3~ (INT1) rather than 2 (INT0) if using interrupts
#define PN532_RESET (4)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Moteino uses SPI for transceiver, we need to use I2C for RFID instead.

// Use this line for a breakout with a software SPI connection (recommended):
// Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
// We need pull up resistors on tx & rx and need to connect x,y (3,4).
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
// also change #define in Adafruit_PN532.cpp library file
   #define Serial SerialUSB
#endif

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        22    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
int TRANSMITPERIOD = 300; //transmit a packet to gateway so often (in ms)
//*********************************************************************************************
#define ACK_TIME      30  // max # of ms to wait for an ack
#define ONBOARDLED     9  // Moteinos have LEDs on D9
//#define LED            5  // MotionOLEDMote has an external LED on D5

// The RFID seems to be using Interupts as well as the event pattern from Moteino.  Must be hidden in the library.  Or, is I2C inherently interrupt driven??
//#define EVENT_PIN     3  // D3
//#define EVENT_IRQ     1  // hardware interrupt 1 (D3) - where motion sensors OUTput is connected, this will generate an interrupt every time there is MOTION

#define BATT_MONITOR  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_FORMULA(reading) reading * 0.00322 * 1.49 // >>> fine tune this parameter to match your voltage when fully charged
                                                       // details on how this works: https://lowpowerlab.com/forum/index.php/topic,1206.0.html
#define DUPLICATE_INTERVAL 20000 //avoid duplicates in 55second intervals (ie mailman sometimes spends 30+ seconds at mailbox)
#define BATT_INTERVAL  300000  // read and report battery voltage every this many ms (approx)

#define SERIAL_EN             //comment this out when deploying to an installed Mote to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#define DEBUGFlush() { Serial.flush(); }
#else
#define DEBUG(input);
#define DEBUGln(input);
#define DEBUGFlush();
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

volatile boolean eventDetected=false;
float batteryVolts = 5;

void eventIRQ(void);
void checkBattery(char *BATstr);

// Let's get rid of this eventually
char sendBuf[32];
byte sendLen;

typedef struct
{	
  int nodeId=22; //store this nodeId
  uint8_t uid[7];  // Buffer to store the returned UID
  uint8_t uidLength;  // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  uint8_t data[32];
  char BATstr[10]; //longest battery voltage reading message = 9chars
} rfidPayload;
rfidPayload rfidData;

void setup(void) {
  #ifndef ESP8266
    while (!Serial); // for Leonardo/Micro/Zero
  #endif
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hello!");
#endif

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

// The RFID must be interrupt driven somewhere in the library or I2C code
//  pinMode(MOTION_PIN, INPUT);
//  attachInterrupt(MOTION_IRQ, motionIRQ, FALLING);

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
  pinMode(ONBOARDLED, OUTPUT);
  radio.sendWithRetry(GATEWAYID, "START", 5);

#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
#ifdef SERIAL_EN
   Serial.print("Didn't find PN53x board");
#endif
    while (1); // halt
  }

#ifdef SERIAL_EN
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
#endif
  
  // configure board to read RFID tags
  nfc.SAMConfig();

#ifdef SERIAL_EN
  Serial.println("Waiting for an ISO14443A Card ...");
#endif
}

// Should we move anything into the IRQ handler???
//void eventIRQ()
//{
//  eventDetected=true;
//  DEBUGln("IRQ");
//}

uint16_t batteryReportCycles=0;
uint32_t time=0, now=0, MLO=0, BLO=0;
byte eventRecentlyCycles=0;

long lastPeriod = -1;
void loop(void) {
  uint8_t success;

  //rfidData.uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    
  now = millis();
  checkBattery(rfidData.BATstr);

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, rfidData.uid, &rfidData.uidLength);
  
// In the Motenio model, there are event interrupts and periodic check-in interrupts.  Not sure if we can do that here.
  if (success) {

    digitalWrite(ONBOARDLED, HIGH);
    MLO = time; //save timestamp of event

//    sprintf(sendBuf, "EVENT BAT:%sv", rfidData.BATstr);
//
//    sendLen = strlen(sendBuf);
//
//    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
//    {
//      DEBUG("MOTION ACK:OK! RSSI:");
//      DEBUG(radio.RSSI);
//      batteryReportCycles = 0;
//    }
//    else DEBUG("MOTION ACK:NOK...");
//
//    DEBUG(" VIN: ");
//    DEBUGln(rfidData.BATstr);
//
//    radio.sleep();

    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(rfidData.uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(rfidData.uid, rfidData.uidLength);
    Serial.println("");

    //////////
  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    //fill in the struct with new values
    rfidData.nodeId = NODEID;
    //rfidData.uptime = millis();
    //rfidData.temp = 91.23; //it's hot!
    
    Serial.print("Sending struct (");
    Serial.print(sizeof(rfidData));
    Serial.print(" bytes) ... ");
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&rfidData), sizeof(rfidData)))
      Serial.print(" ok!");
    else Serial.print(" nothing...");
    Serial.println();
    Blink(ONBOARDLED,3);
    lastPeriod=currPeriod;
}    //////////
    if (rfidData.uidLength == 4)
    {
      // We probably have a Mifare Classic card ... 
      Serial.println("Seems to be a Mifare Classic card (4 byte UID)");
	  
      // Now we need to try to authenticate it for read/write access
      // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
      Serial.println("Trying to authenticate block 4 with default KEYA value");
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	  
	  // Start with block 4 (the first block of sector 1) since sector 0
	  // contains the manufacturer data and it's probably better just
	  // to leave it alone unless you know what you're doing
      success = nfc.mifareclassic_AuthenticateBlock(rfidData.uid, rfidData.uidLength, 4, 0, keya);
	  
      if (success)
      {
        Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
 		
        // If you want to write something to block 4 to test with, uncomment
	 // the following line and this text should be read back in a minute
        //memcpy(rfidData.data, (const uint8_t[]){ 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0 }, sizeof rfidData.data);
        // success = nfc.mifareclassic_WriteDataBlock (4, data);

        // Try to read the contents of block 4
        success = nfc.mifareclassic_ReadDataBlock(4, rfidData.data);
		
        if (success)
        {
          // Data seems to have been read ... spit it out
          Serial.println("Reading Block 4:");
          nfc.PrintHexChar(rfidData.data, 16);
          Serial.println("");
		  
          // Wait a bit before reading the card again
          delay(1000);
        }
        else
        {
          Serial.println("Ooops ... unable to read the requested block.  Try another key?");
        }
      }
      else
      {
        Serial.println("Ooops ... authentication failed: Try another key?");
      }
    }
    
    if (rfidData.uidLength == 7)
    {
      // We probably have a Mifare Ultralight card ...
      Serial.println("Seems to be a Mifare Ultralight tag (7 byte UID)");
	  
      // Try to read the first general-purpose user page (#4)
      Serial.println("Reading page 4");
      success = nfc.mifareultralight_ReadPage (4, rfidData.data);
      if (success)
      {
        // Data seems to have been read ... spit it out
        nfc.PrintHexChar(rfidData.data, 4);
        Serial.println("");
		
        // Wait a bit before reading the card again
        delay(1000);
      }
      else
      {
        Serial.println("Ooops ... unable to read the requested page!?");
      }
    }
    digitalWrite(ONBOARDLED, LOW);
  }
  else if (time-BLO > BATT_INTERVAL)  // This is the periodic check-in interrupt but not sure if that works here
  {
//    sprintf(sendBuf, "BAT:%sv", rfidData.BATstr);
//    
//    sendLen = strlen(sendBuf);
//    BLO = time;
//    radio.sendWithRetry(GATEWAYID, (const void*)(&rfidData), sizeof(rfidData));
//    radio.sleep();
//    batteryReportCycles=0;
  }

  DEBUGFlush();
}


uint32_t BLR=0;
void checkBattery(char *BATstr)
{
  if (time-BLR > 30000) //only read battery every 30s or so
  {
    unsigned int readings=0;
    BLR = time;
    for (byte i=0; i<10; i++) //take 10 samples, and average
      readings+=analogRead(BATT_MONITOR);
    batteryVolts = BATT_FORMULA(readings / 10.0);
    dtostrf(batteryVolts, 3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

