#include <Adafruit_PN532.h>

// **********************************************************************************************************
// Moteino gateway/base sketch that works with Moteinos equipped with RFM69W/RFM69HW/RFM69CW/RFM69HCW
// This is a basic gateway sketch that receives packets from end node Moteinos, formats them as ASCII strings
//      with the end node [ID] and passes them to Pi/host computer via serial port
//     (ex: "messageFromNode" from node 123 gets passed to serial as "[123] messageFromNode")
// It also listens to serial messages that should be sent to listening end nodes
//     (ex: "123:messageToNode" sends "messageToNode" to node 123)
// Make sure to adjust the settings to match your transceiver settings (frequency, HW etc).
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
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
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE (www.arduino.cc)
#include <Adafruit_PN532.h>

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID          1 //the ID of this node
#define NETWORKID     200 //the network ID of all nodes this node listens/talks to
#define FREQUENCY     RF69_433MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "sampleEncryptKey" //identical 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ACK_TIME       30  // # of ms to wait for an ack packet
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75  //target RSSI for RFM69_ATC (recommended > -80)
//*****************************************************************************************************************************
// Serial baud rate must match your Pi/host computer serial port baud rate!
#define SERIAL_EN     //comment out if you don't want any serial verbose output
#define SERIAL_BAUD  115200
//*****************************************************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit Windbond FlashMEM chip

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif
  
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", radio.getFrequency()/1000000);
  DEBUGln(buff);

  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
  {
    DEBUGln("SPI FlashMEM not found (is chip onboard?)");
  }
}

byte ackCount=0;
byte inputLen=0;
char input[64];
byte buff[61];
String inputData;

typedef struct
{  
  int nodeId=22; //store this nodeId
  uint8_t uid[7];  // Buffer to store the returned UID
  uint8_t uidLength;  // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  uint8_t data[32];
  char BATstr[10]; //longest battery voltage reading message = 9chars
} rfidPayload;
rfidPayload theData;

void loop() {
  inputLen = readSerialLine(input);
  inputData = String(input);
  //inputstr.toUpperCase();
  
  if (inputLen > 0)
  {
    if (inputData.equals("KEY?"))
    {
      DEBUG("ENCRYPTKEY:");
      DEBUG(ENCRYPTKEY);
    }

    byte targetId = inputData.toInt(); //extract ID if any
    byte colonIndex = inputData.indexOf(":"); //find position of first colon

    if (targetId > 0)
    {
      inputData = inputData.substring(colonIndex+1); //trim "ID:" if any
    }

    if (targetId > 0 && targetId != NODEID && targetId != RF69_BROADCAST_ADDR && colonIndex>0 && colonIndex<4 && inputData.length()>0)
    {
      inputData.getBytes(buff, 61);
      //DEBUGln((char*)buff);
      //DEBUGln(targetId);
      //DEBUGln(colonIndex);
      if (radio.sendWithRetry(targetId, buff, inputData.length()))
      {
        DEBUGln("ACK:OK");
      }
      else
        DEBUGln("ACK:NOK");
    }
  }

  if (radio.receiveDone())
  {
    int rssi = radio.RSSI;
    DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
    
    switch (radio.SENDERID) 
    {
      case 22:
          theData = *(rfidPayload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
          Serial.println("Switch 22");
          Adafruit_PN532::PrintHex(theData.uid, theData.uidLength);
          //Serial.println(theData.nodeId, DEC);
          //Serial.print(" UID=");
          //Serial.print(theData.uid[0],HEX);
          //Serial.print(theData.uid[1],HEX);
          //Serial.print(theData.uid[2],HEX);
          //Serial.print(theData.uid[3],HEX);
          //Serial.print(theData.uid[4],HEX);
          //Serial.print(theData.uid[5],HEX);
          //Serial.print(theData.uid[6],HEX);

      break;

      default:
      if (radio.DATALEN > 0)
      {
       for (byte i = 0; i < radio.DATALEN; i++)
          DEBUG((char)radio.DATA[i]);
        DEBUG("   [RSSI:");DEBUG(rssi);DEBUG("]");
      }
    }
    

    CheckForWirelessHEX(radio, flash, false); //non verbose DEBUG

    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      DEBUG("[ACK-sent]");
    }
    DEBUGln();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
