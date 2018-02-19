/*
#  SumpPumpAlert based on Moteino, HC-SR04 ultrasonic sensor and RaspberryPi
#
#  Copyright (c) 2013 Felix Rusu (felix@lowpowerlab.com).  All rights reserved.
#
#  This code is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This code is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
*/

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
//#include <SparkFunBME280.h> //get it here: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/tree/master/Libraries/Arduino/src
#include <Wire.h>     //comes with Arduino

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID        47    //unique for each node on same network
#define NETWORKID     200  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
//#define ENABLE_BME280 //uncomment to allow reading the BME280 (if present)
//*********************************************************************************************
#define ACK_TIME      30  // max # of ms to wait for an ack
#define ONBOARDLED     9  // Moteinos have LEDs on D9
#define LED            5  // MotionOLEDMote has an external LED on D5
#define MOTION_PIN     3  // D3
#define MOTION_IRQ     1  // hardware interrupt 1 (D3) - where motion sensors OUTput is connected, this will generate an interrupt every time there is MOTION
#define BATT_MONITOR  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_FORMULA(reading) reading * 0.00322 * 1.49 // >>> fine tune this parameter to match your voltage when fully charged
                                                       // details on how this works: https://lowpowerlab.com/forum/index.php/topic,1206.0.html
#define DUPLICATE_INTERVAL 20000 //avoid duplicates in 55second intervals (ie mailman sometimes spends 30+ seconds at mailbox)
#define BATT_INTERVAL  300000  // read and report battery voltage every this many ms (approx)  300,000 ms = 5 min

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

#ifdef ENABLE_BME280
  BME280 bme280;
#endif

#define ECHO 3
#define TRIG 4

float batteryVolts = 5;
char BATstr[10]; //longest battery voltage reading message = 9chars
char sendBuf[32];
byte sendLen;
#ifdef ENABLE_BME280
  float temperature=0;
  char Fstr[10];
#endif

void motionIRQ(void);
void checkBattery(void);

void setup() {
  Serial.begin(SERIAL_BAUD);
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

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(MOTION_PIN, INPUT);
  attachInterrupt(MOTION_IRQ, motionIRQ, RISING);

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buff);
  pinMode(ONBOARDLED, OUTPUT);
  pinMode(LED, OUTPUT);
  radio.sendWithRetry(GATEWAYID, "START", 5);
  
#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

#ifdef ENABLE_BME280
  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = 0x77;
  bme280.settings.runMode = 3; //Normal mode
  bme280.settings.tStandby = 0;
  bme280.settings.filter = 0;
  bme280.settings.tempOverSample = 1;
  bme280.settings.pressOverSample = 1;
  bme280.settings.humidOverSample = 1;
#endif
//  radio.Sleep();
}

void loop() {
  long distance = getDistance();
  byte sendLen;
  
  if (distance >= 400 || distance <= 0){
    Serial.println("Out of range");
    sprintf(sendBuf, "SUMP_LVL:BAD");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
    sprintf(sendBuf, "SUMP_LVL:%ld", distance);
  }
  Blink(LED, 50);
 
  sendLen = strlen(sendBuf);

    if (radio.sendWithRetry(GATEWAYID, sendBuf, sendLen))
    {
      DEBUG("MOTION ACK:OK! RSSI:");
      DEBUG(radio.RSSI);
//      batteryReportCycles = 0;
    }
    else DEBUG("MOTION ACK:NOK...");

    DEBUG(" VIN: ");
    DEBUGln(BATstr);

    radio.sleep();
    }

long getDistance() {
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME) {
    if (radio.ACKReceived(GATEWAYID))
      return true;
  }
  return false;
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

