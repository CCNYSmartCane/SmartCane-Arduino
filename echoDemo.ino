/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

const int buttonPin = 12;     // the number of the pushbutton pin
const int sensorPin = A0;    // select the input pin for the potentiometer

int buttonCurrent;           // the current reading from the input pin
int buttonPrevious = HIGH;    // the previous reading from the input pin

int sensorCurrent;  // variable to store the value coming from the sensor
int sensorPrevious = 508; // 508 is the inital state of the sensor of the joystick

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long t1 = 0;         // the last time the output pin was toggled
long t2 = 0;
long debounce = 200;   // the debounce time, increase if the output flickers


/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("NEWNAME"); /* 7 characters max! */

  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{ 
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get the floats and print it out
    while (BTLEserial.available()) {
      // Union to share the same memory location for byte -> float conversion
      union u_tag {
        byte b[4];
        float fval;
      } u;
      
      u.b[3] = BTLEserial.read();
      u.b[2] = BTLEserial.read();
      u.b[1] = BTLEserial.read();
      u.b[0] = BTLEserial.read();

      Serial.print(u.fval); Serial.print(" ");
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }

    // Handle button pressed
    buttonCurrent = digitalRead(buttonPin);

    // if the input just went from LOW and HIGH and we've waited long enough
    // to ignore any noise on the circuit, toggle the output pin and remember
    // the time
    if (buttonCurrent == HIGH && buttonPrevious == LOW && millis() - t1 > debounce) {
      Serial.println("Pressed button");
      t1 = millis();    
    }    
    buttonPrevious = buttonCurrent;

    // Handle sensor joystick
    sensorCurrent = analogRead(sensorPin);
  
    if (sensorCurrent == 508 && sensorPrevious != 508 && millis() - t2 > debounce) {
        if (sensorPrevious > 508) {
          Serial.println("Up");
        } else if (sensorPrevious < 508) {
          Serial.println("Down");
        }
        t2 = millis();    
    }    
    sensorPrevious = sensorCurrent;
  }


}
