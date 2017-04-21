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
#define ADAFRUITBLE_RDY 7     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// Vibration motor connected to digital pin 5 and 6
int LeftMotor = 5;
int RightMotor = 6;

/*************************************************************************************/
// Joystick Setup
const int JoystickPin = A0;    // select the input pin for the potentiometer
int JoystickY;  // variable to store the value coming from the sensor
int lastJoystickState = LOW; //LOW not press, HIGH Pressed

long JoystickDebounce = 500;   // the debounce time, increase if the output flickers
long pressedTime;
/*************************************************************************************/

/*************************************************************************************/
// Button Setup
const int buttonPin = A3;     // the number of the pushbutton pin
const int JoystickButtonPin = A2;

int buttonReading;           // the current reading from the input pin
int buttonState;              // the status of the button
int buttonPrevious = HIGH;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long buttonLastDebounceTime = 0;         // the last time the output pin was toggled
long buttonDebounce = 500;   // the debounce time, increase if the output flickers
/*************************************************************************************/

int YesConnected = false;
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  setupIMU();
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  Serial.begin(9600);
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
    //feedback for bluetooth connection established.
    if(YesConnected == false && status == ACI_EVT_CONNECTED){
      YesConnected = true;
      analogWrite(LeftMotor, 255);
      analogWrite(RightMotor, 255);
      delay(500);
      analogWrite(LeftMotor, 0);
      analogWrite(RightMotor, 0);
      delay(500);
      analogWrite(LeftMotor, 255);
      analogWrite(RightMotor, 255);
      delay(500);
      analogWrite(LeftMotor, 0);
      analogWrite(RightMotor, 0);
    }
    
    // Lets see if there's any data for us!
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
      handleRotation(u.fval);
    }
    CaneControlPanel();
  }


}


