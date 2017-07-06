/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more All text above, and the splash screen below must be included in any redistribution
information
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

/*************************************************************************************/
// Bluetooth Setuppress
// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 7     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/*************************************************************************************/

/*************************************************************************************/
// Vibration motor connected to digital pin 5 and 6
int LeftMotor = 6;
int RightMotor = 5;
/*************************************************************************************/

/*************************************************************************************/
// Joystick Setup
const int JoystickPin = A1;    // select the input pin for the potentiometer
int Joystick_Value;  // variable to store the value coming from the sensor
long JoystickDebounce = 1250;
long JoystickLastDebounceTime;
int JoystickState;
/*
A0,
Center  470
Up    252
Down  562
Left    388
Right 620
A1
Center  470
Up    540
Down  230
Left    620
Right 368
 */
/*************************************************************************************/

/*************************************************************************************/
// Button Setup
const int GreenButton = A3;     // the number of the pushbutton pin
const int RedButton = A2;
// High = Pressed Down
int RedButtonReading,GreenButtonReading;           // the current reading from the input pin
int RedButtonState,GreenButtonState;              // the status of the button
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long buttonLastDebounceTime = 0;         // the last time the output pin was toggled
long buttonDebounce = 500;   // the debounce time, increase if the output flickers
/*************************************************************************************/

/*************************************************************************************/
// Red Button variables
int buttonVal = 0; // value read from button
int buttonLast = 0; // buffered value of the button's previous state
long btnDnTime; // time the button was pressed down
long btnUpTime; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered
#define debounce 100 // ms debounce period to prevent flickering when pressing or releasing the button
#define holdTime 2500 // ms hold period: how long to wait for press+hold event
/*************************************************************************************/

/*************************************************************************************/
// Red Reset Button Setup
/*float pressedTime;    // the time the red button is pressed
bool pressed = false;   // default pressed position*/
/*************************************************************************************/

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
  pinMode(GreenButton, INPUT);
  pinMode(RedButton, INPUT);
  
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
        YesConnected = false;
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
        BTLEserial.print("SmartCane Connected!");
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
        BTLEserial.print("SmartCane Disconnected");
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
      Serial.println(u.fval);
      handleRotation(u.fval);
    }
    CaneControlPanel();
  }
}
/*// Read the state of the button
  buttonVal = digitalRead(RedButton);
// Test for button pressed and store the down time
    if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnUpTime) > long(debounce)){
        btnDnTime = millis();
    }
// Test for button release and store the up time
    if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnDnTime) > long(debounce)){
      if (ignoreUp == false) event1();
      else ignoreUp = false;
      btnUpTime = millis();
    }
// Test for button held down for longer than the hold time
    if (buttonVal == HIGH && (millis() - btnDnTime) > long(holdTime)){
      event2();
      ignoreUp = true;
      btnDnTime = millis();
    }
buttonLast = buttonVal;
}

void event1(){
  Serial.println("RedButton PressedTEST");
  BTLEserial.print("4"); //4 - select button clicked
}

void event2(){
  Serial.println("Reset Motors");
  BTLEserial.print("Reset Motors");
  analogWrite(LeftMotor, 0);
  analogWrite(RightMotor, 0);
  return false;
}
*/
