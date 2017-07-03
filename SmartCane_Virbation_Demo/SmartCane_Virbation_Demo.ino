#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Vibration motor connected to digital pin 5 and 6
int LeftMotor = 5;
int RightMotor = 6;
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

int rotation;
bool inputLoop = false;
bool state1 = false;  

void setup() {
  // put your setup code here, to run once:
  setupIMU();
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Handle button pressed
  buttonReading = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if ((millis() - buttonLastDebounceTime) > buttonDebounce) {
    buttonState = LOW;
  }     
  
  if (buttonReading == HIGH && buttonState == LOW) {
      buttonState = HIGH;
      buttonLastDebounceTime = millis();
      rotation = random(45, 170);
      handleRotation(rotation);
      state1 = false;
      inputLoop = false;
  }
  
}

void setupIMU(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

void handleRotation(float rotationNeeded) {

  //waiting for button pressed.
  while (true){
    // Handle button pressed
    buttonReading = digitalRead(buttonPin);
  
    // If the switch changed, due to noise or pressing:
    if ((millis() - buttonLastDebounceTime) > buttonDebounce) {
      buttonState = LOW;
    }
    
    if (buttonReading == HIGH && buttonState == LOW) {
        buttonState = HIGH;
        buttonLastDebounceTime = millis();
        break;
    }
  }
    sensors_event_t event;
  bno.getEvent(&event);
  float startOrientation = event.orientation.x;
  int goalOrientation = (int)(startOrientation - rotationNeeded) % 360;

  int deltaOrientation = 0;
  int fadeValue = 0;
  bool state = false;
  
  while(true && state1 == false) {
    bno.getEvent(&event);
    deltaOrientation = (int)(event.orientation.x - goalOrientation) % 360;
    if (deltaOrientation < 0) {
      deltaOrientation += 360;  
    }
    
    while (deltaOrientation == 0 && inputLoop == false) {
      delay(500);
      Serial.println("Hold button to confirm rotation");
      // Send back user confirmtaion via bluetooth that confirms we good
      if (digitalRead(buttonPin) == HIGH){
        Serial.println("Rotation confirmed");
        analogWrite(RightMotor, 0);
        analogWrite(LeftMotor, 0);
        inputLoop = true;
        state1 = true;
      }
      break;
    }
    if (deltaOrientation != 0 && inputLoop == false) {
      // Use motors
      Serial.println(deltaOrientation);
      if(deltaOrientation < 180) {
        fadeValue = map(deltaOrientation, 0, 179, 30, 255);
        analogWrite(LeftMotor, fadeValue);
        analogWrite(RightMotor, 0);
      } else {
        fadeValue = map(deltaOrientation, 180, 359, 255, 30);
        analogWrite(RightMotor, fadeValue);  
        analogWrite(LeftMotor, 0);
      }
    }
  }
}
