/*************************************************************************************/
// Joystick Setup
const int JoystickPin = A0;    // select the input pin for the potentiometer
int JoystickY;  // variable to store the value coming from the sensor
int lastJoystickState = LOW; //LOW not press, HIGH Pressed

long JoystickDebounce = 500;   // the debounce time, increase if the output flickers
long pressedTime;
/*************************************************************************************/

void CheckJoystick() {
        JoystickY = analogRead(JoystickPin);
      if ((millis() - pressedTime) > JoystickDebounce) {
        lastJoystickState = LOW;
      }     
      if (lastJoystickState == LOW) {
        if (JoystickY > 800) {
          lastJoystickState = HIGH;
          pressedTime = millis();
          Serial.println("Up"); 
          BTLEserial.print("2"); //2 - Up clicked
        } 
        if (JoystickY < 200) {
          lastJoystickState = HIGH;
          pressedTime = millis();
          Serial.println("Down"); 
          BTLEserial.print("3"); //3 - Down clicked
        }
       }
}
