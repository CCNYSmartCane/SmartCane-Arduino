void CaneControlPanel() {

  // Handle button pressed
  buttonReading = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if ((millis() - buttonLastDebounceTime) > buttonDebounce) {
    buttonState = LOW;
  }     
  
  if (buttonReading == HIGH && buttonState == LOW) {
      buttonState = HIGH;
      buttonLastDebounceTime = millis();
      Serial.println("No");
      BTLEserial.print("1"); //1 - select button clicked
  }

  // Handle sensor joystick
  JoystickY = analogRead(JoystickPin);
  
  // allow joystick reading to send only every JoystickDebounce time 500 milliseconds
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


