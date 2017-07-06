void CaneControlPanel() {

  // Handle button pressed
  greenButton("Yes");
  redButton("Yes");
  Joystick();
}

int greenButton(String sendResponseToPhone){
  // Handle button pressed
  GreenButtonReading = digitalRead(GreenButton);
  // If the switch changed, due to noise or pressing:
  if ((millis() - buttonLastDebounceTime) > buttonDebounce) {
    GreenButtonState = LOW;
  }     
  if (GreenButtonReading == HIGH && GreenButtonState == LOW) {
      GreenButtonState = HIGH;
      buttonLastDebounceTime = millis();
      Serial.println("GreenButton Pressed");
      if (sendResponseToPhone == "Yes"){
        Serial.println("GreenButton Send Bluetooth");
        BTLEserial.print("1"); //1 - select button clicked
      } 
      else if (sendResponseToPhone == "Status"){
        Serial.println("GreenButton return true");
        BTLEserial.print("1"); //1 - select button clicked
        return true;
    }
  }
}

int redButton(String sendResponseToPhone){
  // Handle button pressed
  RedButtonReading = digitalRead(RedButton);
  // If the switch changed, due to noise or pressing:
  if ((millis() - buttonLastDebounceTime) > buttonDebounce) {
    RedButtonState = LOW;
  }     
  if (RedButtonReading == HIGH && RedButtonState == LOW) {
      RedButtonState = HIGH;
      buttonLastDebounceTime = millis();
      Serial.println("RedButton Pressed");
      if (sendResponseToPhone == "Yes"){
        Serial.println("RedButton Send Bluetooth");
        BTLEserial.print("4"); //4 - select button clicked
      } 
      else if (sendResponseToPhone == "Status"){
        Serial.println("RedButton return true");
        BTLEserial.print("4"); //4 - select button clicked
        return true;
    }
  }
}

void Joystick(){
    /*
 A1
Center  470
Up    540
Down  230
Left    620
Right 368
  */
  // Handle sensor joystick
  Joystick_Value = analogRead(JoystickPin);
  
  // allow joystick reading to send only every JoystickDebounce time 500 milliseconds
  if ((millis() - JoystickLastDebounceTime) > JoystickDebounce) {
    JoystickState = LOW;
  }     
  if (JoystickState == LOW) {
    if (Joystick_Value < 320) {
      JoystickState = HIGH;
      JoystickLastDebounceTime = millis();
      Serial.println(Joystick_Value);
      Serial.println("Down"); 
      BTLEserial.print("Prior Destination");
      BTLEserial.print("3"); //3 - Down clicked
    }
    if (Joystick_Value > 500) {
      JoystickState = HIGH;
      JoystickLastDebounceTime = millis();
      Serial.println(Joystick_Value);
      Serial.println("Up"); 
      BTLEserial.print("Next Destination");
      BTLEserial.print("2"); //2 - Up clicked
    }
  }
}
