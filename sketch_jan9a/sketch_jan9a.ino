/**
* Source code:
* https://www.italiantechproject.it/tutorial-arduino/motore-dc
*/
#define MOTOR_PIN 3
#define BUTTON_PIN 4
#define POTENTIOMETER_PIN A0
#define BOOT_TIME 400
 
bool enabled = false;
int lastEnableValue = LOW;
unsigned long startTime = 0;
 
void setup(){
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}
 
void loop(){
  int enableValue = digitalRead(BUTTON_PIN);
  if(enableValue != lastEnableValue && enableValue == HIGH){
    if(!enabled){
      startTime = millis();
    }
    enabled = !enabled;
  }
  lastEnableValue = enableValue;
 
  if(enabled){
    if((millis()-startTime) > BOOT_TIME){
      int motorSpeed = map(analogRead(POTENTIOMETER_PIN), 0, 1023, 70, 255);
      analogWrite(MOTOR_PIN, motorSpeed);
    }else{
      analogWrite(MOTOR_PIN, 255);
    }
  }else{
    analogWrite(MOTOR_PIN, 0);
  }
  delay(50);
}
