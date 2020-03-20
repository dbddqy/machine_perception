#include<Servo.h>
Servo myservo;

//define button pin
int buttonPin = 7;

// define angles
int angleCount = 4;
int angles[] = {90, 45, 90, 135};
int angleIndex = 0;

//define button states
bool lastState = LOW;
bool currentState = LOW;

void setup() {
  //set pin modes
  pinMode(buttonPin, OUTPUT);
  myservo.attach(9);
}

void loop() {
  currentState = readButtonDebounce(lastState, buttonPin);
  if (currentState == HIGH && lastState == LOW) {
    lastState = HIGH;
    myservo.write(angles[angleIndex]);
    angleIndex = (angleIndex+1) % angleCount;
  }
  if (currentState == LOW && lastState == HIGH) {
    lastState = LOW;
  }
}

bool readButtonDebounce(bool lastState, int buttonPin) {
  bool currentState = digitalRead(buttonPin);
  if (lastState != currentState) {
    delay(10);
    currentState = digitalRead(buttonPin);
  }
  return currentState;
}
