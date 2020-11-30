#include<Servo.h>
#include <AccelStepper.h>

// all the defines
#define SERVO_PIN 3

Servo servo;

void setup() {
  //set pin modes
  servo.attach(SERVO_PIN);
  // begin serial
  Serial.begin(9600);
  Serial.println("Please enter the angle (0.0 To 90.0)");
}

void loop() {
  while (Serial.available() > 0) {
    float orient = Serial.parseFloat(SKIP_ALL);
    if (orient > 0.0 && orient <= 90.0) {
      Serial.print(" orient: ");
      Serial.println(orient);
      servo.write(orient*1.5+10.0);
    } else if (orient == -1.0) {
      Serial.print(" home");
      servo.write(0.0);
    }
  }
}
