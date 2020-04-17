#include <AccelStepper.h>

#define STEPS 200
#define dirPin 2
#define stepPin 3
#define home_switch 9
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define motorInterfaceType 1
AccelStepper stepper(motorInterfaceType, 5, 4);
int Pval = 0;
int potVal = 0;

void setup() {
  // Set the maximum speed in steps per second:
  // pinMode(stepPin, OUTPUT);
  // pinMode(dirPin, OUTPUT);
  int initial_homing = 1;
  while (!digitalRead(home_switch)) {
    stepper.setMaxSpeed(3000);
    stepper.setAcceleration(1000);
    stepper.moveTo(initial_homing);
    stepper.run();
    initial_homing += 1;
    delay(1);
  }
  Serial.begin(9600);
//  stepper.setSpeed(1000);
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
}
void loop() {
//  if (stepper.distanceToGo() == 0) 
//    stepper.moveTo(-stepper.currentPosition());
  Serial.println(stepper.currentPosition());
  stepper.runToNewPosition(-20.0/40.0*6400); // -20mm
  delay(2000);
  Serial.println(stepper.currentPosition());
  stepper.runToNewPosition(-40.0/40.0*6400); // -40mm
  delay(2000);
}
