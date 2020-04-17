#include<Servo.h>
#include <AccelStepper.h>

// all the defines
#define SERVO_PIN 9
#define BUTTON_PIN 7
#define HOME_SWITCH 7
#define MOTOR_INTERFACE_TYPE 1
#define STEPPER_DIR_PIN 4
#define STEPPER_STEP_PIN 5
#define STEPPER_STEPS 800
#define STEPPER_SPEED 200

Servo servo;
AccelStepper stepper(MOTOR_INTERFACE_TYPE, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// define angles
int poseCount = 3;
int angles[] = {90-35, 90, 90}; // in degree
int positions[] = {0, -7, -175}; // in milimeter

void setup() {
  //set pin modes
  pinMode(BUTTON_PIN, INPUT);
  pinMode(HOME_SWITCH, INPUT);
  servo.attach(SERVO_PIN);
  stepper.setSpeed(STEPPER_SPEED);
  // init stepper
  home(stepper, HOME_SWITCH);
  stepper.setMaxSpeed(4000);
  stepper.setAcceleration(2000);
  delay(2000);
}

int poseIndex = 0;

//define button states
bool lastState = LOW;
bool currentState = LOW;

void loop() {
  currentState = readButtonDebounce(lastState, BUTTON_PIN);
  if (currentState == HIGH && lastState == LOW) {
    lastState = HIGH;
    servo.write(angles[poseIndex]);
    stepper.runToNewPosition(positions[poseIndex]/40.0*6400); // 6400 steps per round, 40mm per round
    poseIndex = (poseIndex+1) % poseCount;
  }
  if (currentState == LOW && lastState == HIGH) {
    lastState = LOW;
  }
}

void home(AccelStepper stepper, int homeSwitch) {
  int initialHoming = 1;
  while (!digitalRead(homeSwitch)) {
    stepper.setMaxSpeed(3000);
    stepper.setAcceleration(1000);
    stepper.moveTo(initialHoming);
    stepper.run();
    initialHoming += 1;
    delay(1);
  }
  while (digitalRead(homeSwitch)) {
    stepper.moveTo(initialHoming);
    stepper.run();
    initialHoming -= 1;
  }
  stepper.setCurrentPosition(0);
}

bool readButtonDebounce(bool lastState, int buttonPin) {
  bool currentState = digitalRead(buttonPin);
  if (lastState != currentState) {
    delay(10);
    currentState = digitalRead(buttonPin);
  }
  return currentState;
}
