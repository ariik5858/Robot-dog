#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "Arduino.h"
#include "leg.h"
#include "utils.h"

LegController control;
int angleDeg = 90;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  myServo.begin();
  myServo.setPWMFreq(60);  // Analog servos run at ~60 Hz
  delay(10);

  Serial.println("Start up test");
  control.startUp();
  Serial.println("Start up test finish");

  // Leg firstLeg = control.getLeg(0);
  // control.setLegPos(0, -50, 50.7, 0);
}

void loop() {
  angleDeg = 180;
  // myServo.setPWM(0,0,map(constrain(angleDeg, 0, 180), 0, 180, SERVOMIN, SERVOMAX)); // theta 1
  // delay(1000);
  // myServo.setPWM(1,0,map(constrain(angleDeg, 0, 180), 0, 180, SERVOMIN, SERVOMAX)); // theta 3
  // delay(1000);
  // myServo.setPWM(2,0,map(constrain(angleDeg, 0, 180), 0, 180, SERVOMIN, SERVOMAX)); // theta 2
  // delay(1000);
  
  // angleDeg = 165.62;
  // myServo.setPWM(2,0,map(constrain(angleDeg, 0, 180), 0, 180, SERVOMIN, SERVOMAX)); // theta 2
  // delay(1000);
  // angleDeg = 175.05;
  // myServo.setPWM(2,0,map(constrain(angleDeg, 0, 180), 0, 180, SERVOMIN, SERVOMAX)); // theta 2
  // delay(1000);

  control.setLegPos(0, -25, 50.7, -25);   delay(5000);
  Serial.print("Angles1: ");
  Serial.print(control.getLeg(0).theta1);
  Serial.print(" ");
  Serial.print(control.getLeg(0).theta2);
  Serial.print(" ");
  Serial.println(control.getLeg(0).theta3);

  control.setLegPos(0, -25, 50.7,   0);   delay(5000);
  Serial.print("Angles2: ");
  Serial.print(control.getLeg(0).theta1);
  Serial.print(" ");
  Serial.print(control.getLeg(0).theta2);
  Serial.print(" ");
  Serial.println(control.getLeg(0).theta3);

  control.setLegPos(0, -50, 50.7, -25);   delay(5000);
  Serial.print("Angles3: ");
  Serial.print(control.getLeg(0).theta1);
  Serial.print(" ");
  Serial.print(control.getLeg(0).theta2);
  Serial.print(" ");
  Serial.println(control.getLeg(0).theta3);
}

// myServo.setPWM(0,0,angleToPulse(135)); // theta3
// myServo.setPWM(1,0,angleToPulse(90)); // theta2
// delay(1000);

// myServo.setPWM(0,0,angleToPulse((theta3 * 180) / PI));
// myServo.setPWM(1,0,angleToPulse((theta2 * 180) / PI));
// delay(1000);

// myServo.setPWM(0,0,angleToPulse(90));
// myServo.setPWM(1,0,angleToPulse(180));
// delay(1000);
