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
}

void loop() {
  /*
    It was found that motor 0 controls theta1,
    motor 1 controls theta3, and motor 2 controls 
    theta2 for leg 1.

    It is likely that either we have to setup the 
    motors such that they follow this for every leg
  */


  // Below is a test path for the leg to follow
  control.setLegPos(0, -25, 50.7, -25);   delay(5000);
  Serial.print("Angles1: ");
  Serial.print(control.getLeg(0).theta1);
  Serial.print(" ");
  Serial.print(control.getLeg(0).theta2);
  Serial.print(" ");
  Serial.println(control.getLeg(0).theta3);

  control.setLegPos(0, -50, 70, -25);   delay(5000);
  Serial.print("Angles3: ");
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