#include <Adafruit_PWMServoDriver.h>

#include "Arduino.h"
#include "leg.h"
#include "utils.h"

Adafruit_PWMServoDriver myServo(0x40);

static uint16_t angleToPulse(int angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  return map(angleDeg, 0, 180, SERVOMIN, SERVOMAX);
}


LegController::LegController()
  : legs{
      Leg(true,  0, 1, 2),    // leg 0
      Leg(true,  3, 4, 5),    // leg 1
      Leg(false, 6, 7, 8),    // leg 2
      Leg(false, 9,10,11)     // leg 3
    } {}

void LegController::setLegPos(int legNum, float x, float y, float z) {
  if (legNum < 0 || legNum >= 4) return;

  Leg &leg = legs[legNum];

  // Compute IK into this leg's joint angles (in degrees)
  ik(x, y, z, leg.side, leg.theta1, leg.theta2, leg.theta3);

  myServo.setPWM(leg.pin1, 0, angleToPulse(leg.theta1));
  myServo.setPWM(leg.pin2, 0, angleToPulse(leg.theta2));
  myServo.setPWM(leg.pin3, 0, angleToPulse(leg.theta3));
  delay(10);
}

Leg& LegController::getLeg(int legNum) {
  return legs[legNum];
}

void LegController::startUp() {
  Serial.println("min angles");
  for (int i = 0; i < 4; i++) {
    Leg &leg = legs[i];
    myServo.setPWM(leg.pin1, 0, angleToPulse(90));
    myServo.setPWM(leg.pin2, 0, angleToPulse(90));
    myServo.setPWM(leg.pin3, 0, angleToPulse(135));
  }
  delay(1000);
  Serial.println("max angles");
  for (int i = 0; i < 4; i++) {
    Leg &leg = legs[i];
    myServo.setPWM(leg.pin1, 0, angleToPulse(180));
    myServo.setPWM(leg.pin2, 0, angleToPulse(180));
    myServo.setPWM(leg.pin3, 0, angleToPulse(90));
  }
  delay(1000);
  Serial.println("min angles");
  for (int i = 0; i < 4; i++) {
    Leg &leg = legs[i];
    myServo.setPWM(leg.pin1, 0, angleToPulse(90));
    myServo.setPWM(leg.pin2, 0, angleToPulse(90));
    myServo.setPWM(leg.pin3, 0, angleToPulse(135));
  }
  delay(1000);
}
