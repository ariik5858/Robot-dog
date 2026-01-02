#include <Adafruit_PWMServoDriver.h>
#include <stdio.h>

#include "Arduino.h"
#include "leg.h"
#include "utils.h"

Adafruit_PWMServoDriver myServo(0x40);

pos fwPathL[3] = {
  {-95.0f, 50.7f, -35.0f},
  {-45.0f, 50.7f, 0.0f},
  {-75.0f, 50.7f, 0.0f}
  
};

pos fwPathR[3] = {
  {-95.0f, -50.7f, -35.0f},
  {-45.0f, -50.7f, 0.0f},
  {-75.0f, -50.7f, 0.0f}
};

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

void LegController::ik(Leg &leg, int side, float x, float y, float z) {
  float phi = atan2(y, x);
  float root = sqrt(x*x + y*y - L1*L1);
  float l;

  l = L1;
  if(!side) l = -L1;

  // Theta 1
    float t1 = phi - atan2(l, -root);
    if (t1 < 0) {
      if (abs(t1) > PI){
        t1 += 2*PI;
      }
    }
    leg.theta1 = t1;

    // Theta 3
    float xPrime = cos(leg.theta1)*x + sin(leg.theta1)*y;
    float r = xPrime - d2z; 
    float num = r*r + z*z - L5*L5 - L3*L3;
    float den = 2*L5*L3;
    float costheta3 = num / den;
    costheta3 = max(min(costheta3, 1), -1);
    leg.theta3 = -safe_acos(costheta3);

    // Theta 2
    float k1 = L5 + L3*cos(leg.theta3);
    float k2 = L3*sin(leg.theta3);

    float rho = atan2(k2, k1);
    float psi = atan2(z, r);
    leg.theta2 = psi - rho;
    if (leg.theta2<0)
    {
        leg.theta2 += 2*PI;
    }
}

void LegController::ikTrue(Leg &leg, int side, float x, float y, float z) {
  ik(leg, side, x, y, z);
  if(side) {
    float phi1 = safe_asin(d2z/d2);
    float phi2 = safe_asin(d2x/d2);

    float r = cosLawC(L4, L5, abs(leg.theta3));
    float tR3 = sinTheta(L4, r, abs(leg.theta3));
    float tR2 = cosTheta(L8, r, L6);

    float tL2 = (2*PI - leg.theta2) - tR3 - tR2;
    float tL1 = (PI - 2*gamma) - tL2;
    float tR4 = PI - tL1 - phi2;
    float r2 = cosLawC(L8, d2, abs(tR4));

    float phi3 = sinTheta(L8, r2, tR4);
    float phi4 = cosTheta(L10, r2, L9);

    float t1 = (leg.theta1 + HALF_PI);
    float t2 = (leg.theta2 - HALF_PI);
    float t3 = (2*PI - abs(phi1 + phi3 + phi4));
    t3 = (PI - abs(leg.theta3)) - ((2*PI - abs(leg.theta2)) - HALF_PI) + 20*DEG_TO_RAD + HALF_PI;

    t1 = PI - wrap(t1);
    t2 = wrap(t2);
    t3 = wrap(t3);

    leg.theta1 = t1 * RAD_TO_DEG;
    leg.theta2 = t2 * RAD_TO_DEG;
    leg.theta3 = t3 * RAD_TO_DEG;
  } else {
    float phi1 = safe_asin(d2z/d2);
    float phi2 = safe_asin(d2x/d2);

    float r = cosLawC(L4, L5, abs(leg.theta3));
    float tR3 = sinTheta(L4, r, abs(leg.theta3));
    float tR2 = cosTheta(L8, r, L6);

    float tL2 = (2*PI - leg.theta2) - tR3 - tR2;
    float tL1 = (PI - 2*gamma) - tL2;
    float tR4 = PI - tL1 - phi2;
    float r2 = cosLawC(L8, d2, abs(tR4));

    float phi3 = sinTheta(L8, r2, tR4);
    float phi4 = cosTheta(L10, r2, L9);

    float t1 = (leg.theta1 + HALF_PI);
    float t2 = (leg.theta2 - HALF_PI);
    float t3 = (2*PI - abs(phi1 + phi3 + phi4));
    t3 = (PI - abs(leg.theta3)) - ((2*PI - abs(leg.theta2)) - HALF_PI) + 20*DEG_TO_RAD + HALF_PI; // possibly wrong


    t1 = PI - wrap(t1);
    t2 = PI - wrap(t2);
    t3 = PI - wrap(t3);

    leg.theta1 = t1 * RAD_TO_DEG;
    leg.theta2 = t2 * RAD_TO_DEG;
    leg.theta3 = t3 * RAD_TO_DEG;
  }
}

void LegController::setLegPos(int legNum, float x, float y, float z) {
  if (legNum < 0 || legNum >= 4) return;

  Leg &leg = legs[legNum];

  // Compute IK into this leg's joint angles
  ikTrue(leg, leg.side, x, y, z);
  myServo.setPWM(leg.pin1, 0, angleToPulse(leg.theta1));
  myServo.setPWM(leg.pin2, 0, angleToPulse(leg.theta2));
  myServo.setPWM(leg.pin3, 0, angleToPulse(leg.theta3));
  delay(10);
}

void LegController::walk(int legNum, pos* path, int size) {
  for (int i = 0; i < size; i++) {
    setLegPos(legNum, path[i].x, path[i].y, path[i].z);
    delay(100);
  }
}

void LegController::forwardWalk() {
  // Footfall order: BL → FL → BR → FR
  int len = sizeof(fwPathR)/sizeof(fwPathR[0]);

  walk(1, fwPathL, len);
  walk(0, fwPathL, len);
  walk(3, fwPathR, len);
  walk(2, fwPathR, len);
}

Leg LegController::getLeg(int legNum) {
  return legs[legNum];
}

void LegController::startUp() {
  setLegPos(0, -75.0f, 50.7f, 0.0f);
  setLegPos(1, -75.0f, 50.7f, 0.0f);
  setLegPos(2, -75.0f, -50.7f, 0.0f);
  setLegPos(3, -75.0f, -50.7f, 0.0f);
  
}

void LegController::callibrate() {
  Serial.println("angles");
  for (int i = 0; i < 4; i++) {
    Leg &leg = legs[i];
    myServo.setPWM(leg.pin1, 0, angleToPulse(0));
    myServo.setPWM(leg.pin2, 0, angleToPulse(0));
    myServo.setPWM(leg.pin3, 0, angleToPulse(0));
  }
  delay(5000);
  Serial.println("min angles");
  for (int i = 0; i < 4; i++) {
    Leg &leg = legs[i];
    myServo.setPWM(leg.pin1, 0, angleToPulse(90));
    myServo.setPWM(leg.pin2, 0, angleToPulse(90));
    myServo.setPWM(leg.pin3, 0, angleToPulse(90));
  }
  delay(5000);
  // Serial.println("max angles");
  // for (int i = 0; i < 4; i++) {
  //   Leg &leg = legs[i];
  //   myServo.setPWM(leg.pin1, 0, angleToPulse(180));
  //   myServo.setPWM(leg.pin2, 0, angleToPulse(180));
  //   myServo.setPWM(leg.pin3, 0, angleToPulse(180));
  // }
  // delay(5000);
  // for (int i = 0; i < 4; i++) {
  //   Leg &leg = legs[i];
  //   myServo.setPWM(leg.pin1, 0, angleToPulse(90));
  //   myServo.setPWM(leg.pin2, 0, angleToPulse(90));
  //   myServo.setPWM(leg.pin3, 0, angleToPulse(90));
  // }
  // delay(5000);
}

