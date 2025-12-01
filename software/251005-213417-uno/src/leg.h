#pragma once
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150  // Pulse length for 0° (adjust if needed)
#define SERVOMAX 600  // Pulse length for 180° (adjust if needed)

extern Adafruit_PWMServoDriver myServo;

struct Leg {
  bool side;       // true = right, false = left
  float theta1, theta2, theta3;
  uint8_t pin1, pin2, pin3;

  Leg(bool s, uint8_t p1, uint8_t p2, uint8_t p3)
    : side(s), theta1(0), theta2(0), theta3(0),
      pin1(p1), pin2(p2), pin3(p3) {}
};

class LegController {
public:
  LegController();
  void startUp();
  void setLegPos(int legNum, float x, float y, float z);
  void ik(Leg *leg, int side, float x, float y, float z);
  void ikTrue(Leg *leg, int side, float x, float y, float z);
  Leg getLeg(int legNum);
private:
  Leg legs[4];
};