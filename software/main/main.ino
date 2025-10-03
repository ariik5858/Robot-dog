#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150  // Pulse length for 0° (adjust if needed)
#define SERVOMAX 600  // Pulse length for 180° (adjust if needed)

uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}


float theta1;
float theta2;
float theta3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Testing PCA9685 with one servo");
  myServo.begin();
  myServo.setPWMFreq(60);  // Analog servos run at ~60 Hz
  delay(10);

  theta1 = 0;
  theta2 = 0;
  theta3 = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  ik(0, 0, 0, false, theta1, theta2, theta3);

}
