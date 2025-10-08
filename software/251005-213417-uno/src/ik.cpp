#include <stdio.h>
#include "utils.h"

void ik(float x, float y, float z, bool side, float &theta1, float &theta2, float &theta3) {
  Serial.println(F("=== IK Debug Start ==="));

  // -- yz plane calculations -- //
  float Ryz = sqrtf(y * y + z * z);
  Serial.print(F("Ryz = ")); Serial.println(Ryz);

  float alpha1 = safe_acos(L1 / Ryz);
  Serial.print(F("alpha1 = ")); Serial.println(alpha1 * RAD_TO_DEG);

  float alpha2 = atan2f(y, z);
  Serial.print(F("alpha2 = ")); Serial.println(alpha2 * RAD_TO_DEG);

  float phi = abs(alpha1) + abs(alpha2);
  Serial.print(F("phi = ")); Serial.println(phi * RAD_TO_DEG);
  
  // for now the code will only look at the right side
  theta1 = phi; // right side theta
  Serial.print(F("theta1 = ")); Serial.println(theta1 * RAD_TO_DEG);

  float rotate_angle = theta1 - H_PI;
  
  // -- rotation about x-axis -- //
  float y_prime = y * cos(rotate_angle) + z * sin(rotate_angle);
  float z_prime = -y * sin(rotate_angle) + z * cos(rotate_angle);
  Serial.print(F("y' = ")); Serial.println(y_prime);
  Serial.print(F("z' = ")); Serial.println(z_prime);
  
  // -- xz plane calculations -- //
	float Rxz_prime = sqrtf(x * x + z_prime * z_prime);
	float d2 = sqrt(d2x * d2x + d2z * d2z);
	Serial.print(F("d2 = ")); Serial.println(d2);
  float phi1 = atan2f(d2z, d2x);
  Serial.print(F("phi1 = ")); Serial.println(phi1 * RAD_TO_DEG);
  float alpha3_prime = atan2f(abs(z_prime), abs(x));
  Serial.print(F("alpha3' = ")); Serial.println(alpha3_prime * RAD_TO_DEG);
  float alpha4 = phi1 + alpha3_prime;
  Serial.print(F("alpha4 = ")); Serial.println(alpha4 * RAD_TO_DEG);
  float b1 = sqrt(Rxz_prime * Rxz_prime + d2 * d2 - 2 * Rxz_prime * d2 * cos(alpha4));
  Serial.print(F("b1 = ")); Serial.println(b1);
  float alpha5 = safe_asin((sin(alpha4) / b1) * Rxz_prime);
  Serial.print(F("alpha5 = ")); Serial.println(alpha5 * RAD_TO_DEG);

  float alpha6 = safe_acos((L3 * L3 + L5 * L5 - b1 * b1) / (2 * L3 * L5));
  Serial.print(F("alpha6 = ")); Serial.println(alpha6 * RAD_TO_DEG);

  float alpha7 = safe_asin((sin(alpha6) / b1) * L3);
  Serial.print(F("alpha7 = ")); Serial.println(alpha7 * RAD_TO_DEG);

  float phi2 = alpha7 + alpha5;
  Serial.print(F("phi2 = ")); Serial.println(phi2 * RAD_TO_DEG);

  float alpha8 = PI - alpha6;
  Serial.print(F("alpha8 = ")); Serial.println(alpha8 * RAD_TO_DEG);
  float b2 = sqrt(L4 * L4 + L5 * L5 - 2 * L4 * L5 * cos(alpha8));
  Serial.print(F("b2 = ")); Serial.println(b2);
	
  float alpha9 = safe_asin((sin(alpha8) / b2) * L4);
  Serial.print(F("alpha9 = ")); Serial.println(alpha9 * RAD_TO_DEG);

  float alpha10 = safe_acos((L8 * L8 + b2 * b2 - L6 * L6) / (2 * L8 * b2));
  Serial.print(F("alpha10 = ")); Serial.println(alpha10 * RAD_TO_DEG);

  float phi3 = alpha10 + alpha9;
  Serial.print(F("phi3 = ")); Serial.println(phi3 * RAD_TO_DEG);

  float alpha11 = 2 * PI - phi2 - phi3 - gamma1;
  Serial.print(F("alpha11 = ")); Serial.println(alpha11 * RAD_TO_DEG);

	float b3 = sqrt(d2 * d2 + L8 * L8 - 2 * d2 * L8 * cos(alpha11));
  Serial.print(F("b3 = ")); Serial.println(b2);

  float alpha12 = safe_asin((sin(alpha11) / b3) * L8);
  Serial.print(F("alpha12 = ")); Serial.println(alpha12 * RAD_TO_DEG);

  float alpha13 = safe_acos((L10 * L10 + b3 * b3 - L9 * L9) / (2 * L10 * b3));
  Serial.print(F("alpha13 = ")); Serial.println(alpha13 * RAD_TO_DEG);
  
	theta2 = (phi1 + alpha12 + alpha13) - H_PI;
	theta3 = phi1 + phi2;

  // convert all thetas to degrees
  theta1 = theta1 * RAD_TO_DEG;
  theta2 = theta2 * RAD_TO_DEG;
  theta3 = theta3 * RAD_TO_DEG;

  Serial.print(F("theta1 = ")); Serial.println(theta1);
  Serial.print(F("theta2 = ")); Serial.println(theta2);
  Serial.print(F("theta3 = ")); Serial.println(theta3);
  Serial.println(F("=== IK Debug End ==="));
}

float clamp(float num) {
  if (num < -1.0f || num > 1.0f) {
    Serial.println(F("Value out of range"));
  }
  return clampf(num, -1.0f, 1.0f);
}