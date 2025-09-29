// Arm lengths
const float L1 = 10;
const float L2 = 10;
const float L3 = 10;
const float L4 = 10;
const float L5 = 10;
const float L6 = 10;
const float L7 = 10;
const float L8 = 10;
const float L9 = 10;
const float L10 = 10;

const float d1 = 10;
const float d2z = 10;
const float d2x = 10;

const float gamma1 = 10;

// const float PI = 3.1415926535897932384626433832795;
const float H_PI = 1.5707963267948966192313216916398;

void ik(float x, float y, float z, bool side, float* theta1, float* theta2, float* theta3) {
  // -- yz plane calculations -- //
  float Ryz = sqrtf(y*y + z*z);
  float alpha1 = acos(L1 / Ryz);
  float alpha2 = atan(y / z);
  float phi = alpha1 + alpha2;
  
  // for now the code will only look at the right side
  *theta1 = phi - H_PI; // right side theta
  // I think left side theta is phi + H_PI 
  
  // -- rotation about x-axis -- //
  float y_prime = y * cos(*theta1) + z * sin(*theta1); // since are all in the 
  float z_prime = -y * sin(*theta1) + z * cos(*theta1);
  
  // -- xz plane calculations -- //
  float Rxz_prime = sqrt(x*x + z_prime*z_prime);
  float alpha3 = atan(z_prime / x);
  float b1 = sqrt(d1*d1 + Rxz_prime*Rxz_prime - 2*d1*Rxz_prime*cos(alpha3));
  
  float alpha3_prime = asin((sin(alpha3)/b1)*Rxz_prime);
  float phi1 = atan(d2z/d2x);
  float alpha4 = phi1 + (Pi - phi1);
  float b2 = sqrt(d2*d2 + b1*b1 - 2*d2*b1);
  float alpha5 = asin((sin(alpha4)/b2)*b1);

  float alpha6 = acos((L3*L3 + L5*L5 - b2*b2) / (2*L3*L5));
  float alpha7 = asin((sin(alpha6)/b2)*L3);
  float phi2 = alpha7 + alpha5;

  float alpha8 = Pi-alpha6;
  float b3 = sqrt(L4*L4 + L5*L5 - 2*L4*L5*cos(alpha8));
  float alpha9 = asin((sin(alpha8)/b3)*L4);

  float alpha10 = acos((L8*L8 + b3*b3 - L6*L6) / (2*L8*B3));
  float phi3 = alpha10 + alpha9;

  float alpha11 = 2*Pi - phi2 - phi3 - gamma1;
  float b4 = sqrt(L8*L8 + d2*d2 - 2*L8*d2*cos(alpha11));
  float alpha12 = asin((sin(alpha11)/b4)*L8);

  float alpha13 = acos((L10*L10 + b4*b4 - L9*L9) / (2*L10*b4));

	// the following could change depending on how the motors rotate
  *theta2 = phi1 + alpha12 + alpha13;
	*theta3 = phi1 + phi2 + Pi;

}