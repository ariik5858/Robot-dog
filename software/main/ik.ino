const float L1 = 10;
// const float PI = 3.1415926535897932384626433832795;
const float H_PI = 1.5707963267948966192313216916398;

void ik(float x, float y, float z, bool side, float* theta1) {
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

}