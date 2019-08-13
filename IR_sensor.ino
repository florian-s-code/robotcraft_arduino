const int adc2 = A2 ;  //Initializing Pin for adc
const int adc3 = A3 ;  //Initializing Pin for adc
const int adc4 = A4 ;  //Initializing Pin for adc

//IR Sensor and distance conversion code
#define DEGREE 4
float polynom_right[DEGREE+1] = {868.12980760,
                                -054.18084250,
                                 001.53271537,
                                -000.01948814,
                                 000.00009091
                                 };
float polynom_left[DEGREE+1] = {882.668269,
                               -56.71887730,
                                1.62773400,
                               -0.02078510,
                                0.00009664
                                };
float polynom_front[DEGREE+1] = {864.048077,
                               -050.825800,
                                001.361118,
                               -000.016553,
                                000.000075
                               };
/* Convert the analog value from the IR to a value in cm using
 * a polynomial approximation
 */
float convert(int sensor_value, float polynom[DEGREE+1])
{
  float result = polynom[0];
  for(int i = 1; i < DEGREE+1; i++) {
    result += polynom[i]*pow(sensor_value, i);
  }
  return result;
}

float sensL(){
  int sensor_left = analogRead(adc2) ; 
  return convert(sensor_left, polynom_left);
}
float sensF(){
  int sensor_front = analogRead(adc3) ; 
  return convert(sensor_front, polynom_front);
}
float sensR(){
  int sensor_right = analogRead(adc4) ; 
  return convert(sensor_right, polynom_right);
}
