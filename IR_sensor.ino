const int adc2 = A2 ;  //Initializing Pin for adc
const int adc3 = A3 ;  //Initializing Pin for adc
const int adc4 = A4 ;  //Initializing Pin for adc

const float power_left = -1.2448947;
const float coeff_left = 18360.9142;
const float power_front = -1.24666891;
const float coeff_front = 19680.97255;
const float power_right = -1.2682982;
const float coeff_right= 21344.2132;
/* Convert the analog value from the IR to a value in cm using
 * a polynomial approximation
 */
float convert(int sensor_value, float power, float coeff)
{
  return coeff*pow(sensor_value, power);
}

float sensL(){
  int sensor_left = analogRead(adc2) ; 
  return convert(sensor_left, power_left, coeff_left);
}
float sensF(){
  int sensor_front = analogRead(adc3) ; 
  return convert(sensor_front, power_front, coeff_front);
}
float sensR(){
  int sensor_right = analogRead(adc4) ; 
  return convert(sensor_right, power_right, coeff_right);
}
