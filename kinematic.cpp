#include <math.h>
#include <stdio.h>
#include <Encoder.h>

#include "kinematic.h"

const float r = 0.016; //wheel radius
const float b = 0.096; //wheel to wheel distance
const float C = 8000.0; //number of steaps of encoder;

void posUpdateOld(int nr, int nl, Robot *robot) {
  float Dr = 2*M_PI*r*nr/C;
  float Dl = 2*M_PI*r*nl/C;
  float D = (Dr+Dl)/2;
  robot->t += (Dr-Dl)/b;
  robot->x += D*cos(robot->t);
  robot->y += D*sin(robot->t);
}
/* Get command from something
 * @parameters float array to store velocities: at 0 the linear V, at 1 the angular V
 */
void get_cmd_vel(float vel[2]) {
  vel[0]=0.1;
  vel[1]=0.2;
}
/* Convert linear V and angular V into wheel velocites
 * @parameters robot parameters + float array to store wheel velocities : at 0 the left wheel V and at 1 the right wheel V
 */
void cmd_vel2wheels(float V, float W, double wheel_vel[2]) {
  float wl = (V - b/2.0*W)/r;
  float wr = (V + b/2.0*W)/r;
  wheel_vel[0] = wl;
  wheel_vel[1] = wr;
}
/* Update the state of the robot with pulses encoder received
 * and the wheel velocites.
 */
void posUpdate(Robot* robot, Encoder encR, Encoder encL, double wheel_vel[2]) {
  //compare the stored pulses count with current pulses count to calculate diff
  long Nr = encR.read();
  long Nl = encL.read();
  long diffNr = Nr - robot->Nr;
  long diffNl = Nl - robot->Nl;
  //update robot state with current pulse count
  robot->Nr = Nr;
  robot->Nl = Nl;
  //calculate real linear and angular velocities with pulse count diff  
  float v = (2.0*M_PI*r/C) * ((float)(diffNr+diffNl)/2.0);
  float w = (2.0*M_PI*r/C) * ((float)(diffNr-diffNl)/b);

  //update position
  robot->t = atan2(sin(robot->t+w), cos(robot->t+w));
  robot->x += v*cos(robot->t);
  robot->y += v*sin(robot->t);

  //update wheel velocities
  cmd_vel2wheels(v, w, wheel_vel);
}
