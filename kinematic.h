#define DELTA_T 50.0 //DELTA_T in ms

typedef struct {
  float x, y, t; //cartesian coordinates and angle/x
  long Nr, Nl; //encoder pulses for right and left
} Robot;

void cmd_vel2wheels(float V, float W, double wheel_vel[2]);
void posUpdate(Robot* robot, Encoder encR, Encoder encL, double wheel_vel[2]);
void get_cmd_vel(double vel[2]);
