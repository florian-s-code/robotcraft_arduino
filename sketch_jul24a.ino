#include <Encoder.h>
#include <PID_v1.h>
#include "kinematic.h"

#define PWM1 4
#define PWM2 9
#define DIR1 5
#define DIR2 6



//-----KINEMATIC SETUP--------
Robot robot;
float cmd_vel[2] = {0, 0}; //v and w command (constant for now)
double cmd_wheel_vel[2] = {0, 0};
double real_wheel_vel[2] = {0, 0};

void kinematic_setup() {
  robot.x = robot.y =robot.t = robot.Nr = robot.Nl = 0;
}
//-----ENCODER SETUP--------
Encoder encL(2,3);
Encoder encR(19,18);

// ---- PID SETUP ---------
double outputR;
double outputL;
PID pidR(&real_wheel_vel[0], &outputR, &cmd_wheel_vel[0], 300, 25000, 0, DIRECT);
PID pidL(&real_wheel_vel[1], &outputL, &cmd_wheel_vel[1], 300, 25000, 0, DIRECT);

void PID_setup() {
  pidR.SetMode(AUTOMATIC);
  pidL.SetMode(AUTOMATIC);
}

//-----UPDATE LOOP --------

unsigned long lastUpdateTime=0;

void updateLoop() {
  unsigned long timeT = millis();
  if(timeT-lastUpdateTime > DELTA_T) {
    lastUpdateTime = timeT;
    posUpdate(&robot, encR, encL, real_wheel_vel);

    PID_loop();
  }
}

unsigned long lastPrintTime=0;
void printLoop() {
  
  unsigned long timeT = millis();
  if(timeT-lastPrintTime > 10) {
    lastPrintTime = timeT;
    //Serial.print(real_wheel_vel[0]*10);
    //Serial.print(" ");
    //Serial.println(cmd_wheel_vel[0]*10);
    //Serial.println(cmd_vel[0]);
  }
}

void PID_loop() { //called in UpdateLoop
  //recalculate desired wheel velocities using desired v and w velocities
  cmd_vel2wheels(cmd_vel[0], cmd_vel[1], cmd_wheel_vel);
  
  pidR.Compute();
  pidL.Compute();

  //Inverse motor direction if ouptut negative and make it positive
  if(outputR < 0) {
    digitalWrite(DIR1, HIGH);
    outputR = -outputR;
  } else {
    digitalWrite(DIR1, LOW);
  }
  if(outputL < 0) {
    digitalWrite(DIR2, HIGH);
    outputL = -outputL;
  } else {
    digitalWrite(DIR2, LOW);
  }
  
  //Ensures output in correct range
  outputR = min(outputR, 255);
  outputL = min(outputL, 255);
  
  analogWrite(PWM1, (int)outputR);
  analogWrite(PWM2, (int)outputL);
}

void setup() {
  kinematic_setup();
  PID_setup();
  // put your setup code here, to run once:
  pinMode(PWM1, OUTPUT); //motor PWM control
  pinMode(PWM2, OUTPUT); //motor PWM control
  pinMode(DIR1, OUTPUT); //motor PWM control
  pinMode(DIR2, OUTPUT); //motor PWM control
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);

  Serial.begin(115200);
}

void loop() {
  updateLoop();
  printLoop();

}