#include <stdint.h>

#include <Encoder.h>
#include <PID_v1.h>
#include "kinematic.h"
#include "IR_sensor.h"
#include "LED.h"

#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>


#define PWM1 9
#define PWM2 4
#define DIR1 6
#define DIR2 5

//-----KINEMATIC SETUP--------
Robot robot;
float cmd_vel[2] = {0.0, 0.0}; //treated velocity (mostly absolute)
double cmd_wheel_vel[2] = {0, 0};
double real_wheel_vel[2] = {0, 0};

void kinematic_setup() {
  robot.x = robot.y =robot.t = robot.Nr = robot.Nl = 0;
}
// ---- PID SETUP ---------
double outputR;
double outputL;
//PID pidR(&real_wheel_vel[0], &outputR, &cmd_wheel_vel[0], 06*0.7, 3.36, 0,013125, DIRECT);
PID pidR(&real_wheel_vel[0], &outputR, &cmd_wheel_vel[0], 300*0.05, 2500*0.05, 0, DIRECT);
PID pidL(&real_wheel_vel[1], &outputL, &cmd_wheel_vel[1], 300*0.05, 2500*0.05, 0, DIRECT);

void PID_setup() {
  pidR.SetMode(AUTOMATIC);
  pidL.SetMode(AUTOMATIC);
  pidR.SetOutputLimits(-200, +200);
  pidL.SetOutputLimits(-200, +200);
}

//----ROS SETUP --------------

ros::NodeHandle  nh;

const unsigned int cmdTimeout = 3000; //Time to wait for a cmd_vel message before stopping the robot
bool hasTimedOut = false;

//publishers
std_msgs::Float32 distance_msg;
ros::Publisher frontDistPub("front_distance", &distance_msg);
ros::Publisher rightDistPub("right_distance", &distance_msg);
ros::Publisher leftDistPub("left_distance", &distance_msg);

geometry_msgs::Pose2D pose_msg;
ros::Publisher posePub("pose", &pose_msg);

//subscribers

void ledsCallback(const std_msgs::UInt8MultiArray msg) {
  uint8_t * data = (uint8_t *)msg.data;
  setColor(data);
}
unsigned long lastCmdTime; //last time we received a command
void cmdVelCallback(const geometry_msgs::Twist msg) {
  lastCmdTime = millis();
  cmd_vel[0] = msg.linear.x; //only the x is interesting because the robot goes in only 1 direction
  cmd_vel[1] = msg.angular.z; //only z because the robot rotate only around this axis
}
void setPoseCallback(const geometry_msgs::Pose2D msg) {
  robot.x = msg.x;
  robot.y = msg.y;
  robot.t = msg.theta;
}
/* Stop the robot and reset PID errors */
void stopAndResetPID() {
  pidR.SetMode(MANUAL);
  pidL.SetMode(MANUAL);
  outputR = outputL = 0;
  cmd_vel[0] = cmd_vel[1] = 0;
  pidR.SetMode(AUTOMATIC);
  pidL.SetMode(AUTOMATIC);
}
void stopAndResetCallback( const std_msgs::Empty& msg ) {
  stopAndResetPID();
}

ros::Subscriber<std_msgs::UInt8MultiArray> ledsSub("rgb_leds", ledsCallback );
ros::Subscriber<geometry_msgs::Twist> cmdSub("cmd_vel", cmdVelCallback );
ros::Subscriber<geometry_msgs::Pose2D> poseSub("set_pose", setPoseCallback );
ros::Subscriber<std_msgs::Empty> stopAndResetSub("stop_reset", stopAndResetCallback );


void ros_setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.advertise(frontDistPub);
  nh.advertise(rightDistPub);
  nh.advertise(leftDistPub);
  nh.advertise(posePub);

  nh.subscribe(ledsSub);
  nh.subscribe(cmdSub);
  nh.subscribe(poseSub);
  nh.subscribe(stopAndResetSub);
}

// ----- ROS LOOP -----------

unsigned long lastROSUpdateTime=0;
const int DELTA_ROS = 100; //ms

void checkCommandTimeout();

void ROSLoop() { //called in loop()
  
  unsigned long timeT = millis();
  if(timeT-lastROSUpdateTime < DELTA_ROS)
    return;

  lastROSUpdateTime= timeT;
  distance_msg.data = robot.front_distance;
  frontDistPub.publish( &distance_msg );
  distance_msg.data = robot.right_distance;
  rightDistPub.publish( &distance_msg );
  distance_msg.data = robot.left_distance;
  leftDistPub.publish( &distance_msg );

  pose_msg.x = robot.x;
  pose_msg.y  = robot.y;
  pose_msg.theta = robot.t;
  posePub.publish( &pose_msg );
 
  nh.spinOnce();

  //check command timeout
  checkCommandTimeout();

}

void checkCommandTimeout() {
  if( (hasTimedOut == false) && (millis() - lastCmdTime > cmdTimeout) ) {
    stopAndResetPID();
    pidR.SetMode(MANUAL);
    pidL.SetMode(MANUAL);
    hasTimedOut = true;
  }
  else if( (hasTimedOut == true) && (millis() - lastCmdTime < cmdTimeout) ) {
    pidR.SetMode(AUTOMATIC);
    pidL.SetMode(AUTOMATIC);
    hasTimedOut = false;
  }
}

//-----ENCODER SETUP--------

Encoder encR(2,3);
Encoder encL(19,18);

//-----UPDATE LOOP --------

unsigned long lastUpdateTime=0;

void updateLoop() {
  unsigned long timeT = millis();
  if(timeT-lastUpdateTime > DELTA_T) {
    lastUpdateTime = timeT;
    posUpdate(&robot, encR, encL, real_wheel_vel);

    PID_loop();
  }

  //IR Sensors
  robot.front_distance = sensF();
  robot.left_distance = sensL();
  robot.right_distance = sensR();
}

unsigned long lastPrintTime=0;
void printLoop() {
  
  unsigned long timeT = millis();
  if(timeT-lastPrintTime > 10) {
    lastPrintTime = timeT;
    //Serial.print(real_wheel_vel[0]);
    //Serial.print(" ");
    //Serial.println(cmd_wheel_vel[0]);
    //Serial.println(cmd_vel[0]);
  }
}

void PID_loop() { //called in UpdateLoop
  //recalculate desired wheel velocities using desired v and w velocities
  cmd_vel2wheels(cmd_vel[0], cmd_vel[1], cmd_wheel_vel);

  pidR.Compute();
  pidL.Compute();

  /*Serial.print("OutputR : ");
  Serial.println(outputR);
  Serial.print("OutputL : ");
  Serial.println(outputL);*/

  double absOutputR = outputR;
  double absOutputL = outputL;

  //Inverse motor direction if ouptut negative and make it positive
  if(absOutputR <= 0) {
    digitalWrite(DIR1, HIGH);
    absOutputR = -absOutputR;
  } else {
    digitalWrite(DIR1, LOW);
  }
  if(absOutputL <= 0) {
    digitalWrite(DIR2, HIGH);
    absOutputL = -absOutputL;
  } else {
    digitalWrite(DIR2, LOW);
  }
  
  //Ensures output in correct range
  absOutputR = min(absOutputR, 255);
  absOutputL = min(absOutputL, 255);
  
  analogWrite(PWM1, (int)absOutputR);
  analogWrite(PWM2, (int)absOutputL);
}

void setup() {
  kinematic_setup();
  PID_setup();
  ros_setup();
  LED_setup();
  
  // put your setup code here, to run once:
  pinMode(PWM1, OUTPUT); //motor PWM control
  pinMode(PWM2, OUTPUT); //motor PWM control
  pinMode(DIR1, OUTPUT); //motor PWM control
  pinMode(DIR2, OUTPUT); //motor PWM control
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
}

void loop() {
  ROSLoop();
  updateLoop();
  //printLoop();
  //delay(100);
}
