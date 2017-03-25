#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

#include <ros.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;

float RAD2DEG = 57.29577951308232;

void servo_cb( const sensor_msgs::JointState& cmd_msg){
  Braccio.ServoMovement(20, (int)(RAD2DEG*cmd_msg.position[0]),  (int)(RAD2DEG*cmd_msg.position[1]), (int)(RAD2DEG*cmd_msg.position[2]), (int)(RAD2DEG*cmd_msg.position[3]), (int)(RAD2DEG*cmd_msg.position[4]), 10);  
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup() {

  // init library
  Braccio.begin();

  // initialize ROS node
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

}

void loop() { 
  nh.spinOnce();
  delay(1);
}
