#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <wiringPi.h>

#include "constants.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "switch_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("switch_state", 1);
  ros::Rate loop_rate(20);
  
  //setup WiringPi and configure switch pin for output
  wiringPiSetup();
  pinMode(BB_PIN_SWITCH, INPUT);
  pullUpDnControl(BB_PIN_SWITCH, PUD_UP);
  
  ROS_INFO("Switch node initialized");

  while (ros::ok())
  {
    std_msgs::Bool msg;
    msg.data = (digitalRead(BB_PIN_SWITCH) != 1);
    
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
