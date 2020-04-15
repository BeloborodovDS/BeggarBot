#include "ros/ros.h"
#include "beggar_bot/DetectionBox.h"
#include "beggar_bot/DetectionList.h"

#include "../include/constants.h"

using namespace beggar_bot;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<DetectionList>("camera_state", 1);
  ros::Rate loop_rate(10); //!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  DetectionList msg;
  DetectionBox box;
  int count = 0;
  
  ROS_INFO("Camera node initialized");

  while (ros::ok())
  {
    for (int i=0; i<count % 3; i++) 
    {
        box.x = 0;
        box.y = 0;
        box.width = count;
        box.height = count % 3;
        msg.detections.push_back(box);
    }
    msg.count = count;
    
    chatter_pub.publish(msg);
    msg.detections.clear();
    ros::spinOnce();
    loop_rate.sleep();
    count ++;
  }

  return 0;
}
