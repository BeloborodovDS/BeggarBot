#include <atomic>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include "wiringPi.h"

#include "../include/constants.h"

std::atomic_bool is_running;

bool terminate_node(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &ignored)
{
    ROS_INFO("Switch node terminated");
    is_running = false;
    return true;
}

int main(int argc, char **argv)
{
    is_running = true;
    
    ros::init(argc, argv, "switch_node");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("terminator", 1);
    ros::ServiceServer term = n.advertiseService("terminate_switch", terminate_node);
    ros::Rate loop_rate(20);
  
    //setup WiringPi and configure switch pin for output
    wiringPiSetup();
    pinMode(BB_PIN_SWITCH, INPUT);
    pullUpDnControl(BB_PIN_SWITCH, PUD_UP);
    // Pulled UP: OFF <=> 1, ON <=> 0
  
    std_msgs::Bool msg;
    msg.data = false;
    bool terminator = false;
    bool info_sent = false;
  
    ROS_INFO("Switch node initialized");

    while (is_running && ros::ok())
    {
        terminator = digitalRead(BB_PIN_SWITCH);
    
        if (terminator)
        {
            chatter_pub.publish(msg);
            if (!info_sent)
            {
                ROS_INFO("Switch node: sending TERMINATOR");
                info_sent = true;
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    usleep(500000);
    return 0;
}
