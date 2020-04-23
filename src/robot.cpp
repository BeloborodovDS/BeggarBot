#include <atomic>
#include <unistd.h>
#include <cmath>

#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "beggar_bot/SensorData.h"
#include "beggar_bot/DetectionBox.h"

#include "std_srvs/Empty.h"
#include "beggar_bot/ServoAction.h"
#include "beggar_bot/ServoSpeed.h"
#include "beggar_bot/ServoHeadPlatform.h"

#include "../include/constants.h"

using namespace std;
using namespace beggar_bot;

std::atomic_bool is_running;
std::atomic_bool face_processed;

float leftSensor, rightSensor;
DetectionBox face;

void terminator_callback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("Beggar Bot: caught TERMINATOR");
    is_running = false;
}

void sensor_callback(const SensorData::ConstPtr& msg)
{
    leftSensor = msg->left;
    rightSensor = msg->right;
}

void camera_callback(const DetectionBox::ConstPtr& msg)
{
    face = *msg;
    face_processed = false;
}

int follow_face(ros::ServiceClient &client_head_platform, ServoHeadPlatform &srv_head_platform)
{
    float x = 0, y = 0;
    
    // wait for another processed frame
    while (face_processed)
    {
        ros::spinOnce();
    }
    face_processed = true;
  
    // no face detected
    if (!face.present)
        return BB_NO_FACE;
  
    // calculate displace of face center from frame center
    y = face.y + 0.5*face.height - 0.5;
    y = y*BB_VFOV;
  
    x = face.x + 0.5*face.width - 0.5;
    x = x*BB_HFOV;
    
    // close enough to center
    if ((abs(y) < BB_FOLLOW_TOLERANCE) and (abs(x) < BB_FOLLOW_TOLERANCE))
    {
        // too small face
        if (face.height * face.width < BB_FACE_AREA)
            return BB_FACE_FAR;
        else
            return BB_FOUND_FACE;
    }
    
    // move robot (head - vertical plane, platform - horizontal plane)
    srv_head_platform.request.head_delta = y;
    srv_head_platform.request.platform_delta = x;
    client_head_platform.call(srv_head_platform);
    usleep(50000);
    
    // face is off limits, cannot reach
    if (!srv_head_platform.response.head_success)
        return BB_FACE_LIMIT;
  
    // skip next frame just in case
    while (face_processed)
    {
        ros::spinOnce();
    }
    face_processed = true;
  
    // moving robot
    return BB_FACE_BUSY;
}

int main( int argc, char** argv )
{
    is_running = true;
    face_processed = true;
    int found_face = BB_NO_FACE;
    leftSensor = 0;
    rightSensor = 0;
    
    ros::init(argc, argv, "beggar_bot");
    ros::NodeHandle n;
    
    ros::Subscriber sub_terminator = n.subscribe("terminator", 1, terminator_callback);
    ros::Subscriber sub_sensor = n.subscribe("sensor_state", 1, sensor_callback);
    ros::Subscriber sub_camera = n.subscribe("camera_state", 1, camera_callback);
    
    ros::ServiceClient client_action = n.serviceClient<ServoAction>("servo_action");
    ServoAction srv_action;
    ros::ServiceClient client_speed = n.serviceClient<ServoSpeed>("servo_speed");
    ServoSpeed srv_speed;
    ros::ServiceClient client_head_platform = n.serviceClient<ServoHeadPlatform>("servo_head_platform");
    ServoHeadPlatform srv_head_platform;
    
    usleep(5000000); // wait for nodes
    
    ROS_INFO("Beggar Bot initialized");
    
    /*
    srv_action.request.action = BB_ACTION_SHAKE;
    client_action.call(srv_action);
    
    srv_action.request.action = BB_ACTION_FROWN_ANGRY;
    client_action.call(srv_action);
    
    srv_head_platform.request.head_delta = -25;
    srv_head_platform.request.platform_delta = -25;
    client_head_platform.call(srv_head_platform);
    
    srv_speed.request.left = -1;
    srv_speed.request.right = 1;
    client_speed.call(srv_speed);
    
    usleep(3000000);
    
    srv_action.request.action = BB_ACTION_RESET_HEAD;
    client_action.call(srv_action);
    
    srv_speed.request.left = 0;
    srv_speed.request.right = 0;
    client_speed.call(srv_speed);
    */
    
    while (is_running && ros::ok())
    {
        int status = follow_face(client_head_platform, srv_head_platform);
        
        ROS_INFO("Status %f", (float) status);
        
        ros::spinOnce();
    }
    
    ros::ServiceClient terminate_switch = n.serviceClient<std_srvs::Empty>("terminate_switch");
    ros::ServiceClient terminate_camera = n.serviceClient<std_srvs::Empty>("terminate_camera");
    ros::ServiceClient terminate_servo = n.serviceClient<std_srvs::Empty>("terminate_servo");
    ros::ServiceClient terminate_sensor = n.serviceClient<std_srvs::Empty>("terminate_sensor");
    std_srvs::Empty srv_terminate;
    
    terminate_switch.call(srv_terminate);
    terminate_sensor.call(srv_terminate);
    terminate_camera.call(srv_terminate);
    terminate_servo.call(srv_terminate);
    
    //-------------------------------------------------MAIN BODY-----------------------------------------------------------
    
    /*
    // for navigation
    float left=0, right=0, alpha=0, sign=1;
    int roam_counter = 0;
    
    //main cycle
    while(is_running)
    {
        // while face is seen, align to it
        found_face = follow_face(&thread_pointers);
        while (found_face == BB_FACE_BUSY and is_running)
            found_face = follow_face(&thread_pointers);
        
        // read sensors
        left = IR_values[0];
        right = IR_values[1];
        
        // face detected => perform
        if (found_face == BB_FOUND_FACE)
        {
            // stop, shake and detect face again
            roam_counter = 0;
            setSpeed(0, 0);
            shake();
            delay(1000);
            found_face = follow_face(&thread_pointers);
            // if face is gone, angry frown, else shake again
            if (found_face == BB_NO_FACE)
                frown(-1);
            else
            {
                frown(1);
                shake();
                delay(1000);
            }
            // turn and go
            driveHead(BB_HEAD_INIT_POS - g_headPos);
            rotatePlatform(180);
            frown(0);
        }
        // if face over limit => angry frown
        else if (found_face == BB_FACE_LIMIT)
        {
            roam_counter = 0;
            setSpeed(0, 0);
            frown(-1);
            delay(1000);
            rotatePlatform(180);
        }
        // no face of face too far: random exploration
        else
        {     
            alpha = rand() % 91; // [0, 90]
            sign = (rand() % 2) * 2 - 1; // {-1, 1}
            
            // obstacles far: go forward
            if (left < 1 and right < 1)
            {
                setSpeed(1, 1);
                roam_counter ++;
            }
            // obstacles close: random rotate [90, 180] u [-180, -90]
            else if (left >=1 and right >=1)
            {
                rotatePlatform((90 + alpha) * sign);
                roam_counter = 0;
                driveHead(BB_HEAD_INIT_POS - g_headPos);
            }
            // obstacles to the right: rotate left 
            else if (left > right)
            {
                rotatePlatform(alpha);
                roam_counter = 0;
                driveHead(BB_HEAD_INIT_POS - g_headPos);
            }
            // obstacles to the left: rotate right
            else
            {
                rotatePlatform(-alpha);
                roam_counter = 0;
                driveHead(BB_HEAD_INIT_POS - g_headPos);
            }
            
            // if going forward for too long (stuck): go back a little and random rotate
            if (roam_counter > BB_ROAM_LIMIT)
            {
                roam_counter = 0;
                setSpeed(-1, -1);
                delay(1000);
                rotatePlatform((90 + alpha) * sign);
                setSpeed(1, 1);
            }
            delay(10);
        }
    }
    
    */
    
    return 0;
}
