#include <atomic>
#include <unistd.h>

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

void terminator_callback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("Beggar Bot: caught TERMINATOR");
    is_running = false;
}

void sensor_callback(const SensorData::ConstPtr& msg)
{
    ; //ROS_INFO("sensor_callback: %f, %f", msg->left, msg->right);
}

void camera_callback(const DetectionBox::ConstPtr& msg)
{
    ; //ROS_INFO("camera_callback: %f, %f, %f, %f, %f, %f", 
             //(float) (msg->count), (float) (msg->present), msg->x, msg->y, msg->width, msg->height);
}


/*
int follow_face()
{
    // TODO: remove H, W
    int H = pointers->ncs->netInputHeight, W = pointers->ncs->netInputWidth;  //net input image size
    float y = 0, x = 0, dist=0, mindist=100000, sign = 0;
    TrackingBox trbox, best_trbox;
  
    // wait for another processed frame
    while ( *(pointers->face_processed))
        delay(10);
    *(pointers->face_processed) = true;
  
    // find face closest to frame center
    pthread_mutex_lock(&face_vector_mutex);//________________LOCK_____________________________
    for (int i=0; i<pointers->trfaces->size(); i++)
    {
        trbox = (*(pointers->trfaces))[i];
        y = trbox.box.y + trbox.box.height/2;
        x = trbox.box.x + trbox.box.width/2;
        dist = abs(y - H/2.0) + abs(x - W/2.0);
        if (dist < mindist)
        {
            mindist = dist; 
            best_trbox = trbox;
        }
    }
    pthread_mutex_unlock(&face_vector_mutex);//________________UNLOCK________________________________
  
    // no face detected
    if (mindist >= 100000)
        return BB_NO_FACE;
  
    // calculate displace of face center from frame center
    y = best_trbox.box.y + best_trbox.box.height/2 - H/2.0;
    y = y*BB_VFOV/H;
    sign = float(y>=0)*2 - 1;
  
    x = best_trbox.box.x + best_trbox.box.width/2 - W/2.0;
    x = x*BB_HFOV/W;
  
    // cannot move head further
    if ((g_headPos+sign < BB_HEAD_MIN_LIMIT) or (g_headPos+sign > BB_HEAD_MAX_LIMIT))
        return BB_FACE_LIMIT;
  
    // close enough to center
    if ((abs(y) < BB_FOLLOW_TOLERANCE * H) and (abs(x) < BB_FOLLOW_TOLERANCE * W))
    {
        // too small face
        if (best_trbox.box.height * best_trbox.box.width / (H*W) < BB_FACE_AREA)
            return BB_FACE_FAR;
        else
            return BB_FOUND_FACE;
    }
  
    // rotate platform and move head asynchronously
    auto waitRotation = std::async(std::launch::async, rotatePlatform, x);
    driveHead(y, 0.1);
    waitRotation.wait();
    delay(200);
  
    // wait for next processed frame just in case
    while ( *(pointers->face_processed))
        delay(10);
    *(pointers->face_processed) = true;
  
    // moving robot
    return BB_FACE_BUSY;
}
*/

int main( int argc, char** argv )
{
    is_running = true;
    face_processed = true;
    int found_face = BB_NO_FACE;
    
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
    
    usleep(1000000); // wait for nodes
    
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
    
    while (is_running && ros::ok())
    {
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
