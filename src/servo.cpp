#include <future>
#include <cmath>

#include "ros/ros.h"
#include "beggar_bot/ServoAction.h"
#include "beggar_bot/ServoSpeed.h"
#include "beggar_bot/ServoHeadPlatform.h"

#include "wiringPi.h"
#include "pca9685.h"

#include "../include/constants.h"

using namespace beggar_bot;
using namespace std;

float g_headPos; //head position

// drive servo PIN to an ANGLE for SG90 or FS5109M servo
// angle: [0,180], pin: PIN_BASE(controller)+SERVO_NUM(0-15) or PIN_BASE(controller)+16 for all servos
void driveDegs(float angle, int pin)
{
    if (angle<0) angle = 0;
    if (angle>180) angle = 180;
    // 20 ms is pulse width (~50 hz)
    int ticks = (int) (BB_PCA_MAX_PWM * 
        (angle/180.0f*(BB_SERVO_MS_MAX-BB_SERVO_MS_MIN) + BB_SERVO_MS_MIN) / 20.0f + 0.5); 
    pwmWrite(pin, ticks);
}

//move eyebrows
void frown(int mode)
{
    if (mode == BB_ACTION_FROWN_NEUTRAL)
        driveDegs(60, BB_PIN_EYEBROW);
    else if (mode == BB_ACTION_FROWN_KIND)
        driveDegs(25, BB_PIN_EYEBROW);
    else if (mode == BB_ACTION_FROWN_ANGRY)
        driveDegs(100, BB_PIN_EYEBROW);
    else
        ROS_ERROR("Servo node: unknown frown mode %s", mode);
    delay(1000);
}

//shake hand
void shake()
{
    driveDegs(65, BB_PIN_ARM);
    delay(500);
    driveDegs(90, BB_PIN_ARM);
    delay(500);
    driveDegs(115, BB_PIN_ARM);
    delay(500);
    driveDegs(90, BB_PIN_ARM);
    delay(500);
}

// Drive head from g_headPos to g_headPos+delta_angle with specified speed
// speed is ratio from SERVO_MAX_SPEED, in [0,1]
bool driveHead(float delta_angle, float speed = 0.1)
{
    // on the limit - cannot move in this direction
    float sign = 1e-3 * (float(delta_angle>=0) * 2 - 1);
    if (g_headPos+sign < BB_HEAD_MIN_LIMIT or g_headPos+sign > BB_HEAD_MAX_LIMIT)
        return false;
    
    if (speed > 1) speed = 1;
    if (speed < 0) speed = 0;
    float an_from = g_headPos, an_to = g_headPos+delta_angle;
    if (an_to > BB_HEAD_MAX_LIMIT) an_to = BB_HEAD_MAX_LIMIT;
    if (an_to < BB_HEAD_MIN_LIMIT) an_to = BB_HEAD_MIN_LIMIT;
    
    float step = BB_HEAD_MAX_STEP*speed; //deg step for each time span
    
    if (an_from < an_to)
        while(an_from < an_to) // forward
        {
            an_from += step;
            driveDegs(an_from, BB_PIN_HEAD);
            delay(BB_HEAD_DT);
        }
    else
        while(an_from > an_to) // backwards
        {
            an_from -= step;
            driveDegs(an_from, BB_PIN_HEAD);
            delay(BB_HEAD_DT);
        }
    
    // fix imprecise position
    driveDegs(an_to, BB_PIN_HEAD);
    g_headPos = an_to;
    delay(BB_HEAD_DT);
    
    return true;
}

// speed: [-1, 1]: 1 => full forward, -1 => full backwards 
// motor:  BB_PIN_RIGHT_MOTOR or BB_PIN_LEFT_MOTOR
void setWheelSpeed(float speed, int motor)
{
    if (speed > 1) speed = 1;
    if (speed < -1) speed = -1;
    //symmetry
    if (motor == BB_PIN_RIGHT_MOTOR)
        speed *= -1;
    int ticks = (int) (BB_PCA_MAX_PWM * (0.5*(speed+1)*(BB_CONT_MS_MAX-BB_CONT_MS_MIN) +
        BB_CONT_MS_MIN) / 20.0f + 0.5); // 20ms is pulse width (~50 hz)
    pwmWrite(motor, ticks);
}

//rotate platform: > 0: clockwise, < 0: counter-clockwise
void rotatePlatform(float degrees)
{
    float sign = 2*float(degrees >= 0)  - 1;
    setWheelSpeed(sign, BB_PIN_LEFT_MOTOR);
    setWheelSpeed(-sign, BB_PIN_RIGHT_MOTOR);
    delay(1000 * sign * degrees / BB_DEG_PER_SECOND);
    setWheelSpeed(0, BB_PIN_LEFT_MOTOR);
    setWheelSpeed(0, BB_PIN_RIGHT_MOTOR);
}

// set initial positions
void reset()
{
    driveDegs(BB_HEAD_INIT_POS, BB_PIN_HEAD);
    g_headPos = BB_HEAD_INIT_POS;
    
    frown(BB_ACTION_FROWN_NEUTRAL);
    driveDegs(45, BB_PIN_ARM);
    
    setWheelSpeed(0, BB_PIN_LEFT_MOTOR);
    setWheelSpeed(0, BB_PIN_RIGHT_MOTOR);
    
    delay(2000);
}

bool servo_action(ServoAction::Request  &req, ServoAction::Response &ignored)
{
    if (req.action == BB_ACTION_RESET)
        reset();
    else if (req.action == BB_ACTION_RESET_HEAD)
        driveHead(BB_HEAD_INIT_POS - g_headPos);
    else if (req.action == BB_ACTION_SHAKE)
        shake();
    else
        frown(req.action);
    return true;
}

bool servo_speed(ServoSpeed::Request  &req, ServoSpeed::Response &ignored)
{
    setWheelSpeed(req.left, BB_PIN_LEFT_MOTOR);
    setWheelSpeed(req.right, BB_PIN_RIGHT_MOTOR);  
    return true;
}

bool servo_head_platform(ServoHeadPlatform::Request  &req, ServoHeadPlatform::Response &res)
{
    float head = req.head_delta, platform = req.platform_delta;
    bool success = true;
    
    // if both specified, rotate platform and move head asynchronously
    if (abs(head) > 1e-3 && abs(platform) > 1e-3) 
    {
        auto waitRotation = std::async(std::launch::async, rotatePlatform, platform);
        success = driveHead(head);
        waitRotation.wait();
    }
    else if (abs(head) > 1e-3)
        success = driveHead(head);
    else if (abs(platform) > 1e-3)
        rotatePlatform(platform);
    
    res.head_success = success;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_node");
    ros::NodeHandle n;
    ros::ServiceServer action_service = n.advertiseService("servo_action", servo_action);
    ros::ServiceServer speed_service = n.advertiseService("servo_speed", servo_speed);
    ros::ServiceServer head_platform_service = n.advertiseService("servo_head_platform", servo_head_platform);
    
    //setup WiringPi
    wiringPiSetup();
    
    // Setup PCA with pinbase 300 and i2c location 0x40 (default for pca9685)
    // PWM period for servos is 20ms (50Hz)
    int pca_fd = pca9685Setup(BB_PCA_PIN_BASE, 0x40, BB_PCA_HERTZ);
    if (pca_fd < 0)
    {
        ROS_ERROR("Servo node: failed to initialize PCA9685");
        return 0;
    }
    
    pca9685PWMReset(pca_fd);
    reset();
    
    ROS_INFO("Servo node and PCA9685 initialized");
    ros::spin();
    
    reset();
    pca9685PWMReset(pca_fd);
    
    return 0;
}
