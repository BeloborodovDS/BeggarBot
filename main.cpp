#include <iostream>
#include <fstream>
#include <set>
#include <cstring>
#include <future>
#include <atomic>
#include <unistd.h>

#include <pthread.h>

#include "pca9685.h"
#include <wiringPi.h>
#include "submodules/mcp3008/mcp3008Spi.h"

#include "constants.h"

using namespace std;

std::atomic_bool is_running;  //is used to stop threads from main()
pthread_mutex_t face_vector_mutex;    //mutex for shared face vectors (probs and faces)

float g_headPos; //head position

//drive servo PIN at angle ANGLE for SG90 servo
//angle: [0,180], pin: PIN_BASE(controller)+SERVO_NUM(0-15) or PIN_BASE(controller)+16 for all servos
void driveDegs(float angle, int pin)
{
  if (angle<0) angle = 0;
  if (angle>180) angle = 180;
  int ticks = (int)(BB_PCA_MAX_PWM * (angle/180.0f*(BB_SERVO_MS_MAX-BB_SERVO_MS_MIN) + BB_SERVO_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(pin, ticks);
}

//drive servo by pulse width value
void driveMs(float Ms, int pin)
{
  float cycleMs = 1000.0f / BB_PCA_HERTZ;
  int ticks = (int)(BB_PCA_MAX_PWM * Ms / cycleMs + 0.5f);
  pwmWrite(pin, ticks);
}

//move eyebrows: 
// 0 for neutral expression
// >0 for kind expression
// <0 for angry expression
void frown(int mode)
{
  if(mode==0)
    driveDegs(60, BB_PIN_EYEBROW);
  else if (mode>0)
    driveDegs(25, BB_PIN_EYEBROW);
  else
    driveDegs(100, BB_PIN_EYEBROW);
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


//Drive head from g_headPos to g_headPos+delta_angle with specified speed
//speed is ratio from SERVO_MAX_SPEED, in [0,1]
void driveHead(float delta_angle, float speed = 0.1)
{
  // limits
  if (speed > 1) speed = 1;
  if (speed < 0) speed = 0;
  float an_from = g_headPos, an_to = g_headPos+delta_angle;
  if (an_to > BB_HEAD_MAX_LIMIT) an_to = BB_HEAD_MAX_LIMIT;
  if (an_to < BB_HEAD_MIN_LIMIT) an_to = BB_HEAD_MIN_LIMIT;
  if (an_from < 0) an_from = 0;
  if (an_from > 180) an_from = 180;
  
  float step = BB_HEAD_MAX_STEP*speed; //deg step for each time span
  
  if (an_from < an_to)  
    while(an_from < an_to) //forward
    {
      an_from += step;
      driveDegs(an_from, BB_PIN_HEAD);
      delay(BB_HEAD_DT);
    }
  else
    while(an_from > an_to) //backwards
    {
      an_from -= step;
      driveDegs(an_from, BB_PIN_HEAD);
      delay(BB_HEAD_DT);
    }
    
  //fix imprecise position
  driveDegs(an_to, BB_PIN_HEAD);
  g_headPos = an_to;
  
  delay(BB_HEAD_DT);
}

//speed: [-1, 1]: 1 => full forward, -1 => full backwards 
void setSpeedLeft(float speed)
{
  if (speed > 1) speed = 1;
  if (speed < -1) speed = -1;
  int ticks = (int)(BB_PCA_MAX_PWM * (0.5*(speed+1)*(BB_CONT_MS_MAX-BB_CONT_MS_MIN) + BB_CONT_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(BB_PIN_LEFT_MOTOR, ticks);
}

//speed: [-1, 1]: 1 => full forward, -1 => full backwards 
void setSpeedRight(float speed)
{
  if (speed > 1) speed = 1;
  if (speed < -1) speed = -1;
  speed *= -1; //symmetry
  int ticks = (int)(BB_PCA_MAX_PWM * (0.5*(speed+1)*(BB_CONT_MS_MAX-BB_CONT_MS_MIN) + BB_CONT_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(BB_PIN_RIGHT_MOTOR, ticks);
}

//set speed for left and right wheels
void setSpeed(float speed_left, float speed_right)
{
  setSpeedLeft(speed_left);
  setSpeedRight(speed_right);  
}

//rotate platform: > 0: clockwise, < 0: counter-clockwise
void rotatePlatform(float degrees)
{
    float sign = 2*float(degrees >= 0)  - 1;
    setSpeedLeft(sign);
    setSpeedRight(-sign);
    delay(1000 * sign * degrees / BB_DEG_PER_SECOND);
    setSpeedLeft(0);
    setSpeedRight(0);
}

//set initial position
void resetRobot()
{
  driveDegs(BB_HEAD_INIT_POS, BB_PIN_HEAD);
  g_headPos = BB_HEAD_INIT_POS;
  frown(0);
  driveDegs(45, BB_PIN_ARM);
  delay(2000);
}

// find closest to center face and try to align to it
int follow_face(thread_pointers_t* pointers)
{
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

int main( int argc, char** argv )
{    
    //------------------------------------------------INIT-----------------------------------------------------------------
    is_running = true; //will be used to terminate threads
    
    int found_face = BB_NO_FACE;
    
    std::atomic_bool face_processed (true);
  
    //setup WiringPi
    wiringPiSetup();
    
    // Setup PCA with pinbase 300 and i2c location 0x40 (default for pca9685)
    // PWM period for SG90 servos is 20ms (50Hz)
    int pca_fd = pca9685Setup(BB_PCA_PIN_BASE, 0x40, BB_PCA_HERTZ);
    if (pca_fd < 0)
    {
        cout<<"Error in init PCA9685!"<<endl;
        return 0;
    }
    // Reset all output
    pca9685PWMReset(pca_fd);
    cout<<"PCA9685 controller connected"<<endl;
    
    resetRobot();
    cout<<"Robot reset"<<endl;
    
    //-------------------------------------------------MAIN BODY-----------------------------------------------------------

    
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
    
    setSpeed(0, 0);


    //--------------------------------------------RESET--------------------------------------------------
    
    resetRobot();
    cout<<"Final reset."<<endl;
    
    pca9685PWMReset(pca_fd);
    
    return 0;
}
